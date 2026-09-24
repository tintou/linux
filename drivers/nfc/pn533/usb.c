// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for NXP PN533 NFC Chip - USB transport layer
 *
 * Copyright (C) 2011 Instituto Nokia de Tecnologia
 * Copyright (C) 2012-2013 Tieto Poland
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/nfc.h>
#include <linux/netdevice.h>
#include <net/nfc/nfc.h>
#include "pn533.h"

#define VERSION "0.1"

#define PN533_VENDOR_ID 0x4CC
#define PN533_PRODUCT_ID 0x2533

#define SCM_VENDOR_ID 0x4E6
#define SCL3711_PRODUCT_ID 0x5591

#define SONY_VENDOR_ID         0x054c
#define PASORI_PRODUCT_ID      0x02e1

#define ACS_VENDOR_ID 0x072f
#define ACR122U_PRODUCT_ID 0x2200

#define NSR106_VENDOR_ID    0x0416
#define NSR106_PRODUCT_ID_B008 0xb008
#define NSR106_PRODUCT_ID_B029 0xb029
#define NSR106_PRODUCT_ID_B030 0xb030
#define NSR106_PRODUCT_ID_B058 0xb058

/* NSR106 uses 64-byte fixed-size HID interrupt transfers */
#define NSR106_FRAME_SIZE 64

static const struct usb_device_id pn533_usb_table[] = {
	{ USB_DEVICE(PN533_VENDOR_ID, PN533_PRODUCT_ID),
	  .driver_info = PN533_DEVICE_STD },
	{ USB_DEVICE(SCM_VENDOR_ID, SCL3711_PRODUCT_ID),
	  .driver_info = PN533_DEVICE_STD },
	{ USB_DEVICE(SONY_VENDOR_ID, PASORI_PRODUCT_ID),
	  .driver_info = PN533_DEVICE_PASORI },
	{ USB_DEVICE(ACS_VENDOR_ID, ACR122U_PRODUCT_ID),
	  .driver_info = PN533_DEVICE_ACR122U },
	{ USB_DEVICE(NSR106_VENDOR_ID, NSR106_PRODUCT_ID_B008),
	  .driver_info = PN533_DEVICE_NSR106 },
	{ USB_DEVICE(NSR106_VENDOR_ID, NSR106_PRODUCT_ID_B029),
	  .driver_info = PN533_DEVICE_NSR106 },
	{ USB_DEVICE(NSR106_VENDOR_ID, NSR106_PRODUCT_ID_B030),
	  .driver_info = PN533_DEVICE_NSR106 },
	{ USB_DEVICE(NSR106_VENDOR_ID, NSR106_PRODUCT_ID_B058),
	  .driver_info = PN533_DEVICE_NSR106 },
	{ }
};
MODULE_DEVICE_TABLE(usb, pn533_usb_table);

/* NSR106 frames encapsulate PN532 commands and append APDU status to replies. */

#define NSR106_TX_FRAME_HEADER_LEN \
	(sizeof(struct pn533_nsr106_tx_frame) + 1) /* +1 for cmd_code in data[0] */
#define NSR106_TX_FRAME_TAIL_LEN   2 /* checksum + end marker */
#define NSR106_RX_FRAME_HEADER_LEN \
	(sizeof(struct pn533_nsr106_rx_frame) + 1) /* +1 for cmd+1 in data[0] */
#define NSR106_RX_FRAME_TAIL_LEN   4 /* APDU SW(2) + checksum(1) + end marker(1) */
#define NSR106_MAX_PAYLOAD_LEN \
	(NSR106_FRAME_SIZE - NSR106_TX_FRAME_HEADER_LEN - NSR106_TX_FRAME_TAIL_LEN)

struct pn533_nsr106_tx_frame {
	u8 type;        /* 0x01 = host→device command */
	u8 len;         /* total frame length */
	__le16 seq;     /* sequence counter */
	u8 preamble;    /* 0xFF */
	u8 reserved[3]; /* 0x00 */
	u8 pn532_len;   /* PN532 content length: TFI(1) + cmd(1) + params */
	u8 tfi;         /* 0xD4 = host→device TFI */
	u8 data[];      /* data[0] = cmd_code, data[1..] = parameters */
} __packed;

struct pn533_nsr106_rx_frame {
	u8 type;        /* 0x02 = device→host response */
	u8 len;         /* total frame length */
	__le16 seq;     /* echo of request sequence */
	u8 tfi;         /* 0xD5 = device→host TFI */
	u8 data[];      /* data[0] = cmd+1, then payload, APDU SW, checksum, 0xFD */
} __packed;

struct pn533_usb_phy {
	struct usb_device *udev;
	struct usb_interface *interface;

	struct urb *out_urb;
	struct urb *in_urb;

	struct urb *ack_urb;
	u8 *ack_buffer;

	u16 nsr106_seq;    /* TX sequence counter, incremented by 2 per frame */

	struct pn533 *priv;
};

static void pn533_recv_response(struct urb *urb)
{
	struct pn533_usb_phy *phy = urb->context;
	struct sk_buff *skb = NULL;

	if (!urb->status) {
		skb = alloc_skb(urb->actual_length, GFP_ATOMIC);
		if (!skb) {
			nfc_err(&phy->udev->dev, "failed to alloc memory\n");
		} else {
			skb_put_data(skb, urb->transfer_buffer,
				     urb->actual_length);
		}
	}

	pn533_recv_frame(phy->priv, skb, urb->status);
}

static int pn533_submit_urb_for_response(struct pn533_usb_phy *phy, gfp_t flags)
{
	phy->in_urb->complete = pn533_recv_response;

	return usb_submit_urb(phy->in_urb, flags);
}

static void pn533_recv_ack(struct urb *urb)
{
	struct pn533_usb_phy *phy = urb->context;
	struct pn533 *priv = phy->priv;
	struct pn533_cmd *cmd = priv->cmd;
	struct pn533_std_frame *in_frame;
	int rc;

	cmd->status = urb->status;

	switch (urb->status) {
	case 0:
		break; /* success */
	case -ECONNRESET:
	case -ENOENT:
		dev_dbg(&phy->udev->dev,
			"The urb has been stopped (status %d)\n",
			urb->status);
		goto sched_wq;
	case -ESHUTDOWN:
	default:
		nfc_err(&phy->udev->dev,
			"Urb failure (status %d)\n", urb->status);
		goto sched_wq;
	}

	in_frame = phy->in_urb->transfer_buffer;

	if (!pn533_rx_frame_is_ack(in_frame)) {
		nfc_err(&phy->udev->dev, "Received an invalid ack\n");
		cmd->status = -EIO;
		goto sched_wq;
	}

	rc = pn533_submit_urb_for_response(phy, GFP_ATOMIC);
	if (rc) {
		nfc_err(&phy->udev->dev,
			"usb_submit_urb failed with result %d\n", rc);
		cmd->status = rc;
		goto sched_wq;
	}

	return;

sched_wq:
	queue_work(priv->wq, &priv->cmd_complete_work);
}

static int pn533_submit_urb_for_ack(struct pn533_usb_phy *phy, gfp_t flags)
{
	phy->in_urb->complete = pn533_recv_ack;

	return usb_submit_urb(phy->in_urb, flags);
}

static int pn533_usb_send_ack(struct pn533 *dev, gfp_t flags)
{
	struct pn533_usb_phy *phy = dev->phy;
	static const u8 ack[6] = {0x00, 0x00, 0xff, 0x00, 0xff, 0x00};
	/* spec 7.1.1.3:  Preamble, SoPC (2), ACK Code (2), Postamble */

	if (!phy->ack_buffer) {
		phy->ack_buffer = kmemdup(ack, sizeof(ack), flags);
		if (!phy->ack_buffer)
			return -ENOMEM;
	}

	phy->ack_urb->transfer_buffer = phy->ack_buffer;
	phy->ack_urb->transfer_buffer_length = sizeof(ack);
	return usb_submit_urb(phy->ack_urb, flags);
}

struct pn533_out_arg {
	struct pn533_usb_phy *phy;
	struct completion done;
};

static int pn533_usb_send_frame(struct pn533 *dev,
				struct sk_buff *out)
{
	struct pn533_usb_phy *phy = dev->phy;
	struct pn533_nsr106_tx_frame *nsr106_tx = NULL;
	struct pn533_out_arg arg;
	void *cntx;
	int rc;

	if (phy->priv == NULL)
		phy->priv = dev;

	if (dev->device_type == PN533_DEVICE_NSR106) {
		/*
		 * Copy the variable-length sk_buff into a kmalloc'd buffer
		 * (DMA-safe), zero-padded to the transfer size.
		 */
		u8 flen, cksum;
		int i;

		if (WARN_ON(out->len > NSR106_FRAME_SIZE))
			return -EMSGSIZE;

		nsr106_tx = kzalloc(NSR106_FRAME_SIZE, GFP_KERNEL);
		if (!nsr106_tx)
			return -ENOMEM;

		memcpy(nsr106_tx, out->data, out->len);

		/* Stamp per-transfer sequence counter (incremented by 2 per frame) */
		nsr106_tx->seq = cpu_to_le16(phy->nsr106_seq);
		phy->nsr106_seq += 2;

		/* Recompute checksum after modifying seq */
		flen  = nsr106_tx->len;
		cksum = 0;
		for (i = 0; i < flen - 2; i++)
			cksum += ((u8 *)nsr106_tx)[i];
		((u8 *)nsr106_tx)[flen - 2] = cksum ^ 0xFF;

		phy->out_urb->transfer_buffer = nsr106_tx;
		phy->out_urb->transfer_buffer_length = NSR106_FRAME_SIZE;
	} else {
		phy->out_urb->transfer_buffer = out->data;
		phy->out_urb->transfer_buffer_length = out->len;
	}

	print_hex_dump_debug("PN533 TX: ", DUMP_PREFIX_NONE, 16, 1,
			     out->data, out->len, false);

	arg.phy = phy;
	init_completion(&arg.done);
	cntx = phy->out_urb->context;
	phy->out_urb->context = &arg;

	rc = usb_submit_urb(phy->out_urb, GFP_KERNEL);
	if (rc) {
		kfree(nsr106_tx);
		return rc;
	}

	wait_for_completion(&arg.done);
	kfree(nsr106_tx);
	phy->out_urb->context = cntx;

	if (dev->protocol_type == PN533_PROTO_REQ_RESP) {
		/* request for response for sent packet directly */
		rc = pn533_submit_urb_for_response(phy, GFP_KERNEL);
		if (rc)
			goto error;
	} else if (dev->protocol_type == PN533_PROTO_REQ_ACK_RESP) {
		/* request for ACK if that's the case */
		rc = pn533_submit_urb_for_ack(phy, GFP_KERNEL);
		if (rc)
			goto error;
	}

	return 0;

error:
	usb_unlink_urb(phy->out_urb);
	return rc;
}

static void pn533_usb_abort_cmd(struct pn533 *dev, gfp_t flags)
{
	struct pn533_usb_phy *phy = dev->phy;

	/* ACR122U and NSR106 do not support abort commands: they don't have an
	 * ACK mechanism and behave incorrectly when the in_urb is cancelled
	 * before the response arrives.
	 */
	if (dev->device_type == PN533_DEVICE_ACR122U ||
	    dev->device_type == PN533_DEVICE_NSR106)
		return;

	/* An ack will cancel the last issued command */
	pn533_usb_send_ack(dev, flags);

	/* cancel the urb request */
	usb_kill_urb(phy->in_urb);
}

static void pn533_nsr106_tx_frame_init(void *_frame, u8 cmd_code)
{
	struct pn533_nsr106_tx_frame *frame = _frame;

	frame->type     = 0x01;
	frame->len      = NSR106_TX_FRAME_HEADER_LEN + NSR106_TX_FRAME_TAIL_LEN;
	frame->seq      = 0;
	frame->preamble = 0xFF;
	memset(frame->reserved, 0, sizeof(frame->reserved));
	frame->pn532_len = 2; /* TFI + cmd_code */
	frame->tfi      = PN533_STD_FRAME_DIR_OUT;
	frame->data[0]  = cmd_code;
}

static void pn533_nsr106_tx_frame_finish(void *_frame)
{
	struct pn533_nsr106_tx_frame *frame = _frame;
	u8 *buf = _frame;
	u8 len = frame->len;
	u8 cksum = 0;
	int i;

	for (i = 0; i < len - 2; i++)
		cksum += buf[i];
	buf[len - 2] = cksum ^ 0xFF;
	buf[len - 1] = 0xFE;
}

static void pn533_nsr106_tx_update_payload_len(void *_frame, int len)
{
	struct pn533_nsr106_tx_frame *frame = _frame;

	frame->len      += len;
	frame->pn532_len += len;
}

static bool pn533_nsr106_is_rx_frame_valid(void *_frame, struct pn533 *dev)
{
	struct pn533_nsr106_rx_frame *frame = _frame;
	u8 *buf = _frame;
	u8 len = frame->len;
	u8 cksum = 0;
	int i;

	if (frame->type != 0x02)
		return false;

	if (frame->tfi != PN533_STD_FRAME_DIR_IN)
		return false;

	if (len < NSR106_RX_FRAME_HEADER_LEN + NSR106_RX_FRAME_TAIL_LEN)
		return false;

	if (buf[len - 1] != 0xFD)
		return false;

	for (i = 0; i < len - 2; i++)
		cksum += buf[i];
	if ((cksum ^ 0xFF) != buf[len - 2])
		return false;

	return true;
}

static int pn533_nsr106_rx_frame_size(void *_frame)
{
	struct pn533_nsr106_rx_frame *frame = _frame;

	return frame->len;
}

static u8 pn533_nsr106_get_cmd_code(void *_frame)
{
	struct pn533_nsr106_rx_frame *frame = _frame;

	return frame->data[0]; /* response command = sent_cmd + 1 */
}

static struct pn533_frame_ops pn533_nsr106_frame_ops = {
	.tx_frame_init          = pn533_nsr106_tx_frame_init,
	.tx_frame_finish        = pn533_nsr106_tx_frame_finish,
	.tx_update_payload_len  = pn533_nsr106_tx_update_payload_len,
	.tx_header_len          = NSR106_TX_FRAME_HEADER_LEN,
	.tx_tail_len            = NSR106_TX_FRAME_TAIL_LEN,

	.rx_is_frame_valid      = pn533_nsr106_is_rx_frame_valid,
	.rx_header_len          = NSR106_RX_FRAME_HEADER_LEN,
	.rx_tail_len            = NSR106_RX_FRAME_TAIL_LEN,
	.rx_frame_size          = pn533_nsr106_rx_frame_size,

	.max_payload_len        = NSR106_MAX_PAYLOAD_LEN,
	.get_cmd_code           = pn533_nsr106_get_cmd_code,
};

/* ACR122 specific structs and functions */

/* ACS ACR122 pn533 frame definitions */
#define PN533_ACR122_TX_FRAME_HEADER_LEN (sizeof(struct pn533_acr122_tx_frame) \
					  + 2)
#define PN533_ACR122_TX_FRAME_TAIL_LEN 0
#define PN533_ACR122_RX_FRAME_HEADER_LEN (sizeof(struct pn533_acr122_rx_frame) \
					  + 2)
#define PN533_ACR122_RX_FRAME_TAIL_LEN 2
#define PN533_ACR122_FRAME_MAX_PAYLOAD_LEN PN533_STD_FRAME_MAX_PAYLOAD_LEN

/* CCID messages types */
#define PN533_ACR122_PC_TO_RDR_ICCPOWERON 0x62
#define PN533_ACR122_PC_TO_RDR_ESCAPE 0x6B

#define PN533_ACR122_RDR_TO_PC_ESCAPE 0x83


struct pn533_acr122_ccid_hdr {
	u8 type;
	u32 datalen;
	u8 slot;
	u8 seq;

	/*
	 * 3 msg specific bytes or status, error and 1 specific
	 * byte for reposnse msg
	 */
	u8 params[3];
} __packed;

struct pn533_acr122_apdu_hdr {
	u8 class;
	u8 ins;
	u8 p1;
	u8 p2;
} __packed;

struct pn533_acr122_tx_frame {
	struct pn533_acr122_ccid_hdr ccid;
	struct pn533_acr122_apdu_hdr apdu;
	u8 datalen;
	u8 data[]; /* pn533 frame: TFI ... */
} __packed;

struct pn533_acr122_rx_frame {
	struct pn533_acr122_ccid_hdr ccid;
	u8 data[]; /* pn533 frame : TFI ... */
} __packed;

static void pn533_acr122_tx_frame_init(void *_frame, u8 cmd_code)
{
	struct pn533_acr122_tx_frame *frame = _frame;

	frame->ccid.type = PN533_ACR122_PC_TO_RDR_ESCAPE;
	/* sizeof(apdu_hdr) + sizeof(datalen) */
	frame->ccid.datalen = sizeof(frame->apdu) + 1;
	frame->ccid.slot = 0;
	frame->ccid.seq = 0;
	frame->ccid.params[0] = 0;
	frame->ccid.params[1] = 0;
	frame->ccid.params[2] = 0;

	frame->data[0] = PN533_STD_FRAME_DIR_OUT;
	frame->data[1] = cmd_code;
	frame->datalen = 2;  /* data[0] + data[1] */

	frame->apdu.class = 0xFF;
	frame->apdu.ins = 0;
	frame->apdu.p1 = 0;
	frame->apdu.p2 = 0;
}

static void pn533_acr122_tx_frame_finish(void *_frame)
{
	struct pn533_acr122_tx_frame *frame = _frame;

	frame->ccid.datalen += frame->datalen;
}

static void pn533_acr122_tx_update_payload_len(void *_frame, int len)
{
	struct pn533_acr122_tx_frame *frame = _frame;

	frame->datalen += len;
}

static bool pn533_acr122_is_rx_frame_valid(void *_frame, struct pn533 *dev)
{
	struct pn533_acr122_rx_frame *frame = _frame;

	if (frame->ccid.type != 0x83)
		return false;

	if (!frame->ccid.datalen)
		return false;

	if (frame->data[frame->ccid.datalen - 2] == 0x63)
		return false;

	return true;
}

static int pn533_acr122_rx_frame_size(void *frame)
{
	struct pn533_acr122_rx_frame *f = frame;

	/* f->ccid.datalen already includes tail length */
	return sizeof(struct pn533_acr122_rx_frame) + f->ccid.datalen;
}

static u8 pn533_acr122_get_cmd_code(void *frame)
{
	struct pn533_acr122_rx_frame *f = frame;

	return PN533_FRAME_CMD(f);
}

static struct pn533_frame_ops pn533_acr122_frame_ops = {
	.tx_frame_init = pn533_acr122_tx_frame_init,
	.tx_frame_finish = pn533_acr122_tx_frame_finish,
	.tx_update_payload_len = pn533_acr122_tx_update_payload_len,
	.tx_header_len = PN533_ACR122_TX_FRAME_HEADER_LEN,
	.tx_tail_len = PN533_ACR122_TX_FRAME_TAIL_LEN,

	.rx_is_frame_valid = pn533_acr122_is_rx_frame_valid,
	.rx_header_len = PN533_ACR122_RX_FRAME_HEADER_LEN,
	.rx_tail_len = PN533_ACR122_RX_FRAME_TAIL_LEN,
	.rx_frame_size = pn533_acr122_rx_frame_size,

	.max_payload_len = PN533_ACR122_FRAME_MAX_PAYLOAD_LEN,
	.get_cmd_code = pn533_acr122_get_cmd_code,
};

struct pn533_acr122_poweron_rdr_arg {
	int rc;
	struct completion done;
};

static void pn533_acr122_poweron_rdr_resp(struct urb *urb)
{
	struct pn533_acr122_poweron_rdr_arg *arg = urb->context;

	print_hex_dump_debug("ACR122 RX: ", DUMP_PREFIX_NONE, 16, 1,
		       urb->transfer_buffer, urb->transfer_buffer_length,
		       false);

	arg->rc = urb->status;
	complete(&arg->done);
}

static int pn533_acr122_poweron_rdr(struct pn533_usb_phy *phy)
{
	/* Power on th reader (CCID cmd) */
	u8 cmd[10] = {PN533_ACR122_PC_TO_RDR_ICCPOWERON,
		      0, 0, 0, 0, 0, 0, 3, 0, 0};
	char *buffer;
	int transferred;
	int rc;
	void *cntx;
	struct pn533_acr122_poweron_rdr_arg arg;

	buffer = kmemdup(cmd, sizeof(cmd), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	init_completion(&arg.done);
	cntx = phy->in_urb->context;  /* backup context */

	phy->in_urb->complete = pn533_acr122_poweron_rdr_resp;
	phy->in_urb->context = &arg;

	print_hex_dump_debug("ACR122 TX: ", DUMP_PREFIX_NONE, 16, 1,
		       cmd, sizeof(cmd), false);

	rc = usb_bulk_msg(phy->udev, phy->out_urb->pipe, buffer, sizeof(cmd),
			  &transferred, 5000);
	kfree(buffer);
	if (rc || (transferred != sizeof(cmd))) {
		nfc_err(&phy->udev->dev,
			"Reader power on cmd error %d\n", rc);
		return rc ?: -EINVAL;
	}

	rc =  usb_submit_urb(phy->in_urb, GFP_KERNEL);
	if (rc) {
		nfc_err(&phy->udev->dev,
			"Can't submit reader poweron cmd response %d\n", rc);
		return rc;
	}

	wait_for_completion(&arg.done);
	phy->in_urb->context = cntx; /* restore context */

	return arg.rc;
}

static void pn533_out_complete(struct urb *urb)
{
	struct pn533_out_arg *arg = urb->context;
	struct pn533_usb_phy *phy = arg->phy;

	switch (urb->status) {
	case 0:
		break; /* success */
	case -ECONNRESET:
	case -ENOENT:
		dev_dbg(&phy->udev->dev,
			"The urb has been stopped (status %d)\n",
			urb->status);
		break;
	case -ESHUTDOWN:
	default:
		nfc_err(&phy->udev->dev,
			"Urb failure (status %d)\n",
			urb->status);
	}

	complete(&arg->done);
}

static void pn533_ack_complete(struct urb *urb)
{
	struct pn533_usb_phy *phy = urb->context;

	switch (urb->status) {
	case 0:
		break; /* success */
	case -ECONNRESET:
	case -ENOENT:
		dev_dbg(&phy->udev->dev,
			"The urb has been stopped (status %d)\n",
			urb->status);
		break;
	case -ESHUTDOWN:
	default:
		nfc_err(&phy->udev->dev,
			"Urb failure (status %d)\n",
			urb->status);
	}
}

static const struct pn533_phy_ops usb_phy_ops = {
	.send_frame = pn533_usb_send_frame,
	.send_ack = pn533_usb_send_ack,
	.abort_cmd = pn533_usb_abort_cmd,
};

static int pn533_usb_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *ep_in, *ep_out;
	struct pn533 *priv;
	struct pn533_usb_phy *phy;
	u32 protocols;
	enum pn533_protocol_type protocol_type = PN533_PROTO_REQ_ACK_RESP;
	struct pn533_frame_ops *fops = NULL;
	unsigned char *in_buf;
	int in_buf_len = PN533_EXT_FRAME_HEADER_LEN +
			 PN533_STD_FRAME_MAX_PAYLOAD_LEN +
			 PN533_STD_FRAME_TAIL_LEN;
	int rc;

	/*
	 * The NSR106 is a composite device (HID NFC + mass storage).
	 * Only claim the HID interface.
	 */
	if (id->driver_info == PN533_DEVICE_NSR106 &&
	    interface->cur_altsetting->desc.bInterfaceClass != USB_CLASS_HID)
		return -ENODEV;

	phy = devm_kzalloc(&interface->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	in_buf = kzalloc(in_buf_len, GFP_KERNEL);
	if (!in_buf)
		return -ENOMEM;

	phy->udev = interface_to_usbdev(interface);
	phy->interface = interface;

	phy->in_urb = usb_alloc_urb(0, GFP_KERNEL);
	phy->out_urb = usb_alloc_urb(0, GFP_KERNEL);
	phy->ack_urb = usb_alloc_urb(0, GFP_KERNEL);

	if (!phy->in_urb || !phy->out_urb || !phy->ack_urb) {
		rc = -ENOMEM;
		goto error;
	}

	if (id->driver_info == PN533_DEVICE_NSR106) {
		struct usb_endpoint_descriptor *ep_in_int, *ep_out_int;

		rc = usb_find_common_endpoints(interface->cur_altsetting,
					       NULL, NULL,
					       &ep_in_int, &ep_out_int);
		if (rc) {
			nfc_err(&interface->dev,
				"Could not find interrupt endpoints\n");
			goto error;
		}

		usb_fill_int_urb(phy->in_urb, phy->udev,
				 usb_rcvintpipe(phy->udev,
						usb_endpoint_num(ep_in_int)),
				 in_buf, NSR106_FRAME_SIZE,
				 NULL, phy, ep_in_int->bInterval);
		usb_fill_int_urb(phy->out_urb, phy->udev,
				 usb_sndintpipe(phy->udev,
						usb_endpoint_num(ep_out_int)),
				 NULL, 0, pn533_out_complete, phy,
				 ep_out_int->bInterval);
		usb_fill_int_urb(phy->ack_urb, phy->udev,
				 usb_sndintpipe(phy->udev,
						usb_endpoint_num(ep_out_int)),
				 NULL, 0, pn533_ack_complete, phy,
				 ep_out_int->bInterval);

	} else {
		rc = usb_find_common_endpoints(interface->cur_altsetting,
					       &ep_in, &ep_out, NULL, NULL);
		if (rc) {
			nfc_err(&interface->dev,
				"Could not find bulk-in or bulk-out endpoint\n");
			goto error;
		}

		usb_fill_bulk_urb(phy->in_urb, phy->udev,
				  usb_rcvbulkpipe(phy->udev,
						  usb_endpoint_num(ep_in)),
				  in_buf, in_buf_len, NULL, phy);
		usb_fill_bulk_urb(phy->out_urb, phy->udev,
				  usb_sndbulkpipe(phy->udev,
						  usb_endpoint_num(ep_out)),
				  NULL, 0, pn533_out_complete, phy);
		usb_fill_bulk_urb(phy->ack_urb, phy->udev,
				  usb_sndbulkpipe(phy->udev,
						  usb_endpoint_num(ep_out)),
				  NULL, 0, pn533_ack_complete, phy);
	}

	switch (id->driver_info) {
	case PN533_DEVICE_STD:
		protocols = PN533_ALL_PROTOCOLS;
		break;

	case PN533_DEVICE_PASORI:
		protocols = PN533_NO_TYPE_B_PROTOCOLS;
		break;

	case PN533_DEVICE_ACR122U:
		protocols = PN533_NO_TYPE_B_PROTOCOLS;
		fops = &pn533_acr122_frame_ops;
		protocol_type = PN533_PROTO_REQ_RESP;

		rc = pn533_acr122_poweron_rdr(phy);
		if (rc < 0) {
			nfc_err(&interface->dev,
				"Couldn't poweron the reader (error %d)\n", rc);
			goto error;
		}
		break;

	case PN533_DEVICE_NSR106:
		protocols = PN533_ALL_PROTOCOLS;
		fops = &pn533_nsr106_frame_ops;
		protocol_type = PN533_PROTO_REQ_RESP;
		break;

	default:
		nfc_err(&interface->dev, "Unknown device type %lu\n",
			id->driver_info);
		rc = -EINVAL;
		goto error;
	}

	priv = pn53x_common_init(id->driver_info, protocol_type,
					phy, &usb_phy_ops, fops,
					&phy->udev->dev);

	if (IS_ERR(priv)) {
		rc = PTR_ERR(priv);
		goto error;
	}

	phy->priv = priv;

	rc = pn533_finalize_setup(priv);
	if (rc)
		goto err_clean;

	usb_set_intfdata(interface, phy);
	rc = pn53x_register_nfc(priv, protocols, &interface->dev);
	if (rc)
		goto err_clean;

	return 0;

err_clean:
	pn53x_common_clean(priv);
error:
	usb_kill_urb(phy->in_urb);
	usb_kill_urb(phy->out_urb);
	usb_kill_urb(phy->ack_urb);

	usb_free_urb(phy->in_urb);
	usb_free_urb(phy->out_urb);
	usb_free_urb(phy->ack_urb);
	kfree(in_buf);
	kfree(phy->ack_buffer);

	return rc;
}

static void pn533_usb_disconnect(struct usb_interface *interface)
{
	struct pn533_usb_phy *phy = usb_get_intfdata(interface);

	if (!phy)
		return;

	pn53x_unregister_nfc(phy->priv);
	pn53x_common_clean(phy->priv);

	usb_set_intfdata(interface, NULL);

	usb_kill_urb(phy->in_urb);
	usb_kill_urb(phy->out_urb);
	usb_kill_urb(phy->ack_urb);

	kfree(phy->in_urb->transfer_buffer);
	usb_free_urb(phy->in_urb);
	usb_free_urb(phy->out_urb);
	usb_free_urb(phy->ack_urb);
	kfree(phy->ack_buffer);

	nfc_info(&interface->dev, "NXP PN533 NFC device disconnected\n");
}

static struct usb_driver pn533_usb_driver = {
	.name =		"pn533_usb",
	.probe =	pn533_usb_probe,
	.disconnect =	pn533_usb_disconnect,
	.id_table =	pn533_usb_table,
};

module_usb_driver(pn533_usb_driver);

MODULE_AUTHOR("Lauro Ramos Venancio <lauro.venancio@openbossa.org>");
MODULE_AUTHOR("Aloisio Almeida Jr <aloisio.almeida@openbossa.org>");
MODULE_AUTHOR("Waldemar Rymarkiewicz <waldemar.rymarkiewicz@tieto.com>");
MODULE_DESCRIPTION("PN533 USB driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
