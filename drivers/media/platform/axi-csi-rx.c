/*
 * axi-csi-rx.c
 *
 * Based on axi-hdmi-rx.c
 *
 * Copyright (C) 2015 Sylvain Munaut <tnt@246tNt.com>
 * Copyright (C) 2012-2013 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-event.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mediabus.h>

#include <linux/amba/xilinx_dma.h>



#define DRIVER_NAME "axi-csi-rx"

/* CSI Registers */
#define AXI_CSI_RX_REG_CR	0x00
#define AXI_CSI_RX_REG_SR	0x04
#define AXI_CSI_RX_REG_ER	0x08
#define AXI_CSI_RX_REG_IMR	0x0C
#define AXI_CSI_RX_REG_PDCR	0x10
#define AXI_CSI_RX_REG_VPCR	0x14
#define AXI_CSI_RX_REG_VCCS	0x20
#define AXI_CSI_RX_REG_VCCE	0x24
#define AXI_CSI_RX_REG_VCLS	0x28
#define AXI_CSI_RX_REG_VCLE	0x2C


#ifndef V4L2_PIX_FMT_SBGGR10P
	#define V4L2_PIX_FMT_SBGGR10P v4l2_fourcc('p', 'B', 'A', 'A')
#endif

#define to_axi_csi_rx_buffer(vb)  container_of(vb, struct axi_csi_rx_buffer, vb)


struct axi_csi_rx_fmt_desc {

	const char *desc;
	u32 vpcr;
	u32 bpp;
	enum v4l2_mbus_pixelcode mbus_code;
};

static const struct axi_csi_rx_fmt_desc axi_csi_rx_fmts[] = {
	{ "Raw Bayer SBGGR10P", 0x000000, 5, V4L2_PIX_FMT_SBGGR10P}, /* x xx xxxxxxxx 00 00 */
	{ "Bayer 10-bit BGGR",  0xa00022, 2, V4L2_PIX_FMT_SBGGR10},  /* 1 10 xxxxxxxx 10 10 */
	{ "Bayer 8-bit BGGR",   0x800023, 1, V4L2_PIX_FMT_SBGGR8},   /* 1 00 xxxxxxxx 10 11 */
	{ "rgbz32",     		0x803930, 4, V4L2_PIX_FMT_RGB32},    /* 1 xx 00111001 11 00 */
	{ "bgrz32",     		0x801b30, 4, V4L2_PIX_FMT_BGR32},    /* 1 xx 00011011 11 00 */
	{ "gray8",      		0x800233, 1, V4L2_PIX_FMT_GREY},     /* 1 xx xxxxxx10 11 11 */
	{ "RGB565 (LE)",	    0x803912, 2, V4L2_PIX_FMT_RGB565},   /* 1 xx 00111001 01 10 */
};

struct axi_csi_rx_framesize {
	u16 width;
	u16 height;
};

static const struct axi_csi_rx_framesize axi_csi_rx_framesizes[] = {
	{
		.width		= 176,
		.height		= 144,
	}, {
		.width		= 320,
		.height		= 240,
	}, {
		.width		= 640,
		.height		= 480,
	}, {
		.width		= 1280,
		.height		= 720,
	}, {
		.width		= 1280,
		.height		= 960,
	}, {
		.width		= 1920,
		.height		= 1080,
	}, {
		.width		= 2560,
		.height		= 1600,
	}, {
		.width		= 2592,
		.height		= 1944,
	}
};


struct axi_csi_rx_buffer {
	struct vb2_buffer vb;
	struct list_head head;
    struct axi_csi_rx *csi;

    dma_addr_t addr;
    unsigned int length;
    unsigned int bytesused;	
};


struct axi_csi_rx_stream {
	struct video_device vdev;
	struct vb2_queue q;
	struct v4l2_subdev *subdev;
	struct mutex lock;
	spinlock_t spinlock;
	u32 width, height;
	u32 stride;
	u32 bpp;
	u32 sequence;
	struct axi_csi_rx_fmt_desc * csi_rx_fmt;

	__u32 pixelformat;

	struct dma_chan *chan;
	struct list_head queued_buffers;

};

struct axi_csi_rx {
	struct v4l2_device v4l2_dev;
	struct vb2_alloc_ctx *alloc_ctx;

	struct axi_csi_rx_stream stream;

	void __iomem *base;
	struct gpio_desc *gpio_led;

	struct v4l2_async_notifier notifier;
	struct v4l2_async_subdev asd;
	struct v4l2_async_subdev *asds[1];
};


/* Helpers ---------------------------------------------------------------- */

static void
axi_csi_rx_write(struct axi_csi_rx *csi,
                 unsigned int reg, unsigned int val)
{
	writel(val, csi->base + reg);
}

static u32
axi_csi_rx_read(struct axi_csi_rx *csi,
                unsigned int reg)
{
	return readl(csi->base + reg);
}



/* V4L device ------------------------------------------------------------- */


 /* File Operations */

static const struct v4l2_file_operations axi_csi_rx_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.unlocked_ioctl	= video_ioctl2,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};


 /* IOCTL Operations */

static int
axi_csi_rx_ioctl_querycap(struct file *file, void *priv_fh,
                          struct v4l2_capability *vcap)
{
	strlcpy(vcap->driver, DRIVER_NAME, sizeof(vcap->driver));
	strcpy(vcap->card, "MIPI Camera Serial Interface");
	snprintf(vcap->bus_info, sizeof(vcap->bus_info), "platform:" DRIVER_NAME);
	vcap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vcap->capabilities = vcap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int
axi_csi_rx_ioctl_enum_fmt_vid_cap(struct file *file, void *priv_fh,
                                  struct v4l2_fmtdesc *f)
{

   if (f->index >= ARRAY_SIZE(axi_csi_rx_fmts))
     return -EINVAL;

   f->pixelformat = axi_csi_rx_fmts[f->index].mbus_code;
   strlcpy(f->description, axi_csi_rx_fmts[f->index].desc, sizeof(f->description));
	
	return 0;
}

static int
axi_csi_rx_ioctl_get_fmt_vid_cap(struct file *file, void *priv_fh,
                                 struct v4l2_format *f)
{
	struct axi_csi_rx *csi_rx = video_drvdata(file);
	struct axi_csi_rx_stream *s = &csi_rx->stream;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width			= s->width;
	pix->height			= s->height;
	pix->bytesperline	= s->stride;
	pix->field 		    = V4L2_FIELD_NONE;
	pix->pixelformat    = s->pixelformat;
	pix->sizeimage      = s->stride * s->height;
	pix->colorspace     = V4L2_COLORSPACE_SRGB;

	return 0;
}

static int
private_try_fmt_vid_cap(struct file *file, void *priv_fh,
                        struct v4l2_format *fmt_csi, struct v4l2_subdev_format *fmt_cam,
                        int *fmt_idx)
{
	struct axi_csi_rx *csi_rx = video_drvdata(file);
	struct axi_csi_rx_stream *s = &csi_rx->stream;
	struct v4l2_pix_format *pix = &fmt_csi->fmt.pix;
	int i;

	v4l_bound_align_image(&pix->width, 176, 2592, 3, &pix->height, 144,
		1944, 3, 0);

	for (i = 0; i < ARRAY_SIZE(axi_csi_rx_fmts); i++){
		if(axi_csi_rx_fmts[i].mbus_code == pix->pixelformat){
			break;
		}
	}

	*fmt_idx = i;

	v4l2_fill_mbus_format(&fmt_cam->format, pix, 0);

	fmt_cam->which = V4L2_SUBDEV_FORMAT_TRY;
	fmt_cam->pad = 0;

	return v4l2_subdev_call(s->subdev, pad, set_fmt, NULL, fmt_cam);

}                       

static int
axi_csi_rx_ioctl_try_fmt_vid_cap(struct file *file, void *priv_fh,
                                 struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format fmt_cam;
	int fmt_idx;

	private_try_fmt_vid_cap(file, priv_fh, f, &fmt_cam, &fmt_idx);

	/* no cropping in raw mode */
	if (pix->pixelformat == V4L2_PIX_FMT_SBGGR10P) {
		pix->width = fmt_cam.format.width;
		pix->height = fmt_cam.format.height;
	}	

	pix->bytesperline = pix->width * axi_csi_rx_fmts[fmt_idx].bpp;
	pix->colorspace = V4L2_COLORSPACE_SRGB;;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->field = V4L2_FIELD_NONE;

	return 0;
}

static int
axi_csi_rx_ioctl_set_fmt_vid_cap(struct file *file, void *priv_fh,
                                 struct v4l2_format *f)
{
	struct axi_csi_rx *csi_rx = video_drvdata(file);
	struct axi_csi_rx_stream *s = &csi_rx->stream;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format fmt_cam;
    struct xilinx_vdma_config config;
	int fmt_idx;
	int ret;
	int tl_bias = 2;
	int d;

	if (private_try_fmt_vid_cap(file, priv_fh, f, &fmt_cam, &fmt_idx))
		return -EINVAL;

	/* reset csi */
	axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_CR, (1<<31));

	if (pix->pixelformat == V4L2_PIX_FMT_SBGGR10P) {

		/* no cropping in raw mode */
		axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_VCCS, 0x000);
		axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_VCCE, 0xfff);
		axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_VCLS, 0x000);
		axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_VCLE, 0xfff);

	} else {

		d = (tl_bias + (fmt_cam.format.width - pix->width - tl_bias) / 2) & ~1;
		axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_VCCS, d);
		axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_VCCE, d + pix->width - 1);

		d = (tl_bias + (fmt_cam.format.height - pix->height - tl_bias) / 2) & ~1;
		axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_VCLS, d);
		axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_VCLE, d + pix->height - 1);

	}

	axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_VPCR, axi_csi_rx_fmts[fmt_idx].vpcr);	/* VPCR */
	axi_csi_rx_write(csi_rx, AXI_CSI_RX_REG_CR,	(1 << 16) |	 /* PP active */
												(1 <<  1) |	 /* Bypass LP detect */
												(1 <<  0) ); /* PHY active */	

	s->width  = pix->width;
	s->height = pix->height;
	s->bpp    = axi_csi_rx_fmts[fmt_idx].bpp;
	s->stride = s->width * s->bpp;


	fmt_cam.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt_cam.pad = 0;
	ret = v4l2_subdev_call(s->subdev, pad, set_fmt, NULL, &fmt_cam);
	if (ret)
		return ret;

     /* Configure the DMA engine. */
    memset(&config, 0, sizeof(config));

    config.reset = 1;
	xilinx_vdma_channel_set_config(s->chan, &config);	

    config.reset = 0;
    config.ext_fsync = 1;
    config.park = 1;
    config.gen_lock = 0;
    config.master = 1;
    config.frm_dly = 0;

	xilinx_vdma_channel_set_config(s->chan, &config);	

	return 0;
}

static int 
axi_csi_rx_ioctl_enum_framesizes(struct file *file, void *priv_fh,
					 struct v4l2_frmsizeenum *fsize)
{

	if (fsize->index >= ARRAY_SIZE(axi_csi_rx_framesizes))
	return -EINVAL;

	fsize->discrete.width  = axi_csi_rx_framesizes[fsize->index].width;
	fsize->discrete.height = axi_csi_rx_framesizes[fsize->index].height;
	fsize->type            = V4L2_FRMSIZE_TYPE_DISCRETE;

	return 0;
}

static int
axi_csi_rx_ioctl_log_status(struct file *file, void *priv_fh)
{
	struct axi_csi_rx *csi = video_drvdata(file);

	v4l2_device_call_all(&csi->v4l2_dev, 0, core, log_status);

	return 0;
}


#ifdef CONFIG_VIDEO_ADV_DEBUG
static int
axi_csi_rx_ioctl_get_register(struct file *file, void *priv_fh,
                              struct v4l2_dbg_register *reg)
{
	struct axi_csi_rx *csi = video_drvdata(file);

	reg->val = axi_csi_rx_read(csi, reg->reg);
	reg->size = 4;

	return 0;
}

static int
axi_csi_rx_ioctl_set_register(struct file *file, void *priv_fh,
                              const struct v4l2_dbg_register *reg)
{
	struct axi_csi_rx *csi = video_drvdata(file);

	axi_csi_rx_write(csi, reg->reg, reg->val);

	return 0;
}
#endif

static const struct v4l2_ioctl_ops axi_csi_rx_ioctl_ops = {
	.vidioc_querycap			= axi_csi_rx_ioctl_querycap,
	.vidioc_enum_fmt_vid_cap	= axi_csi_rx_ioctl_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= axi_csi_rx_ioctl_get_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= axi_csi_rx_ioctl_set_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= axi_csi_rx_ioctl_try_fmt_vid_cap,
	.vidioc_enum_framesizes  	= axi_csi_rx_ioctl_enum_framesizes,
	.vidioc_log_status			= axi_csi_rx_ioctl_log_status,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
	.vidioc_reqbufs				= vb2_ioctl_reqbufs,
	.vidioc_querybuf			= vb2_ioctl_querybuf,
	.vidioc_qbuf				= vb2_ioctl_qbuf,
	.vidioc_dqbuf				= vb2_ioctl_dqbuf,
	.vidioc_create_bufs			= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf			= vb2_ioctl_prepare_buf,
	.vidioc_expbuf              = vb2_ioctl_expbuf,
	.vidioc_streamon			= vb2_ioctl_streamon,
	.vidioc_streamoff			= vb2_ioctl_streamoff,


#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register			= axi_csi_rx_ioctl_get_register,
	.vidioc_s_register			= axi_csi_rx_ioctl_set_register,
#endif
};


 /* Queue Operations */

static void axi_csi_rx_dma_done(void *arg)
{
	struct axi_csi_rx_buffer *buf = arg;
	struct vb2_queue *q = buf->vb.vb2_queue;
	struct axi_csi_rx *csi_rx = vb2_get_drv_priv(q);
	struct axi_csi_rx_stream *s = &csi_rx->stream;
	unsigned long flags;
	
	spin_lock_irqsave(&s->spinlock, flags);
	list_del(&buf->head);
	spin_unlock_irqrestore(&s->spinlock, flags);	

	buf->vb.v4l2_buf.sequence = s->sequence++;
	v4l2_get_timestamp(&buf->vb.v4l2_buf.timestamp);
	vb2_set_plane_payload(&buf->vb, 0, buf->length);
	vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);
}

static int
axi_csi_rx_qops_queue_setup(struct vb2_queue *q,
	const struct v4l2_format *fmt, unsigned int *num_buffers,
	unsigned int *num_planes, unsigned int sizes[], void *alloc_ctxs[])
{
	struct axi_csi_rx *csi = vb2_get_drv_priv(q);
	struct axi_csi_rx_stream *s = &csi->stream;

	if (*num_buffers < 1)
		*num_buffers = 1;

	*num_planes = 1;

	if (fmt)
		sizes[0] = fmt->fmt.pix.sizeimage;
	else
		sizes[0] = s->stride * s->height;
	if (sizes[0] == 0)
		return -EINVAL;

	alloc_ctxs[0] = csi->alloc_ctx;	

	return 0;
}

static int 
axi_csi_rx_qops_buf_prepare(struct vb2_buffer *vb)
{
	struct axi_csi_rx *csi = vb2_get_drv_priv(vb->vb2_queue);
	struct axi_csi_rx_buffer *buf = to_axi_csi_rx_buffer(vb);

    buf->csi = csi;
    buf->addr = vb2_dma_contig_plane_dma_addr(vb, 0);
    buf->length = vb2_plane_size(vb, 0);
    buf->bytesused = 0;

	return 0;
}


static void
axi_csi_rx_qops_buf_queue(struct vb2_buffer *vb)
{
	struct axi_csi_rx_buffer *buf = container_of(vb, struct axi_csi_rx_buffer, vb);
	struct axi_csi_rx *csi = vb2_get_drv_priv(vb->vb2_queue);
	struct axi_csi_rx_stream *s = &csi->stream;
	struct dma_async_tx_descriptor *desc;
	struct vb2_queue *q = vb->vb2_queue;
	struct dma_interleaved_template *xt;
	unsigned long flags;
	dma_cookie_t cookie;

	xt = kzalloc(sizeof(struct dma_async_tx_descriptor) +
				sizeof(struct data_chunk), GFP_KERNEL);
	if (!xt) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	xt->dst_start = buf->addr;
	xt->src_sgl = false;
	xt->dst_sgl = true;
	xt->frame_size = 1;
	xt->numf = s->height;
	xt->sgl[0].size = s->width * s->bpp;
	xt->sgl[1].icg = s->stride - (s->width * s->bpp);
	xt->dir = DMA_DEV_TO_MEM;

	desc = dmaengine_prep_interleaved_dma(s->chan, xt, DMA_PREP_INTERRUPT);
	kfree(xt);
	if (!desc) {
		v4l2_err(&csi->v4l2_dev, "Failed to prepare DMA transfer\n");
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}
	desc->callback = axi_csi_rx_dma_done;
	desc->callback_param = buf;

	cookie = dmaengine_submit(desc);
	if (cookie < 0) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	spin_lock_irqsave(&s->spinlock, flags);
	list_add_tail(&buf->head, &s->queued_buffers);
	spin_unlock_irqrestore(&s->spinlock, flags);

	if (vb2_is_streaming(q))
		dma_async_issue_pending(s->chan);
}

static int
axi_csi_rx_qops_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct axi_csi_rx *csi = vb2_get_drv_priv(q);
	struct axi_csi_rx_stream *s = &csi->stream;
	int ret = 0;

	s->sequence = 0;

	dma_async_issue_pending(s->chan);

	ret = v4l2_subdev_call(s->subdev, video, s_stream, 1);

	gpiod_set_value_cansleep(csi->gpio_led, 1);

	return ret;
}

static int
axi_csi_rx_qops_stop_streaming(struct vb2_queue *q)
{
	struct axi_csi_rx *csi = vb2_get_drv_priv(q);
	struct axi_csi_rx_stream *s = &csi->stream;
	struct axi_csi_rx_buffer *buf;
	struct xilinx_vdma_config config;
	unsigned long flags;

	/* Stop and reset the DMA engine. */
	dmaengine_device_control(s->chan, DMA_TERMINATE_ALL, 0);	

	config.reset = 1;

	xilinx_vdma_channel_set_config(s->chan, &config);

	spin_lock_irqsave(&s->spinlock, flags);

	list_for_each_entry(buf, &s->queued_buffers, head)
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);

	INIT_LIST_HEAD(&s->queued_buffers);

	spin_unlock_irqrestore(&s->spinlock, flags);

	vb2_wait_for_all_buffers(q);

	/* call the camera to stop streaming */
	v4l2_subdev_call(s->subdev, video, s_stream, 0);

	gpiod_set_value_cansleep(csi->gpio_led, 0);

	return 0;
}

static const struct vb2_ops axi_csi_rx_qops = {
	.queue_setup		= axi_csi_rx_qops_queue_setup,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_prepare		= axi_csi_rx_qops_buf_prepare,
	.buf_queue		    = axi_csi_rx_qops_buf_queue,
	.start_streaming	= axi_csi_rx_qops_start_streaming,
	.stop_streaming		= axi_csi_rx_qops_stop_streaming,
};


 /* */

static int
axi_csi_rx_nodes_register(struct axi_csi_rx *csi)
{
	struct axi_csi_rx_stream *s = &csi->stream;
	struct video_device *vdev = &s->vdev;
	struct vb2_queue *q = &s->q;
	int rv;

	mutex_init(&s->lock);

	vdev->v4l2_dev = &csi->v4l2_dev;
	vdev->fops     = &axi_csi_rx_fops;
	vdev->release  = video_device_release_empty;
	vdev->ctrl_handler = s->subdev->ctrl_handler;
	vdev->lock     = &s->lock;
	vdev->queue    = q;

	q->lock = &s->lock;
	INIT_LIST_HEAD(&s->queued_buffers);
	spin_lock_init(&s->spinlock);	

	/* default format */
	s->width = 640;
	s->height = 480;
	s->pixelformat = V4L2_PIX_FMT_RGB565;
	s->bpp = 2;
	s->stride = s->width * s->bpp;

	vdev->ioctl_ops = &axi_csi_rx_ioctl_ops;

	q->type            = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes        = VB2_MMAP | VB2_USERPTR | VB2_READ;
	q->drv_priv        = csi;
	q->buf_struct_size = sizeof(struct axi_csi_rx_buffer);
	q->ops             = &axi_csi_rx_qops;
	q->mem_ops         = &vb2_dma_contig_memops;
	q->timestamp_type  = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	rv = vb2_queue_init(q);
	if (rv)
		return rv;

	return video_register_device(vdev, VFL_TYPE_GRABBER, -1);
}

 /* Async subdev probing */
static struct axi_csi_rx *
notifier_to_axi_csi_rx(struct v4l2_async_notifier *n)
{
	return container_of(n, struct axi_csi_rx, notifier);
}

static int
axi_csi_rx_async_bound(struct v4l2_async_notifier *notifier,
                       struct v4l2_subdev *subdev,
                       struct v4l2_async_subdev *asd)
{
	struct axi_csi_rx *csi = notifier_to_axi_csi_rx(notifier);

	csi->stream.subdev = subdev;

	return 0;
}

static int
axi_csi_rx_async_complete(struct v4l2_async_notifier *notifier)
{
	struct axi_csi_rx *csi = notifier_to_axi_csi_rx(notifier);
	int rv;

	rv = v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
	if (rv < 0)
		return rv;

	return axi_csi_rx_nodes_register(csi);
}




/* Platform Driver  ------------------------------------------------------- */

static int
axi_csi_rx_probe(struct platform_device *pdev)
{
	struct axi_csi_rx *csi;
	struct resource *regs;
	struct device_node *ep_node;
	int irq, rv;

	dev_info(&pdev->dev, "probe\n");

	/* Alloc our private struct */
	csi = devm_kzalloc(&pdev->dev, sizeof(struct axi_csi_rx), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	/* Get info & resources from the DT */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "Invalid (or missing) base address\n");
		return -ENXIO;
	}
	csi->base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(csi->base))
		return PTR_ERR(csi->base);

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "Invalid (or missing) IRQ\n");
		return -ENXIO;
	}

	csi->gpio_led = devm_gpiod_get(&pdev->dev, "led");
	if (!IS_ERR(csi->gpio_led)) {
		gpiod_direction_output(csi->gpio_led, 0);
	} else {
		csi->gpio_led = NULL;
	}

	csi->stream.chan = dma_request_slave_channel(&pdev->dev, "video");
	if (!csi->stream.chan)
		return -EPROBE_DEFER;


	/* V4L2 setup */
	csi->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(csi->alloc_ctx)) {
		rv = PTR_ERR(csi->alloc_ctx);
		dev_err(&pdev->dev, "Failed to init dma ctx: %d\n", rv);
		goto err_dma_release_channel;
	}

	video_set_drvdata(&csi->stream.vdev, csi);
	platform_set_drvdata(pdev, csi);

	rv = v4l2_device_register(&pdev->dev, &csi->v4l2_dev);
	if (rv) {
		dev_err(&pdev->dev, "Failed to register V4L device: %d\n", rv);
		goto err_dma_cleanup_ctx;
	}

	ep_node = v4l2_of_get_next_endpoint(pdev->dev.of_node, NULL);
	if (!ep_node) {
		rv = -EINVAL;
		goto err_device_unregister;
	}

	csi->asd.match_type = V4L2_ASYNC_MATCH_OF;
	csi->asd.match.of.node = v4l2_of_get_remote_port_parent(ep_node);

	csi->asds[0] = &csi->asd;
	csi->notifier.subdevs     = csi->asds;
	csi->notifier.num_subdevs = ARRAY_SIZE(csi->asds);
	csi->notifier.bound       = axi_csi_rx_async_bound;
	csi->notifier.complete    = axi_csi_rx_async_complete;

	rv = v4l2_async_notifier_register(&csi->v4l2_dev, &csi->notifier);
	if (rv) {
		dev_err(&pdev->dev, "Error %d registering device nodes\n", rv);
		goto err_device_unregister;
	}

	/* Init result */
	dev_info(&pdev->dev, "Regs     : 0x%08x -> 0x%08x [%s]\n",
		regs->start, regs->end, regs->name);
	dev_info(&pdev->dev, "IRQ      : %d\n", irq);
	dev_info(&pdev->dev, "GPIO LED : %d\n",
		csi->gpio_led ? desc_to_gpio(csi->gpio_led) : -1);

	return 0;

err_device_unregister:
	v4l2_device_unregister(&csi->v4l2_dev);
err_dma_cleanup_ctx:
	vb2_dma_contig_cleanup_ctx(csi->alloc_ctx);
err_dma_release_channel:
	dma_release_channel(csi->stream.chan);

	return rv;
}

static int
axi_csi_rx_remove(struct platform_device *pdev)
{
	struct axi_csi_rx *csi = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "remove\n");


	if (csi->stream.chan)
		dma_release_channel(csi->stream.chan);

	if (!IS_ERR_OR_NULL(csi->alloc_ctx))
		vb2_dma_contig_cleanup_ctx(csi->alloc_ctx);

	if (&csi->notifier)
		v4l2_async_notifier_unregister(&csi->notifier);
	
	v4l2_device_unregister(&csi->v4l2_dev);

	if (video_is_registered(&csi->stream.vdev))
		video_unregister_device(&csi->stream.vdev);

	return 0;
}


static struct of_device_id axi_csi_rx_of_match[] = {
	{ .compatible = "s47,axi-csi-rx-1.00.a", },
	{ }
};
MODULE_DEVICE_TABLE(of, axi_csi_rx_of_match);

static struct platform_driver axi_csi_rx_driver = {
	.probe  = axi_csi_rx_probe,
	.remove = axi_csi_rx_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
		.of_match_table = axi_csi_rx_of_match,
	},
};
module_platform_driver(axi_csi_rx_driver);

MODULE_AUTHOR("Sylvain Munaut <tnt@246tNt.com>");
MODULE_DESCRIPTION("MIPI CSI RX interface on AXI bus");
MODULE_LICENSE("GPL");
