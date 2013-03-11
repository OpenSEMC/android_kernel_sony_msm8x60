/* Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"

extern int mdp_irq_mask;
extern int mdp_irq_enabled;

static int mddi_state;

#define TOUT_PERIOD	HZ	/* 1 second */
#define MS_100		(HZ/10)	/* 100 ms */

static int vsync_start_y_adjust = 4;

#define MAX_CONTROLLER	1
#define VSYNC_EXPIRE_TICK 10

static struct vsycn_ctrl {
	struct device *dev;
	int inited;
	int update_ndx;
	int expire_tick;
	int blt_wait;
	u32 ov_koff;
	u32 ov_done;
	u32 dmap_koff;
	u32 dmap_done;
	uint32 rdptr_intr_tot;
	uint32 rdptr_sirq_tot;
	atomic_t suspend;
	atomic_t vsync_resume;
	int wait_vsync_cnt;
	int blt_change;
	int blt_free;
	int blt_end;
	int sysfs_created;
	int uevent;
	struct mutex update_lock;
	struct completion ov_comp;
	struct completion dmap_comp;
	struct completion vsync_comp;
	spinlock_t spin_lock;
	struct msm_fb_data_type *mfd;
	struct mdp4_overlay_pipe *base_pipe;
	struct vsync_update vlist[2];
	int vsync_enabled;
	int clk_enabled;
	int clk_control;
	int new_update;
	ktime_t vsync_time;
	struct delayed_work clk_work;
} vsync_ctrl_db[MAX_CONTROLLER];

static void vsync_irq_enable(int intr, int term)
{
	unsigned long flag;

void mdp_dmap_vsync_set(int enable)
{
	dmap_vsync_enable = enable;
}

int mdp_dmap_vsync_get(void)
{
	return dmap_vsync_enable;
}

void mdp4_mddi_vsync_enable(struct msm_fb_data_type *mfd,
		struct mdp4_overlay_pipe *pipe, int which)
{
	uint32 start_y, data, tear_en;

	tear_en = (1 << which);

	if ((mfd->use_mdp_vsync) && (mfd->ibuf.vsync_enable) &&
		(mfd->panel_info.lcd.vsync_enable)) {

		if (mdp_hw_revision < MDP4_REVISION_V2_1) {
			/* need dmas dmap switch */
			if (which == 0 && dmap_vsync_enable == 0 &&
				mfd->panel_info.lcd.rev < 2) /* dma_p */
				return;
		}

		if (vsync_start_y_adjust <= pipe->dst_y)
			start_y = pipe->dst_y - vsync_start_y_adjust;
		else
			start_y = (mfd->total_lcd_lines - 1) -
				(vsync_start_y_adjust - pipe->dst_y);
		if (which == 0)
			MDP_OUTP(MDP_BASE + 0x210, start_y);	/* primary */
		else
			MDP_OUTP(MDP_BASE + 0x214, start_y);	/* secondary */

		data = inpdw(MDP_BASE + 0x20c);
		data |= tear_en;
		MDP_OUTP(MDP_BASE + 0x20c, data);
	} else {
		data = inpdw(MDP_BASE + 0x20c);
		data &= ~tear_en;
		MDP_OUTP(MDP_BASE + 0x20c, data);
	}
}

#define WHOLESCREEN

void mdp4_overlay_update_lcd(struct msm_fb_data_type *mfd)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	uint8 *src;
	int ptype;
	uint32 mddi_ld_param;
	uint16 mddi_vdo_packet_reg;
	struct mdp4_overlay_pipe *pipe;
	int ret;

	if (mfd->key != MFD_KEY)
		return;

	mddi_mfd = mfd;		/* keep it */

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	if (mddi_pipe == NULL) {
		ptype = mdp4_overlay_format2type(mfd->fb_imgType);
		if (ptype < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);
		pipe = mdp4_overlay_pipe_alloc(ptype, MDP4_MIXER0);
		if (pipe == NULL)
			printk(KERN_INFO "%s: pipe_alloc failed\n", __func__);
		pipe->pipe_used++;
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = mfd->fb_imgType;
		mdp4_overlay_panel_mode(pipe->mixer_num, MDP4_PANEL_MDDI);
		ret = mdp4_overlay_format2pipe(pipe);
		if (ret < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);

		mddi_pipe = pipe; /* keep it */
		mddi_ld_param = 0;
		mddi_vdo_packet_reg = mfd->panel_info.mddi.vdopkt;

		if (mdp_hw_revision == MDP4_REVISION_V2_1) {
			uint32	data;

			data = inpdw(MDP_BASE + 0x0028);
			data &= ~0x0300;	/* bit 8, 9, MASTER4 */
			if (mfd->fbi->var.xres == 540) /* qHD, 540x960 */
				data |= 0x0200;
			else
				data |= 0x0100;

			MDP_OUTP(MDP_BASE + 0x00028, data);
		}

		if (mfd->panel_info.type == MDDI_PANEL) {
			if (mfd->panel_info.pdest == DISPLAY_1)
				mddi_ld_param = 0;
			else
				mddi_ld_param = 1;
		} else {
			mddi_ld_param = 2;
		}

		MDP_OUTP(MDP_BASE + 0x00090, mddi_ld_param);

		if (mfd->panel_info.bpp == 24)
			MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC_24 << 16) | mddi_vdo_packet_reg);
		else if (mfd->panel_info.bpp == 16)
			MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC_16 << 16) | mddi_vdo_packet_reg);
		else
			MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC << 16) | mddi_vdo_packet_reg);

		MDP_OUTP(MDP_BASE + 0x00098, 0x01);
		mdp4_init_writeback_buf(mfd, MDP4_MIXER0);
		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr = 0;
	} else {
		pipe = mddi_pipe;
	}

	/* 0 for dma_p, client_id = 0 */
	MDP_OUTP(MDP_BASE + 0x00090, 0);


	src = (uint8 *) iBuf->buf;

#ifdef WHOLESCREEN

	{
		struct fb_info *fbi;

		fbi = mfd->fbi;
		pipe->src_height = fbi->var.yres;
		pipe->src_width = fbi->var.xres;
		pipe->src_h = fbi->var.yres;
		pipe->src_w = fbi->var.xres;
		pipe->src_y = 0;
		pipe->src_x = 0;
		pipe->dst_h = fbi->var.yres;
		pipe->dst_w = fbi->var.xres;
		pipe->dst_y = 0;
		pipe->dst_x = 0;
		pipe->srcp0_addr = (uint32)src;
		pipe->srcp0_ystride = fbi->fix.line_length;
	}

#else
	if (mdp4_overlay_active(MDP4_MIXER0)) {
		struct fb_info *fbi;

		fbi = mfd->fbi;
		pipe->src_height = fbi->var.yres;
		pipe->src_width = fbi->var.xres;
		pipe->src_h = fbi->var.yres;
		pipe->src_w = fbi->var.xres;
		pipe->src_y = 0;
		pipe->src_x = 0;
		pipe->dst_h = fbi->var.yres;
		pipe->dst_w = fbi->var.xres;
		pipe->dst_y = 0;
		pipe->dst_x = 0;
		pipe->srcp0_addr = (uint32) src;
		pipe->srcp0_ystride = fbi->fix.line_length;
	} else {
		/* starting input address */
		src += (iBuf->dma_x + iBuf->dma_y * iBuf->ibuf_width)
					* iBuf->bpp;

		pipe->src_height = iBuf->dma_h;
		pipe->src_width = iBuf->dma_w;
		pipe->src_h = iBuf->dma_h;
		pipe->src_w = iBuf->dma_w;
		pipe->src_y = 0;
		pipe->src_x = 0;
		pipe->dst_h = iBuf->dma_h;
		pipe->dst_w = iBuf->dma_w;
		pipe->dst_y = iBuf->dma_y;
		pipe->dst_x = iBuf->dma_x;
		pipe->srcp0_addr = (uint32) src;
		pipe->srcp0_ystride = iBuf->ibuf_width * iBuf->bpp;
	}
#endif

	pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;

	mdp4_overlay_rgb_setup(pipe);

	mdp4_mixer_stage_up(pipe, 1);

	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_dmap_xy(pipe);

	mdp4_overlay_dmap_cfg(mfd, 0);
	mdp4_mixer_stage_commit(pipe->mixer_num);
	mdp4_mddi_vsync_enable(mfd, pipe, 0);

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
}

static void mdp4_overlay_update_mddi(struct msm_fb_data_type *mfd);

void mdp4_mddi_vsync_ctrl(struct fb_info *info, int enable)
{
	struct vsycn_ctrl *vctrl;
	unsigned long flags;
	int clk_set_on = 0;
	int cndx = 0;

	vctrl = &vsync_ctrl_db[cndx];

	pr_debug("%s: clk_enabled=%d vsycn_enabeld=%d req=%d\n", __func__,
		vctrl->clk_enabled, vctrl->vsync_enabled, enable);

	mutex_lock(&vctrl->update_lock);

	if (vctrl->vsync_enabled == enable) {
		mutex_unlock(&vctrl->update_lock);
		return;
	}

	if (mfd->ov0_wb_buf->write_addr == 0) {
		pr_info("%s: no blt_base assigned\n", __func__);
		return -EBUSY;
	}

	if (mddi_pipe->ov_blt_addr == 0) {
		mdp4_mddi_dma_busy_wait(mfd);
		spin_lock_irqsave(&mdp_spin_lock, flag);
		mddi_pipe->blt_end = 0;
		mddi_pipe->blt_cnt = 0;
		mddi_pipe->ov_cnt = 0;
		mddi_pipe->dmap_cnt = 0;
		mddi_pipe->ov_blt_addr = mfd->ov0_wb_buf->write_addr;
		mddi_pipe->dma_blt_addr = mfd->ov0_wb_buf->write_addr;
		mdp4_stat.blt_mddi++;
		spin_unlock_irqrestore(&mdp_spin_lock, flag);
	return 0;
}

	if (enable) {
		if (vctrl->clk_enabled == 0) {
			pr_debug("%s: SET_CLK_ON\n", __func__);
			mdp_clk_ctrl(1);
			vctrl->clk_enabled = 1;
			clk_set_on = 1;
		}
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->clk_control = 0;
		vctrl->expire_tick = 0;
		vctrl->new_update = 1;
		if (clk_set_on) {
			vsync_irq_enable(INTR_PRIMARY_RDPTR,
						MDP_PRIM_RDPTR_TERM);
		}
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	} else {
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->clk_control = 1;
		if (vctrl->clk_enabled)
			vctrl->expire_tick = VSYNC_EXPIRE_TICK;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	}
	mutex_unlock(&vctrl->update_lock);

	if (vctrl->vsync_enabled &&  atomic_read(&vctrl->suspend) == 0)
		atomic_set(&vctrl->vsync_resume, 1);
}

int mdp4_mddi_overlay_blt_stop(struct msm_fb_data_type *mfd)
{
	unsigned long flag;

	pr_debug("%s: blt_end=%d blt_addr=%x\n",
		 __func__, mddi_pipe->blt_end, (int)mddi_pipe->ov_blt_addr);

	if ((mddi_pipe->blt_end == 0) && mddi_pipe->ov_blt_addr) {
		spin_lock_irqsave(&mdp_spin_lock, flag);
		mddi_pipe->blt_end = 1;	/* mark as end */
		spin_unlock_irqrestore(&mdp_spin_lock, flag);
		return 0;
	}

	return -EBUSY;
}

int mdp4_mddi_overlay_blt_offset(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	req->offset = 0;
	req->width = mddi_pipe->src_width;
	req->height = mddi_pipe->src_height;
	req->bpp = mddi_pipe->bpp;

	return sizeof(*req);
}

void mdp4_mddi_overlay_blt(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	if (req->enable)
		mdp4_mddi_overlay_blt_start(mfd);
	else if (req->enable == 0)
		mdp4_mddi_overlay_blt_stop(mfd);

	if (wait_for_completion_timeout(&vctrl->dmap_comp, HZ/10) <= 0){
		pr_err("DMAP UPDATE FAILED!!! \n");
	}
}

void mdp4_blt_xy_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr, addr2;
	int bpp;
	char *overlay_base;

	if (pipe->ov_blt_addr == 0)
		return;


#ifdef BLT_RGB565
	bpp = 2; /* overlay ouput is RGB565 */
#else
	bpp = 3; /* overlay ouput is RGB888 */
#endif
	off = 0;
	if (pipe->dmap_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;

	addr = pipe->ov_blt_addr + off;

	if (wait_for_completion_timeout(&vctrl->ov_comp, (HZ/10)) <= 0) {
		pr_err("OV UPDATE FAILED!!! \n");
	}
}

void mdp4_primary_rdptr(void)
{
	struct vsycn_ctrl *vctrl;

	vctrl = &vsync_ctrl_db[cndx];
	pr_debug("%s: ISR, cpu=%d\n", __func__, smp_processor_id());
	vctrl->rdptr_intr_tot++;

	spin_lock(&vctrl->spin_lock);
	vctrl->vsync_time = ktime_get();
	complete_all(&vctrl->vsync_comp);
	vctrl->wait_vsync_cnt = 0;

	if (vctrl->expire_tick) {
		vctrl->expire_tick--;
		if (vctrl->expire_tick == 0)
			schedule_delayed_work(&vctrl->clk_work, 0);
	}
	spin_unlock(&vctrl->spin_lock);
}

/*
 * mdp4_dmap_done_mddi: called from isr
 */
void mdp4_dma_p_done_mddi(struct mdp_dma_data *dma)
{
	int diff;

	mddi_pipe->dmap_cnt++;
	diff = mddi_pipe->ov_cnt - mddi_pipe->dmap_cnt;
	pr_debug("%s: ov_cnt=%d dmap_cnt=%d\n",
			__func__, mddi_pipe->ov_cnt, mddi_pipe->dmap_cnt);

	if (mdp_rev <= MDP_REV_41)
		mdp4_mixer_blend_cfg(MDP4_MIXER0);

	if (diff <= 0) {
		spin_lock(&mdp_spin_lock);
		dma->dmap_busy = FALSE;
		complete(&dma->dmap_comp);
		spin_unlock(&mdp_spin_lock);

		if (mddi_pipe->blt_end) {
			mddi_pipe->blt_end = 0;
			mddi_pipe->ov_blt_addr = 0;
			mddi_pipe->dma_blt_addr = 0;
			pr_debug("%s: END, ov_cnt=%d dmap_cnt=%d\n", __func__,
				mddi_pipe->ov_cnt, mddi_pipe->dmap_cnt);
			mdp_intr_mask &= ~INTR_DMA_P_DONE;
			outp32(MDP_INTR_ENABLE, mdp_intr_mask);
		}

		mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_OFF, TRUE);
		mdp_disable_irq_nosync(MDP_DMA2_TERM);  /* disable intr */
		return;
	}

	spin_lock(&mdp_spin_lock);
	dma->busy = FALSE;
	spin_unlock(&mdp_spin_lock);
	complete(&dma->comp);
	if (busy_wait_cnt)
		busy_wait_cnt--;

	pr_debug("%s: kickoff dmap\n", __func__);

	mdp4_blt_xy_update(mddi_pipe);
	/* kick off dmap */
	outpdw(MDP_BASE + 0x000c, 0x0);
	mdp4_stat.kickoff_dmap++;
	mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_OFF, TRUE);
}

/*
 * mdp4_overlay0_done_mddi: called from isr
 */
void mdp4_overlay0_done_mddi(struct mdp_dma_data *dma)
{
	int diff;

	if (mddi_pipe->ov_blt_addr == 0) {
		mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_OFF, TRUE);
		spin_lock(&mdp_spin_lock);
		dma->busy = FALSE;
		spin_unlock(&mdp_spin_lock);
		complete(&dma->comp);

		if (busy_wait_cnt)
			busy_wait_cnt--;
		mdp_disable_irq_nosync(MDP_OVERLAY0_TERM);

		return;
	}

	/* blt enabled */
	if (mddi_pipe->blt_end == 0)
		mddi_pipe->ov_cnt++;

	pr_debug("%s: ov_cnt=%d dmap_cnt=%d\n",
			__func__, mddi_pipe->ov_cnt, mddi_pipe->dmap_cnt);

	if (mddi_pipe->blt_cnt == 0) {
		/* first kickoff since blt enabled */
		mdp_intr_mask |= INTR_DMA_P_DONE;
		outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	}

static void clk_ctrl_work(struct work_struct *work)
{
	struct vsycn_ctrl *vctrl =
		container_of(to_delayed_work(work), typeof(*vctrl), clk_work);
	unsigned long flags;

	mutex_lock(&vctrl->update_lock);
	if (vctrl->clk_control && vctrl->clk_enabled) {

		spin_lock_irqsave(&mdp_spin_lock, flags);
		if (mdp_irq_enabled && (mdp_irq_mask & MDP_DMAP_TERM)) {
			pr_err("AVOID CLOCK DISABLE DURING DMA TRANSFER!!!\n");
			spin_unlock_irqrestore(&mdp_spin_lock, flags);
			mutex_unlock(&vctrl->update_lock);
			schedule_delayed_work(&vctrl->clk_work, (HZ/100));
			return;
		}

		spin_unlock_irqrestore(&mdp_spin_lock, flags);

		spin_lock_irqsave(&vctrl->spin_lock, flags);
		if (vctrl->expire_tick || vctrl->vsync_enabled)
			return;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);

		pr_debug("%s: SET_CLK_OFF\n", __func__);
		mdp_clk_ctrl(0);

		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vsync_irq_disable(INTR_PRIMARY_RDPTR, MDP_PRIM_RDPTR_TERM);
			vctrl->clk_enabled = 0;
		vctrl->clk_control = 0;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		}
	mutex_unlock(&vctrl->update_lock);
}

static ssize_t vsync_show_event(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cndx;
	struct vsycn_ctrl *vctrl;
	ssize_t ret = 0;
	unsigned long flags;
	u64 vsync_tick;

	cndx = 0;
	vctrl = &vsync_ctrl_db[0];

	if (atomic_read(&vctrl->suspend) > 0 ||
		atomic_read(&vctrl->vsync_resume) == 0)
		return 0;

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (vctrl->wait_vsync_cnt == 0)
		INIT_COMPLETION(vctrl->vsync_comp);
	vctrl->wait_vsync_cnt++;
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	ret = wait_for_completion_interruptible_timeout(&vctrl->vsync_comp,
		msecs_to_jiffies(VSYNC_PERIOD * 4));
	if (ret <= 0) {
		vctrl->wait_vsync_cnt = 0;
		return -EBUSY;
	}

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	vsync_tick = ktime_to_ns(vctrl->vsync_time);
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	ret = snprintf(buf, PAGE_SIZE, "VSYNC=%llu", vsync_tick);
	buf[strlen(buf) + 1] = '\0';
	return ret;
}

	mdp4_blt_xy_update(mddi_pipe);
	mdp_enable_irq(MDP_DMA2_TERM);	/* enable intr */
	/* kick off dmap */
	outpdw(MDP_BASE + 0x000c, 0x0);
	mdp4_stat.kickoff_dmap++;
	mdp_disable_irq_nosync(MDP_OVERLAY0_TERM);
}

void mdp4_mddi_overlay_restore(void)
{
	if (mddi_mfd == NULL)
		return;

	pr_debug("%s: resotre, pid=%d\n", __func__, current->pid);

	if (mddi_mfd->panel_power_on == 0)
		return;
	if (mddi_mfd && mddi_pipe) {
		mdp4_mddi_dma_busy_wait(mddi_mfd);
		mdp4_overlay_update_lcd(mddi_mfd);

	vctrl->inited = 1;
	vctrl->update_ndx = 0;
	mutex_init(&vctrl->update_lock);
	init_completion(&vctrl->ov_comp);
	init_completion(&vctrl->dmap_comp);
	init_completion(&vctrl->vsync_comp);
	spin_lock_init(&vctrl->spin_lock);
	atomic_set(&vctrl->vsync_resume, 1);
	INIT_DELAYED_WORK(&vctrl->clk_work, clk_ctrl_work);
}

void mdp4_mddi_blt_dmap_busy_wait(struct msm_fb_data_type *mfd)
{
	unsigned long flag;
	int need_wait = 0;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (mfd->dma->dmap_busy == TRUE) {
		INIT_COMPLETION(mfd->dma->dmap_comp);
		need_wait++;
	}
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	if (need_wait) {
		/* wait until DMA finishes the current job */
		wait_for_completion(&mfd->dma->dmap_comp);
	}
}

/*
 * mdp4_mddi_cmd_dma_busy_wait: check mddi link activity
 * mddi link is a shared resource and it can only be used
 * while it is in idle state.
 * ov_mutex need to be acquired before call this function.
 */
void mdp4_mddi_dma_busy_wait(struct msm_fb_data_type *mfd)
{
	unsigned long flag;
	int need_wait = 0;

	pr_debug("%s: START, pid=%d\n", __func__, current->pid);
	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (mfd->dma->busy == TRUE) {
		if (busy_wait_cnt == 0)
			INIT_COMPLETION(mfd->dma->comp);
		busy_wait_cnt++;
		need_wait++;
	}
	spin_unlock_irqrestore(&mdp_spin_lock, flag);


	if (need_wait) {
		/* wait until DMA finishes the current job */
		pr_debug("%s: PENDING, pid=%d\n", __func__, current->pid);
		wait_for_completion(&mfd->dma->comp);
	}
	pr_debug("%s: DONE, pid=%d\n", __func__, current->pid);
}

void mdp4_mddi_kickoff_video(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
	/*
	 * a video kickoff may happen before UI kickoff after
	 * blt enabled. mdp4_overlay_update_lcd() need
	 * to be called before kickoff.
	 * vice versa for blt disabled.
	 */
	if (mddi_pipe->ov_blt_addr && mddi_pipe->blt_cnt == 0)
		mdp4_overlay_update_lcd(mfd); /* first time */
	else if (mddi_pipe->ov_blt_addr == 0  && mddi_pipe->blt_cnt) {
		mdp4_overlay_update_lcd(mfd); /* last time */
		mddi_pipe->blt_cnt = 0;
	}

	pr_debug("%s: blt_addr=%d blt_cnt=%d\n",
		__func__, (int)mddi_pipe->ov_blt_addr, mddi_pipe->blt_cnt);

	if (mddi_pipe->ov_blt_addr)
		mdp4_mddi_blt_dmap_busy_wait(mddi_mfd);
	mdp4_mddi_overlay_kickoff(mfd, pipe);
}

void mdp4_mddi_kickoff_ui(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
	pr_debug("%s: pid=%d\n", __func__, current->pid);
	mdp4_mddi_overlay_kickoff(mfd, pipe);
}


void mdp4_mddi_overlay_kickoff(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
	unsigned long flag;

	mdp_enable_irq(MDP_OVERLAY0_TERM);
	spin_lock_irqsave(&mdp_spin_lock, flag);
	mfd->dma->busy = TRUE;
	if (mddi_pipe->ov_blt_addr)
		mfd->dma->dmap_busy = TRUE;
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
	/* start OVERLAY pipe */
	mdp_pipe_kickoff(MDP_OVERLAY0_TERM, mfd);
	mdp4_stat.kickoff_ov0++;
}

static void mdp4_overlay_update_mddi(struct msm_fb_data_type *mfd)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	uint32 outBpp = iBuf->bpp;
	uint16 mddi_vdo_packet_reg;
	uint32 dma_s_cfg_reg;

	dma_s_cfg_reg = 0;

	if (vctrl->base_pipe == NULL) {
		ptype = mdp4_overlay_format2type(mfd->fb_imgType);
		if (ptype < 0)
			pr_info("%s: format2type failed\n", __func__);

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	/* PIXELSIZE */
	MDP_OUTP(MDP_BASE + 0xa0004, (pipe->dst_h << 16 | pipe->dst_w));
	MDP_OUTP(MDP_BASE + 0xa0008, pipe->srcp0_addr);	/* ibuf address */
	MDP_OUTP(MDP_BASE + 0xa000c, pipe->srcp0_ystride);/* ystride */

	if (mfd->panel_info.bpp == 24) {
		dma_s_cfg_reg |= DMA_DSTC0G_8BITS |	/* 666 18BPP */
		    DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS;
	} else if (mfd->panel_info.bpp == 18) {
		dma_s_cfg_reg |= DMA_DSTC0G_6BITS |	/* 666 18BPP */
		    DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
	} else {
		dma_s_cfg_reg |= DMA_DSTC0G_6BITS |	/* 565 16BPP */
		    DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
	}

		vctrl->base_pipe = pipe; /* keep it */
		mdp4_init_writeback_buf(mfd, MDP4_MIXER0);
		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr = 0;
	} else {
		pipe = vctrl->base_pipe;
	}

	/* 1 for dma_s, client_id = 0 */
	MDP_OUTP(MDP_BASE + 0x00090, 1);

	mddi_vdo_packet_reg = mfd->panel_info.mddi.vdopkt;

	if (mfd->panel_info.bpp == 24)
		MDP_OUTP(MDP_BASE + 0x00094,
			(MDDI_VDO_PACKET_DESC_24 << 16) | mddi_vdo_packet_reg);
	else if (mfd->panel_info.bpp == 16)
		MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC_16 << 16) | mddi_vdo_packet_reg);
	else
		MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC << 16) | mddi_vdo_packet_reg);

	MDP_OUTP(MDP_BASE + 0x00098, 0x01);

	MDP_OUTP(MDP_BASE + 0xa0000, dma_s_cfg_reg);

	mdp4_mddi_vsync_enable(mfd, pipe, 1);

	mdp4_mixer_stage_up(pipe, 0);

	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_dmap_xy(pipe);

	mdp4_overlay_dmap_cfg(mfd, 0);

	wmb();
}

void mdp4_mddi_blt_start(struct msm_fb_data_type *mfd)
{
	mdp4_mddi_do_blt(mfd, 1);
}

void mdp4_mddi_blt_stop(struct msm_fb_data_type *mfd)
{
	mdp4_mddi_do_blt(mfd, 0);
}

void mdp4_mddi_overlay_blt(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	mdp4_mddi_do_blt(mfd, req->enable);
}

static DEVICE_ATTR(vsync_event, S_IRUGO, vsync_show_event, NULL);
static struct attribute *vsync_fs_attrs[] = {
	&dev_attr_vsync_event.attr,
	NULL,
};
static struct attribute_group vsync_fs_attr_group = {
	.attrs = vsync_fs_attrs,
};
int mdp4_mddi_on(struct platform_device *pdev)
{
	int ret = 0;
	int cndx = 0;
	struct msm_fb_data_type *mfd;
	struct vsycn_ctrl *vctrl;

	pr_debug("%s+:\n", __func__);

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	vctrl = &vsync_ctrl_db[cndx];
	vctrl->mfd = mfd;
	vctrl->dev = mfd->fbi->dev;

	mdp_clk_ctrl(1);
	mdp4_overlay_update_mddi(mfd);
	mdp_clk_ctrl(0);

	mdp4_iommu_attach();

	atomic_set(&vctrl->suspend, 0);
	pr_info("%s-:\n", __func__);

	if (!vctrl->sysfs_created) {
		ret = sysfs_create_group(&vctrl->dev->kobj,
			&vsync_fs_attr_group);
		if (ret) {
			pr_err("%s: sysfs group creation failed, ret=%d\n",
				__func__, ret);
			return ret;
		}

		kobject_uevent(&vctrl->dev->kobj, KOBJ_ADD);
		pr_debug("%s: kobject_uevent(KOBJ_ADD)\n", __func__);
		vctrl->sysfs_created = 1;
	}

	return ret;
}

void mdp4_mddi_overlay_dmas_restore(void)
{
	int ret = 0;
	int cndx = 0;
	struct msm_fb_data_type *mfd;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;

	pr_debug("%s+:\n", __func__);

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;
	if (pipe == NULL) {
		pr_err("%s: NO base pipe\n", __func__);
		return ret;
	}

	atomic_set(&vctrl->suspend, 1);
	atomic_set(&vctrl->vsync_resume, 0);
	complete_all(&vctrl->vsync_comp);

	/* sanity check, free pipes besides base layer */
	mdp4_overlay_unset_mixer(pipe->mixer_num);
	mdp4_mixer_stage_down(pipe, 1);
	mdp4_overlay_pipe_free(pipe);
	vctrl->base_pipe = NULL;

	if (vctrl->clk_enabled) {
		/*
		 * in case of suspend, vsycn_ctrl off is not
		 * received from frame work which left clock on
		 * then, clock need to be turned off here
		 */
		mdp_clk_ctrl(0);
	}

	vctrl->clk_enabled = 0;
	vctrl->vsync_enabled = 0;
	vctrl->clk_control = 0;
	vctrl->expire_tick = 0;

	vsync_irq_disable(INTR_PRIMARY_RDPTR, MDP_PRIM_RDPTR_TERM);

	pr_debug("%s-:\n", __func__);

	/*
	 * footswitch off
	 * this will casue all mdp register
	 * to be reset to default
	 * after footswitch on later
	 */

	return ret;
}

void mdp_mddi_overlay_suspend(struct msm_fb_data_type *mfd)
{
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;
	/* dis-engage rgb0 from mixer0 */
	if (pipe) {
		if (mfd->ref_cnt == 0) {
			/* adb stop */
			if (pipe->pipe_type == OVERLAY_TYPE_BF)
				mdp4_overlay_borderfill_stage_down(pipe);

			/* pipe == rgb1 */
			mdp4_overlay_unset_mixer(pipe->mixer_num);
			vctrl->base_pipe = NULL;
		} else {
			mdp4_mixer_stage_down(pipe, 1);
			mdp4_overlay_iommu_pipe_free(pipe->pipe_ndx, 1);
		}
	}
}

void mdp4_mddi_overlay(struct msm_fb_data_type *mfd)
{
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	unsigned long flags;

	mutex_lock(&mfd->dma->ov_mutex);
	vctrl = &vsync_ctrl_db[cndx];

	if (!mfd->panel_power_on) {
		mutex_unlock(&mfd->dma->ov_mutex);
		return;
	}

	pipe = vctrl->base_pipe;
	if (pipe == NULL) {
		pr_err("%s: NO base pipe\n", __func__);
		mutex_unlock(&mfd->dma->ov_mutex);
		return;
	}

	mutex_lock(&vctrl->update_lock);
	if (!vctrl->clk_enabled) {
		pr_err("%s: mdp clocks disabled\n", __func__);
		mutex_unlock(&vctrl->update_lock);
		mutex_unlock(&mfd->dma->ov_mutex);
		return;

	}
	mutex_unlock(&vctrl->update_lock);

	spin_lock_irqsave(&vctrl->spin_lock, flags);

		/*
		 * in the middle of shutting clocks down
		 * delay to allow pan display to go through
		 */
		vctrl->expire_tick = VSYNC_EXPIRE_TICK;

	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (mfd && mfd->panel_power_on) {
		mdp4_mddi_dma_busy_wait(mfd);

		if (mddi_pipe && mddi_pipe->ov_blt_addr)
			mdp4_mddi_blt_dmap_busy_wait(mfd);
		mdp4_overlay_mdp_perf_upd(mfd, 0);
		mdp4_overlay_update_lcd(mfd);

		mdp4_overlay_mdp_perf_upd(mfd, 1);
		if (mdp_hw_revision < MDP4_REVISION_V2_1) {
			/* dmas dmap switch */
			if (mdp4_overlay_mixer_play(mddi_pipe->mixer_num)
						== 0) {
				mdp4_dma_s_update_lcd(mfd, mddi_pipe);
				mdp4_mddi_dma_s_kickoff(mfd, mddi_pipe);
			} else
				mdp4_mddi_kickoff_ui(mfd, mddi_pipe);
		} else	/* no dams dmap switch  */
			mdp4_mddi_kickoff_ui(mfd, mddi_pipe);

	/* signal if pan function is waiting for the update completion */
		if (mfd->pan_waiting) {
			mfd->pan_waiting = FALSE;
			complete(&mfd->pan_comp);
		}
	}

	mdp4_overlay_mdp_pipe_req(pipe, mfd);

	mdp4_overlay_mdp_perf_req(mfd, pipe);

	mdp4_overlay_mdp_perf_upd(mfd, 1);

	mdp4_mddi_pipe_commit();

	mdp4_overlay_mdp_perf_upd(mfd, 0);
	mutex_unlock(&mfd->dma->ov_mutex);
}

int mdp4_mddi_overlay_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	struct msm_fb_data_type *mfd = info->par;
	mutex_lock(&mfd->dma->ov_mutex);
	if (mfd && mfd->panel_power_on) {
		mdp4_mddi_dma_busy_wait(mfd);
		mdp_hw_cursor_update(info, cursor);
	}
	mutex_unlock(&mfd->dma->ov_mutex);
	return 0;
}
