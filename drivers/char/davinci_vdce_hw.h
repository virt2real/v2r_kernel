/* *
 * Copyright (C) 2007 Texas Instruments	Inc
 *
 * This	program	is free	software; you can redistribute it and/or modify
 * it under the	terms of the GNU General Public	License	as published by
 * the Free Software Foundation; either	version	2 of the License, or
 * (at your option) any	later version.
 *
 * This	program	is distributed in the hope that	it will	be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A	PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59	Temple Place, Suite 330, Boston, MA 02111-1307	USA
 */
/* davinci_vdce_hw.h	file */
#ifndef	DAVINCI_VDCE_HW_H
#define	DAVINCI_VDCE_HW_H

#ifdef __KERNEL__
#include <mach/hardware.h>
#include <asm/io.h>
#endif

#define	MAX_BLEND_TABLE	(4)

/* Register	offset mapping*/
#ifdef __KERNEL__
#define	BITSET(variable,bit)		((variable)| (1<<bit))
#define	BITRESET(variable,bit)		((variable)& (~(0x00000001<<(bit))))
#define	BITGET(variable,bit)		(((variable)& (1<<bit))>>(bit))

/* Bit position for various bits . Used in BITSET */
#define	 SET_CCV_V_TYPE		(2)
#define	 SET_CCV_H_TYPE		(3)
#define	 SET_CCV_IN_MPEG1	(1)
#define	 SET_CCV_OUT_MPEG1	(0)
#define	 SET_CCV_ENABLE		(10)
#define	 VDCE_RASTER_SCANNING	(0)
/* Bits position number to be used in bitset for setting various registers */
#define	SET_HRSZ_ENABLE		(0)
#define	SET_VRSZ_ENABLE		(4)
#define	SET_HRSZ_ALF_ENABLE	(8)
#define	SET_VRSZ_ALF_ENABLE	(12)
#define	SET_RSZ_H_TYPE		(1)
#define	SET_RSZ_V_TYPE		(5)
#define	SET_RSZ_H_ALF_mode	(9)
#define	SET_LUMA_ENABLE		(1)
#define	SET_CHROMA_ENABLE	(2)
#define	SET_TOP_ENABLE		(7)
#define	SET_BOT_ENABLE		(6)
#define	SET_SRC_MODE		(14)
#define	SET_RES_MODE		(13)
#define	SET_BMP_MODE		(15)
#define	SET_PRO_MODE		(12)
/* various modes supported in hardware. The value generated by them */
#define	MODE_PRECODEC		(2)
#define	MODE_POSTCODEC		(3)
#define	MODE_TRANSCODEC		(1)
#define	MODE_EPAD		(0)

#define	SET_RSZ_ENABLE		(8)
#define	SET_RMAP_ENABLE		(9)
#define	SET_BLEND_ENABLE	(11)
#define	FOUR_TWO_TWO		(1)
#define	SET_RMAP_YENABLE	(3)
#define	SET_RMAP_CENABLE	(11)

/* various shifts and masks of register */
#define	LUMA_CHROMA_ENABLE_MASK	(0x6)
#define	LUMA_CHROMA_ENABLE_SHIFT (0x1)
#define	TOP_BOTTOM_ENABLE_MASK	(0xc0)
#define	TOP_BOTTOM_ENABLE_SHIFT	(0x6)

/* Mask and shift values for range mapping registers */
#define	RANGE_MAP_Y_SHIFT	(0)
#define	RANGE_MAP_Y_MASK	(0x7<<RANGE_MAP_Y_SHIFT)
#define	RANGE_MAP_C_SHIFT	(8)
#define	RANGE_MAP_C_MASK	(0x7<<RANGE_MAP_C_SHIFT)
/* Mask and shift values for Blending registers */
#define	BLD_LUT_Y_SHIFT		(16)
#define	BLD_LUT_Y_MASK		(0xff<<BLD_LUT_Y_SHIFT)
#define	BLD_LUT_CB_SHIFT	(0)
#define	BLD_LUT_CB_MASK		(0xff<<BLD_LUT_CB_SHIFT)
#define	BLD_LUT_CR_SHIFT	(8)
#define	BLD_LUT_CR_MASK		(0xff<<BLD_LUT_CR_SHIFT)
#define	BLD_LUT_FCT_SHIFT	(24)
#define	BLD_LUT_FCT_MASK	(0xff<<BLD_LUT_FCT_SHIFT)
/* Mask and shift values for Edgepadding registers */
#define	EPD_C_HEXT_SHIFT	(0)
#define	EPD_C_HEXT_MASK		(0x3f<<EPD_C_HEXT_SHIFT)
#define	EPD_C_VEXT_SHIFT	(8)
#define	EPD_C_VEXT_MASK		(0x3f<<EPD_C_VEXT_SHIFT	 )
#define	EPD_Y_VEXT_SHIFT	(8)
#define	EPD_Y_VEXT_MASK		(0x3f<<EPD_Y_VEXT_SHIFT)
#define	EPD_Y_HEXT_SHIFT	(0)
#define	EPD_Y_HEXT_MASK		(0x3f<<EPD_Y_HEXT_SHIFT	)
/* Mask and shift values for Resizing registers */
#define	RSZ_H_MAG_SHIFT		(0)
#define	RSZ_H_MAG_MASK		(0xfff<<RSZ_H_MAG_SHIFT	)
#define	RSZ_V_MAG_SHIFT		(0)
#define	RSZ_V_MAG_MASK		(0xfff<<RSZ_V_MAG_SHIFT	)

#define	VDCE_REQ_SZ_SHIFT		(0)
#define	VDCE_REQ_SZ_MASK		(0x1ff<<VDCE_REQ_SZ_SHIFT	)
#define	VDCE_PRCS_UNIT_SIZE_SHIFT	 (0)
#define	VDCE_PRCS_UNIT_SIZE_MASK	 (0x1ff<<VDCE_PRCS_UNIT_SIZE_SHIFT )
/* source and result image size shift and mask */
#define	SRC_Y_HSZ_SHIFT		(0)
#define	SRC_Y_HSZ_MASK		(0xfff<<SRC_Y_HSZ_SHIFT	)
#define	SRC_Y_VSZ_SHIFT		(16)
#define	SRC_Y_VSZ_MASK		(0xfff<<SRC_Y_VSZ_SHIFT	)
#define	RES_Y_HSZ_SHIFT		(0)
#define	RES_Y_HSZ_MASK		(0xfff<<SRC_Y_HSZ_SHIFT	)
#define	RES_Y_VSZ_SHIFT		(16)
#define	RES_Y_VSZ_MASK		(0xfff<<RES_Y_VSZ_SHIFT	)
#define	SRC_C_HSZ_SHIFT		(0)
#define	SRC_C_HSZ_MASK		(0xfff<<SRC_C_HSZ_SHIFT	)
#define	SRC_C_VSZ_SHIFT		(16)
#define	SRC_C_VSZ_MASK		(0xfff<<SRC_C_VSZ_SHIFT	 )
#define	RES_C_HSZ_SHIFT		(0)
#define	RES_C_HSZ_MASK		(0xfff<<RES_C_HSZ_SHIFT	)
#define	RES_C_VSZ_SHIFT		(16)
#define	RES_C_VSZ_MASK		(0xfff<<RES_C_VSZ_SHIFT)
#define	SRC_BMP_HSZ_SHIFT	(0)
#define	SRC_BMP_HSZ_MASK	(0xfff<<SRC_BMP_HSZ_SHIFT )
#define	SRC_BMP_VSZ_SHIFT	(16)
#define	SRC_BMP_VSZ_MASK	(0xfff<<SRC_BMP_VSZ_SHIFT )
#define	SRC_BMP_STRT_HPS_SHIFT	  (0)
#define	SRC_BMP_STRT_HPS_MASK	  (0xfff<<SRC_BMP_STRT_HPS_SHIFT)
#define	RES_BMP_STRT_VPS_SHIFT	(16)
#define	RES_BMP_STRT_VPS_MASK	(0xfff<<RES_BMP_STRT_VPS_SHIFT )

#define	VDCE_MODE_SHIFT		(4)
#define	VDCE_MODE_MASK		(0x3 <<VDCE_MODE_SHIFT )
#define	RSZ_ALF_INTENSITY_SHIFT	(0)
#define	RSZ_ALF_INTENSITY_MASK	(0xff)
/* Base address and offset for various ge registers */
#define	VDCE_IOBASE_VADDR	IO_ADDRESS(0x01c12800)
/*#define	VDCE_IOBASE_VADDR	(0x01c12800)*/
#define	VDCE_ENABLE		(0x0)
#define	VDCE_CTRL		(0x04)
#define	VDCE_INTEN		(0x08)
#define	VDCE_INTEN_SET		(0x0c)
#define	VDCE_EMULATION_CTRL	(0x1c)
#define	VDCE_STATUS		(0x14)
#define	VDCE_STATUS_CLR		(0x18)
#define	VDCE_SDR_FMT		(0x20)
#define	VDCE_REQ_SZ		(0x24)
#define	VDCE_PRCS_UNIT_SIZE	(0x28)
#define	VDCE_SRC_STRT_ADD_YTOP	(0x40)
#define	VDCE_SRC_STRT_OFF_YTOP	(0x48)
#define	VDCE_SRC_STRT_ADD_YBOT	(0x4c)
#define	VDCE_SRC_STRT_OFF_YBOT	(0x54)
#define	VDCE_SRC_STRT_ADD_CTOP	(0x58)
#define	VDCE_SRC_STRT_OFF_CTOP	(0x60)
#define	VDCE_SRC_STRT_ADD_CBOT	(0x64)
#define	VDCE_SRC_STRT_OFF_CBOT	(0x6c)
#define	VDCE_SRC_STRT_ADD_BMP_TOP	(0x70)
#define	VDCE_SRC_STRT_OFF_BMP_TOP	(0x74)
#define	VDCE_SRC_STRT_ADD_BMP_BOT	(0x78)
#define	VDCE_SRC_STRT_OFF_BMP_BOT	(0x7c)
#define	VDCE_RES_STRT_ADD_YTOP	(0x80)
#define	VDCE_RES_STRT_OFF_YTOP	(0x88)
#define	VDCE_RES_STRT_ADD_YBOT	(0x8c)
#define	VDCE_RES_STRT_OFF_YBOT	(0x94)
#define	VDCE_RES_STRT_ADD_CTOP	(0x98)
#define	VDCE_RES_STRT_OFF_CTOP	(0xa0)
#define	VDCE_RES_STRT_ADD_CBOT	(0xa4)
#define	VDCE_RES_STRT_OFF_CBOT	(0xac)
#define	SRC_Y_STRT_PS		(0xc0)
#define	SRC_Y_SZ		(0xc4)
#define	SRC_C_STRT_PS		(0xc8)
#define	SRC_C_SZ		(0xcc)
#define	SRC_BMP_STRT_PS		(0xd0)
#define	SRC_BMP_SZ		(0xd4)
#define	RES_Y_STRT_PS		(0xe0)
#define	RES_Y_SZ		(0xe4)
#define	RES_C_STRT_PS		(0xe8)
#define	RES_C_SZ		(0xec)
#define	RES_BMP_STRT_PS		(0xf0)
#define	RSZ_MODE		(0x100)
#define	RSZ_H_MAG		(0x104)
#define	RSZ_V_MAG		(0x108)
#define	RSZ_H_PHASE		(0x10c)
#define	RSZ_V_PHASE		(0x110)
#define	RSZ_ALF_INTENSITY	(0x114)
#define	CCV_MODE		(0x120)
#define	BLD_LUT_00		(0x140)
#define	BLD_LUT_01		(0x144)
#define	BLD_LUT_02		(0x148)
#define	BLD_LUT_03		(0x14c)
#define	RGMP_CTRL		(0x160)
#define	EPD_LUMA_WIDTH		(0x184)
#define	EPD_CHROMA_WIDTH	(0x188)
/* Register read/write */
#define	regw(val,reg)		__raw_writel(val,((reg)+ VDCE_IOBASE_VADDR))
#define	regr(reg)		__raw_readl((reg)+VDCE_IOBASE_VADDR)

#define vdce_isbusy()		(regr(VDCE_CTRL) & 0x01)

/* register mapping structure */
typedef struct vdce_hw_config {
	unsigned int vdce_ctrl;
	unsigned int vdce_emulation_ctrl;
	unsigned int vdce_sdr_fmt;
	unsigned int vdce_req_sz;
	unsigned int vdce_prcs_unit_size;
	u32 src_Y_strt_ps;
	u32 src_Y_sz;
	u32 src_C_strt_ps;
	u32 src_C_sz;
	u32 src_bmp_strt_ps;
	u32 src_bmp_sz;
	u32 res_Y_strt_ps;
	u32 res_Y_sz;
	u32 res_C_strt_ps;
	u32 res_C_sz;
	u32 res_bmp_strt_ps;
	unsigned int vdce_src_strt_add_ytop;
	unsigned int vdce_src_strt_add_ctop;
	unsigned int vdce_src_strt_add_ybot;
	unsigned int vdce_src_strt_add_cbot;
	unsigned int vdce_res_strt_add_ytop;
	unsigned int vdce_res_strt_add_ctop;
	unsigned int vdce_res_strt_add_ybot;
	unsigned int vdce_res_strt_add_cbot;
	unsigned int vdce_src_add_ofst_ytop;
	unsigned int vdce_src_add_ofst_ctop;
	unsigned int vdce_src_add_ofst_ybot;
	unsigned int vdce_src_add_ofst_cbot;
	unsigned int vdce_src_strt_add_bmp_top;
	unsigned int vdce_src_strt_add_bmp_bot;
	unsigned int vdce_src_strt_off_bmp_top;
	unsigned int vdce_src_strt_off_bmp_bot;
	unsigned int vdce_res_strt_off_ytop;
	unsigned int vdce_res_strt_off_ctop;
	unsigned int vdce_res_strt_off_ybot;
	unsigned int vdce_res_strt_off_cbot;
	u32 rsz_mode;
	u32 rsz_h_mag;
	u32 rsz_v_mag;
	u32 rsz_h_phase;
	u32 rsz_v_phase;
	u32 rsz_alf_intensity;
	u32 ccv_mode;
	u32 bld_lut[MAX_BLEND_TABLE];
	u32 rgmp_ctrl;
	u32 epd_luma_width;
	u32 epd_chroma_width;
} vdce_hw_config_t;

int vdce_enable_int(void);
int vdce_clear_status(void);
int vdce_enable(vdce_hw_config_t *);
void vdce_hw_setup(vdce_hw_config_t *);
#endif
#endif
