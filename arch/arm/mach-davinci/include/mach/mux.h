/*
 * Table of the DAVINCI register configurations for the PINMUX combinations
 *
 * Author: Vladimir Barinov, MontaVista Software, Inc. <source@mvista.com>
 *
 * Based on linux/include/asm-arm/arch-omap/mux.h:
 * Copyright (C) 2003 - 2005 Nokia Corporation
 *
 * Written by Tony Lindgren
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Copyright (C) 2008 Texas Instruments.
 */

#ifndef __INC_MACH_MUX_H
#define __INC_MACH_MUX_H

struct mux_config {
	const char *name;
	const char *mux_reg_name;
	const unsigned char mux_reg;
	const unsigned char mask_offset;
	const unsigned char mask;
	const unsigned char mode;
	bool debug;
};

enum davinci_dm365_index {
	/* MMC/SD 0 */
	DM365_MMCSD0,

	/* MMC/SD 1 */
	DM365_SD1_CLK,
	DM365_SD1_CMD,
	DM365_SD1_DATA3,
	DM365_SD1_DATA2,
	DM365_SD1_DATA1,
	DM365_SD1_DATA0,

	/* I2C */
	DM365_I2C_SDA,
	DM365_I2C_SCL,

	/* AEMIF */
	DM365_AEMIF_CE0,
	DM365_AEMIF_CE1,

	/* ASP0 function */
	/* SPI0 */

	/* UART */
	DM365_UART0_RXD,
	DM365_UART0_TXD,
	/* EMAC */
	DM365_EMAC_TX_EN,
	DM365_EMAC_TX_CLK,
	DM365_EMAC_COL,
	DM365_EMAC_TXD3,
	DM365_EMAC_TXD2,
	DM365_EMAC_TXD1,
	DM365_EMAC_TXD0,
	DM365_EMAC_RXD3,
	DM365_EMAC_RXD2,
	DM365_EMAC_RXD1,
	DM365_EMAC_RXD0,
	DM365_EMAC_RX_CLK,
	DM365_EMAC_RX_DV,
	DM365_EMAC_RX_ER,
	DM365_EMAC_CRS,
	DM365_EMAC_MDIO,
	DM365_EMAC_MDCLK,

	/* Key Scan */

	/* PWM */
	DM365_PWM0,
	DM365_PWM0_G23,
	DM365_PWM1,
	DM365_PWM1_G25,
	DM365_PWM2_G87,
	DM365_PWM2_G88,
	DM365_PWM2_G89,
	DM365_PWM2_G90,
	DM365_PWM3_G80,
	DM365_PWM3_G81,
	DM365_PWM3_G85,
	DM365_PWM3_G86,

	/* SPI1 */

	/* SPI2 */

	/* SPI3 */

	/* SPI4 */
        DM365_SPI4_SDENA0,
	/* GPIO */
	DM365_GPIO20,
	DM365_GPIO22,
	DM365_GPIO23,
	DM365_GPIO24,
	DM365_GPIO25,
	DM365_GPIO26,
	DM365_GPIO27,
	DM365_GPIO28,
	DM365_GPIO29,
	DM365_GPIO30,
	DM365_GPIO31,
	DM365_GPIO32,
	DM365_GPIO33,
	DM365_GPIO34,
	DM365_GPIO35,
	DM365_GPIO44,
	DM365_GPIO45,
	DM365_GPIO46,
	DM365_GPIO47,
	DM365_GPIO48,
	DM365_GPIO49,
	DM365_GPIO50,
	DM365_GPIO51,
	DM365_GPIO66,
	DM365_GPIO67,
	DM365_GPIO79,
	DM365_GPIO80,
	DM365_GPIO81,
	DM365_GPIO82,
	DM365_GPIO83,
	DM365_GPIO84,
	DM365_GPIO85,
	DM365_GPIO86,
	DM365_GPIO87,
	DM365_GPIO88,
	DM365_GPIO89,
	DM365_GPIO90,
	DM365_GPIO91,
	DM365_GPIO92,
	DM365_GPIO100,
	DM365_GPIO101,
	DM365_GPIO102,
	DM365_GPIO103,
	

	/* CLKOUT */
	DM365_CLKOUT1,
	DM365_CLKOUT2,

	/* Video */
	DM365_EXTCLK,
	DM365_VIN_CAM_WEN,
	DM365_VIN_CAM_VD,
	DM365_VIN_CAM_HD,
	DM365_VIN_YIN4_7_EN,
	DM365_VIN_YIN0_3_EN,

	/* IRQ muxing */
	DM365_INT_EDMA_CC,
	DM365_INT_EDMA_TC0_ERR,
	DM365_INT_EDMA_TC1_ERR,
	DM365_INT_EDMA_TC2_ERR,
	DM365_INT_EDMA_TC3_ERR,
	DM365_INT_PRTCSS,
	DM365_INT_EMAC_RXTHRESH,
	DM365_INT_EMAC_RXPULSE,
	DM365_INT_EMAC_TXPULSE,
	DM365_INT_EMAC_MISCPULSE,
	DM365_INT_IMX0_ENABLE,
	DM365_INT_IMX0_DISABLE,
	DM365_INT_HDVICP_ENABLE,
	DM365_INT_HDVICP_DISABLE,
	DM365_INT_IMX1_ENABLE,
	DM365_INT_IMX1_DISABLE,
	DM365_INT_NSF_ENABLE,
	DM365_INT_NSF_DISABLE,

	/* EDMA event muxing */
	DM365_EVT2_ASP_TX,
	DM365_EVT3_ASP_RX,
	DM365_EVT2_VC_TX,
	DM365_EVT3_VC_RX,
	DM365_EVT26_MMC0_RX,

	/* Video Output RG888*/
};

#ifdef CONFIG_DAVINCI_MUX
/* setup pin muxing */
extern int davinci_cfg_reg(unsigned long reg_cfg);
#else
/* boot loader does it all (no warnings from CONFIG_DAVINCI_MUX_WARNINGS) */
static inline int davinci_cfg_reg(unsigned long reg_cfg) { return 0; }
#endif

#endif /* __INC_MACH_MUX_H */
