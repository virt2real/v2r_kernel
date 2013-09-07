/*
 * DM365 Leopard Board
 *
 * Derived from: arch/arm/mach-davinci/board-dm365-evm.c
 * RidgeRun Copyright (C) 2010.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**************************************************************************
 * Included Files
 **************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/autoconf.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/leds.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/mux.h>
#include <mach/hardware.h>
#include <mach/dm365.h>
#include <mach/psc.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/serial.h>
#include <mach/mmc.h>
#include <mach/nand.h>
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <linux/videodev2.h>
#include <media/davinci/videohd.h>
#include <media/davinci/dm365_generic_prgb_encoder.h>
#include <media/tvp514x.h>
#include <media/tvp7002.h>
#include "dm365pwcr.h"

#define DM365_EVM_PHY_MASK		(0x2)
#define DM365_EVM_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

#define DM365_ASYNC_EMIF_CONTROL_BASE	0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE	0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE	0x04000000
//volatile unsigned char* out_port = (volatile unsigned char*)DM365_ASYNC_EMIF_DATA_CE1_BASE;
//static void out_debug(const char c){
//     *out_port = c;
//}
static struct snd_platform_data dm365_leopard_snd_data = {
	.eventq_no = EVENTQ_3,
};

/*
static struct i2c_board_info i2c_info[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{
		I2C_BOARD_INFO("ths7303", 0x2c),
	}
};
*/

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= 400	/* kHz */,
	.bus_delay	= 0	/* usec */,
	.sda_pin        = 21,
	.scl_pin        = 20,
};

/* Input available at the ov7690 */
//Shadrin camera
static struct v4l2_input mt9p031_inputs[] = {
	{
		.index = 0,
		.name = "Camera",
		.type = V4L2_INPUT_TYPE_CAMERA,
	}
};

//Shadrin camera
static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		//Clock for camera????
		.module_name = "mt9p031",
		.is_camera = 1,
		.grp_id = VPFE_SUBDEV_MT9P031,
		.num_inputs = ARRAY_SIZE(mt9p031_inputs),
		.inputs = mt9p031_inputs,
		.ccdc_if_params = {
			.if_type = VPFE_RAW_BAYER,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("mt9p031", 0x30),
			/* this is for PCLK rising edge */
			.platform_data = (void *)1,
		},
	}
};

static struct vpfe_config vpfe_cfg = {
       .num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
       .sub_devs = vpfe_sub_devs,
       .card_name = "DM365 Leopard",
       .ccdc = "DM365 ISIF",
       .num_clocks = 1,
       .clocks = {"vpss_master"},
};

/*Need to review if this is necessary*/
static struct davinci_mmc_config dm365leopard_mmc_config = {
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};


static void dm365leopard_emac_configure(void)
{
	/*
	 * EMAC pins are multiplexed with GPIO and UART
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 125 - 127
	 */
	davinci_cfg_reg(DM365_EMAC_TX_EN);
	davinci_cfg_reg(DM365_EMAC_TX_CLK);
	davinci_cfg_reg(DM365_EMAC_COL);
	davinci_cfg_reg(DM365_EMAC_TXD3);
	davinci_cfg_reg(DM365_EMAC_TXD2);
	davinci_cfg_reg(DM365_EMAC_TXD1);
	davinci_cfg_reg(DM365_EMAC_TXD0);
	davinci_cfg_reg(DM365_EMAC_RXD3);
	davinci_cfg_reg(DM365_EMAC_RXD2);
	davinci_cfg_reg(DM365_EMAC_RXD1);
	davinci_cfg_reg(DM365_EMAC_RXD0);
	davinci_cfg_reg(DM365_EMAC_RX_CLK);
	davinci_cfg_reg(DM365_EMAC_RX_DV);
	davinci_cfg_reg(DM365_EMAC_RX_ER);
	davinci_cfg_reg(DM365_EMAC_CRS);
	davinci_cfg_reg(DM365_EMAC_MDIO);
	davinci_cfg_reg(DM365_EMAC_MDCLK);

	/*
	 * EMAC interrupts are multiplexed with GPIO interrupts
	 * Details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 133 - 134
	 */
	davinci_cfg_reg(DM365_INT_EMAC_RXTHRESH);
	davinci_cfg_reg(DM365_INT_EMAC_RXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_TXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_MISCPULSE);
}

static void dm365leopard_mmc_configure(void)
{
	/*
	 * MMC/SD pins are multiplexed with GPIO and EMIF
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 118, 128 - 131
	 */
	davinci_cfg_reg(DM365_SD1_CLK);
	davinci_cfg_reg(DM365_SD1_CMD);
	davinci_cfg_reg(DM365_SD1_DATA3);
	davinci_cfg_reg(DM365_SD1_DATA2);
	davinci_cfg_reg(DM365_SD1_DATA1);
	davinci_cfg_reg(DM365_SD1_DATA0);
}


static void dm365leopard_usb_configure(void)
{
	davinci_cfg_reg(DM365_GPIO66);
	gpio_request(66, "usb");
	gpio_direction_output(66, 1);
	setup_usb(500, 8);
}

static void dm365leopard_wifi_configure(void)
{
	/*
	* CLKOUT1 pin is multiplexed with GPIO35 and SPI4
	* Further details are available at the DM365 ARM
	* Subsystem Users Guide(sprufg5.pdf) pages 118, 127 - 129
	*/
	/* Setup PWCTRO2 to generate 32kHz
	 * Setup GPIO36 as output 1 (WLAN RESET)
	 * Setup GPIO69 as output 1 (WLAN_SHDN)
	 * Setup sd2/mmc2 interface
	 * The HDG104 driver will be loaded later
	 */
	//Setup PWCTRO2 to generate 32kHz
	printk("Before start WiFi 32k\r\n");
	//configure_prtcss_32k();
	printk("After start WiFi 32k\r\n");
	//SET WLAN RESET
	{
		volatile int i = 0;
		for (i = 0; i < 65536; i++){
			if ((i%4096) == 0) printk("*");

		}
	}
	gpio_request(36, "wifi_reset");
	gpio_direction_output(36, 1);
	//SET WLAN_SHDN
	gpio_request(69, "wifi_shdn");
	gpio_direction_output(69, 1);
	//LETS CONSIDER SD2 AS INITIALIZED

#if 0
	struct clk *clkout1_clk;

	davinci_cfg_reg(DM365_CLKOUT1);

	clkout1_clk = clk_get(NULL, "clkout1");
	if (IS_ERR(clkout1_clk))
		return;
	clk_enable(clkout1_clk);

	/*
	* Configure CLKOUT1 OBSCLK registers
	*/

	/* (reg OCSEL) Setting OBSCLK source with Oscillator divider output enable */
	__raw_writel(0x0,IO_ADDRESS(0x01C40C00 + 0x104));

	/* (reg OSCDIV1) Setting the Oscillator divider enable with a divider ratio of 1 */
	__raw_writel(0x8000,IO_ADDRESS(0x01C40C00 + 0x124));

	/* (reg CKEN) Setting the OBSCLK clock enable */
	__raw_writel(0x02,IO_ADDRESS(0x01C40C00 + 0x148));
#endif
}
static void dm365leopard_camera_configure(void){
	struct clk *clkout0_clk;
	clkout0_clk = clk_get(NULL, "clkout0");
	clk_enable(clkout0_clk);

	davinci_cfg_reg(DM365_SPI4_SDENA0);
        davinci_cfg_reg(DM365_VIN_YIN0_3_EN);
	gpio_request(98, "Camera_on_off");
	gpio_direction_output(98, 0);
	gpio_request(99, "Camera_reset");
	gpio_direction_output(99, 1);//Reset is active low, thus set this pin to high
	davinci_cfg_reg(DM365_EXTCLK);

}
static void dm365leopard_gpio_configure(void){
//
	gpio_request(22, "GPIO22");
	gpio_direction_input(22);
	davinci_cfg_reg(DM365_GPIO22);
//
	gpio_request(23, "GPIO23");
	gpio_direction_input(23);
	davinci_cfg_reg(DM365_GPIO23);
//
	gpio_request(24, "GPIO24");
	gpio_direction_input(24);
	davinci_cfg_reg(DM365_GPIO24);
//
	gpio_request(25, "GPIO25");
	gpio_direction_input(25);
	davinci_cfg_reg(DM365_GPIO25);
//
	gpio_request(26, "GPIO26");
	gpio_direction_input(26);
	davinci_cfg_reg(DM365_GPIO26);
//
	gpio_request(27, "GPIO27");
	gpio_direction_input(27);
	davinci_cfg_reg(DM365_GPIO27);
//
	gpio_request(28, "GPIO28");
	gpio_direction_input(28);
	davinci_cfg_reg(DM365_GPIO28);
//
	gpio_request(29, "GPIO29");
	gpio_direction_input(29);
	davinci_cfg_reg(DM365_GPIO29);
//
	gpio_request(30, "GPIO30");
	gpio_direction_input(30);
	davinci_cfg_reg(DM365_GPIO30);
//
	gpio_request(31, "GPIO31");
	gpio_direction_input(31);
	davinci_cfg_reg(DM365_GPIO31);
//
	gpio_request(32, "GPIO32");
	gpio_direction_input(32);
	davinci_cfg_reg(DM365_GPIO32);
//
	gpio_request(33, "GPIO33");
	gpio_direction_input(33);
	davinci_cfg_reg(DM365_GPIO33);
//
	gpio_request(34, "GPIO34");
	gpio_direction_input(34);
	davinci_cfg_reg(DM365_GPIO34);
//
	gpio_request(35, "GPIO35");
	gpio_direction_input(35);
	davinci_cfg_reg(DM365_GPIO35);
//
	gpio_request(44, "GPIO44");
	gpio_direction_input(44);
	davinci_cfg_reg(DM365_GPIO44);
//
	gpio_request(45, "GPIO45");
	gpio_direction_input(45);
	davinci_cfg_reg(DM365_GPIO45);
//
	gpio_request(46, "GPIO46");
	gpio_direction_input(46);
	davinci_cfg_reg(DM365_GPIO46);
//
	gpio_request(47, "GPIO47");
	gpio_direction_input(47);
	davinci_cfg_reg(DM365_GPIO47);
//
	gpio_request(48, "GPIO48");
	gpio_direction_input(48);
	davinci_cfg_reg(DM365_GPIO48);
//
	gpio_request(49, "GPIO49");
	gpio_direction_input(49);
	davinci_cfg_reg(DM365_GPIO49);
//
	gpio_request(50, "GPIO50");
	gpio_direction_input(50);
	davinci_cfg_reg(DM365_GPIO50);
//
	gpio_request(51, "GPIO51");
	gpio_direction_input(51);
	davinci_cfg_reg(DM365_GPIO51);
//
	gpio_request(67, "GPIO67");
	gpio_direction_input(67);
	davinci_cfg_reg(DM365_GPIO67);
//
	gpio_request(79, "GPIO79");
	gpio_direction_input(79);
	davinci_cfg_reg(DM365_GPIO79);
//
	gpio_request(80, "GPIO80");
	gpio_direction_input(80);
	davinci_cfg_reg(DM365_GPIO80);
//
	gpio_request(81, "GPIO81");
	gpio_direction_input(81);
	davinci_cfg_reg(DM365_GPIO81);
//
	gpio_request(82, "GPIO82");
	gpio_direction_input(82);
	davinci_cfg_reg(DM365_GPIO82);
//
	gpio_request(83, "GPIO83");
	gpio_direction_input(83);
	davinci_cfg_reg(DM365_GPIO83);
//
	gpio_request(84, "GPIO84");
	gpio_direction_input(84);
	davinci_cfg_reg(DM365_GPIO84);
//
	gpio_request(85, "GPIO85");
	gpio_direction_input(85);
	davinci_cfg_reg(DM365_GPIO85);
//
	gpio_request(86, "GPIO86");
	gpio_direction_input(86);
	davinci_cfg_reg(DM365_GPIO86);
//
	gpio_request(87, "GPIO87");
	gpio_direction_input(87);
	davinci_cfg_reg(DM365_GPIO87);
//
	gpio_request(88, "GPIO88");
	gpio_direction_input(88);
	davinci_cfg_reg(DM365_GPIO88);
//
	gpio_request(89, "GPIO89");
	gpio_direction_input(89);
	davinci_cfg_reg(DM365_GPIO89);
//
	gpio_request(90, "GPIO90");
	gpio_direction_input(90);
	davinci_cfg_reg(DM365_GPIO90);
//
	gpio_request(91, "GPIO91");
	gpio_direction_input(91);
	davinci_cfg_reg(DM365_GPIO91);
//
	gpio_request(92, "GPIO92");
	gpio_direction_input(92);
	davinci_cfg_reg(DM365_GPIO92);
//
	gpio_request(100, "GPIO100");
	gpio_direction_input(100);
	davinci_cfg_reg(DM365_GPIO100);
//
	gpio_request(101, "GPIO101");
	gpio_direction_input(101);
	davinci_cfg_reg(DM365_GPIO101);
//
	gpio_request(102, "GPIO102");
	gpio_direction_input(102);
	davinci_cfg_reg(DM365_GPIO102);
//
	gpio_request(103, "GPIO103");
	gpio_direction_input(103);
	davinci_cfg_reg(DM365_GPIO103);
}
#if 0
static void dm365leopard_prgb_out_configure(void)
{
	/*
	 * R/G/B 0 and 1 pins are multiplexed with GPIOs
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 118, 128 - 131
	 */
	davinci_cfg_reg(DM365_VOUT_B0);
	davinci_cfg_reg(DM365_VOUT_B1);
	davinci_cfg_reg(DM365_VOUT_B2);
	davinci_cfg_reg(DM365_VOUT_R0);
	davinci_cfg_reg(DM365_VOUT_R1);
	davinci_cfg_reg(DM365_VOUT_R2);
	davinci_cfg_reg(DM365_VOUT_G0);
	davinci_cfg_reg(DM365_VOUT_G1);
	davinci_cfg_reg(DM365_VOUT_COUTL_EN);
	davinci_cfg_reg(DM365_VOUT_COUTH_EN);
	davinci_cfg_reg(DM365_VOUT_LCD_OE);
	davinci_cfg_reg(DM365_VOUT_HVSYNC);
}
#endif

void enable_lcd(void)
{
}
EXPORT_SYMBOL(enable_lcd);

void enable_hd_clk(void)
{
}
EXPORT_SYMBOL(enable_hd_clk);


static void __init leopard_init_i2c(void)
{
	davinci_cfg_reg(DM365_GPIO20);
	gpio_request(20, "i2c-scl");
	gpio_direction_output(20, 0);
	davinci_cfg_reg(DM365_I2C_SCL);

	davinci_init_i2c(&i2c_pdata);
	//i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info)); //Because we do not use McBSP codec and hi res video out
}

//int set_prgb_pixel_clk_output(unsigned int required_clk_rate);

/*static struct davinci_gen_prgb_pdata gen_prgb_data = {
	.xres = 1280,
	.yres = 720,
	.fps = {60, 1},
	.pixel_clock_khz = 0,
	.left_margin = 300,
	.right_margin = 70,
	.upper_margin = 26,
	.lower_margin = 3,
	.hsync_len = 80,
	.vsync_len = 5,
	.flags = 0,
	.clock_set_function = NULL,
};

static struct platform_device gen_prgb_device = {
	.name			= PRGB_ENCODER_DRV_NAME,
	.id				= 0,
	.num_resources	= 0,
	.resource		= NULL,
	.dev			= {
		.platform_data	= &gen_prgb_data,
	},
};*///Commented because we do not use LCD in v2r design

static struct davinci_nand_pdata davinci_nand_data = {
	.mask_chipsel		= 0,
	.ecc_mode		= NAND_ECC_HW,
	.ecc_bits		= 4,
};
//!!!!!!!!!!!!!!!!!!!!!! Find out the matter
static struct resource davinci_nand_resources[] = {
	{
		.start		= DM365_ASYNC_EMIF_DATA_CE0_BASE,
		.end		= DM365_ASYNC_EMIF_DATA_CE0_BASE + SZ_32M - 1,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= DM365_ASYNC_EMIF_CONTROL_BASE,
		.end		= DM365_ASYNC_EMIF_CONTROL_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device davinci_nand_device = {
	.name			= "davinci_nand",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(davinci_nand_resources),
	.resource		= davinci_nand_resources,
	.dev			= {
		.platform_data	= &davinci_nand_data,
	},
};
static struct resource davinci_usb_console_resources[] = {
		{
			.start		= DM365_ASYNC_EMIF_DATA_CE1_BASE,
			.end		= DM365_ASYNC_EMIF_DATA_CE1_BASE + SZ_32M - 1,
			.flags		= IORESOURCE_MEM,
		}
};
static struct platform_device davinci_usb_console_device = {
	.name			= "davinci_usb_console",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(davinci_usb_console_resources),
	.resource		= davinci_usb_console_resources
};
static struct platform_device *dm365_leopard_devices[] __initdata = {
	&davinci_nand_device,
	&davinci_usb_console_device
	//&gen_prgb_device,
};


static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};
static void __init dm365_leopard_map_io(void)
{
	/* setup input configuration for VPFE input devices */
	dm365_set_vpfe_config(&vpfe_cfg);
	dm365_init();
}

static __init void dm365_leopard_init(void)
{
	//out_debug('X');	out_debug('X');out_debug('X');
	leopard_init_i2c();
	davinci_serial_init(&uart_config);
	//out_debug('X');	out_debug('X');out_debug('X');
	printk("LEOPARD BOARD INIT\r\n");
	dm365leopard_emac_configure();
	printk("EMAC CONFIGURED\r\n");
	dm365leopard_usb_configure();
	printk("USB CONFIGURED\r\n");
	dm365leopard_mmc_configure();
	printk("MMC CONFIGURED\r\n");
	//dm365leopard_prgb_out_configure();
	davinci_setup_mmc(0, &dm365leopard_mmc_config);
	davinci_setup_mmc(1, &dm365leopard_mmc_config);
	dm365_init_vc(&dm365_leopard_snd_data);
	dm365_init_rtc();
	printk("RTC CONFIGURED\r\n");
	dm365leopard_wifi_configure();
	printk("WIFI PARTIALLY CONFIGURED\r\n");
	dm365leopard_camera_configure();
	platform_add_devices(dm365_leopard_devices, ARRAY_SIZE(dm365_leopard_devices));
        dm365leopard_gpio_configure();
        printk("GPIO CONFIGURED\r\n");
}

static __init void dm365_leopard_irq_init(void)
{
	davinci_irq_init();
}

#ifdef CONFIG_V2R_PARSE_CMDLINE
static void v2r_parse_cmdline(char * string)
{

    char *p;
    char *temp_string;
    char *temp_param;
    char *param_name;
    char *param_value;
    printk(KERN_INFO "Parse kernel cmdline:\n");
    temp_string = kstrdup(string, GFP_KERNEL);

    do
    {
	p = strsep(&temp_string, " ");
	if (p) {
	    // split param string into two parts
	    temp_param = kstrdup(p, GFP_KERNEL);
	    param_name = strsep(&temp_param, "=");
	    if (!param_name) continue;
	    //printk(KERN_INFO "%s\n", temp_value);
	    param_value = strsep(&temp_param, " ");
	    if (!param_value) continue;
	    //printk(KERN_INFO "%s\n", param_value);
	    //printk (KERN_INFO "param %s = %s\n", param_name, param_value);
	    
	    // i'd like to use switch, but fig tam
	    
	    if (!strcmp(param_name, "pwrled")) {
		if (!strcmp(param_value, "on")) {
		    printk(KERN_INFO "Power LED set ON\n");
		    // turn on blue led
		    u8 result = 0;
		    result = davinci_rtcss_read(0x00);
		    result |= (1<<3);
		    davinci_rtcss_write(result, 0x00);
		}
	    }

	    #if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	    if (!strcmp(param_name, "redled")) {
		v2r_led[0].default_trigger = param_value;
	    }

	    if (!strcmp(param_name, "greenled")) {
		v2r_led[1].default_trigger = param_value;
	    }
	    #endif

	    if (!strcmp(param_name, "wifi")) {
		if (!strcmp(param_value, "on")) {
		    printk(KERN_INFO "Wi-Fi board enabled\n");
		    davinci_setup_mmc(1, &dm365evm_mmc_config);
		    /* maybe setup mmc1/etc ... _after_ mmc0 */
		    //dm365_wifi_configure();
		}
	    }

	    if (!strcmp(param_name, "1wire")) {
		int temp;
		kstrtoint(param_value, 10, &temp);
		w1_gpio_pdata.pin = temp;
		printk(KERN_INFO "Use 1-wire on GPIO%d\n", temp);
		w1_run = 1;
		
	    }

	    if (!strcmp(param_name, "1wirepullup")) {
		int temp;
		kstrtoint(param_value, 10, &temp);
		w1_gpio_pdata.ext_pullup_enable_pin = temp;
		printk(KERN_INFO "Use 1-wire pullup resistor on GPIO%d\n", temp);
	    }

	    if (!strcmp(param_name, "camera")) {
		if (!strcmp(param_value, "ov5642")) {
		    printk(KERN_INFO "Use camera OmniVision OV5642\n");
		}
		if (!strcmp(param_value, "ov7675")) {
		    printk(KERN_INFO "Use camera OmniVision OV7675\n");
		}
		if (!strcmp(param_value, "ov9710")) {
		    printk(KERN_INFO "Use camera OmniVision OV9710\n");
		}
	    }
	    
	}

    } while(p);

}

#endif

MACHINE_START(DM365_LEOPARD, "DM365 Leopard")
	.phys_io	= IO_PHYS,
	.io_pg_offst	= (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params	= (0x80000100),
	.map_io		= dm365_leopard_map_io,
	.init_irq	= dm365_leopard_irq_init,
	.timer		= &davinci_timer,
	.init_machine	= dm365_leopard_init,
MACHINE_END

