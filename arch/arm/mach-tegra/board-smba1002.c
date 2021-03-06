/*
 * arch/arm/mach-tegra/board-smba1002.c
 *
 * Copyright (C) 2011 Eduardo Jos� Tagle <ejtagle@tutopia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_devices.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/pda_power.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/i2c-tegra.h>
#include <linux/i2c.h>
#include <linux/memblock.h>
#include <linux/rfkill-gpio.h>

#include <sound/alc5623.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/io.h>
#include <mach/w1.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/nand.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/gpio.h>
#include <mach/spdif.h>
#include <mach/audio.h>  
#include <mach/clk.h>
#include <mach/usb_phy.h>
#include <mach/i2s.h>
#include <mach/system.h>
#include <linux/nvmap.h>

#include <mach/tegra_alc5623_pdata.h>
#include <linux/i2c/at168_ts.h>

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/f_accessory.h>

#include "board.h"
#include "board-smba1002.h"
#include "clock.h"
#include "gpio-names.h"
#include "devices.h"
#include "pm.h"
#include "wakeups-t2.h"
#include "wdt-recovery.h"

static struct rfkill_gpio_platform_data smba1002_bt_rfkill_pdata = {
		.name           = "bluetooth_rfkill",
		.reset_gpio     = SMBA1002_BT_RESET,
		.shutdown_gpio  = -1,
		.power_clk_name = "bcm4329_32k_clk",
		.type           = RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device smba1002_bt_rfkill_device = {
	.name = "rfkill_gpio",
	.id   = -1,
	.dev  = {
	        .platform_data = &smba1002_bt_rfkill_pdata,
	 },
};

void __init smba_bt_rfkill(void)
{
	/*Add Clock Resource*/
	clk_add_alias("bcm4329_32k_clk", smba1002_bt_rfkill_device.name, \
    				"blink", NULL);
	
	return;
}

static struct resource smba_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = SMBA1002_BT_IRQ,
			.end    = SMBA1002_BT_IRQ,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(SMBA1002_BT_IRQ),
			.end    = TEGRA_GPIO_TO_IRQ(SMBA1002_BT_IRQ),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device smba_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(smba_bluesleep_resources),
	.resource       = smba_bluesleep_resources,
};

void __init smba_setup_bluesleep(void)
{
	platform_device_register(&smba_bluesleep_device);
	tegra_gpio_enable(SMBA1002_BT_IRQ);
	tegra_gpio_enable(SMBA1002_BT_RESET);
	return;
}

static __initdata struct tegra_clk_init_table smba_clk_init_table[] = {
	/* name			parent		 rate		enabled */
	{ "cdev1",		NULL,		     0, true },
	{ "i2s1",		"pll_a_out0", 	     0,	false},
	{ "i2s2",		"pll_a_out0", 	     0,	false},
	{ "spdif_out",	"pll_a_out0",  	     0,	false},
	{ "sdmmc1",		"clk_m",	  48000000,	true },
	{ "sdmmc2",		"clk_m",	  48000000,	true },
	{ "sdmmc4",		"clk_m",	  48000000,	true },
	{ "ndflash",	"pll_p",	 108000000,	true },
	{ "pwm",		"clk_32k",		 32768,	false},	
	{ "usbd",		"clk_m",	  12000000,	true },		/* fsl-tegra-udc , utmip-pad , tegra_ehci.0 , tegra_otg - we need this to be always on to always get hotplug events */
	{ "usb2",		"clk_m",	  12000000,	false},		/* tegra_ehci.1 - Really unused*/
	{ "usb3",		"clk_m",	  12000000,	true },		/* tegra_ehci.2 - we need this to be always on to always get hotplug events */
	{ "i2c1",		"clk_m",	    800000,	false},		/* tegra-i2c.0 */
	{ "i2c2",		"clk_m",	    315789,	false},		/* tegra-i2c.1 */
	{ "i2c3",		"clk_m",	    800000,	false},		/* tegra-i2c.2 */
	{ "dvc",		"clk_m",	   2400000,	false},		/* tegra-i2c.3 */
	//{ "pll_p_out4",	"pll_p",	24000000,	true },
	{ "blink",	"clk_32k",	32768,		false},
	
	{ NULL,			NULL,		         0,		0},
}; 

static struct tegra_i2c_platform_data smba_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data smba_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 100000, 100000 },
	//FIX ME
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
};

static struct tegra_i2c_platform_data smba_i2c3_platform_data = {
	.adapter_nr     = 3,
	.bus_count      = 1,
	.bus_clk_rate   = { 100000, 0 },
};

static struct tegra_i2c_platform_data smba_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_dvc		= true,
};

int __init smba_i2c_register_devices(void)
{
	
	tegra_i2c_device1.dev.platform_data = &smba_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &smba_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &smba_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &smba_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);
	
	return 0;
}

static struct tegra_audio_platform_data tegra_spdif_pdata = {
	.dma_on			= true,  /* use dma by default */
	.mask			= TEGRA_AUDIO_ENABLE_TX | TEGRA_AUDIO_ENABLE_RX,
	.stereo_capture = true,
};

static struct tegra_audio_platform_data tegra_audio_pdata[] = {
	/* For I2S1 - Hifi */
	[0] = {
#ifdef ALC5623_IS_MASTER
		.i2s_master		= false,	/* CODEC is master for audio */
		.dma_on			= true,  	/* use dma by default */
		.i2s_clk_rate 	= 2822400,
		.dap_clk	  	= "cdev1",
		.audio_sync_clk = "audio_2x",
		.mode			= I2S_BIT_FORMAT_I2S,
		.fifo_fmt		= I2S_FIFO_16_LSB,
		.bit_size		= I2S_BIT_SIZE_16,
#else
		.i2s_master		= true,		/* CODEC is slave for audio */
		.dma_on			= true,  	/* use dma by default */
#ifdef SMBA1002_48KHZ_AUDIO						
		.i2s_master_clk = 48000,
		.i2s_clk_rate 	= 12288000,
#else
		.i2s_master_clk = 44100,
		.i2s_clk_rate 	= 11289600,
#endif
		.dap_clk	  	= "cdev1",
		.audio_sync_clk = "audio_2x",
		.mode			= I2S_BIT_FORMAT_I2S,
		.fifo_fmt		= I2S_FIFO_PACKED,
		.bit_size		= I2S_BIT_SIZE_16,
		.i2s_bus_width	= 32,
#endif
		.mask			= TEGRA_AUDIO_ENABLE_TX | TEGRA_AUDIO_ENABLE_RX,
		.stereo_capture = true,
	},
	/* For I2S2 - Bluetooth */
	[1] = {
		.i2s_master		= false,	/* bluetooth is master always */
		.dma_on			= true,  /* use dma by default */
		.i2s_master_clk = 8000,
		.dsp_master_clk = 8000,
		.i2s_clk_rate	= 2000000,
		.dap_clk		= "cdev1",
		.audio_sync_clk = "audio_2x",
		.mode			= I2S_BIT_FORMAT_DSP,
		.fifo_fmt		= I2S_FIFO_16_LSB,
		.bit_size		= I2S_BIT_SIZE_16,
		.i2s_bus_width 	= 32,
		.dsp_bus_width 	= 16,
		.mask			= TEGRA_AUDIO_ENABLE_TX | TEGRA_AUDIO_ENABLE_RX,
		.stereo_capture = true,
	}
}; 

static struct alc5623_platform_data smba_alc5623_pdata = {
	.add_ctrl	= 0xD300,
	.jack_det_ctrl	= 0,
	.avdd_mv		= 3300,	/* Analog vdd in millivolts */

	.mic1bias_mv		= 2475,	/* MIC1 bias voltage */
	.mic1boost_db    	= 20,  /* MIC1 gain boost */
	.mic2boost_db   	= 20,  /* MIC2 gain boost */

	.default_is_mic2 	= false,	/* SMBA1002 uses MIC1 as the default capture source */

};

static struct i2c_board_info __initdata smba_i2c_bus0_board_info[] = {
	{
		I2C_BOARD_INFO("alc5623", 0x1a),
		.platform_data = &smba_alc5623_pdata,
	},
};


static struct tegra_alc5623_platform_data smba_audio_pdata = {
        .gpio_spkr_en           = -2,
        .gpio_hp_det            = SMBA1002_HP_DETECT,
	.gpio_int_mic_en 	= SMBA1002_INT_MIC_EN,
	.hifi_codec_datafmt = SND_SOC_DAIFMT_I2S,	/* HiFi codec data format */
#ifdef ALC5623_IS_MASTER
	.hifi_codec_master  = true,					/* If Hifi codec is master */
#else
	.hifi_codec_master  = false,				/* If Hifi codec is master */
#endif
	.bt_codec_datafmt   = SND_SOC_DAIFMT_DSP_A,	/* Bluetooth codec data format */
	.bt_codec_master    = true,					/* If bt codec is master */

};

static struct platform_device tegra_generic_codec = {
	.name = "tegra-generic-codec",
	.id   = -1,
};

static struct platform_device smba_audio_device = {
	.name = "tegra-snd-alc5623",
	.id   = 0,
        .dev    = {
                .platform_data  = &smba_audio_pdata,
        },

};

static struct platform_device *smba_i2s_devices[] __initdata = {
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_spdif_device,
	&tegra_das_device,
	&spdif_dit_device,
	&tegra_pcm_device,
	&tegra_generic_codec,
	&smba_audio_device, /* this must come last, as we need the DAS to be initialized to access the codec registers ! */
};


int  __init smba_audio_register_devices(void)
{
	int ret;

        pr_info("%s++ \n", __func__);

	/* Patch in the platform data */
	tegra_i2s_device1.dev.platform_data = &tegra_audio_pdata[0];
	tegra_i2s_device2.dev.platform_data = &tegra_audio_pdata[1];
	tegra_spdif_device.dev.platform_data = &tegra_spdif_pdata;

        ret = i2c_register_board_info(0, smba_i2c_bus0_board_info,
                ARRAY_SIZE(smba_i2c_bus0_board_info));
        if (ret)
                return ret;

        return platform_add_devices(smba_i2s_devices, ARRAY_SIZE(smba_i2s_devices));
}




struct at168_i2c_ts_platform_data at168_pdata = {
	.gpio_reset = SMBA1002_TS_RESET,
	.gpio_power = SMBA1002_TS_POWER,
};

static struct i2c_board_info __initdata smba_i2c_bus0_touch_info_at168[] = {
	{
		I2C_BOARD_INFO("at168_touch", 0x5c),
		.irq = TEGRA_GPIO_TO_IRQ(SMBA1002_TS_IRQ),
		.platform_data = &at168_pdata,
	},
};


int __init smba_touch_register_devices(void)
{
	tegra_gpio_enable(SMBA1002_TS_IRQ);
	gpio_request(SMBA1002_TS_IRQ, "at168_touch");
	gpio_direction_input(SMBA1002_TS_IRQ);
	
	i2c_register_board_info(0, smba_i2c_bus0_touch_info_at168, 1);

	return 0;
}




#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resources[] = {
	{
		.flags = IORESOURCE_MEM,
	},
 };
 
 static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resources),
	.resource       = ram_console_resources,
};

static void __init tegra_ramconsole_reserve(unsigned long size)
{
	struct resource *res;
	long ret;

	res = platform_get_resource(&ram_console_device, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("Failed to find memory resource for ram console\n");
		return;
	}
	res->start = memblock_end_of_DRAM() - size;
	res->end = res->start + size - 1;
	ret = memblock_remove(res->start, size);
	if (ret) {
		ram_console_device.resource = NULL;
		ram_console_device.num_resources = 0;
		pr_err("Failed to reserve memory block for ram console\n");
	}
}
#endif

#ifdef SMBA1002_GPS
/*Fosser2's GPS MOD*/
static atomic_t smba_gps_mag_powered = ATOMIC_INIT(0);
void smba_gps_mag_poweron(void)
{
	if (atomic_inc_return(&smba_gps_mag_powered) == 1) {
		pr_info("Enabling GPS/Magnetic module\n");
		/* 3G/GPS power on sequence */
		gpio_set_value(SMBA1002_GPSMAG_DISABLE, 1); /* Enable power */
		msleep(2);
	}
}
EXPORT_SYMBOL_GPL(smba_gps_mag_poweron);

void smba_gps_mag_poweroff(void)
{
	if (atomic_dec_return(&smba_gps_mag_powered) == 0) {
		pr_info("Disabling GPS/Magnetic module\n");
		/* 3G/GPS power on sequence */
		gpio_set_value(SMBA1002_GPSMAG_DISABLE, 0); /* Disable power */
		msleep(2);
	}
}
EXPORT_SYMBOL_GPL(smba_gps_mag_poweroff);

static atomic_t smba_gps_mag_inited = ATOMIC_INIT(0);
void smba_gps_mag_init(void)
{
	if (atomic_inc_return(&smba_gps_mag_inited) == 1) {
		gpio_request(SMBA1002_GPSMAG_DISABLE, "gps_disable");
		gpio_direction_output(SMBA1002_GPSMAG_DISABLE, 0);
	}
}
EXPORT_SYMBOL_GPL(smba_gps_mag_init);

void smba_gps_mag_deinit(void)
{
	atomic_dec(&smba_gps_mag_inited);
}
EXPORT_SYMBOL_GPL(smba_gps_mag_deinit);

#endif

static struct platform_device *smba_devices[] __initdata = {
    &tegra_pmu_device,
    &smba1002_bt_rfkill_device,
    &smba_bluesleep_device,
    &tegra_spi_device1,
	&tegra_spi_device2,
	&tegra_spi_device3,
	//&tegra_spi_device4,
	&tegra_aes_device,
	&tegra_wdt_device,
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
        .port_otg = true,
        .has_hostpc = false,
        .phy_intf = TEGRA_USB_PHY_INTF_UTMI,
        .op_mode = TEGRA_USB_OPMODE_DEVICE,
        .u_data.dev = {
                .vbus_pmu_irq = 0,
                .vbus_gpio = -1,
                .charging_supported = false,
                .remote_wakeup_supported = false,
        },
        .u_cfg.utmi = {
                .hssync_start_delay = 0,
                .elastic_limit = 16,
                .idle_wait_delay = 17,
                .term_range_adj = 6,
                .xcvr_setup = 9,
                .xcvr_lsfslew = 2,
                .xcvr_lsrslew = 2,
                .xcvr_setup_offset = 0,
                .xcvr_use_fuses = 1,
        },
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
        .port_otg = true,
        .has_hostpc = false,
        .phy_intf = TEGRA_USB_PHY_INTF_UTMI,
        .op_mode = TEGRA_USB_OPMODE_HOST,
        .u_data.host = {
                .vbus_gpio = SMBA1002_USB0_VBUS,
                .vbus_reg = NULL,
                .hot_plug = true,
                .remote_wakeup_supported = false,
                .power_off_on_suspend = false,
        },
        .u_cfg.utmi = {
                .hssync_start_delay = 9,
                .elastic_limit = 16,
                .idle_wait_delay = 17,
                .term_range_adj = 6,
                .xcvr_setup = 9,
                .xcvr_lsfslew = 2,
                .xcvr_lsrslew = 2,
        },
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
        .port_otg = false,
        .has_hostpc = false,
        .phy_intf = TEGRA_USB_PHY_INTF_UTMI,
        .op_mode        = TEGRA_USB_OPMODE_HOST,
        .u_data.host = {
                .vbus_gpio = -1,
                .vbus_reg = NULL,
                .hot_plug = true,
                .remote_wakeup_supported = false,
                .power_off_on_suspend = true,
        },
        .u_cfg.utmi = {
                .hssync_start_delay = 9,
                .elastic_limit = 16,
                .idle_wait_delay = 17,
                .term_range_adj = 6,
                .xcvr_setup = 9,
                .xcvr_lsfslew = 2,
                .xcvr_lsrslew = 2,
        },
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
}; 

static void smba_usb_init(void)
	{
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	platform_device_register(&tegra_udc_device);

	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);
};

int __init smba_usb_register_devices(void)
{
	smba_usb_init();
	return 0;
}

static void __init tegra_smba_init(void)
{
	struct clk *clk;

	/* force consoles to stay enabled across suspend/resume */
	// console_suspend_enabled = 0;

	/* Init the suspend information */
	//tegra_init_suspend(&smba_suspend);


	/* Set the SDMMC1 (wifi) tap delay to 6.  This value is determined
	 * based on propagation delay on the PCB traces. */
	clk = clk_get_sys("sdhci-tegra.0", NULL);
	if (!IS_ERR(clk)) {
		tegra_sdmmc_tap_delay(clk, 6);
		clk_put(clk);
	} else {
		pr_err("Failed to set wifi sdmmc tap delay\n");
	}

	/* Initialize the pinmux */
	smba_pinmux_init();

	/* Initialize the clocks - clocks require the pinmux to be initialized first */
    tegra_clk_init_from_table(smba_clk_init_table);

	platform_add_devices(smba_devices,ARRAY_SIZE(smba_devices));
	/* Register i2c devices - required for Power management and MUST be done before the power register */
	smba_i2c_register_devices();

	/* Register the power subsystem - Including the poweroff handler - Required by all the others */
	smba_charge_init();
	smba_regulator_init();
        smba_charger_init();

	/* Register the USB device */
	smba_usb_register_devices();

	/* Register UART devices */
	smba_uart_register_devices();

	/* Register GPU devices */
	smba_panel_init();

	/* Register Audio devices */
	smba_audio_register_devices();

	/* Register Jack devices */
	//smba_jack_register_devices();

	/* Register all the keyboard devices */
	smba_keys_init();

	/* Register touchscreen devices */
	smba_touch_register_devices();

	/* Register accelerometer device */
	smba_sensors_register_devices();
	
	/* Register Bluetooth powermanagement devices */
	smba_bt_rfkill();
	smba_setup_bluesleep();

	/* Register Camera powermanagement devices */
	smba_camera_register_devices();

	/* Register NAND flash devices */
	smba_nand_register_devices();
	
	/* Register SDHCI devices */
	smba1002_sdhci_init();	
	
#ifdef SMBA1002_GPS
	/* Register gps powermanagement devices */
	smba_gps_pm_register_devices();
#endif	
	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
#if 0
	/* Finally, init the external memory controller and memory frequency scaling
   	   NB: This is not working on SMBA1002. And seems there is no point in fixing it,
	   as the EMC clock is forced to the maximum speed as soon as the 2D/3D engine
	   starts.*/
	smba_init_emc();
#endif

#ifdef _DUMP_WBCODE
	dump_warmboot(tegra_lp0_vec_start,tegra_lp0_vec_size);
#endif

#ifdef _DUMP_BOOTCAUSE
	dump_bootflags();
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	/* Register the RAM console device */
	platform_device_register(&ram_console_device);
#endif

	/* Release the tegra bootloader framebuffer */
	tegra_release_bootloader_fb();
}

static void __init tegra_smba_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	/* Reserve the graphics memory */		
#if defined(DYNAMIC_GPU_MEM)
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)	
	tegra_reserve(0, SMBA1002_FB1_MEM_SIZE, SMBA1002_FB2_MEM_SIZE);
#else
	tegra_reserve(SMBA1002_GPU_MEM_SIZE, SMBA1002_FB1_MEM_SIZE, SMBA1002_FB2_MEM_SIZE);
#endif
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	/* Reserve 1M memory for the RAM console */
	tegra_ramconsole_reserve(SZ_1M);
#endif
}

static void __init tegra_smba_fixup(struct machine_desc *desc,
	struct tag *tags, char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = SMBA1002_MEM_BANKS;
	mi->bank[0].start = PHYS_OFFSET;
#if defined(DYNAMIC_GPU_MEM)
	mi->bank[0].size  = SMBA1002_MEM_SIZE;
#else
	mi->bank[0].size  = SMBA1002_MEM_SIZE - SMBA1002_GPU_MEM_SIZE;
#endif
} 

/* the Shuttle bootloader identifies itself as MACH_TYPE_HARMONY [=2731]
   or as MACH_TYPE_LEGACY[=3333]. We MUST handle both cases in order
   to make the kernel bootable */
MACHINE_START(HARMONY, "harmony")
	.boot_params	= 0x00000100,
	.map_io         = tegra_map_common_io,
	.init_early     = tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine	= tegra_smba_init,
	.reserve		= tegra_smba_reserve,
	.fixup			= tegra_smba_fixup,
MACHINE_END

#ifdef MACH_TYPE_TEGRA_LEGACY
MACHINE_START(TEGRA_LEGACY, "tegra_legacy")
#else
MACHINE_START(LEGACY, "legacy")
#endif
	.boot_params	= 0x00000100,
	.map_io         = tegra_map_common_io,
	.init_early     = tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer, 	
	.init_machine	= tegra_smba_init,
	.reserve		= tegra_smba_reserve,
	.fixup			= tegra_smba_fixup,
MACHINE_END
