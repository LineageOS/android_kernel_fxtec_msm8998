/* Copyright (C) MicroArray
 * MicroArray Fprint Driver Code
 * mtk-settings.h
 * Date: 2016-11-17
 * Version: v4.0.03
 * Author: guq
 * Contact: guq@microarray.com.cn
 */

#ifndef __QCOM_SETTINGS_H_
#define __QCOM_SETTINGS_H_



#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
//#include "madev.h"
#include <linux/spi/spi.h>
#include <linux/wakelock.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

//macro settings
#define MA_DRV_NAME             "madev"

#define MA_DTS_NAME            "mediatek,hct_finger"

#define MA_EINT_DTS_NAME        "mediatek,hct_finger"
//macro settings end

extern int mas_probe(struct spi_device *spi);
extern int mas_remove(struct spi_device *spi);

/* add for spi cls ctl start */
struct mt_spi_t {
        struct platform_device *pdev;
        void __iomem *regs;
        int irq;
        int running;
        struct wake_lock wk_lock;
        struct mt_chip_conf *config;
        struct spi_master *master;

        struct spi_transfer *cur_transfer;
        struct spi_transfer *next_transfer;

        spinlock_t lock;
        struct list_head queue;
#if !defined(CONFIG_MTK_CLKMGR)
        struct clk *clk_main;
#endif
};

//kingsun/zlc: This struct is porting from mtk, It's no use actually at qcom platform
struct qcom_chip_conf {
        int setuptime;
        int holdtime;
	 int high_time;
	 int low_time;
	 int cs_idletime;
	 int ulthgh_thrsh;
	 int cpol;
	 int cpha;
	 int rx_mlsb;
	 int tx_mlsb;
	 int tx_endian;
	 int rx_endian;
	 int com_mod;
	 int pause;
	 int finish_intr;
	 int deassert;
	 int ulthigh;
	 int tckdly;
};

/* add for spi cls ctl end this func only used in tee enviroment*/
//packaging
//void mas_enable_spi_clock(struct spi_device *spi);
//void mas_diasble_spi_clock(struct spi_device *spi);
//packaging end

//the interface called by madev
void mas_select_transfer(struct spi_device *spi, int len);
int mas_finger_get_gpio_info(struct platform_device *pdev);
int mas_finger_set_gpio_info(int cmd);
void mas_enable_spi_clock(struct spi_device *spi);
void mas_disable_spi_clock(struct spi_device *spi);
unsigned int mas_get_irq(void);
int mas_get_platform(void);
int mas_remove_platform(void);
int mas_power(int cmd);
int get_screen(void);
void ma_spi_change(struct spi_device *spi, unsigned int speed, int flag);
#endif
