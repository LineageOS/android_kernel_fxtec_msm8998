/* Copyright (C) MicroArray
 * MicroArray Fprint Driver Code
 * mtk-settings.c
 * Date: 2016-11-17
 * Version: v4.0.03
 * Author: guq
 * Contact: guq@microarray.com.cn
 */

#include "qcom-settings.h"

struct mas_platform_data {
	int irq_gpio;
	int finger_en_gpio;
	int reset_gpio;
	//struct regulator *vcc_io_l6;    
	int vcc_en_gpio;
	int cs_gpio;
	int external_supply_mv;
	int txout_boost;
	int force_hwid;
//kingsun/zlc:
	int clk_enabled;
	struct clk *core_clk;
       struct clk *iface_clk;
	unsigned int int_irq;
//end
};

struct mas_platform_data *mas_pdata;
int first_int_after_suspend=0;               //kingsun/zlc:
#ifdef CONFIG_OF
/* -------------------------------------------------------------------- */
int mas_suspend(struct spi_device *spi, pm_message_t mesg)
{  
    printk("%s start\n",__func__);
    if (device_may_wakeup(&spi->dev))
	 enable_irq_wake(mas_pdata->int_irq);
	
    printk("mas_ioctl_clk_disable\n");
    mas_disable_spi_clock(spi);
    first_int_after_suspend=1;
    printk("%s end\n",__func__);
    return 0;
}


/* -------------------------------------------------------------------- */
int mas_resume(struct spi_device *spi)
{ 
   printk("%s start\n",__func__);
   if (device_may_wakeup(&spi->dev))	
	disable_irq_wake(mas_pdata->int_irq);

    printk("mas_enable_spi_clock\n");
    mas_enable_spi_clock(spi);
    printk("%s end\n",__func__);
    return 0;
}


static struct of_device_id mas_of_match[] = {
	{.compatible = "kingsun,fingerprint",},
	{}
};

MODULE_DEVICE_TABLE(of, mas_of_match);
#endif

struct spi_device_id sdev_id = {MA_DRV_NAME, 0};
struct spi_driver sdrv = {
        .driver = {
                .name = MA_DRV_NAME,
                .bus = &spi_bus_type,
                .owner = THIS_MODULE,
#ifdef CONFIG_OF
		 .of_match_table = mas_of_match,
#endif
        },
        .id_table = &sdev_id,
        .probe = mas_probe,
        .remove = mas_remove,
		//.suspend = mas_suspend,
       //.resume = mas_resume,
};
//driver end

/**
 *  the spi struct date start,for getting the spi_device to set the spi clock enable end
 */


void mas_select_transfer(struct spi_device *spi, int len) {
   return ;
}

/*
 *  set spi speed, often we must check whether the setting is efficient
 */

void ma_spi_change(struct spi_device *spi, unsigned int  speed, int flag)
{
    struct qcom_chip_conf *mcc = (struct qcom_chip_conf *)spi->controller_data;
    if(flag == 0) { 
        mcc->com_mod = 0;
    } else {
        mcc->com_mod = 1;
    }    
    mcc->high_time = speed;
    mcc->low_time = speed;
    if(spi_setup(spi) < 0){
        printk("change the spi error!\n");
    }    
}


static long spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
    long lowest_available, nearest_low, step_size, cur;
    long step_direction = -1;
    long guess = rate;
    int max_steps = 10;

  //  FUNC_ENTRY();
    cur = clk_round_rate(clk, rate);
    if (cur == rate)
        return rate;

    /* if we got here then: cur > rate */
    lowest_available = clk_round_rate(clk, 0);
    if (lowest_available > rate)
        return -EINVAL;

    step_size = (rate - lowest_available) >> 1;
    nearest_low = lowest_available;

    while (max_steps-- && step_size) {
        guess += step_size * step_direction;
        cur = clk_round_rate(clk, guess);

        if ((cur < rate) && (cur > nearest_low))
            nearest_low = cur;
        /*
         * if we stepped too far, then start stepping in the other
         * direction with half the step size
         */
        if (((cur > rate) && (step_direction > 0))
                || ((cur < rate) && (step_direction < 0))) {
            step_direction = -step_direction;
            step_size >>= 1;
        }
    }
    return nearest_low;
}

static void spi_clock_set(struct mas_platform_data *data, int speed)
{
    long rate;
    int rc;
  //  FUNC_ENTRY();
    return ;     //kingsun/zlc: bypass the operation of spi clock
    rate = spi_clk_max_rate(data->core_clk, speed);
    if (rate < 0) {
        pr_info("%s: no match found for requested clock frequency:%d",
                __func__, speed);
        return;
    }

    rc = clk_set_rate(data->core_clk, rate);
}


static int mas_ioctl_clk_init(struct spi_device *spi, struct mas_platform_data *data)
{    return 0;     //kingsun/zlc: bypass the operation of spi clock
    pr_info("%s: enter\n", __func__);

    data->clk_enabled = 0;
    data->core_clk = clk_get(&spi->dev, "core_clk");
    if (IS_ERR_OR_NULL(data->core_clk)) {
        pr_err("%s: fail to get core_clk\n", __func__);
        return -1;
    }
    data->iface_clk = clk_get(&spi->dev, "iface_clk");
    if (IS_ERR_OR_NULL(data->iface_clk)) {
        pr_err("%s: fail to get iface_clk\n", __func__);
        clk_put(data->core_clk);
        data->core_clk = NULL;
        return -2;
    }
    return 0;
}

static int mas_ioctl_clk_uninit(struct mas_platform_data *data)
{    return 0;     //kingsun/zlc: bypass the operation of spi clock
    pr_info("%s: enter\n", __func__);

    if (!IS_ERR_OR_NULL(data->core_clk)) {
        clk_put(data->core_clk);
        data->core_clk = NULL;
    }

    if (!IS_ERR_OR_NULL(data->iface_clk)) {
        clk_put(data->iface_clk);
        data->iface_clk = NULL;
    }

    return 0;
}

static int mas_ioctl_clk_enable(struct mas_platform_data *data)
{
    int err;
    return 0;     //kingsun/zlc: bypass the operation of spi clock
    pr_debug("%s: enter\n", __func__);

    if (data->clk_enabled)
        return 0;

    err = clk_prepare_enable(data->core_clk);
    if (err) {
        pr_err("%s: fail to enable core_clk\n", __func__);
        return -1;
    }

    err = clk_prepare_enable(data->iface_clk);
    if (err) {
        pr_err("%s: fail to enable iface_clk\n", __func__);
        clk_disable_unprepare(data->core_clk);
        return -2;
    }

    data->clk_enabled = 1;

    return 0;
}

static int mas_ioctl_clk_disable(struct mas_platform_data *data)
{    return 0;     //kingsun/zlc: bypass the operation of spi clock
    if (!data->clk_enabled)
        return 0;

    clk_disable_unprepare(data->core_clk);
    clk_disable_unprepare(data->iface_clk);
    data->clk_enabled = 0;

    return 0;
}


int mas_get_platform(void) {
   int ret;
	ret = spi_register_driver(&sdrv);
	if(ret) {
		printk("spi_register_driver");
	}
	return ret;
}

int mas_remove_platform(void){
	spi_unregister_driver(&sdrv);
	return 0;
}

static int mas_get_of_pdata(struct device *dev)
{
     struct device_node *node = dev->of_node;
	mas_pdata->irq_gpio = of_get_named_gpio(node, "ks,irq-gpio", 0);
	pr_info("%s:irq-gpio = %d\n", __func__, mas_pdata->irq_gpio);
	if (mas_pdata->irq_gpio < 0) {
		pr_err("irq gpio is missing\n");
		goto of_err;
	}
    
      mas_pdata->finger_en_gpio = of_get_named_gpio(node, "ks,power-gpio", 0);
	pr_info("%s:power-gpio = %d\n", __func__,mas_pdata->finger_en_gpio);
	if (mas_pdata->finger_en_gpio < 0) {
		pr_err("finger_en gpio is missing\n");
		goto of_err;
	}
	  /*
      mas_pdata->vcc_io_l6 = regulator_get(dev, "vcc_io");
        if (IS_ERR(mas_pdata->vcc_io_l6)){ 
	    	pr_info("%s:Failed to get the regulator for vcc_io. \n", __func__);
		goto of_err;
        }
	 else
		pr_info("%s:Get the regulator for vcc_io sucessfully.\n", __func__);
	*/
     dev_info(dev, "end parse_dt\n");
     return 0;

of_err:
	return -ENODEV;
}

static int mas_gpio_configure(bool on)
{
	int err = 0;
	if (on) {
		if (gpio_is_valid(mas_pdata->irq_gpio)) {
			err = gpio_request(mas_pdata->irq_gpio,
						"mas_irq_gpio");
			if (err) {
				pr_info("%s:irq gpio request failed\n",__func__);
				goto err_irq_gpio_req;
			}
			err = gpio_direction_input(mas_pdata->irq_gpio);
			if (err) {
				pr_info("%s:set_direction for irq gpio failed\n",__func__);
				goto err_irq_gpio_dir;
			}
		}

		if (gpio_is_valid(mas_pdata->finger_en_gpio)) {
			err = gpio_request(mas_pdata->finger_en_gpio,
						"mas_en_gpio");
			if (err) {
				pr_info("%s:en gpio request failed\n",__func__);
				goto err_irq_gpio_dir;
			}

			err = gpio_direction_output(mas_pdata->finger_en_gpio, 1);
			if (err) {
				pr_info("%s:set_direction for en gpio failed\n",__func__);
				goto err_en_gpio_dir;
			}
		}
		printk("end mas_gpio_configure\n");
		return 0;
	} else {
		if (gpio_is_valid(mas_pdata->irq_gpio))
			gpio_free(mas_pdata->irq_gpio);
		if (gpio_is_valid(mas_pdata->finger_en_gpio)) {
			gpio_free(mas_pdata->finger_en_gpio);
		}
		return 0;
	}

err_en_gpio_dir:
	if (gpio_is_valid(mas_pdata->finger_en_gpio))
		gpio_free(mas_pdata->finger_en_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(mas_pdata->irq_gpio))
		gpio_free(mas_pdata->irq_gpio);
err_irq_gpio_req:
	return err;
}


 int mas_fingerprint_power(bool flags)
 {
    int rc=0;
    //int vcc_io;

 	if (flags) {
	    if (gpio_is_valid(mas_pdata->finger_en_gpio)) {
                gpio_set_value(mas_pdata->finger_en_gpio, 1);
           }
           msleep(10);
		/*
	    rc=regulator_enable(mas_pdata->vcc_io_l6);
	    if (rc < 0){
		 pr_err("%s: regulator_enable failed\n", __func__);
		 goto enable_err;
	    }
	   else
		pr_info("%s: regulator_enable is successful.\n", __func__);
		
	    vcc_io=regulator_get_voltage(mas_pdata->vcc_io_l6);
	    pr_info("%s:regulator_get_voltage =%d\n", __func__,vcc_io);
		*/
	    pr_info("---- power on ok ----\n");
 	} else {
	    if (gpio_is_valid(mas_pdata->finger_en_gpio)) {
                gpio_set_value(mas_pdata->finger_en_gpio, 0);
              }
           msleep(10);
		/*
	    rc=regulator_disable(mas_pdata->vcc_io_l6);
	    if (rc < 0){
		  pr_err("%s:regulator_disable failed\n", __func__);
		  goto enable_err;
	    }
	   else
		  pr_info("%s:regulator_disable is successful.\n", __func__);
	    */
	    pr_info("---- power off ok ----\n");
	}
//enable_err:
	return rc;
}


int mas_qcm_platform_uninit(struct spi_device *spi)
{ int ret=0;
    mas_ioctl_clk_uninit(mas_pdata);
    mas_fingerprint_power(false);
   //if(mas_pdata->vcc_io_l6)
   //  regulator_put(mas_pdata->vcc_io_l6);
   mas_gpio_configure(false);
   mas_pdata->int_irq=0;
   kfree(mas_pdata);
   return ret;
}


int mas_qcm_platform_init(struct spi_device *spi)
{ int ret=0;
    mas_pdata = kmalloc(sizeof(struct mas_platform_data), GFP_KERNEL);
    if (mas_pdata == NULL) {
	pr_info("%s:Failed to allocate buffer\n", __func__);
	ret=-ENOMEM;
	goto err_devm_kzalloc;
    } 
   ret = mas_get_of_pdata(&spi->dev);
   if(ret<0)
   	goto get_of_pdata_err;
   
   ret = mas_gpio_configure(true);
   if(ret<0)
   	goto fingeriprnt_gpio_configure_err;
   
   ret = mas_fingerprint_power(true);
   if(ret<0)
   	goto fingeriprnt_power_err;

   if (mas_ioctl_clk_init(spi, mas_pdata))
       goto fingeriprnt_power_err;

   if (mas_ioctl_clk_enable(mas_pdata))
       goto fingeriprnt_clk_enable_failed;
    spi_clock_set(mas_pdata, 9600000);
   return ret;
fingeriprnt_clk_enable_failed:
   mas_ioctl_clk_uninit(mas_pdata);
fingeriprnt_power_err:
   //if(mas_pdata->vcc_io_l6)
      //regulator_put(mas_pdata->vcc_io_l6);	
fingeriprnt_gpio_configure_err:
get_of_pdata_err:
   kfree(mas_pdata);
err_devm_kzalloc:
   return ret;
}

void mas_enable_spi_clock(struct spi_device *spi)
{
    mas_ioctl_clk_enable(mas_pdata);
}

void mas_disable_spi_clock(struct spi_device *spi)
{
    mas_ioctl_clk_disable(mas_pdata);
}


unsigned int mas_get_irq(void){
  unsigned int irq=0;
  irq =  gpio_to_irq(mas_pdata->irq_gpio);
  if(irq)
      printk("mas_get_irq: %d\n",irq);
  mas_pdata->int_irq=irq;
  return irq;
}


void mas_set_wakeup(struct spi_device *spi)
{
    device_init_wakeup(&spi->dev, 1);
}

/*
 * this function used for check the interrupt gpio state
 * @index 0 gpio level 1 gpio mode, often use 0
 * @return 0 gpio low 1 gpio high if index = 1,the return is the gpio mode
 *  		under 0  the of_property_read_u32_index return errno,check the dts as below:
 * last but not least use this function must checkt the label on dts file, after is an example:
 * ma_finger: ma_finger{
 *		compatible = "mediatek,afs120x";
 *		finger_int_pin = <100 0>;
 * }
 */
int mas_get_interrupt_gpio(unsigned int index){
	 return gpio_get_value(mas_pdata->irq_gpio);
}
