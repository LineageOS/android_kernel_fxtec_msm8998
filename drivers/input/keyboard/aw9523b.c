#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
//#include <linux/sensors.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif
#include <linux/device.h>


#include "aw9523b.h"
#define AWINIC_NAME                 "aw9523b"
struct aw9523b_platform_data {

};

struct aw9523b_data {
    struct i2c_client *client;    
    spinlock_t irq_lock;
    bool irq_is_disabled; 
#ifdef USEVIO	
    struct regulator *vio;
	bool power_enabled;
#endif
    int gpio_rst;
    int gpio_caps_led;
    int gpio_irq; 
    struct work_struct  work;
};

#define AW9523B_VIO_MIN_UV       1800000
#define AW9523B_VIO_MAX_UV       1800000


#define PLATFORM_QUALCOMM
#ifdef PLATFORM_QUALCOMM


#endif
typedef enum {
    P0_0,
    P0_1, 
    P0_2, 
    P0_3, 
    P0_4, 
    P0_5, 
    P0_6, 
    P0_7    
} P0_Enum;
 
typedef enum {
    P1_0, 
    P1_1, 
    P1_2, 
    P1_3, 
    P1_4,   
    P1_5, 
    P1_6, 
    P1_7    
} P1_Enum;
 
typedef enum {
    KEY_STATE_PRESSED,
    KEY_STATE_RELEASED,
    KEY_STATE_LONGPRESS,
    KEY_STATE_REPEATED, 
    KEY_STATE_NULL 
}TOUCHKEY_STATE;

/* 当有一部分P0\P1口用做扩展GPIO来控制子系统时,为了使软件控制方便,最好遵循一个原则:
 * 即当大部分P0口用做键盘扫描的输入时,那么其他的P0口作为输出来控制子系统;
 *   当大部分P1口用做键盘扫描的输出时,那么其他的P1口作为输入来控制子系统.
*/
#if 0
//下面这个数组定义了哪些P0口是用做键盘扫描的,1为使用,0为不使用.在键盘扫描时,不用做键盘扫描的P0口,其读出的值会被屏蔽
static unsigned char p0_kbd_used[8] = {
    1,/*P0_0*/
    1,/*P0_1*/
    1,/*P0_2*/
    1,/*P0_3*/
    1,/*P0_4*/
    1,/*P0_5*/
    1,/*P0_6*/
    1,/*P0_7*/
};

//下面这个数组定义了哪些P1口是用做键盘扫描的,1为使用,0为不使用.
static unsigned char p1_kbd_used[8] = {   
    1,/*P1_0*/
    1,/*P1_1*/
    1,/*P1_2*/
    1,/*P1_3*/
    1,/*P1_4*/
    1,/*P1_5*/
    1,/*P1_6*/
    1,/*P1_7*/ 
};
 #endif
//下面这个数组定义列是由哪几个P0口（或P1口）从左到右顺序组成的，根据键盘布局来定义
static const P0_Enum col[X_NUM] = {
    P0_0,
    P0_1,
    P0_2,
    P0_3,
    P0_4,
    P0_5,
    P0_6,
    P0_7
};

//下面这个数组定义行是由哪几个P1口（或P0口）从上到下顺序组成的，根据键盘布局来定义 
static const P1_Enum row[Y_NUM] = {
    P1_0,
    P1_1,
    P1_2,
    P1_3,
    P1_4,
    P1_5,
    P1_6,
    P1_7
    
};
 

static u8 p0_kbd_used_mask = 0x0;
static u8 p1_kbd_used_mask = 0x0;

//以下P0_X,P1_Y这两个数组的值会通过aw9523b_setting.h配置文件里的参数动态算出来
static const u8 p0_port_be_press_value[2][X_NUM] = {
													//{0x7e,0x3f,0x5f,0x6f,0x77,0x7d,0x7b,0x7f},
													{0xfe,0xbf,0xdf,0xef,0xf7,0xfd,0xfb,0x7f},
													{0xfa,0xbb,0xdb,0xeb,0xf3,0xf9,0xf7,0x7b},
													}; /*根据硬件设计， 每一列按下去P0口的值，通过这个值，可以判断是哪列被按下*/
static u8 p1_port_output_value[Y_NUM] ; /*根据硬件设计， P1行值的设置，通过这个值去扫描判断哪一行被按下*/  
//本例中计算出的P0_X[0:3]={0xfe,0xfd,0xfb,0xf7}
//本例中计算出的P1_Y[0:5]={0x3e,0x3d,0x3b,0x37,0x2f,0x1f}
 
 
 
static u8  aw9523b_chip_id = 0;

static struct input_dev *aw9523b_input_dev;
static struct i2c_client *g_client = NULL;

#define AW9523B_KEY_WWW  253
#define AW9523B_KEY_NULL 0x0
static const unsigned short func_key_array[8] = {KEY_LEFTSHIFT,KEY_LEFTALT,KEY_LEFTCTRL,KEY_RIGHTSHIFT,KEY_RIGHTALT,KEY_RIGHTCTRL};
static const unsigned short  key_array[Y_NUM][X_NUM] = {
        {KEY_7, KEY_Y,	  KEY_DELETE,    KEY_UP,       KEY_DOWN,  	 KEY_H,        KEY_B,         KEY_DOT},
        {KEY_3, KEY_W,	  KEY_9,         KEY_I,        KEY_M,        KEY_S,        KEY_Z,         KEY_J},
        {KEY_6, KEY_T,	  KEY_ENTER,     KEY_RIGHT,    KEY_LEFT,     KEY_G,        KEY_V,         KEY_COMMA},
        {KEY_2, KEY_Q,	  KEY_SUBTITLE,  KEY_P,        KEY_TAB,      KEY_A,        KEY_RIGHTBRACE,KEY_L},
        {KEY_4, KEY_E,	  KEY_TITLE,     KEY_SEMICOLON,KEY_K,        KEY_D,        KEY_X,         KEY_APOSTROPHE},
        {KEY_1, KEY_GRAVE,KEY_0,         KEY_O,        KEY_B,        KEY_BACKSLASH,KEY_LEFTBRACE, KEY_K},
        {KEY_5, KEY_R, 	  KEY_8,         KEY_U,        KEY_N,        KEY_F,        KEY_C,         KEY_SPACE},
        {KEY_ESC,KEY_HOME,KEY_BACKSPACE, KEY_SYM,      KEY_LEFTCTRL, KEY_CAPSLOCK, KEY_RIGHTSHIFT,KEY_LEFTALT},
};

#define P1_DEFAULT_VALUE  1  /*p1用来输出，这个值是p1的初始值*/
#define OUT_LOW_VALUE 0
#define OUT_HIGH_VALUE 1

static int aw9523b_i2c_read(struct i2c_client *client, char *writebuf,
               int writelen, char *readbuf, int readlen)
{
    int ret;

    if (writelen > 0) {
        struct i2c_msg msgs[] = {
            {
                 .addr = client->addr,
                 .flags = 0,
                 .len = writelen,
                 .buf = writebuf,
             },
            {
                 .addr = client->addr,
                 .flags = I2C_M_RD,
                 .len = readlen,
                 .buf = readbuf,
             },
        };
        ret = i2c_transfer(client->adapter, msgs, 2);
        if (ret < 0)
            dev_err(&client->dev, "%s: i2c read error.\n",
                __func__);
    } else {
        struct i2c_msg msgs[] = {
            {
                 .addr = client->addr,
                 .flags = I2C_M_RD,
                 .len = readlen,
                 .buf = readbuf,
             },
        };
        ret = i2c_transfer(client->adapter, msgs, 1);
        if (ret < 0)
            dev_err(&client->dev, "%s:i2c read error.\n", __func__);
    }
    return ret;
}

static int aw9523b_i2c_write(struct i2c_client *client, char *writebuf,
                int writelen)
{
    int ret;

    struct i2c_msg msgs[] = {
        {
             .addr = client->addr,
             .flags = 0,
             .len = writelen,
             .buf = writebuf,
         },
    };
    ret = i2c_transfer(client->adapter, msgs, 1);
    if (ret < 0)
        dev_err(&client->dev, "%s: i2c write error.\n", __func__);

    return ret;
}

static int aw9523b_write_reg(u8 addr, const u8 val)
{
    u8 buf[2] = {0};

    buf[0] = addr;
    buf[1] = val;

    return aw9523b_i2c_write(g_client, buf, sizeof(buf));
}



static int aw9523b_read_reg(u8 addr, u8 *val)
{
    int ret;
    ret = aw9523b_i2c_read(g_client, &addr, 1, val, 1);
    if (ret < 0)
            dev_err(&g_client->dev, "%s:i2c read error.\n", __func__);
    return ret;
}
#if 1
static u8 aw9523b_read_byte(u8 addr)
{   
    u8 val;
    aw9523b_read_reg(addr,&val);
    return val;
}
#endif
static int aw9523b_hw_reset(struct aw9523b_data *data)
{
    int ret = 0;
    
    ret = gpio_direction_output(data->gpio_rst, 1);
    if(ret){
        dev_err(&data->client->dev,"set_direction for pdata->gpio_rst failed\n");
    }   

    udelay(50);

    ret = aw9523b_write_reg(REG_SOFT_RESET,0x00);//softrest 
    if(ret < 0)
    {
        //can not communicate with aw9523b
        dev_err(&data->client->dev,"*****can not communicate with aw9523b\n");
        return 0xff;
    }

    ret = gpio_direction_output(data->gpio_rst, 0);
    if(ret){
        dev_err(&data->client->dev,"set_direction for pdata->gpio_rst failed\n");
    }     

    udelay(250);
    ret = gpio_direction_output(data->gpio_rst, 1);
      if(ret){
        dev_err(&data->client->dev,"set_direction for pdata->gpio_rst failed\n");
    }   

    udelay(50); 

    return ret;
}


static int aw9523b_i2c_test(struct aw9523b_data *data)
{ 
    aw9523b_read_reg(IC_ID, &aw9523b_chip_id);
    if(aw9523b_chip_id == 0x23 ) // read chip_id =0x23h   reg_addr=0x10h
    {
        printk("aw9523b get chip_id success,chip_id = %d\n", aw9523b_chip_id);
        return 0;
    }
    else
    {
        printk("aw9523b get chip_id failed, error chip_id = %d\n", aw9523b_chip_id);
        return -1;
    }
}



#if 0
static void aw9523b_config_P0_output(void)
{    
    aw9523b_write_reg(0x04, 0x00);
}

static void aw9523b_config_P1_input(void)
{
    aw9523b_write_reg(0x05, 0xFF);
}

static void aw9523b_enable_P1_interupt(void)
{
    aw9523b_write_reg(0x07, 0x00);
}


static void aw9523b_set_P0_value(u8 data)
{
    aw9523b_write_reg(0x02, data);
}

#endif


static void aw9523b_config_P1_output(void)
{    
    aw9523b_write_reg(0x05, 0x00);
}

static void aw9523b_config_P0_input(void)
{
    aw9523b_write_reg(0x04, 0xFF);
}

static void aw9523b_enable_P0_interupt(void)
{
    aw9523b_write_reg(0x06, 0x00);
}

static void aw9523b_disable_P0_interupt(void)
{
    aw9523b_write_reg(0x06, 0xff);
}

static void aw9523b_disable_P1_interupt(void)
{
    aw9523b_write_reg(0x07, 0xff);
}

static u8 aw9523b_get_P0_value(void)
{
    u8 value = 0;
    aw9523b_read_reg(0x00,&value);
    return value;
}


static u8 aw9523b_get_P1_value(void) 
{    
    u8 value = 0;
    aw9523b_read_reg(0x01,&value);
    return value;    
}

static void aw9523b_set_P1_value(u8 data)
{
    aw9523b_write_reg(0x03, data);
}

static void init_p0_p1_lookup_table(void)
{
    int i = 0;

    for (i = 0;i < X_NUM; i++) {
        p0_kbd_used_mask = p0_kbd_used_mask | (1 << col[i]);
    }
    AW9523_LOG("col_temp = %x \n", p0_kbd_used_mask);
#if 0
    for (i = 0; i < X_NUM; i++) {
        p0_port_be_press_value[i] = p0_kbd_used_mask & (~(1 << col[i]));
        AW9523_LOG("p0_port_be_press_value[%d] = %x \n", i, p0_port_be_press_value[i]);
    }
#endif
    for (i = 0; i < Y_NUM; i++) {
        p1_kbd_used_mask = p1_kbd_used_mask | (1 << row[i]);
    }
    AW9523_LOG("p1_kbd_used_mask = %x \n", p1_kbd_used_mask);

    for (i = 0; i < Y_NUM; i++) {
        p1_port_output_value[i] = p1_kbd_used_mask & (~(1 << row[i]));
        AW9523_LOG("p1_port_output_value[%d] = %x \n", i, p1_port_output_value[i]);
    }


}


static void default_p0_p1_settings(void)
{
    init_p0_p1_lookup_table();
    aw9523b_config_P0_input();
    aw9523b_enable_P0_interupt();    
    aw9523b_config_P1_output();
    aw9523b_disable_P1_interupt();
        
    aw9523b_set_P1_value(P1_DEFAULT_VALUE);
    //aw9523b_set_P1_value(0x55);
}

#if 0
static void aw9523b_P0_P1_init(void)
{
    AW9523_FUN(f);
    //aw9523b_p0_int_disable();
    default_p0_p1_settings();

    aw9523b_get_P0_value();
    aw9523b_get_P1_value();
}
#endif

u16 keyboard_get_func_press_key(void)
{
	u8 p0_value = 0xff, p1_index = 0XFF,i=0;

    p0_value = aw9523b_get_P0_value();
	AW9523_LOG("%s p0_value=%x  \n", __func__,aw9523b_get_P0_value());
    //p0_value &= 0x80;
    p0_value = ~(p0_value&(0xff));
	AW9523_LOG("%s p0_value=%x  \n", __func__,p0_value);
    if (p0_value)
    {
        return 0xFF;
    }

	for (i = 0;i < Y_NUM; i++) {
        aw9523b_set_P1_value( p1_port_output_value[i]);
		p0_value = aw9523b_get_P0_value();		
		AW9523_LOG("%s [%d]:p0_value=%x  \n", __func__,i,p0_value);
		p0_value &= 0x80;
        if(!p0_value) 
        {
            p1_index = i;
            //break;
        }
    }
    AW9523_LOG("%s p1_index = %d \n", __func__,p1_index);
	aw9523b_set_P1_value(OUT_LOW_VALUE);  

    if (p1_index!= 0xFF ) {
        return func_key_array[p1_index];
    } else {
        return 0xFF;
    }

}

u16 keyboard_get_press_key(u16 *funckey)
{
    u8 p0_index = 0xFF, p1_index = 0XFF,key_index=0xff;
    u8 i = 0,j=0;
    u8 p0_value = 0xff;
	u16 checkkey = 0xff;
    p0_value = (aw9523b_get_P0_value()&p0_kbd_used_mask);

    AW9523_LOG("X p0_value=0x%x \n", p0_value);
    if (!((~p0_value)& p0_kbd_used_mask))
    {	
		*funckey = checkkey = 0xff;
        return 0xFF;
    }
	
	for(j = 0; j < 3; j++){
		for (i = 0; i < X_NUM; i++) {		 
			if(p0_value == p0_port_be_press_value[j][i]){
				p0_index = i;
				break;
			}
		}
		if(p0_index!=0xFF){
			key_index = j;
			break;
		}
    }
	AW9523_LOG("key_index=%d \n", key_index);
    if (p0_index == 0xFF|| key_index == 0xFF) {
		
        AW9523_LOG("------p0_value can not found in table---- \n");
        return 0xFF;
    }

    for (i = 0;i < Y_NUM; i++) {
        aw9523b_set_P1_value( p1_port_output_value[i]);
        p0_value = (aw9523b_get_P0_value()&p0_kbd_used_mask);
		AW9523_LOG("Y p0_value=%x  \n", p0_value);
        if(((p0_value)&(p0_kbd_used_mask)) == p0_port_be_press_value[0][p0_index]||((p0_value)&(p0_kbd_used_mask)) == p0_port_be_press_value[1][p0_index]) 
        {
            p1_index = i;
			checkkey = key_array[p1_index][p0_index];
			if(checkkey==KEY_LEFTCTRL || checkkey==KEY_RIGHTSHIFT ||checkkey ==KEY_LEFTALT){
				*funckey = checkkey;
				continue;
			}
			else
				break;
        }
    }
    AW9523_LOG("p0_index = %d, p1_index = %d \n", p0_index, p1_index);

    if (p0_index != 0xFF && p1_index!= 0xFF && key_array[p1_index][p0_index] != *(funckey)) {
		
        return key_array[p1_index][p0_index];
    } else {
        return 0xFF;
    }
} 
//static void aw9523b_work_func(unsigned long data);
//static DECLARE_WORK(aw9523b_work, aw9523b_work_func);



void aw9523b_irq_disable(struct aw9523b_data *data)
{
    unsigned long irqflags;

    spin_lock_irqsave(&data->irq_lock, irqflags);
    if (!data->irq_is_disabled) {
        data->irq_is_disabled = true;
        disable_irq_nosync(data->client->irq);
    }
    spin_unlock_irqrestore(&data->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void aw9523b_irq_enable(struct aw9523b_data *data)
{
    unsigned long irqflags = 0;

    spin_lock_irqsave(&data->irq_lock, irqflags);
    if (data->irq_is_disabled) {
        enable_irq(data->client->irq);
        data->irq_is_disabled = false;
    }
    spin_unlock_irqrestore(&data->irq_lock, irqflags);
}

static void aw9523b_work_func(struct work_struct *work)
{
    struct aw9523b_data *pdata = NULL;
    static u16 keycode = 0xFF;
    static u16 pre_keycode = 0xFF;
    static int pressed = 0;
    static u8 capslock_led_enable = 0;
#if 1
	static u16 func_keycode = 0xFF;
	static u16 pre_func_keycode = 0xFF; 
    static int func_pressed = 0;
#endif
    pdata = container_of(work, struct aw9523b_data, work);
	AW9523_LOG("aw9523b_work_func  enter \n");

    aw9523b_disable_P0_interupt();

    //aw9523b_get_P0_value();
    //aw9523b_get_P1_value();
#if 0
	func_keycode = keyboard_get_func_press_key();
	
	AW9523_LOG("keyboard_get_func_press_key  func_keycode=%x \n",func_keycode);
	if(!func_pressed) {
        if(func_keycode == 0xFF)
        {
            goto normal_key;
        }

        func_pressed = 1;
        input_report_key(aw9523b_input_dev, func_keycode, 1);
        input_sync(aw9523b_input_dev);
        pre_func_keycode = func_keycode;
        AW9523_LOG("(func press) keycode = %d \n", func_keycode);
    } else {

		if (pre_func_keycode==func_keycode)
		{
			goto normal_key;
		}
        func_pressed = 0;
        input_report_key(aw9523b_input_dev, pre_func_keycode, 0);
        input_sync(aw9523b_input_dev);
        AW9523_LOG("(func released) keycode = %d \n", pre_func_keycode);
    }
#endif

    keycode = keyboard_get_press_key(&func_keycode);
	AW9523_LOG("keyboard_get_press_key  keycode=%x func_keycode=0x%x \n",keycode,func_keycode);
	
	if(!func_pressed) {
        if(func_keycode == 0xFF)
        {
            goto normal_key;
        }

        func_pressed = 1;
        input_report_key(aw9523b_input_dev, func_keycode, 1);
        input_sync(aw9523b_input_dev);
        pre_func_keycode = func_keycode;
        AW9523_LOG("(func press) keycode = %x \n", func_keycode);
    } else if(keycode ==0xff&&func_keycode==0xff){

		if (pre_func_keycode==func_keycode)
		{
			goto normal_key;
		}
        func_pressed = 0;
        input_report_key(aw9523b_input_dev, pre_func_keycode, 0);
        input_sync(aw9523b_input_dev);
        AW9523_LOG("(func released) keycode = %x \n", pre_func_keycode);
    }
normal_key:
	if(!pressed) {
        if(keycode == 0xFF)
        {
            goto func_exit;
        }
		if(keycode == KEY_CAPSLOCK){
			if(capslock_led_enable == 0)
				gpio_direction_output(pdata->gpio_caps_led, 1);
			capslock_led_enable++;
		}
        pressed = 1;
        input_report_key(aw9523b_input_dev, keycode, 1);
        input_sync(aw9523b_input_dev);
        pre_keycode = keycode;
        AW9523_LOG("(press) keycode = %d \n", keycode);
    } else {		
		if(capslock_led_enable>=2){
			gpio_direction_output(pdata->gpio_caps_led, 0);
			capslock_led_enable = 0;
		}
        pressed = 0;
        input_report_key(aw9523b_input_dev, pre_keycode, 0);
        input_sync(aw9523b_input_dev);
        AW9523_LOG("(released) keycode = %d \n", pre_keycode);
    }
 func_exit:
    //aw9523b_enable_P0_interupt();
    //aw9523b_get_P0_value();
    //aw9523b_get_P1_value();
    //aw9523b_set_P1_value(OUT_LOW_VALUE);
	aw9523b_irq_enable(pdata);
	aw9523b_config_P0_input();
    aw9523b_enable_P0_interupt();    
    aw9523b_config_P1_output();
    aw9523b_disable_P1_interupt();
	aw9523b_set_P1_value(OUT_LOW_VALUE);
}

static irqreturn_t aw9523b_irq_handler(int irq, void *dev_id)
{
    struct aw9523b_data *pdata = dev_id;

    printk("%s enter\n",__func__);

    aw9523b_irq_disable(pdata);

    schedule_work(&pdata->work);

    return IRQ_HANDLED;
}
#if 1

static ssize_t aw9523b_show_chip_id(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    ssize_t res;
    //struct aw9523b_data *data = dev_get_drvdata(dev);

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", aw9523b_chip_id);

    return res; 
}

static ssize_t aw9523b_show_reg(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    ssize_t res = 0;
    char *ptr = buf;
    
    ptr += sprintf(ptr, "INPUT_PORT0: 0x%x\n", aw9523b_read_byte(INPUT_PORT0));
    ptr += sprintf(ptr, "INPUT_PORT1: 0x%x\n", aw9523b_read_byte(INPUT_PORT1));
    ptr += sprintf(ptr, "OUTPUT_PORT0: 0x%x\n", aw9523b_read_byte(OUTPUT_PORT0));
    ptr += sprintf(ptr, "OUTPUT_PORT1: 0x%x\n", aw9523b_read_byte(OUTPUT_PORT1));
    ptr += sprintf(ptr, "CONFIG_PORT0: 0x%x\n", aw9523b_read_byte(CONFIG_PORT0));
    ptr += sprintf(ptr, "CONFIG_PORT1: 0x%x\n", aw9523b_read_byte(CONFIG_PORT1));
    ptr += sprintf(ptr, "INT_PORT0: 0x%x\n", aw9523b_read_byte(INT_PORT0));
    ptr += sprintf(ptr, "INT_PORT1: 0x%x\n", aw9523b_read_byte(INT_PORT1));
    ptr += sprintf(ptr, "IC_ID: 0x%x\n", aw9523b_read_byte(IC_ID));
    ptr += sprintf(ptr, "CTL: 0x%x\n", aw9523b_read_byte(CTL));
    ptr += sprintf(ptr, "\n");
    res = ptr - buf;

    return res;
}

static DEVICE_ATTR(aw9523b_reg, (S_IRUGO | S_IWUSR | S_IWGRP),
            aw9523b_show_reg,
            NULL);
static DEVICE_ATTR(aw9523b_chip_id, (S_IRUGO | S_IWUSR | S_IWGRP),
            aw9523b_show_chip_id,
            NULL);


static struct attribute *aw9523b_attrs[] = {
    &dev_attr_aw9523b_reg.attr,
    &dev_attr_aw9523b_chip_id.attr,
    NULL
};

static const struct attribute_group aw9523b_attr_grp = {
    .attrs = aw9523b_attrs,
};

#endif
#if 0
static DRIVER_ATTR(aw9523b_reg, S_IRUGO, aw9523b_show_reg, NULL);
static DRIVER_ATTR(aw9523b_chip_id, S_IRUGO, aw9523b_show_chip_id, NULL);

static struct driver_attribute *aw9523b_attr_list[] = {
        &driver_attr_aw9523b_chip_id,
        &driver_attr_aw9523b_reg,
};

static int aw9523b_create_attr(struct device_driver *driver)
{
        int idx,err=0;
        int num = (int)(sizeof(aw9523b_attr_list)/sizeof(aw9523b_attr_list[0]));

        if (driver == NULL)
                return -EINVAL;

        for(idx = 0; idx < num; idx++) {
                if((err = driver_create_file(driver, aw9523b_attr_list[idx]))) {
                        printk("driver_create_file (%s) = %d\n", aw9523b_attr_list[idx]->attr.name, err);
                        break;
                }
        }

        return err;
}

static struct platform_driver aw9523b_pdrv;
#endif
static int register_aw9523b_input_dev(struct device *pdev)
{
    int r,i,j;

    AW9523_FUN(f);

    aw9523b_input_dev = input_allocate_device();
    if (!aw9523b_input_dev)
        return -ENOMEM;

    aw9523b_input_dev->name = "aw9523b";
    aw9523b_input_dev->id.bustype = BUS_HOST;
    aw9523b_input_dev->id.vendor = 0x0;
    aw9523b_input_dev->id.product = 0x0;
    aw9523b_input_dev->id.version = 0x0;

    __set_bit(EV_KEY, aw9523b_input_dev->evbit);

	for (i=0;i<X_NUM;i++)
		for (j=0;j<Y_NUM;j++)
			if (key_array[i][j]!=0xff)
				input_set_capability(aw9523b_input_dev, EV_KEY, key_array[i][j]);
    
    
    aw9523b_input_dev->dev.parent = pdev;
    r = input_register_device(aw9523b_input_dev);
    if (r) {
        input_free_device(aw9523b_input_dev);
        return r;
    }
    return 0;
}

static int aw9523b_power_ctl(struct aw9523b_data *data, bool on)
{
    int ret = 0;
#ifdef USEVIO
    if (!on && data->power_enabled) {
        ret = regulator_disable(data->vio);
        if (ret) {
            dev_err(&data->client->dev,
                "Regulator vio disable failed ret=%d\n", ret);
            return ret;
        }
        
        data->power_enabled = on;
    } else if (on && !data->power_enabled) {
        ret = regulator_enable(data->vio);
        if (ret) {
            dev_err(&data->client->dev,
                "Regulator vio enable failed ret=%d\n", ret);
            return ret;
        }        
        data->power_enabled = on;
    } else {
        dev_info(&data->client->dev,
                "Power on=%d. enabled=%d\n",
                on, data->power_enabled);
    }
#endif
    return ret;

}

static int aw9523b_power_init(struct aw9523b_data *data)
{
        int ret = 0;
#ifdef USEVIO    
        data->vio = regulator_get(&data->client->dev, "vio");
        if (IS_ERR(data->vio)) {
            ret = PTR_ERR(data->vio);
            dev_err(&data->client->dev,
                "Regulator get failed vdd ret=%d\n", ret);
            return ret;
        }
    
        if (regulator_count_voltages(data->vio) > 0) {
            ret = regulator_set_voltage(data->vio,
                    AW9523B_VIO_MIN_UV,
                    AW9523B_VIO_MAX_UV);
            if (ret) {
                dev_err(&data->client->dev,
                    "Regulator set failed vio ret=%d\n",
                    ret);
                goto reg_vio_put;
            }
        }
    
        return 0;
        
reg_vio_put:
        regulator_put(data->vio);
#endif
        return ret;

}

static int aw9523b_power_deinit(struct aw9523b_data *data)
{
#ifdef USEVIO
        if (regulator_count_voltages(data->vio) > 0)
            regulator_set_voltage(data->vio,
                    0, AW9523B_VIO_MAX_UV);
    
        regulator_put(data->vio);            
#endif    
        return 0;
}

#ifdef CONFIG_OF
static int aw9523b_parse_dt(struct device *dev,
            struct aw9523b_data *pdata)
{
    int err = 0;
    struct device_node *np = dev->of_node;
    
    pdata->gpio_rst = of_get_named_gpio(np, "awinic,reset-gpio", 0);
    if (gpio_is_valid(pdata->gpio_rst)) {
        err = gpio_request(pdata->gpio_rst, "aw9523b_reset_gpio");
        if (err) {
            dev_err(&pdata->client->dev, "pdata->gpio_rst gpio request failed");
            return -ENODEV;
        }
        err = gpio_direction_output(pdata->gpio_rst, 1);
        if (err) {
            dev_err(&pdata->client->dev,
                "set_direction for pdata->gpio_rst failed\n");
            return -ENODEV;
        }
    }
    else
    {
        dev_err(dev, "pdata->gpio_rst is error\n");
        return pdata->gpio_rst;
    }
	pdata->gpio_caps_led = of_get_named_gpio(np, "awinic,caps-gpio", 0);
	 if (gpio_is_valid(pdata->gpio_caps_led)) {
        err = gpio_request(pdata->gpio_caps_led, "aw9523b_gpio_caps_led");
        if (err) {
            dev_err(&pdata->client->dev, "pdata->gpio_caps_led gpio request failed");
            return -ENODEV;
        }       
    }
    else
    {
        dev_err(dev, "pdata->gpio_caps_led is error\n");
        return pdata->gpio_caps_led;
    }

    pdata->gpio_irq = of_get_named_gpio(np, "awinic,irq-gpio", 0);
    if (gpio_is_valid(pdata->gpio_irq)) {
        err = gpio_request(pdata->gpio_irq, "aw9523b_irq_gpio");
        if (err) {
            dev_err(&pdata->client->dev, "pdata->gpio_rst gpio request failed");
            return -ENODEV;
        }
        err = gpio_direction_input(pdata->gpio_irq);
        //err = gpio_direction_output(pdata->gpio_rst,0);
        if (err) {
            dev_err(&pdata->client->dev,
                "set_direction for pdata->gpio_irq failed\n");
            return -ENODEV;
        }
    }
    else
    {
        dev_err(dev, "pdata->gpio_irq is error\n");
        return pdata->gpio_irq;
    }
    
    return 0;
}
#else
static int aw9523b_parse_dt(struct device *dev,
            struct aw9523b_platform_data *pdata)
{
    return -EINVAL;
}
#endif


static int aw9523b_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int err = 0;
    int devic_id = 0;
    struct aw9523b_data *pdata;
    
    printk("%s begin\n",__func__);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "i2c_check_functionality error\n");
        err = -EPERM;
        goto exit;
    }
    pdata = kzalloc(sizeof(struct aw9523b_data), GFP_KERNEL);
    if (!pdata) {
        err = -ENOMEM;
        goto exit;
    }
    
    if (client->dev.of_node) {
        err = aw9523b_parse_dt(&client->dev, pdata);
        if (err) {
            dev_err(&client->dev, "Failed to parse device tree\n");
            err = -EINVAL;
            goto pdata_free_exit;
        }
    } 
    printk ("rst_gpio=%d irq_gpio=%d irq=%d\n",pdata->gpio_rst,pdata->gpio_irq,client->irq);

    if (!pdata) {
        dev_err(&client->dev, "Cannot get device platform data\n");
        err = -EINVAL;
        goto kfree_exit;
    }
    
    spin_lock_init(&pdata->irq_lock);   
    g_client = client;
	i2c_set_clientdata(client, pdata);
    pdata->client = client;

    

    err = aw9523b_power_init(pdata);
    if (err) {
        dev_err(&client->dev, "Failed to get aw9523b regulators\n");
        err = -EINVAL;
        goto free_input_dev;
    }
    err = aw9523b_power_ctl(pdata, true);
    if (err) {
        dev_err(&client->dev, "Failed to enable aw9523b power\n");
        err = -EINVAL;
        goto deinit_power_exit;
    }

    
	err = aw9523b_hw_reset(pdata);
	if(err == 0xff)
	{
		dev_err(&client->dev, "aw9523b failed to write \n");
		err = -EINVAL;
		goto deinit_power_exit;
	}
	if (err) {
		dev_err(&client->dev, "aw9523b failed to reset\n");
	}
		
    devic_id = aw9523b_i2c_test(pdata);
    if(devic_id < 0)
    {
        dev_err(&client->dev, "aw9523b failed to read \n\n\n\n");
        err = -EINVAL;
        goto deinit_power_exit;
    }
	
	err = register_aw9523b_input_dev(&client->dev);
    if (err) {
        dev_err(&client->dev, "Failed to get aw9523b regulators\n");
        err = -EINVAL;
        goto free_i2c_clientdata_exit;
    }

 //   err = sysfs_create_group(&client->dev.kobj, &aw9523b_attr_grp);
 //   if (err < 0) {
 //       dev_err(&client->dev, "sys file creation failed.\n");
 //       goto deinit_power_exit;
 //   }    

    
    default_p0_p1_settings();     //io_init
    aw9523b_get_P0_value();
	aw9523b_get_P1_value();

    
   // debug_printk("%s device_id = %d\n",__func__,devic_id);
    INIT_WORK(&pdata->work, aw9523b_work_func);
    pdata->irq_is_disabled = true;
    err = request_irq(client->irq, 
            aw9523b_irq_handler,
            IRQ_TYPE_LEVEL_LOW,
            client->name, pdata);
	printk("request_threaded_irq %d\n",err);
    //schedule_delayed_work(&pdata->keypad_work, 0);
 	aw9523b_irq_enable(pdata);
    printk("%s exit success\n",__func__);
    return 0;

//exit_remove_sysfs:
//    sysfs_remove_group(&client->dev.kobj, &aw9523b_attr_grp);
deinit_power_exit:
    aw9523b_power_deinit(pdata);
free_input_dev:
    input_free_device(aw9523b_input_dev);
pdata_free_exit:
    if (pdata && (client->dev.of_node))
        devm_kfree(&client->dev, pdata);
free_i2c_clientdata_exit:
    i2c_set_clientdata(client, NULL);    
kfree_exit:
    kfree(pdata);
exit:
    return err;
}


static int aw9523b_remove(struct i2c_client *client)
{
    struct aw9523b_data *data = i2c_get_clientdata(client);
    
    //cancel_delayed_work_sync(&data->p1_012_work);
    aw9523b_power_deinit(data);
    i2c_set_clientdata(client, NULL);

    kfree(data);

    return 0;
}


static const struct i2c_device_id aw9523b_id[] = {
    { AWINIC_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, aw9523b_id);

static const struct of_device_id aw9523b_of_match[] = {
    { .compatible = "awinic,aw9523b", },
    { },
};

static struct i2c_driver aw9523b_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = AWINIC_NAME,
        .of_match_table = aw9523b_of_match,
    },
    .id_table   = aw9523b_id,
    .probe      = aw9523b_probe,
    .remove     = aw9523b_remove,
};

static int __init AW9523B_init(void)
{
    return i2c_add_driver(&aw9523b_driver);
}

static void __exit AW9523B_exit(void)
{
    i2c_del_driver(&aw9523b_driver);
}

MODULE_AUTHOR("contact@AWINIC TECHNOLOGY");
MODULE_DESCRIPTION("AW9523B LED OF GPIO DRIVER");
MODULE_LICENSE("GPL v2");

module_init(AW9523B_init);
module_exit(AW9523B_exit);

