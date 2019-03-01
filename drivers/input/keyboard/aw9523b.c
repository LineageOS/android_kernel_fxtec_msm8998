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
#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/syscore_ops.h>

struct gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;

	struct timer_list release_timer;
	unsigned int release_delay;	/* in msecs, for IRQ-only buttons */

	struct delayed_work work;
	unsigned int software_debounce;	/* in msecs, for GPIO-driven buttons */

	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
};

struct gpio_keys_drvdata {
	const struct gpio_keys_platform_data *pdata;
	struct pinctrl *key_pinctrl;
	struct input_dev *input;
	struct mutex disable_lock;
	struct gpio_button_data data[0];
};

static struct device *global_dev;
static struct syscore_ops gpio_keys_syscore_pm_ops;

static void gpio_keys_syscore_resume(void);
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
        {KEY_F1,        KEY_Y,	  KEY_ENTER,  KEY_UP,       KEY_7,        KEY_H,        KEY_B,         KEY_COMMA},
        {KEY_3,         KEY_W,	  KEY_9,      KEY_I,        KEY_M,        KEY_S,        KEY_Z,         KEY_J},
        {KEY_LEFT,      KEY_T,	  KEY_DELETE, KEY_RIGHT,    KEY_6,        KEY_G,        KEY_V,         KEY_DOT},
        {KEY_F2,        KEY_Q,	  KEY_MINUS,  KEY_P,        KEY_HOMEPAGE, KEY_A,        KEY_RIGHTBRACE,KEY_L},
        {KEY_BACKSPACE, KEY_E,	  KEY_EQUAL,  KEY_SEMICOLON,KEY_K,        KEY_D,        KEY_X,         KEY_APOSTROPHE},
        {KEY_CAPSLOCK,  KEY_GRAVE,KEY_0,      KEY_O,        KEY_DOWN,     KEY_BACKSLASH,KEY_LEFTBRACE, KEY_K},
        {KEY_SPACE,     KEY_R, 	  KEY_8,      KEY_U,        KEY_N,        KEY_F,        KEY_C,         KEY_5},
        {KEY_ESC,       KEY_TAB,  KEY_4,      KEY_2 ,       0XFF,         KEY_1, 0XFF,0XFF},
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
    int i,j;

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
   // r = input_register_device(aw9523b_input_dev);
   // if (r) {
   //     input_free_device(aw9523b_input_dev);
   //     return r;
    //}
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

/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and associated timer/work structure.
		 */
		disable_irq(bdata->irq);

		if (gpio_is_valid(bdata->button->gpio))
			cancel_delayed_work_sync(&bdata->work);
		else
			del_timer_sync(&bdata->release_timer);

		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(bdata->irq);
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
	}

	ret = scnprintf(buf, PAGE_SIZE - 1, "%*pbl", n_events, bits);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordingly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	if (i == ddata->pdata->nbuttons) {
		error = -EINVAL;
		goto out;
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state;

	state = (__gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;
	if (state < 0) {
		dev_err(input->dev.parent, "failed to get gpio state\n");
		return;
	}

	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
		input_event(input, type, button->code, !!state);
	}
	input_sync(input);
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work.work);

	gpio_keys_gpio_report_event(bdata);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	if (bdata->button->wakeup)
		pm_stay_awake(bdata->input->dev.parent);

	mod_delayed_work(system_wq,
			 &bdata->work,
			 msecs_to_jiffies(bdata->software_debounce));

	return IRQ_HANDLED;
}

static void gpio_keys_irq_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, bdata->button->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}
	spin_unlock_irqrestore(&bdata->lock, flags);
}

static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (!bdata->key_pressed) {
		if (bdata->button->wakeup)
			pm_wakeup_event(bdata->input->dev.parent, 0);

		input_event(input, EV_KEY, button->code, 1);
		input_sync(input);

		if (!bdata->release_delay) {
			input_event(input, EV_KEY, button->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

	if (bdata->release_delay)
		mod_timer(&bdata->release_timer,
			jiffies + msecs_to_jiffies(bdata->release_delay));
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}

static void gpio_keys_quiesce_key(void *data)
{
	struct gpio_button_data *bdata = data;

	if (gpio_is_valid(bdata->button->gpio))
		cancel_delayed_work_sync(&bdata->work);
	else
		del_timer_sync(&bdata->release_timer);
}

static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_button_data *bdata,
				const struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq;
	int error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	if (gpio_is_valid(button->gpio)) {

		error = devm_gpio_request_one(&pdev->dev, button->gpio,
					      GPIOF_IN, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		if (button->debounce_interval) {
			error = gpio_set_debounce(button->gpio,
					button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->software_debounce =
						button->debounce_interval;
		}

		if (button->irq) {
			bdata->irq = button->irq;
		} else {
			irq = gpio_to_irq(button->gpio);
			if (irq < 0) {
				error = irq;
				dev_err(dev,
					"Unable to get irq number for GPIO %d, error %d\n",
					button->gpio, error);
				return error;
			}
			bdata->irq = irq;
		}

		INIT_DELAYED_WORK(&bdata->work, gpio_keys_gpio_work_func);

		isr = gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	} else {
		if (!button->irq) {
			dev_err(dev, "No IRQ specified\n");
			return -EINVAL;
		}
		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->release_delay = button->debounce_interval;
		setup_timer(&bdata->release_timer,
			    gpio_keys_irq_timer, (unsigned long)bdata);

		isr = gpio_keys_irq_isr;
		irqflags = 0;
	}

	input_set_capability(input, button->type ?: EV_KEY, button->code);

	/*
	 * Install custom action to cancel release timer and
	 * workqueue item.
	 */
	error = devm_add_action(&pdev->dev, gpio_keys_quiesce_key, bdata);
	if (error) {
		dev_err(&pdev->dev,
			"failed to register quiesce action, error: %d\n",
			error);
		return error;
	}

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = devm_request_any_context_irq(&pdev->dev, bdata->irq,
					     isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return error;
	}

	return 0;
}

static void gpio_keys_report_state(struct gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (gpio_is_valid(bdata->button->gpio))
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}

static int gpio_keys_pinctrl_configure(struct gpio_keys_drvdata *ddata,
							bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"idea_gpio_key_active");
		if (IS_ERR(set_state)) {
			dev_err(&ddata->input->dev,
				"cannot get ts pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"idea_gpio_key_suspend");
		if (IS_ERR(set_state)) {
			dev_err(&ddata->input->dev,
				"cannot get gpiokey pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(ddata->key_pinctrl, set_state);
	if (retval) {
		dev_err(&ddata->input->dev,
				"cannot set ts pinctrl active state\n");
		return retval;
	}

	return 0;
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;
	int error;

	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error)
			return error;
	}

	/* Report current state of buttons that are connected to GPIOs */
	gpio_keys_report_state(ddata);

	return 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}

/*
 * Handlers for alternative sources of platform_data
 */

#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

	nbuttons = of_get_child_count(node);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nbuttons * sizeof(*button),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->buttons = (struct gpio_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	pdata->rep = !!of_get_property(node, "autorepeat", NULL);
	pdata->name = of_get_property(node, "input-name", NULL);
	pdata->use_syscore = of_property_read_bool(node, "use-syscore");

	i = 0;
	for_each_child_of_node(node, pp) {
		enum of_gpio_flags flags;

		button = &pdata->buttons[i++];

		button->gpio = of_get_gpio_flags(pp, 0, &flags);
		if (button->gpio < 0) {
			error = button->gpio;
			if (error != -ENOENT) {
				if (error != -EPROBE_DEFER)
					dev_err(dev,
						"Failed to get gpio flags, error: %d\n",
						error);
				return ERR_PTR(error);
			}
		} else {
			button->active_low = flags & OF_GPIO_ACTIVE_LOW;
		}

		button->irq = irq_of_parse_and_map(pp, 0);

		if (!gpio_is_valid(button->gpio) && !button->irq) {
			dev_err(dev, "Found button without gpios or irqs\n");
			return ERR_PTR(-EINVAL);
		}

		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				button->gpio);
			return ERR_PTR(-EINVAL);
		}

		button->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = of_property_read_bool(pp, "wakeup-source") ||
				 /* legacy name */
				 of_property_read_bool(pp, "gpio-key,wakeup");

		button->can_disable = !!of_get_property(pp, "linux,can-disable", NULL);

		if (of_property_read_u32(pp, "debounce-interval",
					&button->debounce_interval))
			button->debounce_interval = 5;
	}

	if (pdata->nbuttons == 0)
		return ERR_PTR(-EINVAL);

	return pdata;
}

static const struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "idea-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

#else

static inline struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

#endif

static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	size_t size;
	int i, error;
	int wakeup = 0;
	struct pinctrl_state *set_state;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	size = sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data);
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	input = aw9523b_input_dev;//devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	global_dev = dev;
	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	//input->name = GPIO_KEYS_DEV_NAME;
	//input->phys = "gpio-keys/input0";
	//input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	//input->id.bustype = BUS_HOST;
	input->id.vendor = 0x181d;
	input->id.product = 0x5018;
	input->id.version = 0x0001;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	/* Get pinctrl if target uses pinctrl */
	ddata->key_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ddata->key_pinctrl)) {
		if (PTR_ERR(ddata->key_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		pr_debug("Target does not use pinctrl\n");
		ddata->key_pinctrl = NULL;
	}

	if (ddata->key_pinctrl) {
		error = gpio_keys_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(dev, "cannot set ts pinctrl active state\n");
			return error;
		}
	}

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto err_setup_key;

		if (button->wakeup)
			wakeup = 1;
	}

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto err_create_sysfs;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto err_remove_group;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	if (pdata->use_syscore)
		gpio_keys_syscore_pm_ops.resume = gpio_keys_syscore_resume;

	register_syscore_ops(&gpio_keys_syscore_pm_ops);

	return 0;

err_remove_group:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
err_create_sysfs:
err_setup_key:
	if (ddata->key_pinctrl) {
		set_state =
		pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_gpio_key_suspend");
		if (IS_ERR(set_state))
			dev_err(dev, "cannot get gpiokey pinctrl sleep state\n");
		else
			pinctrl_select_state(ddata->key_pinctrl, set_state);
	}

	return error;
}

static int gpio_keys_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	unregister_syscore_ops(&gpio_keys_syscore_pm_ops);

	device_init_wakeup(&pdev->dev, 0);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void gpio_keys_syscore_resume(void)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(global_dev);
	struct input_dev *input = ddata->input;
	struct gpio_button_data *bdata = NULL;
	int error = 0;
	int i;

	if (ddata->key_pinctrl) {
		error = gpio_keys_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(global_dev, "failed to put the pin in resume state\n");
			return;
		}
	}

	if (device_may_wakeup(global_dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				disable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_keys_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return;

	gpio_keys_report_state(ddata);
}

static int gpio_keys_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i, ret;

	if (ddata->key_pinctrl) {
		ret = gpio_keys_pinctrl_configure(ddata, false);
		if (ret) {
			dev_err(dev, "failed to put the pin in suspend state\n");
			return ret;
		}
	}

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				enable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			gpio_keys_close(input);
		mutex_unlock(&input->mutex);
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error = 0;
	int i;

	if (ddata->pdata->use_syscore == true) {
		dev_dbg(global_dev, "Using syscore resume, no need of this resume.\n");
		return 0;
	}

	if (ddata->key_pinctrl) {
		error = gpio_keys_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(dev, "failed to put the pin in resume state\n");
			return error;
		}
	}

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				disable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_keys_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return error;

	gpio_keys_report_state(ddata);
	return 0;
}

#else

static void gpio_keys_syscore_resume(void){}

static int gpio_keys_suspend(struct device *dev)
{
	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	return 0;
}

#endif

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "idea-keys",
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = of_match_ptr(gpio_keys_of_match),
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);


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

