/* Device Driver for DAC7311 array */

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/ioport.h>
#include <linux/gpio/driver.h>
#include <linux/timekeeping.h>
#include <asm/pgtable.h>
#include <linux/hrtimer.h>

#include "../../drivers/pinctrl/sunxi/pinctrl-sunxi.h" 
#include "../../drivers/gpio/gpiolib.h"

MODULE_AUTHOR("kricnam<me.xinxin@gmail.com>");

#define SIG_GEN_NAME "dac7311"
#define SG_CMD_START 0
#define SG_CMD_STOP  1

#define REG_BASE 
#define REG_PG_DAT REG_BASE + 0xE8

static unsigned int pulse_major = 0;
static struct cdev c_dev;
static struct class * pulse_class;
static dev_t dev=0;

static struct task_struct *thread1;
struct resource * res_pg_dat;

static struct gpio_desc * pgpio;
struct gpio_chip *chip ;
struct sunxi_pinctrl *pctl;
u32 reg;

#define DIN1_GPIO_NUM 114
#define DIN2_GPIO_NUM 115
#define DIN3_GPIO_NUM 116
#define DIN4_GPIO_NUM 117
#define DIN5_GPIO_NUM 118
#define DIN6_GPIO_NUM 119
#define DIN7_GPIO_NUM 120
#define DIN8_GPIO_NUM 121
#define DIN9_GPIO_NUM 122
#define DIN10_GPIO_NUM 123
#define SYNC1_GPIO_NUM 98
#define SYNC2_GPIO_NUM 99
#define SYNC3_GPIO_NUM 100
#define SYNC4_GPIO_NUM 101
#define SYNC5_GPIO_NUM 102
#define SYNC6_GPIO_NUM 103
#define SYNC7_GPIO_NUM 106
#define SYNC8_GPIO_NUM 107
#define SYNC9_GPIO_NUM 108
#define SYNC10_GPIO_NUM 109
#define SCLK_GPIO_NUM 110

static struct gpio dac7311_gpios[] = {
  {SYNC1_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC1"},
  {SYNC2_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC2"},
  {SYNC3_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC3"},
  {SYNC4_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC4"},
  {SYNC5_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC5"},
  {SYNC6_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC6"},
  {SYNC7_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC7"},
  {SYNC8_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC8"},
  {SYNC9_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC9"},
  {SYNC10_GPIO_NUM, GPIOF_OUT_INIT_LOW, "SYNC10"},
  {SCLK_GPIO_NUM, GPIOF_OUT_INIT_HIGH, "SCLK"},
  {DIN1_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN1"},
  {DIN2_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN2"},
  {DIN3_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN3"},
  {DIN4_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN4"},
  {DIN5_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN5"},
  {DIN6_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN6"},
  {DIN7_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN7"},
  {DIN8_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN8"},
  {DIN9_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN9"},
  {DIN10_GPIO_NUM, GPIOF_OUT_INIT_LOW, "DIN10"},
} ; 


#define DAC7311_CLK_BIT 14
#define REG_DAT_START_BIT 18
static void __iomem* dac7311_reg;
static volatile unsigned long reg_output;

static unsigned short dac7311_outputs[10];


static int dac7311_get_register(void)
{
  struct gpio_desc* gpio_dac;
  struct gpio_chip* chip;
  struct sunxi_pinctrl* pctl;
  gpio_dac = gpio_to_desc(SYNC1_GPIO_NUM);
  printk("find dac7311 gpio: %s-%s\n",gpio_dac->label,gpio_dac->name);
  chip = gpiod_to_chip(gpio_dac);
  pctl = gpiochip_get_data(chip);
  dac7311_reg = pctl->membase + sunxi_data_reg(gpio_chip_hwgpio(gpio_dac));
  return 0;
};

static int dac7311_port_init(void)
{
  int n;
  n = gpio_request_array(dac7311_gpios,ARRAY_SIZE(dac7311_gpios));

  if (n)
    pr_err("DAC7311 port init fail = %d.\n",n);
  else
   {
    gpio_free_array(dac7311_gpios,ARRAY_SIZE(dac7311_gpios));
    n = dac7311_get_register();
    }
  return n;
};



static inline void assemble_bits(int i)
{
  int n;
  unsigned short mask;
  mask = 1UL << i;
  reg_output=0UL;
  for(n =0 ; n < 10; n++)
    {
      if ( dac7311_outputs[i] & mask)
        __set_bit(REG_DAT_START_BIT+n,&reg_output);
    }    
  
};


static inline void set_dac7311_output(void)
{
  int i;
  
  for(i=1;i>0;i--)
    {
      assemble_bits(i);
      __set_bit(DAC7311_CLK_BIT,&reg_output);
      writel(reg_output,dac7311_reg);
      __clear_bit(DAC7311_CLK_BIT,&reg_output);
      writel(reg_output,dac7311_reg);
    }
  
};

static int wave_pulse(void* data)
{
  volatile unsigned long i;
  struct timespec next;
  struct timespec now;
  next = current_kernel_time();
  now = current_kernel_time();
  i=0xffffffff;
  next.tv_nsec += 100;
  next.tv_sec += next.tv_nsec / 1000000000;
  next.tv_nsec = next.tv_nsec % 1000000000;
  printk("now:\t%u:%u\n",now.tv_sec,now.tv_nsec);
  printk("next:\t%u.%u\n",next.tv_sec,next.tv_nsec);
  do
    {
      do
        {
          now = current_kernel_time();
         }
      while((now.tv_sec <= next.tv_sec) && (now.tv_nsec < next.tv_nsec));
        
      
      //udelay(3);
      //i = __raw_readl(reg_dat);
      //printk("read [%x]\n",i);
      //__change_bit(3,reg_dat);
      i ^= 0x08;
      //printk("write [%x]\n",i);
      //__raw_writew(i,pg_reg_dat);
      //gpio_set_value(355,i);;
       writel(i,reg_dat);
      //chip->set(chip,3,i);
       set_dac7311_output();  
      //i = ~i;
      //set_current_state(TASK_INTERRU;
      //schedule();
      next.tv_nsec+=100;
      next.tv_sec += next.tv_nsec / 1000000000;
      next.tv_nsec = next.tv_nsec % 1000000000;
      //printk("next\n");
      //printk("now:\t%u.%09u\n",now.tv_sec,now.tv_nsec);
      //printk("next:\t%u.%u\n",next.tv_sec,next.tv_nsec);
      
    }while(!kthread_should_stop());
  return 0;
}

static int thread_init(void)
{
  char name[]="pulse_wave";
  printk("in thread\n");
  thread1 = kthread_create(wave_pulse,NULL,name);
  if (thread1)
    {
      printk(KERN_INFO "run thread\n");
      wake_up_process(thread1);
    }
  else
    {
      printk(KERN_INFO "thread fail\n");
    }
  return 0;
}

static void thread_cleanup(void)
{
  int ret;
  ret = kthread_stop(thread1);
  if (!ret)
    printk(KERN_INFO "therad stopped");
}

static int dev_port_init(void)
{
  int n = 0;
  n = gpio_request_array(dac0800_gpios, ARRAY_SIZE(dac0800_gpios));
  if (n)
    printk(KERN_ALERT "gpio request error.\n");
  else
    gpio_free_array(dac0800_gpios,ARRAY_SIZE(dac0800_gpios));
  return n;
}



static int start(void)
{
  return 0;
}

static int stop(void)
{
  return 0;
}

static int dev_sg_open(struct inode *inode, struct file *filp) {
  printk("<1>DEV_SG_: open\n");
  return 0;
}

static int dev_sg_close(struct inode *inode, struct file *filp) {
  printk("<1>DEV_SG_: close\n");
  return 0;
}

static ssize_t dev_sg_read(struct file *filp, char *buf, size_t size, loff_t *f_pos) {
  printk("<1>DEV_SG_: read  (size=%zu)\n", size);
  return 0;
}

static ssize_t dev_sg_write(struct file *filp, const char *buf, size_t size, loff_t *f_pos) {
  printk("<1>DEV_SG_: write  (size=%zu)\n", size);
  return size;
}

static long dev_sg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg )
{
  switch(cmd)
    {
    case SG_CMD_START:
      return start();
      break;
    case SG_CMD_STOP:
      return stop();
      break;
    default:
      return -EINVAL;
    }
  return -EINVAL;
}

static struct file_operations dev_sg_fops = {
  .open = dev_sg_open,
  .release = dev_sg_close,
  .read = dev_sg_read,
  .write = dev_sg_write,
  .unlocked_ioctl = dev_sg_ioctl,
};


static int signal_generator_init(void)
{
  int result = 0;

  printk("<1>Inserting sg module\n");
  result = dev_port_init();
  result = alloc_chrdev_region(&dev,0,1,SIG_GEN_NAME);
  if (0 > result)    {
    printk(KERN_ALERT "Device Registration failed.\n");
    return result;
  }
  pulse_major = MAJOR(dev);
  
  pulse_class = class_create(THIS_MODULE,"chardrv");
  if (IS_ERR(pulse_class)) {
    result = PTR_ERR(pulse_class);
    printk(KERN_ALERT "class_create fail\n");
    goto class_fail;
  }

  if (NULL == device_create(pulse_class,NULL,dev,NULL,SIG_GEN_NAME)){
    result = -1;
    printk(KERN_ALERT "device_create fail.\n");
    goto device_fail;
  }

  cdev_init(&c_dev,&dev_sg_fops);
  
  if (cdev_add(&c_dev,dev,1) == -1){
    printk(KERN_ALERT "cdev_add fail.\n");
    result = -1;
    device_destroy(pulse_class,dev);
  }
  else
    goto success;
  

 device_fail:
  class_destroy(pulse_class);  
 class_fail:
  unregister_chrdev_region(dev,1);
 success:
  request_register();
  //thread_init();
  pulse_timer_init();
  return result;
}
 
static void waves_dac7311_exit(void)
{
  printk("<1>Exiting sg module\n");
  thread_cleanup();

  /* Unregister character device */
  cdev_del(&c_dev);
  device_destroy(pulse_class,dev);
  class_destroy(pulse_class);
  unregister_chrdev_region(dev,1);
  
  release_register();
  
  return;
}



MODULE_LICENSE("Dual BSD/GPL");
 
module_init(waves_dac7311_init);
module_exit(waves_dac7311_exit);
