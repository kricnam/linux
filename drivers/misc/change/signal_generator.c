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
#include <asm/pgtable.h>


MODULE_AUTHOR("kricnam<me.xinxin@gmail.com>");

#define SIG_GEN_NAME "pulse"
#define SG_CMD_START 0
#define SG_CMD_STOP  1

#define REG_BASE 
#define REG_PG_DAT REG_BASE + 0xE8

static unsigned int pulse_major = 0;
static struct cdev c_dev;
static struct class * pulse_class;
static dev_t dev=0;

/* TODO: make de DTS compatitable code */
static struct gpio dac0800_gpios[] = {
  { 192, GPIOF_OUT_INIT_LOW, "D0" },
  { 193, GPIOF_OUT_INIT_LOW,  "D1" }, 
  { 194, GPIOF_OUT_INIT_LOW,  "D2"   },
  { 195, GPIOF_OUT_INIT_LOW,  "D3"  },
  { 196,GPIOF_OUT_INIT_LOW,"D4"},
  { 197,GPIOF_OUT_INIT_LOW,"D5"},
  { 198,GPIOF_OUT_INIT_LOW,"D6"},
  { 199,GPIOF_OUT_INIT_LOW,"D7"},
  { 200,GPIOF_OUT_INIT_LOW,"OE"},
  { 355,GPIOF_OUT_INIT_LOW,"A0"},
  { 356,GPIOF_OUT_INIT_LOW,"A1"},
  { 357,GPIOF_OUT_INIT_LOW,"A3"},
  { 358,GPIOF_OUT_INIT_LOW,"A4"},
  { 359,GPIOF_OUT_INIT_HIGH,"E0"},
  { 360,GPIOF_OUT_INIT_HIGH,"E1"},
};

static struct task_struct *thread1;
struct resource * res_pg_dat;
static void __iomem *   pg_reg_dat;
static struct gpio_desc * pgpio;

static int request_register(void)
{
  res_pg_dat = request_mem_region(REG_PG_DAT,SECTION_SIZE,"PG");
  if (!res_pg_dat)
    {
      pr_err("mm request fail\n");
      return -1;
      }
  pg_reg_dat = ioremap(REG_PG_DAT,SECTION_SIZE);
  if (!pg_reg_dat)
    {
      release_mem_region(REG_PG_DAT,SECTION_SIZE);
      pr_err("assign reg failed.");
      return -1;
      }

  printk("goto iomap %x\n",(int)pg_reg_dat);
  pgpio = gpio_to_desc(355);
  if (pgpio)
    printk("reg iomap:%x\n",(int)(gpiod_to_chip(pgpio)->reg_set));
  return 0;
}

static void release_register(void)
{
  if (pg_reg_dat)
    iounmap(pg_reg_dat);
  if (res_pg_dat)
    release_mem_region(REG_PG_DAT,SECTION_SIZE);
}

static int wave_pulse(void* data)
{
  u16 i;
  i=0xffff;
  do
    {
      udelay(2);
      //i = __raw_readw(pg_reg_dat);
      //printk("read [%x]\n",i);
      //__change_bit(3,&i);
      //printk("write [%x]\n",i);
      //__raw_writew(i,pg_reg_dat);
      //gpio_set_value(355,i);;
writew(i,pg_reg_dat);

      
i = ~i;
      //set_current_state(TASK_INTERRU;
      //schedule();
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
  thread_init();
  
  return result;
}
 
static void signal_generator_exit(void)
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
 
module_init(signal_generator_init);
module_exit(signal_generator_exit);
