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

MODULE_AUTHOR("kricnam<me.xinxin@gmail.com>");

#define SIG_GEN_NAME "pulse"
#define SG_CMD_START 0
#define SG_CMD_STOP  1
 
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
  return result;
}
 
static void signal_generator_exit(void)
{
  printk("<1>Exiting sg module\n");
  /* Unregister character device */
  cdev_del(&c_dev);
  device_destroy(pulse_class,dev);
  class_destroy(pulse_class);
  unregister_chrdev_region(dev,1);
    
  //unregister_chrdev(SIG_GEN_MAJOR, SIG_GEN_NAME); 
  return;
}



MODULE_LICENSE("Dual BSD/GPL");
 
module_init(signal_generator_init);
module_exit(signal_generator_exit);
