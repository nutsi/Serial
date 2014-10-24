#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/interrupt.h>

#define UART_ADDR_BASE 0x03F8

#define UART_THR (UART_ADDR_BASE + 0)
#define UART_RBR (UART_ADDR_BASE + 0)
#define UART_DLL (UART_ADDR_BASE + 0)
#define UART_IER (UART_ADDR_BASE + 1)
#define UART_DLH (UART_ADDR_BASE + 1)
#define UART_IIR (UART_ADDR_BASE + 2)
#define UART_FCR (UART_ADDR_BASE + 2)
#define UART_LCR (UART_ADDR_BASE + 3)
#define UART_MCR (UART_ADDR_BASE + 4)
#define UART_LSR (UART_ADDR_BASE + 5)
#define UART_MSR (UART_ADDR_BASE + 6)
#define UART_SR  (UART_ADDR_BASE + 7)

#define UART_MOD_MAJOR  4242

wait_queue_head_t wq_recv;
wait_queue_head_t wq_xmit;

char module_name[] = "serial_epitech";
static int mod_major = 0;
static struct class *serial_class = NULL;
static struct cdev serial_dev;
static dev_t dev = 0;


ssize_t serial_dev_read(struct file *file, char *buf, size_t len, loff_t *pos);
ssize_t serial_dev_write(struct file *file, const char *buf, size_t len, loff_t *pos);
int serial_device_ioctl(struct inode *inode, struct file *file,
    unsigned int ioctl_num, unsigned long ioctl_param);

static struct file_operations fops = {
  .compat_ioctl = serial_device_ioctl,
  .write = serial_dev_write,
  .read = serial_dev_read,
  .owner = THIS_MODULE,
};

irqreturn_t handle_uart_irq(int irq, void *devinfo)
{
  return IRQ_HANDLED;
}

int serial_device_ioctl(struct inode *inode, struct file *file,
    unsigned int ioctl_num, unsigned long ioctl_param)
{
  switch (ioctl_num) {
    default:
      printk(KERN_WARNING "serial: %d ioctl request is not supported.\n", ioctl_num);
      return -1;
  };
  return 0;
}

ssize_t serial_dev_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
    int i = 0;
    unsigned char d;
    u8 lsr = inb(UART_LSR);

    while (i < len) {

      d = inb(UART_RBR);

      if (copy_to_user(buf+i, &d, 1)) {
        return -EFAULT;
      }
      lsr = inb(UART_LSR);
      i++;
    }
    return i;
}

ssize_t serial_dev_write(struct file *file, const char *buf, size_t len, loff_t *pos)
{
  u8 msr;
  int i = 0;
  unsigned char d;

  msr = inb(UART_MSR);

  if ((msr & 0x10) != 0x10) {
    return -1;
  }

  while (i < len) {
    if ((msr & 0x10) != 0x10)
      break;
    if (copy_from_user(&d, buf + i, 1))
      return -EFAULT;

    outb(d, UART_THR);
    i++;
  }
  return i;
}

static int __init   init_driver(void)
{
  int ret = 0;

  init_waitqueue_head(&wq_xmit);
  init_waitqueue_head(&wq_recv);
  printk(KERN_INFO "serial: Driver is loading.\n");

  outb(0x00, UART_IER); // Disable receiving data
  outb(0xC7, UART_FCR); // 11000111 enable FIFO and clear fifo buffer
  //outb(0x83, UART_LCR); // 8 bits word
  outb(0x01, UART_DLL);
  outb(0x00, UART_DLH);
  outb(0x03, UART_LCR); // 8 bits word
  outb(0x03, UART_MCR);

  if ((ret = request_irq(4, handle_uart_irq, IRQF_SHARED, module_name, &module_name)) < 0) {
    return ret;
  }

  outb(0x01, UART_IER);
  outb(0x0B, UART_MCR);


  ret = alloc_chrdev_region(&dev, 0, 1, module_name);
  if (ret < 0) {
    printk(KERN_WARNING "serial: alloc_chrdev_failed.\n");
    return ret;
  }
  mod_major = MAJOR(dev);
  serial_class = class_create(THIS_MODULE, module_name);
  if (IS_ERR(serial_class)) {
    ret = PTR_ERR(serial_class);
    return ret;
  }

  if (device_create(serial_class, NULL, dev, NULL, module_name) == NULL) {
    class_destroy(serial_class);
    unregister_chrdev_region(dev, 1);
    return -1;
  }
  cdev_init(&serial_dev, &fops);
  if (cdev_add(&serial_dev, dev, 1) < 0)
  {
    device_destroy(serial_class, dev);
    class_destroy(serial_class);
    unregister_chrdev_region(dev, 1);
    return -1;
  }
  printk(KERN_INFO "serial: Driver loaded.\n");
  return ret;


}

static void __exit exit_driver(void)
{
  printk(KERN_INFO "serial: Unloaded module.\n");
  device_destroy(serial_class, dev);
  class_destroy(serial_class);
  unregister_chrdev(dev, module_name);
  outb(0x00, UART_IER); // Disable receiving data
  free_irq(4, &module_name);
}

module_init(init_driver);
module_exit(exit_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("nuts");
