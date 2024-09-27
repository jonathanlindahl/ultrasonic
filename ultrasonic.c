#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/timekeeping.h>
#include <linux/math64.h>

#define GPIO_18_OUT (18)
#define GPIO_24_IN (24)

static ktime_t time_start;
static ktime_t time_end;

unsigned int gpio_irq_number;

static unsigned int valid_value;

dev_t dev = 0;
static struct class *dev_class;
static struct cdev usnc_cdev;
 
// TODO: put prototypes in a header
static int __init ultrasonic_init(void);
static void __exit ultrasonic_exit(void);

// Driver functions
static int usnc_open(struct inode *inode, struct file *file);
static int usnc_release(struct inode *inode, struct file *file);
static ssize_t usnc_read(struct file *filp, char __user *buf, size_t len, loff_t *off);
static ssize_t usnc_write(struct file *filp, const char *buf, size_t len, loff_t *off);
static irqreturn_t handle_gpio_irq(int irq, void *dev_id);

// fops structure
static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.read = usnc_read,
	.write = usnc_write,
	.open = usnc_open,
	.release = usnc_release,
};

// handle interrupts from GPIO_24_IN
static irqreturn_t handle_gpio_irq(int irq, void *dev_id)
{
	ktime_t ktime_temp;

	if (valid_value == 0) {
		ktime_temp = ktime_get();
		if (gpio_get_value(GPIO_24_IN) == 1) {
			time_start = ktime_temp;
		} else {
			time_end = ktime_temp;
			valid_value = 1;
		}
	}

	return IRQ_HANDLED;
}

static int usnc_open(struct inode *inode, struct file *file)
{
	pr_info("ultrasonic: device file opened\n");
	return 0;
}

static int usnc_release(struct inode *inode, struct file *file)
{
	pr_info("ultrasonic: device file closed\n");
	return 0;
}

static ssize_t usnc_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
	uint8_t gpio_state = 0;
	uint8_t gpio_in_state = 0;

	// read gpio value
	gpio_state = gpio_get_value(GPIO_18_OUT);
	gpio_in_state = gpio_get_value(GPIO_24_IN);

	// write to user
	len = 1;
	if (copy_to_user(buf, &gpio_state, len) > 0)
		pr_err("ultrasonic: ERROR: not all bytes have been copied to user\n");

	pr_info("ultrasonic: read: GPIO_18_OUT %d GPIO_24_IN: %d\n", gpio_state, gpio_in_state);

	return 0;
}

static ssize_t usnc_write(
	struct file *filp, const char __user *buf, size_t len, loff_t *off
) {
	uint8_t rec_buf[10] = {0};
	int counter;
	unsigned long long result;

	if (copy_from_user(rec_buf, buf, len) > 0)
		pr_err("ultrasonic: ERROR: not all bytes have been copied from user\n");

	if (rec_buf[0] == '1') {
		pr_info("ultrasonic: write: starting trigger\n");

		// trigger ultrasonic pulse
		gpio_set_value(GPIO_18_OUT, 1);
		udelay(10);
		gpio_set_value(GPIO_18_OUT, 0);

		valid_value = 0;

		counter = 0;
		while (valid_value == 0) {
			// out of range
			if (++counter > 35200) {
				pr_info("ultrasonic: write: counter timeout\n");
				return len;
			}
			udelay(1);
		}
	} else
		pr_err("ultrasonic: unknown command, must be 1\n");

	result = ktime_to_us(ktime_sub(time_end, time_start));
	pr_info("ultrasonic: write: RESULT: %lld\n", result);
	// calculate measurement in CM
	pr_info("ultrasonic: write: RESULT CM: %lld\n", div_u64(result, 58));

	return len;
}

static int __init ultrasonic_init(void)
{
	// allocate major number
	if ((alloc_chrdev_region(&dev, 0, 1, "usnc_dev")) < 0) {
		pr_err("ultrasonic: couldn't allocate major number");
		goto r_unreg;
	}
	pr_info("ultrasonic: major = %d minor = %d\n", MAJOR(dev), MINOR(dev));

	// create cdev structure
	cdev_init(&usnc_cdev, &fops);

	// add character device
	if ((cdev_add(&usnc_cdev, dev, 1)) < 0) {
		pr_err("ultrasonic: couldn't add device\n");
		goto r_del;
	}

	// create class
	if (IS_ERR(dev_class = class_create(THIS_MODULE, "usnc_class"))) {
		pr_err("ultrasonic: couldn't create struct class\n");
		goto r_class;
	}

	// create device
	if (IS_ERR(device_create(dev_class, NULL, dev, NULL, "usnc_device"))) {
		pr_err("ultrasonic: couldn't create device\n");
		goto r_device;
	}

	// Output GPIO configuration

	// check GPIO validity
	if (gpio_is_valid(GPIO_18_OUT) == false) {
		pr_err("ultrasonic: GPIO %d not valid\n", GPIO_18_OUT);
		goto r_device;
	}

	if (gpio_request(GPIO_18_OUT, "GPIO_18_OUT") < 0) {
		pr_err("ultrasonic: ERROR: GPIO %d request\n", GPIO_18_OUT);
		goto r_gpio_out;
	}

	// configure GPIO 18 as output
	gpio_direction_output(GPIO_18_OUT, 0);

	// Input GPIO configuration
	
	// check GPIO validity
	if (gpio_is_valid(GPIO_24_IN) == false) {
		pr_err("ultrasonic: ERROR: GPIO %d not valid\n", GPIO_24_IN);
		goto r_gpio_in;
	}

	if (gpio_request(GPIO_24_IN, "GPIO_24_IN") < 0) {
		pr_err("ultrasonic: ERROR: GPIO %d request\n", GPIO_24_IN);
		goto r_gpio_in;
	}

	// configure GPIO as input
	gpio_direction_input(GPIO_24_IN);

	gpio_irq_number = gpio_to_irq(GPIO_24_IN);
	pr_info("ultrasonic: gpio_irq_number = %d\n", gpio_irq_number);

	if (
		request_irq(
			gpio_irq_number,
			(void *)handle_gpio_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"usnc_device",
			NULL
		)
	) {
		pr_err("ultrasonic: cannot register IRQ\n");
		goto r_irq;
	}

	/* Exposes GPIO_18 to userspace through /sys/class/gpio
	 * It can now be used to change gpio values:
	 * echo 1 >> /sys/class/gpio/gpio18/value (sets pin high)
	 * echo 0 >> /sys/class/gpio/gpio18/value (sets pin low)
	 * cat /sys/class/gpio/gpio18/value (read current value)
	 *
	 * The second argument prevents the direction from being changed
	 *
	 * Commented as it was part of an older test version and not relevant for final version
	 */
	//gpio_export(GPIO_18, false);

	pr_info("ultrasonic: init done\n");
	return 0;

r_irq:
	free_irq(gpio_irq_number, NULL);
r_gpio_in:
	gpio_free(GPIO_24_IN);
r_gpio_out:
	gpio_free(GPIO_18_OUT);
r_device:
	device_destroy(dev_class, dev);
r_class:
	class_destroy(dev_class);
r_del:
	cdev_del(&usnc_cdev);
r_unreg:
	unregister_chrdev_region(dev, 1);

	return -1;
}

static void __exit ultrasonic_exit(void)
{
	free_irq(gpio_irq_number, NULL);
	gpio_free(GPIO_24_IN);
	//gpio_unexport(GPIO_18);
	gpio_free(GPIO_18_OUT);
	device_destroy(dev_class, dev);
	class_destroy(dev_class);
	cdev_del(&usnc_cdev);
	unregister_chrdev_region(dev, 1);
	pr_info("ultrasonic: driver removed\n");
}

module_init(ultrasonic_init);
module_exit(ultrasonic_exit);

MODULE_LICENSE("GPL");
