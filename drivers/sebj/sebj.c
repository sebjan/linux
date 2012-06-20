/*
 * test module
 */
#include <linux/init.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

static struct cdev sebj_cdev;
static dev_t sebj_dev;// = mkdev(255, 0);
static struct class * class_sebj;
static char mod_name[] = "sebj";
#define SEBJ_TRACE(fmt, ...) \
	printk(KERN_ERR "sebj chardev - %s - " pr_fmt(fmt), __func__, ##__VA_ARGS__)


static ssize_t sebj_read(struct file * filp, char * buffer, size_t count,
							loff_t * ppos)
{
	SEBJ_TRACE("read function called");
	*ppos += count;
	return 0;		/* number of transferred bytes */
}

extern void omap4_pm_off_mode_enable(int);

static ssize_t sebj_write(struct file *file, const char __user * buf,
						size_t count, loff_t *ppos)
{
	char var;
	/* right now, only consider the 1st char */
	copy_from_user(&var, buf, 1);
	switch (var) {
		case 's':
			omap4_pm_off_mode_enable(1);
		break;
	}

	SEBJ_TRACE("handled '%c' command", var);
	*ppos += count;
	return count;		/* number of transferred bytes */
}


static const struct file_operations sebj_fops = {
//	.owner = THIS_MODULE,
	.write = sebj_write,
	.read = sebj_read,
};

static int __init sebj_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&sebj_dev, 0, 1, mod_name);
	if (ret < 0)
		goto fail_chrdev;

	class_sebj = class_create(THIS_MODULE, "sebj_class");
	if (IS_ERR(class_sebj)) {
		ret = -EINVAL;
		goto fail_class;
	}
	device_create(class_sebj, NULL, sebj_dev,
				NULL, mod_name);

	cdev_init(&sebj_cdev, &sebj_fops);
	ret = cdev_add(&sebj_cdev, sebj_dev, 1);
	if (ret) {
		pr_err("Could not register chrdev\n");
		goto fail_cdev;
	}

	SEBJ_TRACE("sebj module initialized\n");

	return 0;

fail_cdev:
	device_destroy(class_sebj, sebj_dev);
	class_destroy(class_sebj);
fail_class:
	unregister_chrdev_region(sebj_dev, 1);
fail_chrdev:

	return ret;
}
module_init(sebj_init);

static void __exit sebj_exit(void)
{
	cdev_del(&sebj_cdev);
	device_destroy(class_sebj, sebj_dev);
	class_destroy(class_sebj);
	unregister_chrdev_region(sebj_dev, 1);
}
module_exit(sebj_exit);

MODULE_LICENSE("GPL");

