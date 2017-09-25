/*
 * Driver for RPMsg on iMX6SX
 *
 * Copyright (c) 2017 F&S Elektronik Systeme GmbH
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/virtio.h>
#include <linux/rpmsg.h>
#include <linux/spinlock.h>
#define MSG   "Discovery-Channel"
#define ELEM  (10)      /* Number of elements in Buffer (max 500) */

struct imx_rpmsg {
        unsigned int  msg_buf[ELEM];
        struct kobj_attribute *kattr;
        struct attribute_group attr_group;
        struct kobject *imx_rpmsg_kobj;
        struct platform_device *pdev;
} irpmsg;

struct ring_buff {
	spinlock_t lock;
	unsigned int * rd_ptr;
        unsigned int * wr_ptr;
        char full;
} rbuff;


/* Read function. */
static ssize_t imx_rpmsg_show(struct kobject *kobj,
                struct kobj_attribute *attr,
                char *buf)
{
	int pos = 0;
	buf[0] = '\0';
	/*Lock Semaphore so we can work on the ringbuffer */
	spin_lock(&rbuff.lock);
	/* Concersion from int array to string. The flag
 	 * indicates, that the buffer has been filled once */
	while(rbuff.rd_ptr != rbuff.wr_ptr || rbuff.full == 1 ) {
		pos += snprintf(buf + pos, PAGE_SIZE - pos, "%1d.%02dV",
                        (*(rbuff.rd_ptr) / 100),
                        (*(rbuff.rd_ptr) % 100));
		rbuff.rd_ptr++;
        	rbuff.full = 0;
	 /* Ringbuffer*/
		if (rbuff.rd_ptr >= irpmsg.msg_buf + ELEM) {
                	rbuff.rd_ptr = irpmsg.msg_buf;
         	}
		if (rbuff.rd_ptr != rbuff.wr_ptr)
			pos += snprintf(buf + pos, PAGE_SIZE - pos, ", ");
		else
			pos += snprintf(buf + pos, PAGE_SIZE - pos, "\n");
	}
	spin_unlock(&rbuff.lock);

        return pos;
}

static struct rpmsg_device_id imx_rpmsg_id_table[] = {
        { .name = "rpmsg-openamp-demo-channel" },
        { },
};
MODULE_DEVICE_TABLE(rpmsg, imx_rpmsg_id_table);

/* RPMsg part */
static void imx_rpmsg_callback(struct rpmsg_channel *rpdev,
                void *data,
                int len,
                void *priv,
                u32 src)
{
	spin_lock(&rbuff.lock);
	/* Move read-pointer to get only fresh values */
	if (rbuff.wr_ptr == rbuff.rd_ptr && rbuff.full == 1) {
                 rbuff.rd_ptr++;
         }
	/* Save data into an integer array */
	*rbuff.wr_ptr=*((unsigned int *) data);
	rbuff.wr_ptr++;

	/* Ringbuffer*/
	if (rbuff.wr_ptr >= irpmsg.msg_buf + ELEM) {
		rbuff.wr_ptr = irpmsg.msg_buf;
		rbuff.full = 1;
        }
        /* Ringbuffer*/
        if (rbuff.rd_ptr >= irpmsg.msg_buf + ELEM) {
       		rbuff.rd_ptr = irpmsg.msg_buf;
	}
	spin_unlock(&rbuff.lock);
}


static void imx_rpmsg_remove(struct rpmsg_channel *rpdev)
{
        dev_info(&rpdev->dev, "imx_rpmsg_driver is removed\n");
        if (irpmsg.imx_rpmsg_kobj) {
                kobject_put(irpmsg.imx_rpmsg_kobj);
                irpmsg.imx_rpmsg_kobj = NULL;
        }
}

static int imx_rpmsg_probe(struct rpmsg_channel *rpdev)
{
        int err;

        /* Inform Cortex-M4 about our existence; this will kick off the demo */
        dev_info(&rpdev->dev, "New channel: 0x%x -> 0x%x!\n",
                        rpdev->src, rpdev->dst);
        err = rpmsg_send(rpdev, MSG, strlen(MSG));
        if (err) {
                dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", err);
                return err;
        }
        return 0;
}

static struct rpmsg_driver imx_rpmsg_driver = {
        .drv.name   = KBUILD_MODNAME,
        .drv.owner  = THIS_MODULE,
        .id_table   = imx_rpmsg_id_table,
        .probe      = imx_rpmsg_probe,
        .callback   = imx_rpmsg_callback,
        .remove     = imx_rpmsg_remove,
};

static struct kobj_attribute kattr[] = {
        {
                .attr = {.name = "buffer", .mode = 0400 },
                .show = imx_rpmsg_show,
                .store = NULL,
        },
};

static int __init imx_rpmsg_init(void)
{
        /* Register RPMsg driver */
        if (register_rpmsg_driver(&imx_rpmsg_driver))
                return -ENOMEM;

        /* Alloc memory for attribute */
        memset(&irpmsg.attr_group, 0, sizeof(struct attribute_group));
        irpmsg.attr_group.attrs = kzalloc(2 * sizeof(struct attribute *),
                        GFP_KERNEL);
        if (!irpmsg.attr_group.attrs) {
                dev_err(&irpmsg.pdev->dev,
                                "Cannot allocate attribute pointers!\n");
                return -ENOMEM;
        }

        irpmsg.attr_group.attrs[0] = &kattr[0].attr;
        irpmsg.attr_group.attrs[1] = NULL;

        /* Create /sys/fs_rpmsg */
        irpmsg.imx_rpmsg_kobj = kobject_create_and_add("fs_rpmsg", NULL);
        if (!irpmsg.imx_rpmsg_kobj) {
                dev_err(&irpmsg.pdev->dev, "Cannot create kobject!\n");
                return -ENOMEM;
        }

        /* Create attributes */
        if (sysfs_create_group(irpmsg.imx_rpmsg_kobj, &irpmsg.attr_group)) {
                dev_err(&irpmsg.pdev->dev, "Cannot create group!\n");
                kobject_put(irpmsg.imx_rpmsg_kobj);
                irpmsg.imx_rpmsg_kobj = NULL;
                return -ENOMEM;
        }

	rbuff.rd_ptr = irpmsg.msg_buf;
	rbuff.wr_ptr = irpmsg.msg_buf;
	rbuff.full = 0;

        return 0;
}
module_init(imx_rpmsg_init);

static void __exit imx_rpmsg_exit(void)
{
        unregister_rpmsg_driver(&imx_rpmsg_driver);
}
module_exit(imx_rpmsg_exit);

MODULE_AUTHOR("Manuel-Tobias Csapo @F&S Elektronik Systeme <csapo@fs-net.de>");
MODULE_DESCRIPTION("Driver for accessing RPMsg content via sysfs");
MODULE_LICENSE("GPL");
