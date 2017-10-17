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

static struct ring_buff {
	spinlock_t lock;
        unsigned int  msg_buf[ELEM];
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
		if (rbuff.rd_ptr >= rbuff.msg_buf + ELEM) {
                	rbuff.rd_ptr = rbuff.msg_buf;
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
	if (rbuff.wr_ptr >= rbuff.msg_buf + ELEM) {
		rbuff.wr_ptr = rbuff.msg_buf;
		rbuff.full = 1;
        }
        /* Ringbuffer*/
        if (rbuff.rd_ptr >= rbuff.msg_buf + ELEM) {
       		rbuff.rd_ptr = rbuff.msg_buf;
	}
	spin_unlock(&rbuff.lock);
}

/* Set sysfs attributes */
static struct kobj_attribute kobj_attribute =__ATTR(buffer, 0400,
						    imx_rpmsg_show, NULL);

static int imx_rpmsg_probe(struct rpmsg_channel *rpdev)
{
        int err;
   	/* Create sysfs file */
        if (sysfs_create_file(&rpdev->dev.kobj,&kobj_attribute.attr)) {
                dev_err(&rpdev->dev, "Cannot create group!\n");
                return -ENOMEM;
        }

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

static void imx_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	sysfs_remove_file(&rpdev->dev.kobj,&kobj_attribute.attr);
}
static struct rpmsg_driver imx_rpmsg_driver = {
        .drv.name   = KBUILD_MODNAME,
        .drv.owner  = THIS_MODULE,
        .id_table   = imx_rpmsg_id_table,
        .probe      = imx_rpmsg_probe,
        .callback   = imx_rpmsg_callback,
        .remove     = imx_rpmsg_remove,
};


static int __init imx_rpmsg_init(void)
{
        /* Register RPMsg driver */
        if (register_rpmsg_driver(&imx_rpmsg_driver))
                return -ENOMEM;

	rbuff.rd_ptr = rbuff.msg_buf;
	rbuff.wr_ptr = rbuff.msg_buf;
	rbuff.full = 0;

        return 0;
}
module_init(imx_rpmsg_init);

static void __exit imx_rpmsg_exit(void)
{
        unregister_rpmsg_driver(&imx_rpmsg_driver);
}
module_exit(imx_rpmsg_exit);

MODULE_AUTHOR("F&S Elektronik Systeme");
MODULE_DESCRIPTION("Driver for accessing RPMsg content via sysfs");
MODULE_LICENSE("GPL");
