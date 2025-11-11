// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 STMicroelectronics - All Rights Reserved
 *
 * Copyright (C) 2025 F&S Elektronik Systeme GmbH
 *
 * The rpmsg tty driver implements serial communication on the RPMsg bus to makes
 * possible for user-space programs to send and receive rpmsg messages as a standard
 * tty protocol.
 *
 * The remote processor can instantiate a new tty by requesting a "rpmsg-tty" RPMsg service.
 * The "rpmsg-tty" service is directly used for data exchange. No flow control is implemented yet.
 */

#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/io.h>
#include <linux/imx_rpmsg.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#define TTY_RPMSG_TIMEOUT_MS		500

#define TTY_RPMSG_CATEGORY		0x0b
#define TTY_RPMSG_VERSION		0x1
#define TTY_RPMSG_TYPE_REQUEST		0x0
#define TTY_RPMSG_TYPE_RESPONSE         0x1
#define TTY_RPMSG_TYPE_NOTIFICATION     0x2
#define TTY_RPMSG_COMMAND_INIT		0x0
#define TTY_RPMSG_COMMAND_DEINIT	0x1
#define TTY_RPMSG_COMMAND_WRITE		0x2
#define TTY_RPMSG_COMMAND_READ		0x3
#define TTY_RPMSG_COMMAND_STOP		0x4
#define TTY_RPMSG_COMMAND_SET_BAUD	0x5

#define RPMSG_TTY_NAME	"ttyRPMSG"
#define MAX_TTY_RPMSG	32

static DEFINE_IDR(tty_idr);	/* tty instance id */
static DEFINE_MUTEX(idr_lock);	/* protects tty_idr */

static struct tty_driver *rpmsg_tty_driver;

struct tty_rpmsg_msg {
	struct imx_rpmsg_head header;
	uint8_t bus_id;			/* Bus ID for the tty port */
	uint8_t ctrl;			/* Flow control: 0 = none, 1 = CRTSCTS */
	uint32_t baudrate;		/* Baudrate in bps */
	int8_t len; 			/* Length of the data in buf */
	uint8_t buf[MAX_TTY_RPMSG];	/* Data buffer for the message */
} __packed __aligned(1);

struct tty_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct mutex lock;
	struct completion cmd_complete;
};

static struct tty_rpmsg_info tty_rpmsg = {0};

struct rpmsg_tty_port {
	struct tty_port		port;	 /* TTY port data */
	int			id;	 /* TTY rpmsg index */
};

static int rpmsg_tty_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src)
{
	struct tty_rpmsg_msg *rmsg = data;
	struct rpmsg_tty_port *cport = NULL;
	int copied;

	if (rmsg->header.type != TTY_RPMSG_TYPE_NOTIFICATION)
		return -EINVAL;

	/* Handle RX data */
	if (rmsg->header.cmd == TTY_RPMSG_COMMAND_READ) {
		cport = idr_find(&tty_idr, rmsg->bus_id);
		if (!cport) {
			dev_err(&rpdev->dev, "No tty port found for bus_id %d\n", rmsg->bus_id);
			return -ENODEV;
		}

		copied = tty_insert_flip_string(&cport->port, rmsg->buf, rmsg->len);

		if (copied != rmsg->len)
		dev_warn(&rpdev->dev, "Trunc buffer: available space is %d\n", copied);

		tty_flip_buffer_push(&cport->port);
		return 0;
	}

	/* ACK command INIT */
	if (rmsg->header.cmd == TTY_RPMSG_COMMAND_INIT ||
			rmsg->header.cmd == TTY_RPMSG_COMMAND_DEINIT ||
			rmsg->header.cmd == TTY_RPMSG_COMMAND_WRITE) {
		complete(&tty_rpmsg.cmd_complete);
		return 0;
	}

	return -EINVAL;
}

static int rpmsg_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct rpmsg_tty_port *cport = idr_find(&tty_idr, tty->index);
	struct tty_port *port;

	tty->driver_data = cport;

	port = tty_port_get(&cport->port);
	return tty_port_install(port, driver, tty);
}

static void rpmsg_tty_cleanup(struct tty_struct *tty)
{
	tty_port_put(tty->port);
}

static int rpmsg_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct rpmsg_tty_port *cport = tty->driver_data;
	int ret;

	if ( !cport || !tty_rpmsg.rpdev || !tty_rpmsg.rpdev->ept) {
		pr_err("rpmsg_tty_open: invalid rpdev or endpoint\n");
		return -ENODEV;
	}

	if ((filp->f_flags & O_ACCMODE) == O_RDONLY ||
			(filp->f_flags & O_ACCMODE) == O_RDWR) {

		struct tty_rpmsg_msg rmsg;

		memset(&rmsg, 0, sizeof(struct tty_rpmsg_msg));
		rmsg.header.cate  = TTY_RPMSG_CATEGORY;
		rmsg.header.major = TTY_RPMSG_VERSION & 0xFF;
		rmsg.header.minor = TTY_RPMSG_VERSION >> 8;
		rmsg.header.type  = TTY_RPMSG_TYPE_REQUEST;
		rmsg.header.cmd   = TTY_RPMSG_COMMAND_READ;
		rmsg.bus_id       = cport->id;

		mutex_lock(&tty_rpmsg.lock);
		ret = rpmsg_send(tty_rpmsg.rpdev->ept, &rmsg, sizeof(struct tty_rpmsg_msg));
		mutex_unlock(&tty_rpmsg.lock);
		if (ret){
			dev_err(&tty_rpmsg.rpdev->dev, "Failed to open tty: %d\n", ret);
			return ret;
		}
	}

	return tty_port_open(tty->port, tty, filp);
}

static void rpmsg_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct rpmsg_tty_port *cport = tty->driver_data;
	struct tty_rpmsg_msg rmsg;
	int ret;

	if (!cport || !tty_rpmsg.rpdev || !tty_rpmsg.rpdev->ept)
		return;

	memset(&rmsg, 0, sizeof(struct tty_rpmsg_msg));
	rmsg.header.cate  = TTY_RPMSG_CATEGORY;
	rmsg.header.major = TTY_RPMSG_VERSION & 0xFF;
	rmsg.header.minor = TTY_RPMSG_VERSION >> 8;
	rmsg.header.type  = TTY_RPMSG_TYPE_REQUEST;
	rmsg.header.cmd   = TTY_RPMSG_COMMAND_STOP;
	rmsg.bus_id       = cport->id;

	mutex_lock(&tty_rpmsg.lock);
	ret = rpmsg_send(tty_rpmsg.rpdev->ept, &rmsg, sizeof(struct tty_rpmsg_msg));
	mutex_unlock(&tty_rpmsg.lock);
	if (ret)
		dev_err(&tty_rpmsg.rpdev->dev, "Failed to send COMMAND_STOP: %d\n", ret);

	return tty_port_close(tty->port, tty, filp);
}

static ssize_t rpmsg_tty_write(struct tty_struct *tty, const u8 *buf,
			       size_t len)
{
	struct rpmsg_tty_port *cport = tty->driver_data;
	struct tty_rpmsg_msg rmsg;
	size_t rlen = len;
	const u8 *pbuf = buf;
	int ret;

	if (!tty_rpmsg.rpdev || !tty_rpmsg.rpdev->ept || len == 0) {
		pr_err("rpmsg_tty_write: invalid rpdev or endpoint\n");
		return 0;
	}

	while(rlen > 0) {
		memset(&rmsg, 0, sizeof(struct tty_rpmsg_msg));
		rmsg.header.cate  = TTY_RPMSG_CATEGORY;
		rmsg.header.major = TTY_RPMSG_VERSION & 0xFF;
		rmsg.header.minor = TTY_RPMSG_VERSION >> 8;
		rmsg.header.type  = TTY_RPMSG_TYPE_REQUEST;
		rmsg.header.cmd   = TTY_RPMSG_COMMAND_WRITE;
		rmsg.bus_id       = cport->id;
		rmsg.len = rlen > MAX_TTY_RPMSG ? MAX_TTY_RPMSG : rlen;

		memcpy(rmsg.buf, pbuf, rmsg.len);
		rlen -= rmsg.len;
		pbuf = (void *)pbuf + rmsg.len;

		mutex_lock(&tty_rpmsg.lock);
		reinit_completion(&tty_rpmsg.cmd_complete);
		rpmsg_send(tty_rpmsg.rpdev->ept, &rmsg, sizeof(struct tty_rpmsg_msg));
		ret = wait_for_completion_timeout(&tty_rpmsg.cmd_complete,
				msecs_to_jiffies(TTY_RPMSG_TIMEOUT_MS));
		mutex_unlock(&tty_rpmsg.lock);
		if (!ret) {
			pr_err("timeout waiting for rpmsg init response\n");
			return 0;
		}
	}

	return len;
}

static unsigned int rpmsg_tty_write_room(struct tty_struct *tty)
{
	return MAX_TTY_RPMSG;
}

static void rpmsg_tty_hangup(struct tty_struct *tty)
{
	tty_port_hangup(tty->port);
}

static void rpmsg_tty_set_termios(struct tty_struct *tty, const struct ktermios *old)
{
	struct rpmsg_tty_port *cport = tty->driver_data;
	uint32_t baud = tty_get_baud_rate(tty);
	struct tty_rpmsg_msg rmsg;

	memset(&rmsg, 0, sizeof(struct tty_rpmsg_msg));
	rmsg.header.cate  = TTY_RPMSG_CATEGORY;
	rmsg.header.major = TTY_RPMSG_VERSION & 0xFF;
	rmsg.header.minor = TTY_RPMSG_VERSION >> 8;
	rmsg.header.type  = TTY_RPMSG_TYPE_REQUEST;
	rmsg.header.cmd   = TTY_RPMSG_COMMAND_SET_BAUD;
	rmsg.bus_id       = cport->id;
	rmsg.ctrl = !!(tty->termios.c_cflag & CRTSCTS);
	rmsg.baudrate = baud;
	rmsg.len = 0;

	mutex_lock(&tty_rpmsg.lock);
	int ret = rpmsg_send(tty_rpmsg.rpdev->ept, &rmsg, sizeof(struct tty_rpmsg_msg));
	mutex_unlock(&tty_rpmsg.lock);
	if (ret)
		dev_err(&tty_rpmsg.rpdev->dev, "Failed to set baudrate: %d\n", ret);
}

static const struct tty_operations rpmsg_tty_ops = {
	.install	= rpmsg_tty_install,
	.open		= rpmsg_tty_open,
	.close		= rpmsg_tty_close,
	.write		= rpmsg_tty_write,
	.write_room	= rpmsg_tty_write_room,
	.hangup		= rpmsg_tty_hangup,
	.cleanup	= rpmsg_tty_cleanup,
	.set_termios	= rpmsg_tty_set_termios,
};

static void rpmsg_tty_destruct_port(struct tty_port *port)
{
	struct rpmsg_tty_port *cport = container_of(port, struct rpmsg_tty_port, port);

	mutex_lock(&idr_lock);
	idr_remove(&tty_idr, cport->id);
	mutex_unlock(&idr_lock);

	kfree(cport);
}

static const struct tty_port_operations rpmsg_tty_port_ops = {
	.destruct = rpmsg_tty_destruct_port,
};

static int tty_rpchip_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rpmsg_tty_port *cport;
	struct device *tty_dev;
	struct tty_rpmsg_msg rmsg = {0};
	int alias_id;
	int ret;

	if (!tty_rpmsg.rpdev || !tty_rpmsg.rpdev->ept)
		return -EPROBE_DEFER;

	alias_id = of_alias_get_id(np, "serial");
	if (alias_id < 0) {
		dev_err(dev, "invalid or missing tty alias id: %d\n", alias_id);
		return -EINVAL;
	}

	cport = devm_kzalloc(&pdev->dev, sizeof(struct rpmsg_tty_port), GFP_KERNEL);
	if (!cport)
		return -ENOMEM;

	cport->id = alias_id;

	/* Announce TTY-Port to Remote-Processor */
	mutex_lock(&tty_rpmsg.lock);
	reinit_completion(&tty_rpmsg.cmd_complete);
	rmsg.header.cate  = TTY_RPMSG_CATEGORY;
	rmsg.header.major = TTY_RPMSG_VERSION & 0xFF;
	rmsg.header.minor = TTY_RPMSG_VERSION >> 8;
	rmsg.header.type  = TTY_RPMSG_TYPE_REQUEST;
	rmsg.header.cmd   = TTY_RPMSG_COMMAND_INIT;
	rmsg.bus_id       = cport->id;

	rpmsg_send(tty_rpmsg.rpdev->ept, &rmsg, sizeof(struct tty_rpmsg_msg));
	ret = wait_for_completion_timeout(&tty_rpmsg.cmd_complete,
					  msecs_to_jiffies(TTY_RPMSG_TIMEOUT_MS));
	mutex_unlock(&tty_rpmsg.lock);
	if(!ret){
		dev_err(dev, "timeout waiting for rpmsg init response\n");
		return -ETIMEDOUT;
	}

	/* init port */
	mutex_lock(&idr_lock);
	ret = idr_alloc(&tty_idr, cport, alias_id, alias_id + 1, GFP_KERNEL);
	mutex_unlock(&idr_lock);
	if (ret < 0) {
		dev_err(dev, "failed to register tty id %d in idr: %d\n", alias_id, ret);
		return ret;
	}

	tty_port_init(&cport->port);
	cport->port.ops = &rpmsg_tty_port_ops;

	platform_set_drvdata(pdev, cport);

	tty_dev = tty_port_register_device(&cport->port, rpmsg_tty_driver,
					   cport->id, dev);
	if (IS_ERR(tty_dev)) {
		ret = PTR_ERR(tty_dev);
		dev_err(dev, "failed to register tty device: %d\n", ret);
		tty_port_put(&cport->port);

		mutex_lock(&idr_lock);
		idr_remove(&tty_idr, cport->id);
		mutex_unlock(&idr_lock);
		devm_kfree(&pdev->dev, cport);
		return ret;
	}

	dev_info(dev, "rpmsg tty device registered: ttyRPMSG%d\n", cport->id);

	return 0;
}

static const struct of_device_id imx_rpmsg_tty_dt_ids[] = {
	{ .compatible = "fus,tty-rpchip", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_tty_dt_ids);

static int tty_rpmsg_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rpmsg_tty_port *cport = platform_get_drvdata(pdev);
	struct tty_rpmsg_msg rmsg = {0};
	int ret;

	if (!cport || !tty_rpmsg.rpdev || !tty_rpmsg.rpdev->ept) {
		dev_err(dev, "%s: invalid rpdev or endpoint\n", __func__);
		return -ENODEV;
	}

	rmsg.header.cate  = TTY_RPMSG_CATEGORY;
	rmsg.header.major = TTY_RPMSG_VERSION & 0xFF;
	rmsg.header.minor = TTY_RPMSG_VERSION >> 8;
	rmsg.header.type  = TTY_RPMSG_TYPE_REQUEST;
	rmsg.header.cmd   = TTY_RPMSG_COMMAND_DEINIT;
	rmsg.bus_id       = cport->id;

	mutex_lock(&tty_rpmsg.lock);
	reinit_completion(&tty_rpmsg.cmd_complete);
	rpmsg_send(tty_rpmsg.rpdev->ept, &rmsg, sizeof(struct tty_rpmsg_msg));
	ret = wait_for_completion_timeout(&tty_rpmsg.cmd_complete,
		msecs_to_jiffies(TTY_RPMSG_TIMEOUT_MS));
	mutex_unlock(&tty_rpmsg.lock);
	if (!ret) {
		dev_err(dev, "timeout waiting for rpmsg deinit response\n");
		return -ETIMEDOUT;
	}

	return 0;
};

static int tty_rpmsg_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rpmsg_tty_port *cport = platform_get_drvdata(pdev);
	struct tty_rpmsg_msg rmsg = {0};
	int ret;

	if (!cport || !tty_rpmsg.rpdev || !tty_rpmsg.rpdev->ept) {
		dev_err(dev, "%s: invalid rpdev or endpoint\n", __func__);
		return -ENODEV;
	}

	rmsg.header.cate  = TTY_RPMSG_CATEGORY;
	rmsg.header.major = TTY_RPMSG_VERSION & 0xFF;
	rmsg.header.minor = TTY_RPMSG_VERSION >> 8;
	rmsg.header.type  = TTY_RPMSG_TYPE_REQUEST;
	rmsg.header.cmd   = TTY_RPMSG_COMMAND_INIT;
	rmsg.bus_id       = cport->id;

	mutex_lock(&tty_rpmsg.lock);
	reinit_completion(&tty_rpmsg.cmd_complete);
	rpmsg_send(tty_rpmsg.rpdev->ept, &rmsg, sizeof(struct tty_rpmsg_msg));
	ret = wait_for_completion_timeout(&tty_rpmsg.cmd_complete,
		msecs_to_jiffies(TTY_RPMSG_TIMEOUT_MS));
	mutex_unlock(&tty_rpmsg.lock);
	if (!ret) {
		dev_err(dev, "timeout waiting for rpmsg init response\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static const struct dev_pm_ops rpmsg_tty_pm_ops = {
	RUNTIME_PM_OPS(tty_rpmsg_suspend,
			   tty_rpmsg_resume, NULL)
	NOIRQ_SYSTEM_SLEEP_PM_OPS(tty_rpmsg_suspend,
				     tty_rpmsg_resume)
	SYSTEM_SLEEP_PM_OPS(tty_rpmsg_suspend, tty_rpmsg_resume)
};

static struct platform_driver imx_rpmsg_tty_driver = {
	.driver = {
		.name	= "rpmsg_tty",
		.of_match_table = imx_rpmsg_tty_dt_ids,
		.pm	= &rpmsg_tty_pm_ops,
	},
	.probe	= tty_rpchip_probe,
};

static int rpmsg_tty_probe(struct rpmsg_device *rpdev)
{
	int ret = 0;

	if (!rpdev) {
		dev_info(&rpdev->dev, "%s failed, rpdev=NULL\n", __func__);
		return -EINVAL;
	}

	mutex_init(&tty_rpmsg.lock);
	init_completion(&tty_rpmsg.cmd_complete);

	tty_rpmsg.rpdev = rpdev;
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
						rpdev->src, rpdev->dst);

	return ret;
}

static void rpmsg_tty_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "removing channel: 0x%x -> 0x%x!\n",
						rpdev->src, rpdev->dst);
}

static struct rpmsg_device_id rpmsg_driver_tty_id_table[] = {
	{ .name	= "rpmsg-tty-channel" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_tty_id_table);

static struct rpmsg_driver rpmsg_tty_rpmsg_drv = {
	.drv.name	= "rpmsg_tty",
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_tty_id_table,
	.probe		= rpmsg_tty_probe,
	.callback	= rpmsg_tty_cb,
	.remove		= rpmsg_tty_remove,
};

static int __init rpmsg_tty_init(void)
{
	int ret = 0;

	rpmsg_tty_driver = tty_alloc_driver(MAX_TTY_RPMSG, TTY_DRIVER_REAL_RAW |
					    TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(rpmsg_tty_driver))
		return PTR_ERR(rpmsg_tty_driver);

	rpmsg_tty_driver->driver_name = "rpmsg_tty";
	rpmsg_tty_driver->name = RPMSG_TTY_NAME;
	rpmsg_tty_driver->major = 0;
	rpmsg_tty_driver->type = TTY_DRIVER_TYPE_CONSOLE;

	/* Disable unused mode by default */
	rpmsg_tty_driver->init_termios = tty_std_termios;
	rpmsg_tty_driver->init_termios.c_lflag &= ~(ECHO | ICANON);
	rpmsg_tty_driver->init_termios.c_iflag |= ICRNL;
	rpmsg_tty_driver->init_termios.c_oflag |= (OPOST | ONLCR);

	tty_set_operations(rpmsg_tty_driver, &rpmsg_tty_ops);

	ret = tty_register_driver(rpmsg_tty_driver);
	if (ret < 0) {
		pr_err("Couldn't install driver: %d\n", ret);
		tty_driver_kref_put(rpmsg_tty_driver);
		return ret;
	}

	ret = register_rpmsg_driver(&rpmsg_tty_rpmsg_drv);
	if (ret < 0) {
		pr_err("Couldn't register rpmsg driver: %d\n", ret);
		tty_unregister_driver(rpmsg_tty_driver);
		tty_driver_kref_put(rpmsg_tty_driver);
		return ret;
	}

	return platform_driver_register(&imx_rpmsg_tty_driver);
}

static void __exit rpmsg_tty_exit(void){
	platform_driver_unregister(&imx_rpmsg_tty_driver);
	unregister_rpmsg_driver(&rpmsg_tty_rpmsg_drv);
	tty_unregister_driver(rpmsg_tty_driver);
 	tty_driver_kref_put(rpmsg_tty_driver);
	idr_destroy(&tty_idr);
}

module_init(rpmsg_tty_init);
module_exit(rpmsg_tty_exit);

MODULE_AUTHOR("Arnaud Pouliquen <arnaud.pouliquen@foss.st.com>");
MODULE_DESCRIPTION("remote processor messaging tty driver");
MODULE_LICENSE("GPL v2");
