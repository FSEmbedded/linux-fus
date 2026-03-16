// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 F&S Elektronik Systeme GmbH
 */

#define pr_fmt(fmt)             KBUILD_MODNAME ": " fmt
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/imx_rpmsg.h>

#define SPI_RPMSG_MAX_BUF_SIZE          475
#define SPI_RPMSG_TIMEOUT               500 /* ms */

#define SPI_RPMSG_CATEGORY              0x0C
#define SPI_RPMSG_VERSION               0x0001
#define SPI_RPMSG_TYPE_REQUEST          0x00
#define SPI_RPMSG_TYPE_RESPONSE         0x01
#define SPI_RPMSG_TYPE_NOTIFICATION     0x02
#define SPI_RPMSG_COMMAND_TRANSFER      0x01
#define SPI_RPMSG_FLAG_CPHA             BIT(0)
#define SPI_RPMSG_FLAG_CPOL             BIT(1)
#define SPI_RPMSG_FLAG_KEEP_CS          BIT(2)

struct spi_rpmsg_msg {
    struct imx_rpmsg_head header;
    u8 bus_id;
    u16 addr;
    u8 flags;
    u32 speed_hz;
    u16 len;
    u8 buf[SPI_RPMSG_MAX_BUF_SIZE];
} __packed __aligned(1);

struct spi_rpmsg {
    struct rpmsg_device *rpdev;
    struct device *dev;
    struct spi_controller *controller;
    struct completion cmd_complete;
    struct mutex lock;
    struct spi_rpmsg_msg rx_msg;
};

struct spi_rpmsg spi_rpmsg;

static int spi_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
                            void *priv, u32 src)
{
    struct spi_rpmsg *rmsg = dev_get_drvdata(&rpdev->dev);
    struct spi_rpmsg_msg *msg = data;

    if (!rmsg) {
        dev_err(&rpdev->dev, "Received NULL message\n");
        return -EINVAL;
    }

    if (msg->header.type != SPI_RPMSG_TYPE_RESPONSE && msg->header.type != SPI_RPMSG_TYPE_NOTIFICATION) {
        dev_err(&rpdev->dev, "Invalid message type: %d\n", msg->header.type);
        return -EINVAL;
    }

    if (msg->header.cmd == SPI_RPMSG_COMMAND_TRANSFER) {
        if (msg->len > SPI_RPMSG_MAX_BUF_SIZE) {
            dev_err(&rpdev->dev, "Msg too large: %d\n", msg->len);
            return -EMSGSIZE;
        }

        size_t copy = min_t(size_t, len, sizeof(rmsg->rx_msg));
        memcpy(&rmsg->rx_msg, msg, copy);
        complete(&rmsg->cmd_complete);
    }


    return 0;
}

static int spi_rpmsg_transfer_one_message(struct spi_controller *ctlr,
                                                   struct spi_message *msg)
{
    struct spi_rpmsg *rmsg = spi_controller_get_devdata(ctlr);
    struct spi_transfer *xfer;
    struct spi_rpmsg_msg req = {0};
    size_t total_len = 0;
    size_t len = 0;
    uint8_t *buf_ptr;
    int xfer_count = 0;
    bool cs_change = false;
    int ret = 0;

    list_for_each_entry(xfer, &msg->transfers, transfer_list) {
        xfer_count++;
        total_len += xfer->len;
        req.speed_hz = xfer->speed_hz;
        if (xfer->cs_change)
            cs_change = true;
    }

    if (total_len > SPI_RPMSG_MAX_BUF_SIZE) {
        dev_err(ctlr->dev.parent, "Message too large: %zu\n", total_len);
        return -EMSGSIZE;
    }

    req.header.cate  = SPI_RPMSG_CATEGORY;
    req.header.major = SPI_RPMSG_VERSION >> 8;
    req.header.minor = SPI_RPMSG_VERSION & 0xff;
    req.header.type  = SPI_RPMSG_TYPE_REQUEST;
    req.header.cmd   = SPI_RPMSG_COMMAND_TRANSFER;

    req.bus_id = ctlr->bus_num;
    req.addr   = msg->spi->chip_select;
    req.flags  = msg->spi->mode & (SPI_CPOL | SPI_CPHA);
    req.len    = total_len;

    if (xfer_count > 1 && !cs_change) {
        req.flags |= SPI_RPMSG_FLAG_KEEP_CS;
        dev_dbg(ctlr->dev.parent, "Transfer: KEEP_CS set (xfer_count=%d)\n", xfer_count);
    }

    buf_ptr = req.buf;
    list_for_each_entry(xfer, &msg->transfers, transfer_list) {
        if (xfer->tx_buf)
            memcpy(buf_ptr, xfer->tx_buf, xfer->len);
        else
            memset(buf_ptr, 0, xfer->len);
        buf_ptr += xfer->len;
    }

    reinit_completion(&rmsg->cmd_complete);

    size_t msg_size = offsetof(struct spi_rpmsg_msg, buf) + total_len;

    ret = rpmsg_send(rmsg->rpdev->ept, &req, msg_size);
    if (ret) {
        dev_err(ctlr->dev.parent, "rpmsg_send failed: %d\n", ret);
        return ret;
    }

    ret = wait_for_completion_timeout(&rmsg->cmd_complete,
                                      msecs_to_jiffies(SPI_RPMSG_TIMEOUT));
    if (!ret) {
        dev_err(ctlr->dev.parent, "SPI transfer timeout\n");
        return -ETIMEDOUT;
    }

    buf_ptr = rmsg->rx_msg.buf;
    len = 0;

    list_for_each_entry(xfer, &msg->transfers, transfer_list) {
        if (len + xfer->len > rmsg->rx_msg.len) {
            dev_err(ctlr->dev.parent, "Remote core sent too few bytes: %d < %zu\n",
                    rmsg->rx_msg.len, total_len);
            return -EIO;
        }

        if (xfer->rx_buf) {
            memcpy(xfer->rx_buf, buf_ptr, xfer->len);
        }

        buf_ptr += xfer->len;
        len += xfer->len;
    }

    msg->actual_length = total_len;
    msg->status = 0;

    spi_finalize_current_message(ctlr);

    return 0;
}

static int spi_rpbus_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct device_node *np = dev->of_node;
    struct spi_master *master;
    struct spi_rpmsg *info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
    int ret;

    master = spi_alloc_master(&pdev->dev, 0);
    if (!master) {
        dev_err(&pdev->dev, "failed to allocate spi master\n");
        return -ENOMEM;
    }
    platform_set_drvdata(pdev, master);

    info = &spi_rpmsg;
    if (!info) {
        dev_err(dev, "failed to get master devdata\n");
        spi_master_put(master);
        return -ENOMEM;
    }

    if (!spi_rpmsg.rpdev || !spi_rpmsg.rpdev->ept) {
        spi_master_put(master);
        return -EPROBE_DEFER;
    }

    mutex_init(&info->lock);
    init_completion(&info->cmd_complete);
    info->dev = &pdev->dev;

    info->rpdev = spi_rpmsg.rpdev;
    info->controller = master;

    spi_controller_set_devdata(master, info);

    master->bus_num = of_alias_get_id(np, "spi");
    if (master->bus_num < 0) {
        dev_err(dev, "invalid or missing spi alias id: %d\n", master->bus_num);
        spi_master_put(master);
        return -EINVAL;
    }
    if (master->bus_num >= 256) {
        dev_err(dev, "spi bus_num out of range\n");
        spi_master_put(master);
        return -EINVAL;
    }

    master->num_chipselect = 1;
    master->mode_bits = SPI_CPOL | SPI_CPHA;
    master->bits_per_word_mask = SPI_BPW_MASK(8);
    master->transfer_one_message = spi_rpmsg_transfer_one_message;
    master->dev.of_node = np;

    ret = spi_register_master(master);
    if (ret < 0) {
        dev_err(&pdev->dev, "failed to register spi master: %d\n", ret);
        spi_master_put(master);
        return ret;
    }

    dev_info(&pdev->dev, "registered spi master:spi %d\n", master->bus_num);

    return 0;
}

static int spi_rpbus_remove(struct platform_device *pdev)
{
    struct spi_controller *ctlr = platform_get_drvdata(pdev);

    if (ctlr) {
        spi_unregister_master(ctlr);
        platform_set_drvdata(pdev, NULL);
    }

    return 0;
}

static const struct of_device_id spi_rpmsg_dt_ids[] = {
    { .compatible = "spi_rpbus", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_rpmsg_dt_ids);

static struct platform_driver spi_rpmsg_platform_driver = {
    .driver = {
        .name   = "spi-rpmsg",
        .of_match_table = spi_rpmsg_dt_ids,
    },
    .probe  = spi_rpbus_probe,
    .remove = spi_rpbus_remove,
};

static int real_rpmsg_spi_probe(struct rpmsg_device *rpdev)
{
    struct spi_rpmsg *g = &spi_rpmsg;

    if (!rpdev || !rpdev->ept) {
        pr_err("%s: invalid rpdev\n", __func__);
        return -EINVAL;
    }

    g->rpdev = rpdev;
    g->dev = &rpdev->dev;

    dev_set_drvdata(&rpdev->dev, g);

    dev_info(&rpdev->dev, "spi_rpmsg_probe successfully\n");

    return 0;
}

static void real_rpmsg_spi_remove(struct rpmsg_device *rpdev)
{
    struct spi_rpmsg *rmsg = dev_get_drvdata(&rpdev->dev);

    if (rmsg && rmsg->controller)
        spi_unregister_controller(rmsg->controller);
}

static struct rpmsg_device_id spi_rpmsg_id_table[] = {
    { .name = "rpmsg-spi-channel" },
    { },
};
MODULE_DEVICE_TABLE(rpmsg, spi_rpmsg_id_table);

static struct rpmsg_driver spi_rpmsg_driver = {
    .drv.name   = "spi-rpmsg",
    .drv.owner  = THIS_MODULE,
    .id_table   = spi_rpmsg_id_table,
    .probe      = real_rpmsg_spi_probe,
    .remove     = real_rpmsg_spi_remove,
    .callback   = spi_rpmsg_cb,
};

static int __init spi_rpmsg_init(void)
{
    int ret;

    ret = platform_driver_register(&spi_rpmsg_platform_driver);
    if (ret)
        return ret;

    ret = register_rpmsg_driver(&spi_rpmsg_driver);
    if (ret) {
        platform_driver_unregister(&spi_rpmsg_platform_driver);
        return ret;
    }

    return 0;
}

static void __exit spi_rpmsg_exit(void)
{
    unregister_rpmsg_driver(&spi_rpmsg_driver);
    platform_driver_unregister(&spi_rpmsg_platform_driver);
}
module_init(spi_rpmsg_init);
module_exit(spi_rpmsg_exit);

MODULE_DESCRIPTION("SPI over RPMSG driver");
MODULE_LICENSE("GPL v2");