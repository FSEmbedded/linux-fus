/*
 Touchscreen driver for ESMT
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input/mt.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input/touchscreen.h>

/* Device Tree */
#include <linux/of_gpio.h>

#define 	MAX_TOUCH_NUM 10
#define 	GET_FW_VERSION 0x00
#define  	DRIVER_VERSION_1 0x2016
#define  	DRIVER_VERSION_2 0x0513
#define		DRIVER_VERSION_3 0x0002
#define 	READ_ALL_BYTES 0 // read back Max bytes
#define		READ_VAR_BYTES 1
#define		READ_8_BYTES 2 // For MTK
#define 	PointType READ_ALL_BYTES

#define 	ESMTDRIVERNAME "esmt_ts"
#define 	ESMTCLASSNAME "esmt"

/* creates sysfs entrys to download new FW */
//#define 	ESMT_DEBUG 1
#ifdef ESMT_DEBUG
#define		APROM_DOWNLOAD 0x61
#define		APROM_ERASE 0x60
#define 	APBOOT 0x65
#define 	LDBOOT 0x64
#define  	CHECK_ROM 0x08
#define 	LDROM_NOW 0x82
#define		CMD_BYTE 0xAE
#endif

#define 	ESMT_KEY_NUMBER 3
#define 	GET_POINT_INFO 0x0A

#define 	TP_RESOLUTION_X 1280
#define 	TP_RESOLUTION_Y 800

#define		max_buf_size ((MAX_TOUCH_NUM *5) + 4)

static struct workqueue_struct *esmt_wq;
static char *APROM_FILE_PATH = "/sdcard/APROM2.bin";

struct esmt_i2c_data *esmt_i2c_data_ptr;

struct esmt_i2c_data {
	struct i2c_client *client;
	struct work_struct get_work;
	struct input_dev *input_dev;
	int reset_pin;
	char phys[32];
	int fingers;
	int touch_num;
	int x_rev;
	int y_rev;
	int xy_swap;
};

int esmt_touch_key[ESMT_KEY_NUMBER] = {
	KEY_BACK,
	KEY_HOMEPAGE,
	KEY_MENU
};

#ifdef ESMT_DEBUG
static dev_t dev_id2;
static struct cdev *chardev;
struct class *esmt_i2c_class2;
#endif

module_param(APROM_FILE_PATH, charp, S_IRUGO);

// ----------------------------------------------------------------------------
static int esmt_i2c_wr(char *writeDataCmd, int lenOfCmd, char *rxdata,
								int length)
{
	int ret=0;
	struct i2c_msg msgs[] = {
		{
			.addr	= esmt_i2c_data_ptr->client->addr,
			.flags	= 0,      //W
			.len	= lenOfCmd,
			.buf	= writeDataCmd,
		},

		{
			.addr	= esmt_i2c_data_ptr->client->addr,
			.flags	= I2C_M_RD,   //R
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(esmt_i2c_data_ptr->client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&esmt_i2c_data_ptr->client->dev, "msg %s i2c read "
						"error: %d\n", __func__, ret);
	return ret;
}
// ----------------------------------------------------------------------------
static int esmt_get_fw_version(void)
{
	u8 	ret = 0;
	u8 	fw_version[2];
	u8	getVersionCmd = GET_FW_VERSION;
	ret = esmt_i2c_wr(&getVersionCmd,1,fw_version,2);
	if (ret<0)
		dev_err(&esmt_i2c_data_ptr->client->dev, "Get FW Version Fail:"
								" %d\n", ret);
	else
		dev_info(&esmt_i2c_data_ptr->client->dev, "APROM FW Version: "
				"%02X%02X \n", fw_version[0], fw_version[1]);
	dev_info(&esmt_i2c_data_ptr->client->dev, "DRIVER VER:%04X%04X_%04X"
		"\n", DRIVER_VERSION_1, DRIVER_VERSION_2, DRIVER_VERSION_3);
	return ret;
}
// ----------------------------------------------------------------------------
static void disable_interrupt(void)
{
	disable_irq_nosync(esmt_i2c_data_ptr->client->irq);
}
// ----------------------------------------------------------------------------
static void enable_interrupt(void)
{
	enable_irq(esmt_i2c_data_ptr->client->irq);
}
// ----------------------------------------------------------------------------
#ifdef ESMT_DEBUG
static int esmt_i2c_create_attrs(struct device * dev)
{
	int i=0, rc=0;

	for (i = 0; i < ARRAY_SIZE(esmt_i2c_attrs); i++) {
		rc = device_create_file(dev, &esmt_i2c_attrs[i]);
		if (rc)
		goto esmt_i2c_attrs_failed;
	}
	goto succeed;

	esmt_i2c_attrs_failed:
	while (i--)
		device_remove_file(dev, &esmt_i2c_attrs[i]);
	succeed:
	return rc;
}
// ----------------------------------------------------------------------------
struct file_operations esmt_ioctl_fops = {
	.owner   = THIS_MODULE,
};
// ----------------------------------------------------------------------------
static int register_esmt_char_dev(void)
{
	int ret = 0;
	struct device *dev = &esmt_i2c_data_ptr->client->dev;

	chardev = cdev_alloc();
	if(chardev == NULL){
		return -1;
	}

    if(alloc_chrdev_region(&dev_id2, 0, 1, ESMTCLASSNAME))  {
		dev_err(dev,"Register char dev error\n");
		return -1;
    }
	cdev_init(chardev, &esmt_ioctl_fops);
	if(cdev_add(chardev, dev_id2, 1)){
		dev_err(dev,"Add char dev error!\n");
	}

	esmt_i2c_class2 = class_create(THIS_MODULE, "es");
	if(IS_ERR(esmt_i2c_class2)) {
		dev_err(dev, "failed in creating class2. :%d :%s\n",__LINE__,
								__FUNCTION__);
		return -1;
	}
	dev = device_create(esmt_i2c_class2, dev, dev_id2, NULL,
								ESMTCLASSNAME);
	ret = esmt_i2c_create_attrs(dev);

	return 0;
}
// ----------------------------------------------------------------------------
static int readFile(struct file *fp,char *buf,int readlen)
{
	if (fp->f_op && fp->f_op->read)
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos);
	else
		return -1;
}
// ----------------------------------------------------------------------------
static int esmt_i2c_txdata(char *txdata, int length)
{
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= esmt_i2c_data_ptr->client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	ret = i2c_transfer(esmt_i2c_data_ptr->client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
// ----------------------------------------------------------------------------
static int Check_LDROM(void)
{
	int ret = 0;
	u8 CheckRomCmd = CHECK_ROM;
	u8 RomIs = 0;

	ret = esmt_i2c_wr(&CheckRomCmd, 1, &RomIs, 1);
	if (RomIs == LDROM_NOW)
		return 1;
	else
		return 0;
}
// ----------------------------------------------------------------------------
static int LD_Boot(void)
{
	int ret = 0;
	u8 txbuf[2] = {LDBOOT, CMD_BYTE};

	ret = esmt_i2c_txdata(txbuf, 2);
	return ret;
}
// ----------------------------------------------------------------------------
static int AP_Boot(void)
{
	int ret = 0;
	u8 txbuf[2] = {APBOOT, CMD_BYTE};

	ret = esmt_i2c_txdata(txbuf, 2);
	return ret;
}
// ----------------------------------------------------------------------------
static int aprom_update(char *fileLocation)
{
	mm_segment_t oldfs;
	int binSize = 0;
	int i = 0, ret = 0;
	struct file *fp_bin = NULL;
	int partSize = 128;
	int partFinished = 0;
	int yetSize = 0;
	char fwData[(partSize+2)];
	char EraseCmd[2] = {APROM_ERASE, CMD_BYTE};
	char DataCmd = APROM_DOWNLOAD;
	char rcvdata[10];
	static char FileBuffer[24000];

	memset(FileBuffer, 0xFF, sizeof(FileBuffer));
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	fp_bin = filp_open(fileLocation,O_RDONLY,0);
	if(IS_ERR(fp_bin))
	{
		dev_err(&esmt_i2c_data_ptr->client->dev, "%s, Can not open %s,"
		"err:0x%lx\n", __func__, fileLocation, (long)PTR_ERR(fp_bin));
		set_fs(oldfs);
		return -1;
	}

	binSize = readFile(fp_bin, FileBuffer, sizeof(FileBuffer));
	filp_close(fp_bin, NULL);
	set_fs(oldfs);

	if (binSize > 0)
	{
		disable_interrupt();
		ret = LD_Boot();
		if (ret <0)
		{
			dev_err(&esmt_i2c_data_ptr->client->dev, "tx LD fail: "
								"%d\n", ret);
			enable_interrupt();
			return -1;
		}
		msleep(15);
		i = 0;
		do {
			ret = Check_LDROM();
			if (!ret)
			{
				i++;
			}
			if (i>3)
				return -1;

			msleep(10);
		} while(ret == 0);

		dev_info(&esmt_i2c_data_ptr->client->dev, "LDROM boot OK, "
							"Erase APROM now\n");
		ret = esmt_i2c_wr(EraseCmd, 2, rcvdata, 2);
		if (ret <0)
		{
			dev_err(&esmt_i2c_data_ptr->client->dev, "tx erase "
							"fail:%d\n", ret);
			return -1;
		}
		msleep(50);
		dev_info(&esmt_i2c_data_ptr->client->dev, "APROM erase OK;"
						"\nAPROM Downloading...\n");

		fwData[0] = DataCmd;
		fwData[1] = CMD_BYTE;
		do
		{
			yetSize = binSize-partFinished;

			for (i=0; i<(yetSize > partSize ? partSize : yetSize);
									i++)
			{
				fwData[i+2] = FileBuffer[partFinished+i];
			}
			if (yetSize>=partSize)
				ret = esmt_i2c_wr(fwData,(partSize+2),
								rcvdata, 1);
			else
				ret = esmt_i2c_wr(fwData,(yetSize+2),
								rcvdata, 1);

			if (ret <0){
				dev_err(&esmt_i2c_data_ptr->client->dev, "tx "
						"write data fail:%d\n", ret);
				return -1;
			}
			msleep(10);

			if (yetSize>=partSize)
				partFinished+=partSize;
			else
				partFinished+=yetSize;

		} while(partFinished<binSize);
		dev_info(&esmt_i2c_data_ptr->client->dev, "Download "
								"Finished\n");
		msleep(5);

		ret = AP_Boot();
		if (ret <0)
		{
			dev_err(&esmt_i2c_data_ptr->client->dev, "tx AP "
							"fail:%d\n", ret);
			return -1;
		}
		msleep(5);
		dev_info(&esmt_i2c_data_ptr->client->dev, "AP Boot\n");
		ret = esmt_get_fw_version();
		enable_interrupt();
	}
	else
		dev_err(&esmt_i2c_data_ptr->client->dev, "File Location "
						"Error: %s\n", fileLocation);

	return 0;
}
// ----------------------------------------------------------------------------
static ssize_t show_download(struct device *dev, struct device_attribute *attr,
								char *buf)
{
	dev_info(dev, "===========================\n");
	dev_info(dev, "== Download Command List ==\n");
	dev_info(dev, "==  echo xxx > download  ==\n");
	dev_info(dev, "== xxx: Path of bin file ==\n");
	dev_info(dev, "===========================\n");
	return 0;

}
// ----------------------------------------------------------------------------
static ssize_t store_download(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int buflen = 128;
	char *filepath = NULL;
	char *locbuf = NULL;
	if(count <= 1)
	{	//for echo ""> and echo >   from adb, the count is 1
		filepath = APROM_FILE_PATH;
		dev_info(dev, "%s, using default path:%s\n", __func__,
								filepath);
	}
	else
	{
		while(buflen < count)
		{
			buflen = buflen << 1;
		}
		locbuf = kmalloc(buflen, GFP_KERNEL);
		if(locbuf == NULL)
		{
			dev_err(dev, "%s, out of memory\n", __func__);
			goto store_fwupdate_exit;
		}
		memcpy(locbuf, buf, count);
		//the last byte should be checked
		if (locbuf[count-1] == 0x0A){
			locbuf[count-1] = '\0';
		}
		filepath = locbuf;
	}

	aprom_update(filepath);

store_fwupdate_exit:
	if(locbuf)
	{
		kfree(locbuf);
	}

    return strnlen(buf, PAGE_SIZE);
}
// ----------------------------------------------------------------------------
static ssize_t show_cmd(struct device *dev, struct device_attribute *attr,
								char *buf)
{
	dev_info(dev, "==================================================\n");
	dev_info(dev, "== ver      : Check the FW version              ==\n");
	dev_info(dev, "==================================================\n");
	return 0;

}
// ----------------------------------------------------------------------------
static ssize_t store_cmd(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	int buflen = 128;
	char *getCmd = NULL;

	if(count <= 1)
		dev_warn(dev, "%s, no cmd, try: more cmd !!!\n", __func__);
	else{
		while(buflen < count)
		{
			buflen = buflen << 1;
		}
		getCmd = kmalloc(buflen, GFP_KERNEL);
		if(getCmd == NULL)
		{
			dev_err(dev, "%s, out of memory\n", __func__);
			goto store_getCmd_exit;
		}
		memcpy(getCmd, buf, count);

		if (getCmd[count-1] == 0x0A)
			getCmd[count-1] = '\0';
		if (strcmp(getCmd,"ver")==0)
			esmt_get_fw_version();
		else
			dev_info(dev, "No command\n");
	}

store_getCmd_exit:
	if(getCmd)
	{
		kfree(getCmd);
	}
	return strnlen(buf, PAGE_SIZE);
}
// ----------------------------------------------------------------------------
static struct device_attribute esmt_i2c_attrs[] = {
	{
        .attr   = {
            .name = "download",
            .mode = S_IRWXU | S_IRWXG | S_IRWXO,
        },
        .show  = show_download,
        .store = store_download,
    },
	{
        .attr   = {
            .name = "cmd",
            .mode = S_IRWXU | S_IRWXG | S_IRWXO,
        },
        .show  = show_cmd,
        .store = store_cmd,
    },
};
#endif
// ----------------------------------------------------------------------------
static void esmt_ts_release(void)
{
	struct esmt_i2c_data *ts = esmt_i2c_data_ptr;

	int i = 0, temp = 0;
	for (i=0; i<3; i++)
		input_report_key(ts->input_dev, esmt_touch_key[i], 0);
	temp = ts->fingers;
	ts->fingers = 0;
	for(i=0; i<ts->touch_num; i++) {
		if(temp & (1<<i)) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev,
							MT_TOOL_FINGER, 0);
		}
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_report_pointer_emulation(ts->input_dev, false);
	input_sync(ts->input_dev);

}
//-----------------------------------------------------------------------------
static inline int esmt_i2c_get_report(struct i2c_client *client)
{
	struct esmt_i2c_data *ts = esmt_i2c_data_ptr;
	struct input_dev *input_dev = ts->input_dev;
	int i = 0, ret = 0, finger_number = 0, fingers=0;
	u8 report_len = 0;
	u8 buf[max_buf_size];
	u8 ids = 0;
	u16 xs, ys = 0;
	u8 readCmd[2] = {GET_POINT_INFO, 0x00};

	/* Reading coordinates */
	if (PointType == READ_ALL_BYTES) {
		ret = esmt_i2c_wr(&readCmd[0], 1,  buf, max_buf_size);
		report_len = buf[0];
	} //end of if(readallbytes)
	else if (PointType == READ_VAR_BYTES){
		ret = esmt_i2c_wr(&readCmd[0], 1,  buf, 9);
		if (ret < 0) {
			dev_err(&client->dev, "%s read_data i2c_rxdata failed:"
						" %d\n", __func__, ret);
			return ret;
		}
		if ((report_len > 0) && (report_len <= max_buf_size) ){
			ret = esmt_i2c_wr(&readCmd[0], 1, buf, report_len);
		}
		else
			return ret;
	}
	else if (PointType == READ_8_BYTES)
	{
		esmt_i2c_wr(readCmd, 2,  &report_len, 1);
		if ((report_len > 0) && (report_len <= max_buf_size) )
		{
			for (i=0; i<((report_len>>3) + 1); i++)
			{
				readCmd[1] = i;
				ret = esmt_i2c_wr(readCmd, 2, &buf[i<<3],
						(((i+1)<<3)<report_len) ? 8:
							(report_len - (i<<3)));
				if (ret < 0) {
					dev_err(&client->dev, "%s read_data "
					"i2c_rxdata failed: %d\n", __func__,
									ret);
					return ret;
				}
			}
		}
	}  // end of !(readallbytes)

	if ((report_len >=4) && (report_len <=max_buf_size))
	{
		if (buf[report_len - 1] != 0x85)
		{
			dev_err(&client->dev, "data check failed!\n");
			return -1;
		}
		if (buf[0] == 0x04)
		{
			if (buf[1] == 0)
			{
				esmt_ts_release();
				return 1;
			}
			else
			{
				dev_err(&client->dev, "wrong data format: "
							"%x \n", buf[1]);
				return 2;
			}
		}
		else
		{
			int temp;
			finger_number = (buf[0]-4)/5;
			for (i=0; i<finger_number; i++)
			{
				ids  = (buf[i*5+2]&0x1F);
				fingers |= 1<<ids;
				xs = (u16)(buf[i*5+3] <<8) | (u16)buf[i*5+4];
				ys = (u16)(buf[i*5+5] <<8) | (u16)buf[i*5+6];

				if(ts->x_rev)
					xs = TP_RESOLUTION_X - xs;
				if(ts->y_rev)
					ys = TP_RESOLUTION_Y - ys;

				input_mt_slot(input_dev, ids);
				input_mt_report_slot_state(input_dev,
							MT_TOOL_FINGER, 1);
				if(ts->xy_swap) {
					input_report_abs(input_dev,
							ABS_MT_POSITION_X, ys);
					input_report_abs(input_dev,
							ABS_MT_POSITION_Y, xs);
				}
				else {
					input_report_abs(input_dev,
							ABS_MT_POSITION_X, xs);
					input_report_abs(input_dev,
							ABS_MT_POSITION_Y, ys);
				}
				input_report_abs(input_dev,
						ABS_MT_TOUCH_MAJOR, 255);
			}
			temp = ts->fingers & ~fingers;
			ts->fingers = fingers;
			for(i=0; i<ts->touch_num; i++) {
				if(temp & (1<<i)) {
					input_mt_slot(input_dev, i);
					input_mt_report_slot_state(input_dev,
							MT_TOOL_FINGER, 0);
				}
			}
			input_mt_report_pointer_emulation(input_dev, false);
			input_sync(input_dev);
		}
	}
	else
	{
		dev_err(&client->dev, "wrong report_len=%d\n", report_len);
		return -1;
	}

	return 0;
}
//-----------------------------------------------------------------------------
static void esmt_ts_work(struct work_struct *work)
{
	esmt_i2c_get_report(esmt_i2c_data_ptr->client);
}
//-----------------------------------------------------------------------------
static irqreturn_t esmt_tp_isr(int irq, void *handle)
{
	struct esmt_i2c_data *ts = (struct esmt_i2c_data *)handle;

	disable_interrupt();
	schedule_work(&ts->get_work);
	enable_interrupt();
	return IRQ_HANDLED;
}
// ----------------------------------------------------------------------------
#ifdef OMAP_EARLYSUSPEND
static void register_esmt_earlysuspend(struct esmt_i2c_data *ts)
{
	dev_info(&ts->client->dev, "%s: %d  %s!\n", __FILE__, __LINE__,
								__FUNCTION__);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.suspend = esmt_early_suspend;
	ts->early_suspend.resume =  esmt_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
}
#endif
// ----------------------------------------------------------------------------
static int esmt_suspend(struct device *dev, pm_message_t state)
{
	//### todo
	dev_info(dev, "suspend #todo\n");

	return 0;
}
// ----------------------------------------------------------------------------
static int esmt_resume(struct device *dev)
{
	//### todo
	dev_info(dev, "resume #todo\n");

	return 0;
}
// ----------------------------------------------------------------------------
#ifdef CONFIG_OF
static int esmt_ts_probe_dt(struct esmt_i2c_data *data)
{
	u32 val;
	struct device_node *np = data->client->dev.of_node;

	if ((of_property_read_u32(np, "touch_num", &val) == 0) && (val > 0)) {
		if (val > MAX_TOUCH_NUM)
			val = MAX_TOUCH_NUM;
		data->touch_num = val;
	}
	else
		data->touch_num = MAX_TOUCH_NUM;

	data->reset_pin = of_get_named_gpio(np, "reset-gpios", 0);
	data->x_rev = of_property_read_bool(np, "x-rev");
	data->y_rev = of_property_read_bool(np, "y-rev");
	data->xy_swap = of_property_read_bool(np, "swap-xy");

	return 0;
}
#endif
// ----------------------------------------------------------------------------
static int esmt_i2c_msg_probe(struct i2c_client *client,
                      const struct i2c_device_id *id)
{
	struct esmt_i2c_data *data = NULL;
	struct input_dev *input_dev;
	int err = 0, ret = 0;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_I2C)) {
		ret = -ENODEV;
		dev_err(&client->dev,"%d,%s\n",__LINE__,__FUNCTION__);
		goto exit;
	}

	data = kzalloc(sizeof(struct esmt_i2c_data), GFP_KERNEL);

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;

#ifdef CONFIG_OF
	/* Get platform data from device tree */
	err = esmt_ts_probe_dt(data);
	if (err) {
		dev_err(&client->dev, "No valid device tree data\n");
		return err;
	}
#endif

	/* Toggle reset line */
	if (gpio_is_valid(data->reset_pin)) {
		devm_gpio_request_one(&client->dev, data->reset_pin,
				      GPIOF_OUT_INIT_LOW, "esmt");
		gpio_set_value(data->reset_pin, 0);
		msleep(50);
		gpio_set_value(data->reset_pin, 1);
		msleep(100);
	}

	input_dev->name = ESMTDRIVERNAME;
	input_dev->phys = "esmt_ts/input0";
	input_dev->id.bustype = BUS_HOST;

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH,input_dev->keybit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(KEY_MENU, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X, 0, TP_RESOLUTION_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, TP_RESOLUTION_Y, 0, 0);

	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TP_RESOLUTION_X,
									0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TP_RESOLUTION_Y,
									0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
						data->touch_num - 1, 0, 0);

	/* Parse touchscreen-size-x, touchscreen-size-y, etc */
	touchscreen_parse_of_params(input_dev);
	dev_info(&client->dev, "Setting resolution to %d x %d\n",
		 input_abs_get_max(input_dev, ABS_MT_POSITION_X),
		 input_abs_get_max(input_dev, ABS_MT_POSITION_Y));

	err = input_mt_init_slots(input_dev, data->touch_num, 0);
	if(err) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		goto free_mem;
	}

	i2c_set_clientdata(client, data);
	INIT_WORK(&data->get_work, esmt_ts_work);
	err = request_irq(client->irq, esmt_tp_isr, IRQF_TRIGGER_FALLING,
							client->name, data);
	if (err) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ\n");
		goto exit;
	}

	err = input_register_device(input_dev);
	if (err < 0) {
		input_free_device(input_dev);
		goto free_mem;
	}

	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	strcpy(data->client->name, ESMTDRIVERNAME);
	esmt_i2c_data_ptr = data;

#ifdef ESMT_DEBUG
	ret = register_esmt_char_dev();
#endif

#ifdef OMAP_EARLYSUSPEND
	register_esmt_earlysuspend(data);
#endif

	dev_info(&client->dev, "client-name=%s, %d X %d \n", client->name,
					TP_RESOLUTION_X, TP_RESOLUTION_Y);

	esmt_get_fw_version();

	return 0;

free_mem:
	input_free_device(input_dev);
	kfree(data);
	return ret;
exit:
	return err;

}
// ----------------------------------------------------------------------------
static int esmt_i2c_msg_remove(struct i2c_client *client)
{
	struct esmt_i2c_data *ts = i2c_get_clientdata(client);
	disable_interrupt();
	msleep(100);
	free_irq(client->irq, ts);
	gpio_free(ts->reset_pin);
	input_unregister_device(ts->input_dev);

	kfree(ts);
	return 0;
}
// ----------------------------------------------------------------------------
static const struct i2c_device_id esmt_i2c_msg_id[] = {
	{ ESMTDRIVERNAME, 0 },   // important name
	{ }
};
//-----------------------------------------------------------------------------
static struct i2c_driver esmt_i2c_msg_driver = {
        .driver = {
		.name   = ESMTDRIVERNAME,
		.owner	= THIS_MODULE,
		.suspend = esmt_suspend,
		.resume = esmt_resume,
        },
        .probe          = esmt_i2c_msg_probe,
        .remove         = esmt_i2c_msg_remove,
        .id_table       = esmt_i2c_msg_id,
};
// ----------------------------------------------------------------------------
static int __init esmt_i2c_msg_init(void)
{
	esmt_wq = create_singlethread_workqueue("esmt_wq");
	return i2c_add_driver(&esmt_i2c_msg_driver);
}
// ----------------------------------------------------------------------------
static void __exit esmt_i2c_msg_exit(void)
{
	i2c_del_driver(&esmt_i2c_msg_driver);
}
// ----------------------------------------------------------------------------
MODULE_AUTHOR("ESMT");
MODULE_DESCRIPTION("ESMT");
MODULE_LICENSE("GPL");

module_init(esmt_i2c_msg_init);
module_exit(esmt_i2c_msg_exit);
