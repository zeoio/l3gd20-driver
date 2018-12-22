/*
 * @file l3gd20.c v0.1_debug
 * Driver for the ST L3GD20 MEMS and L3GD20H mems gyros connected via I2C
 *
 * Write by Sarlor, 11/4/2016
 *  */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* register addresses */
#define L3G_ADDR_WHO_AM_I			0x0F
#define L3G_ADDR_CTRL_REG1			0x20
#define L3G_ADDR_CTRL_REG2			0x21
#define L3G_ADDR_CTRL_REG3			0x22
#define L3G_ADDR_CTRL_REG4			0x23
#define L3G_ADDR_CTRL_REG5			0x24
#define L3G_ADDR_REFERENCE			0x25
#define L3G_ADDR_OUT_TEMP			0x26
#define L3G_ADDR_STATUS_REG			0x27
#define L3G_ADDR_OUT_X_L			0x28
#define L3G_ADDR_OUT_X_H			0x29
#define L3G_ADDR_OUT_Y_L			0x2A
#define L3G_ADDR_OUT_Y_H			0x2B
#define L3G_ADDR_OUT_Z_L			0x2C
#define L3G_ADDR_OUT_Z_H			0x2D
#define L3G_ADDR_FIFO_CTRL_REG			0x2E
#define L3G_ADDR_FIFO_SRC_REG			0x2F
#define L3G_ADDR_INT1_CFG			0x30
#define L3G_ADDR_INT1_SRC			0x31
#define L3G_ADDR_INT1_TSH_XH			0x32
#define L3G_ADDR_INT1_TSH_XL			0x33
#define L3G_ADDR_INT1_TSH_YH			0x34
#define L3G_ADDR_INT1_TSH_YL			0x35
#define L3G_ADDR_INT1_TSH_ZH			0x36
#define L3G_ADDR_INT1_TSH_ZL			0x37
#define L3G_ADDR_INT1_DURATION			0x38

/* Orientation on board */
#define L3G_SENSOR_BOARD_ROTATION_000_DEG	0
#define L3G_SENSOR_BOARD_ROTATION_090_DEG	1
#define L3G_SENSOR_BOARD_ROTATION_180_DEG	2
#define L3G_SENSOR_BOARD_ROTATION_270_DEG	3

/* Internal configuration values */
/* keep lowpass low to avoid noise issues */
#define L3G_REG1_RATE_BITS			((1<<7) | (1<<6))
#define L3G_REG1_RATE_95HZ			((0<<7) | (0<<6))
#define L3G_REG1_RATE_190HZ			((0<<7) | (1<<6))
#define L3G_REG1_RATE_380HZ			((1<<7) | (0<<6))
#define L3G_REG1_RATE_760HZ			((1<<7) | (1<<6))
#define L3G_REG1_BANDWTH_BITS			((1<<5) | (1<<4))

#define L3G_REG1_CONF_BITS	 	 (L3G_REG1_RATE_BITS | L3G_REG1_BANDWTH_BITS)
#define L3G_REG1_RATE_95HZ_LP_12_5HZ	 (L3G_REG1_RATE_95HZ  | (0<<5) | (0<<4))
#define L3G_REG1_RATE_95HZ_LP_25HZ	 (L3G_REG1_RATE_95HZ  | (0<<5) | (1<<4))
#define L3G_REG1_RATE_190HZ_LP_12_5HZ 	 (L3G_REG1_RATE_190HZ | (0<<5) | (0<<4))
#define L3G_REG1_RATE_190HZ_LP_25HZ	 (L3G_REG1_RATE_190HZ | (0<<5) | (1<<4))
#define L3G_REG1_RATE_190HZ_LP_50HZ	 (L3G_REG1_RATE_190HZ | (1<<5) | (0<<4))
#define L3G_REG1_RATE_190HZ_LP_70HZ	 (L3G_REG1_RATE_190HZ | (1<<5) | (1<<4))
#define L3G_REG1_RATE_380HZ_LP_20HZ	 (L3G_REG1_RATE_380HZ | (0<<5) | (0<<4))
#define L3G_REG1_RATE_380HZ_LP_25HZ	 (L3G_REG1_RATE_380HZ | (0<<5) | (1<<4))
#define L3G_REG1_RATE_380HZ_LP_50HZ	 (L3G_REG1_RATE_380HZ | (1<<5) | (0<<4))
#define L3G_REG1_RATE_380HZ_LP_100HZ	 (L3G_REG1_RATE_380HZ | (1<<5) | (1<<4))
#define L3G_REG1_RATE_760HZ_LP_30HZ	 (L3G_REG1_RATE_760HZ | (0<<5) | (0<<4))
#define L3G_REG1_RATE_760HZ_LP_35HZ	 (L3G_REG1_RATE_760HZ | (0<<5) | (1<<4))
#define L3G_REG1_RATE_760HZ_LP_50HZ	 (L3G_REG1_RATE_760HZ | (1<<5) | (0<<4))
#define L3G_REG1_RATE_760HZ_LP_100HZ	 (L3G_REG1_RATE_760HZ | (1<<5) | (1<<4))

#define L3G_REG1_PD_BITS			(1<<3)
#define L3G_REG1_PD_NORMAL			(1<<3)
#define L3G_REG1_PD_ENABLE			(0<<3)

#define L3G_REG1_Z_BITS				(1<<2)
#define L3G_REG1_Z_ENABLE			(1<<2)
#define L3G_REG1_Z_DISABLE			(0<<2)
#define L3G_REG1_Y_BITS				(1<<1)
#define L3G_REG1_Y_ENABLE			(1<<1)
#define L3G_REG1_Y_DISABLE			(0<<1)
#define L3G_REG1_X_BITS				(1<<0)
#define L3G_REG1_X_ENABLE			(1<<0)
#define L3G_REG1_X_DISABLE			(0<<0)

#define L3G_REG2_HIGHPASS_MODE_BITS		((1<<5) | (1<<4))
#define L3G_REG2_HIGHPASS_MODE_NOR_RESET	((0<<5) | (0<<4))
#define L3G_REG2_HIGHPASS_MODE_REF		((0<<5) | (1<<4))
#define L3G_REG2_HIGHPASS_MODE_NOR		((1<<5) | (0<<4))
#define L3G_REG2_HIGHPASS_MODE_AUTO_IRQ		((1<<5) | (1<<4))

#define L3G_REG2_HIGHPASS_CUTOFF_FRE_BITS ((1<<3) | (1<<2) | (1<<1) | (1<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_0    ((0<<3) | (0<<2) | (0<<1) | (0<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_1    ((0<<3) | (0<<2) | (0<<1) | (1<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_2    ((0<<3) | (0<<2) | (1<<1) | (0<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_3    ((0<<3) | (0<<2) | (1<<1) | (1<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_4    ((0<<3) | (1<<2) | (0<<1) | (0<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_5    ((0<<3) | (1<<2) | (0<<1) | (1<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_6    ((0<<3) | (1<<2) | (1<<1) | (0<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_7    ((0<<3) | (1<<2) | (1<<1) | (1<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_8    ((1<<3) | (0<<2) | (0<<1) | (0<<0))
#define L3G_REG2_HIGHPASS_CUTOFF_FRE_9    ((1<<3) | (0<<2) | (0<<1) | (1<<0))

#define L3G_REG3_INT1_ENABLE			(1<<7)
#define L3G_REG3_BOOT_INT1_ENABLE		(1<<6)
#define L3G_REG3_INT1_ACTIVE_ENABLE		(1<<5)
#define L3G_REG3_PUSH_PULL_ENABLE		(1<<4)
#define L3G_REG3_DATA_READY_INT2_ENABLE		(1<<3)
#define L3G_REG3_FIFO_WATERMARK_INT2_ENABLE	(1<<2)
#define L3G_REG3_FIFO_OVERRUN_INT2_ENABLE	(1<<1)
#define L3G_REG3_FIFO_EMPTY_INT2_ENABLE		(1<<0)
#define L3G_REG3_INT1_DISABLE			(0<<7)
#define L3G_REG3_BOOT_INT1_DISABLE		(0<<6)
#define L3G_REG3_INT1_ACTIVE_DISABLE		(0<<5)
#define L3G_REG3_PUSH_PULL_DISABLE		(0<<4)
#define L3G_REG3_DATA_READY_INT2_DISABLE	(0<<3)
#define L3G_REG3_FIFO_WATERMARK_INT2_DISABLE	(0<<2)
#define L3G_REG3_FIFO_OVERRUN_INT2_DISABLE	(0<<1)
#define L3G_REG3_FIFO_EMPTY_INT2_DISABLE	(0<<0)

#define L3G_REG4_BDU_BITS			(1<<7)
#define L3G_REG4_BLE_BITS			(1<<6)
#define L3G_REG4_BDU_NOT_UPDATE			(1<<7)
#define L3G_REG4_BLE_MSB			(1<<6)
#define L3G_REG4_SIMI_3_INTFC			(1<<0)
#define L3G_REG4_BDU_CONTU_UPDATE		(0<<7)
#define L3G_REG4_BLE_LSB			(0<<6)
#define L3G_REG4_SIMI_4_INTFC			(0<<0)

#define L3G_REG4_RANGE_BITS			((1<<5) | (1<<4))
#define L3G_RANGE_250DPS			((0<<5) | (0<<4))
#define L3G_RANGE_500DPS			((0<<5) | (1<<4))
#define L3G_RANGE_2000DPS			((1<<5) | (1<<4))

#define L3G_REG5_REBOOT_MEMORY			(1<<7)
#define L3G_REG5_BOOT_NORMAL			(0<<7)
#define L3G_REG5_FIFO_ENABLE_BITS		(1<<6)
#define L3G_REG5_FIFO_ENABLE			(1<<6)
#define L3G_REG5_FIFO_DISABLE			(0<<6)
#define L3G_REG5_HIGHPASS_FILTER_BITS		(1<<4)

#define L3G_REG5_HIGHPASS_FILTER_DISABLE	(0<<4)
#define L3G_REG5_INT1_DEF_CONF			((0<<3) | (0<<2))
#define L3G_REG5_OUT_DEF_CONF			((0<<1) | (0<<0))

#define L3G_REG_FIFO_MODE_BITS			((1<<7) | (1<<6) | (1<<5))
#define L3G_REG_FIFO_WATWRMK_BITS		((1<<4) | (1<<3) | (1<<2) \
							|(1<<1) | (1<<0))
#define L3G_FIFO_CTRL_BYPASS_MODE		(0<<5)
#define L3G_FIFO_CTRL_FIFO_MODE			(1<<5)
#define L3G_FIFO_CTRL_STREAM_MODE		(1<<6)
#define L3G_FIFO_CTRL_STREAM_TO_FIFO_MODE	(3<<5)
#define L3G_FIFO_CTRL_BYPASS_TO_STREAM_MODE	(1<<7)

#define L3G_DEFAULT_RATE			760
#define L3G_DEFAULT_RANGE_DPS			2000
#define L3G_DEFAULT_FILTER_FREQ			30
#define L3G_TEMP_OFFSET_CELSIUS			40

/* device info */
#define L3G_SENSOR_NAME				"l3gd20"
#define L3G_I2C_ADDR				0x6A	 /* Slave Address */
#define L3G_ABSMIN				-32
#define L3G_ABSMAX				31
#define L3G_FUZZ				1
#define L3G_LSG					21
#define L3G_MAX_DELAY				50

#define L3G_MODE_STANDBY			0x00	/* Power-dwon mode */
#define L3G_MODE_ACTIVE				0x01	/* Normal mode 	   */

#define L3G_GYRO_SAMPLERATE_DEFAULT    		760

struct l3gd20_gyro {
	s16 x;
	s16 y;
	s16 z;
};

struct l3gd20_data {
	struct i2c_client *l3gd20_client;
	struct input_dev *input;
	struct mutex enable_mutex;
	struct delayed_work work;
	struct l3gd20_gyro offset;
	atomic_t delay;
	atomic_t enable;
	atomic_t position;
	atomic_t calibrated;
	atomic_t fuzz;
};

/*
 * converts the complement code to the original code (word).
 * @number: complement will to converted.
 *
 * Returning the value of the primitive to be converted.
 *  */
static int l3gd20_complement_to_original(unsigned int number)
{
	s16 complement;
	if ((int)number < 0)
		complement = (s16)((~(number-0x0001))|0x8000);
	else
		complement = (s16)number;

	return complement;
}

/*
 * converts the complement code to the original code (byte).
 * @number: complement will to converted.
 *
 * Returning the value of the primitive to be converted.
 *  */
static char l3gd20_complement_to_original_byte(unsigned char num)
{
	if ((char)num >= 0)
		return num;
	return -(~num + 1);
}

/*
 * l3gd20 read byte.
 * @client: Handle to slave device.
 * @reg_addr: Byte interpreted by slave.
 * @data: Byte being stored.
 *
 * From the specified register 'reg_addr' to read a byte of data, stored in
 * the '*data'. Returning negative errno else zero on success.
 *  */
static int l3gd20_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if(dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

/*
 * l3gd20 write byte.
 * @client: Handle to slave device.
 * @reg: Byte interpreted by slave.
 * @data: Byte being written.
 *
 * Tells the specified contents '*data' to be written to the specified
 * register 'reg'. Returning negative errno else zero on success.
 *  */
static int l3gd20_smbus_write_byte(struct i2c_client *client, unsigned char reg,
	       	unsigned char *data)
{
	if (i2c_smbus_write_byte_data(client, reg, *data) < 0)
		return -EAGAIN;
	return 0;
}

static int l3gd20_smbus_read_bits(struct i2c_client *client, unsigned char reg,
	       	unsigned char *data, unsigned char start, unsigned char len)
{
	unsigned char tmp;

	if(start >= 8 || len > 8)
		return -EINVAL;
	if (l3gd20_smbus_read_byte(client, reg, &tmp) < 0)
		return -EAGAIN;

	*data = (tmp >> start) & (0xff >> (8-len));
	return 0;
}

/*
 * modify the value of the register.
 * @client: Handle to slave device.
 * @reg: modified register.
 * @clearbits: mask value.
 * @setbits: set value.
 *
 * Modify the value of the register 'reg' to 'setbits'. Returning negative errno
 * else zero on success.
 *  */
static int modify_reg(struct i2c_client *client, unsigned char reg,
		uint8_t clearbits, uint8_t setbits)
{
	unsigned char val;

	if (l3gd20_smbus_read_byte(client, reg, &val) < 0)
		return -EAGAIN;

	val &= ~clearbits;
	val |= setbits;

	if (l3gd20_smbus_write_byte(client, reg, &val) < 0)
		return -EAGAIN;
	return 0;
}

/*
 * l3gd20 power-down mode enable.
 * @client: Handle to slave device.
 * @mode: power-mode mode.
 *
 * Sets the power mode of the device. 0 is power-down mode, 1 is normal mode or
 * sleep mode. Returning negative errno else zero on success.
 *  */
static int l3gd20_set_mode(struct i2c_client *client, unsigned char mode)
{
	unsigned char clearbits = L3G_REG1_PD_BITS;
	unsigned char setbits = (mode & 0x01) << 3;

	if (modify_reg(client, L3G_ADDR_CTRL_REG1, clearbits, setbits) < 0)
		return -EAGAIN;
	return 0;
}

static void l3gd20_set_enable(struct device *dev, int enable);

/*
 * When the new settings and the old settings are not equal, use the new
 * settings.
 *    */
static void update_ctrl_reg1(struct device *dev, unsigned char data)
{
	unsigned char enable = (data & L3G_REG1_PD_BITS >> 3);
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (atomic_read(&l3gd20->enable) != enable)
		l3gd20_set_enable(dev, enable);
}

/*
 * updates the configuration values.
 * @dev: device.
 * @addr: the updated register address.
 *
 * When the register value is modified, there are some configurations that do
 * not take effect and need to be manually updated to take effect. Returning
 * negative errno else zero on success.
 *  */
static int reg_update(struct device *dev, unsigned char addr)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_byte(l3gd20->l3gd20_client, addr, &data) < 0)
		return -EAGAIN;

	switch (addr) {
		case L3G_ADDR_CTRL_REG1:
			update_ctrl_reg1(dev, data);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

/*
 * l3gd20 initialization.
 * @client: Handle to slave device.
 *  */
#define REG_DEF_NUM	7
static int l3gd20_hw_init(struct i2c_client *client)
{
	unsigned char clearbits, setbits;
	int i, comres = 0, tmp = 0;
	unsigned char reg_addr[] = {
		L3G_ADDR_CTRL_REG1,
		L3G_ADDR_CTRL_REG2,
		L3G_ADDR_CTRL_REG3,
		L3G_ADDR_CTRL_REG4,
		L3G_ADDR_CTRL_REG5,
		L3G_ADDR_CTRL_REG5,
		L3G_ADDR_FIFO_CTRL_REG
	};
	unsigned char reg_dat[] = {
		L3G_REG1_PD_NORMAL | L3G_REG1_Z_ENABLE | L3G_REG1_Y_ENABLE
			| L3G_REG1_X_ENABLE,
		0x00,
		0x08,
		L3G_REG4_BDU_NOT_UPDATE,
		0x00,
		L3G_REG5_FIFO_ENABLE,
		L3G_FIFO_CTRL_BYPASS_MODE
	};

	tmp = i2c_smbus_read_byte_data(client, L3G_ADDR_WHO_AM_I);
	printk("%s:addr = 0x%x, Read ID value is :%d\n",
		        __func__, client->addr, tmp);

	for(i = 0; i < REG_DEF_NUM; i++) {
		comres += i2c_smbus_write_byte_data(client, reg_addr[i],
				reg_dat[i]);
		reg_update(&client->dev, reg_addr[i]);
	}

	clearbits = L3G_REG1_CONF_BITS;
	setbits = L3G_REG1_RATE_760HZ_LP_50HZ;
	modify_reg(client, L3G_ADDR_CTRL_REG1, clearbits, setbits);

	clearbits = L3G_REG4_RANGE_BITS;
	setbits = L3G_RANGE_2000DPS;
	modify_reg(client, L3G_ADDR_CTRL_REG4, clearbits, setbits);

	return -(comres != 0);
}

/*
 * l3gd20 read x/y/z axis angular rate data.
 * @client: Handle to slave device.
 * @gyro: to saving gyro data.
 *
 * The value is expressed as two's complement. So, you need to convert it
 * to the original code.
 *  */
static int l3gd20_read_data(struct i2c_client *client,
		struct l3gd20_gyro *gyro)
{
	unsigned char tmp_data[7], i;
	u16 temp[3];

	for (i=0; i<7; i++)
		l3gd20_smbus_read_byte(client, L3G_ADDR_OUT_X_L+i, tmp_data+i);

	temp[0] = ((tmp_data[1] << 8) & 0xff00) | tmp_data[0];
	temp[1] = ((tmp_data[3] << 8) & 0xff00) | tmp_data[2];
	temp[2] = ((tmp_data[5] << 8) & 0xff00) | tmp_data[4];

	gyro->x = l3gd20_complement_to_original(temp[0]);
	gyro->y = l3gd20_complement_to_original(temp[1]);
	gyro->z = l3gd20_complement_to_original(temp[2]);

	return 0;
}

static ssize_t l3gd20_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int addr, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	sscanf(buf, "0x%x:0x%x", &addr, &value);
	if (l3gd20_smbus_write_byte(l3gd20->l3gd20_client,
			(unsigned char)addr, (unsigned char *)&value) < 0)
		return -EINVAL;

	reg_update(dev, (unsigned char)addr);
	return count;
}

static ssize_t l3gd20_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char i, data = 0;
	size_t count = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_byte(l3gd20->l3gd20_client, L3G_ADDR_WHO_AM_I,
				&data) >= 0)
		count = sprintf(buf+count, "0x%02x:0x%02x ", L3G_ADDR_WHO_AM_I, data);

	for (i = L3G_ADDR_CTRL_REG1; i <= L3G_ADDR_INT1_DURATION; i++) {
		if (l3gd20_smbus_read_byte(l3gd20->l3gd20_client, i,&data) >= 0)
			count += sprintf(buf+count, "0x%02x:0x%02x ", i, data);
	}

	buf[count-1] = '\0';

	return count;
}

static ssize_t l3gd20_gyro_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_data *l3gd20 = input_get_drvdata(input);
	struct l3gd20_gyro gyro;

	if (l3gd20_read_data(l3gd20->l3gd20_client, &gyro) < 0)
		return sprintf(buf, "%d", -EINVAL);
	return sprintf(buf, "%d %d %d", gyro.x, gyro.y, gyro.z);
}

static ssize_t l3gd20_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	return sprintf(buf, "%d", atomic_read(&l3gd20->delay));
}

static ssize_t l3gd20_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	atomic_set(&l3gd20->delay, (unsigned int)data);
	return count;
}

static ssize_t l3gd20_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	return sprintf(buf, "%d", atomic_read(&l3gd20->enable));
}

/*
 * the device is switched off. Enable is 1 to start the device, 0 is to
 * turn off the device. When the device is activated periodically read
 * the gyro data value.
 *  */
static void l3gd20_do_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (enable) {
		l3gd20_set_mode(l3gd20->l3gd20_client, L3G_MODE_ACTIVE);
		schedule_delayed_work(&l3gd20->work,
				msecs_to_jiffies(atomic_read(&l3gd20->delay)));
	} else {
		l3gd20_set_mode(l3gd20->l3gd20_client, L3G_MODE_STANDBY);
		cancel_delayed_work_sync(&l3gd20->work);
	}
}

/* set the device on or off. */
static void l3gd20_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&l3gd20->enable);

	mutex_lock(&l3gd20->enable_mutex);
	if (enable != pre_enable) {
		l3gd20_do_enable(dev, enable);
		atomic_set(&l3gd20->enable, enable);
	}
	mutex_unlock(&l3gd20->enable_mutex);
}

static ssize_t l3gd20_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if ((data == 0) || (data == 1))
		l3gd20_set_enable(dev, data);

	return count;
}

static ssize_t l3gd20_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	data = atomic_read(&(l3gd20->position));
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_position_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtol(buf, 10, &data);
	if (err)
		return err;

	atomic_set(&(l3gd20->position), (int)data);
	return count;
}

static ssize_t l3gd20_axis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG1,
				&data, 0, 3) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_axis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &mode);
	if (err)
		return err;

	clearbits = L3G_REG1_Z_BITS | L3G_REG1_Y_BITS | L3G_REG1_X_BITS;
	setbits = mode & 0x07;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG1, clearbits,
				setbits) < 0)
		return -EAGAIN;

	return count;
}

static int get_range(struct i2c_client *client)
{
	unsigned char data;

	if (l3gd20_smbus_read_bits(client, L3G_ADDR_CTRL_REG4, &data, 4, 2) < 0)
	        return -EAGAIN;

	if (data == 0)
		return 250;
	else if (data == 1)
		return 500;
	else
		return 2000;
}

static ssize_t l3gd20_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	return sprintf(buf, "%d", get_range(l3gd20->l3gd20_client));
}

static int set_range(struct i2c_client *client, unsigned long range)
{
	unsigned char clearbits, setbits;

	if (range == 250)
		range = 0;
	else if (range == 500)
		range = 1;
	else
		range = 2;

	clearbits = L3G_REG4_RANGE_BITS;
	setbits = (range & 0x03) << 4;

	if (modify_reg(client, L3G_ADDR_CTRL_REG4, clearbits, setbits) < 0)
		return -1;
	return 0;
}

static ssize_t l3gd20_range_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long range;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &range);
	if (err)
		return err;

	if (set_range(l3gd20->l3gd20_client, range) < 0)
		return -EAGAIN;
	return count;
}

static int get_sample(struct i2c_client *client)
{
	unsigned char data;

	if (l3gd20_smbus_read_bits(client, L3G_ADDR_CTRL_REG1, &data, 6, 2) < 0)
		return -EAGAIN;

	if (data == 0)
		return 95;
	else if (data == 1)
		return 190;
	else if (data == 2)
		return 380;
	else
		return 760;
}

static ssize_t l3gd20_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	return sprintf(buf, "%d", get_sample(l3gd20->l3gd20_client));
}

static int set_sample(struct i2c_client *client, unsigned long rate)
{
	unsigned char clearbits, setbits;

	if (rate == 95)
		rate = 0;
	else if (rate == 190)
		rate = 1;
	else if (rate == 380)
		rate = 2;
	else
		rate = 3;

	clearbits = L3G_REG1_RATE_BITS;
	setbits = (rate & 0x03) << 6;

	if (modify_reg(client, L3G_ADDR_CTRL_REG1, clearbits, setbits) < 0)
		return -EAGAIN;
	return 0;
}

static ssize_t l3gd20_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long rate;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &rate);
	if (err)
		return err;

	if (set_sample(l3gd20->l3gd20_client, rate) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t l3gd20_bandwth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG1,
				&data, 4, 2) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_bandwth_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long bandwth;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &bandwth);
	if (err)
		return err;

	clearbits = L3G_REG1_BANDWTH_BITS;
	setbits = (bandwth & 0x03) << 4;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG1,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t l3gd20_highpass_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG2,
			       &data, 4, 2) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_highpass_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &mode);
	if (err)
		return err;

	clearbits = L3G_REG2_HIGHPASS_MODE_BITS;
	setbits = (mode & 0x03) << 4;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG2,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t l3gd20_highpass_cutoff_fre_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG2,
				&data, 0, 4) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_highpass_cutoff_fre_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &mode);
	if (err)
		return err;

	clearbits = L3G_REG2_HIGHPASS_CUTOFF_FRE_BITS;
	setbits = 0x0f & mode;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG2,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t l3gd20_bdu_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG4,
				&data, 7, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_bdu_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &mode);
	if (err)
		return err;

	clearbits = L3G_REG4_BDU_BITS;
	setbits = (mode & 0x01) << 7;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG4,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t l3gd20_temperature_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_byte(l3gd20->l3gd20_client, L3G_ADDR_OUT_TEMP,
				&data) < 0)
		return sprintf(buf, "%d", -EAGAIN);

	return sprintf(buf, "%d", l3gd20_complement_to_original_byte(data));
}

static ssize_t l3gd20_hpen_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG5,
				&data, 4, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_hpen_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	clearbits = L3G_REG5_HIGHPASS_FILTER_BITS;
	setbits = (data & 0x01) << 4;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG5,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t l3gd20_fifo_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG5,
				&data, 6, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_fifo_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	clearbits = L3G_REG5_FIFO_ENABLE_BITS;
	setbits = (data & 0x01) << 6;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG5,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t l3gd20_zyxor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_STATUS_REG,
				&data, 7, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_zor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_STATUS_REG,
				&data, 6, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_yor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_STATUS_REG,
				&data, 5, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_xor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_STATUS_REG,
				&data, 4, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_zyxda_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_STATUS_REG,
				&data, 3, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_zda_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_STATUS_REG,
				&data, 2, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_yda_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_STATUS_REG,
				&data, 1, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_xda_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_STATUS_REG,
				&data, 0, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_fifomode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client,L3G_ADDR_FIFO_CTRL_REG,
				&data, 5, 3) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_fifomode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	clearbits = L3G_REG_FIFO_MODE_BITS;
	setbits = (data & 0x07) << 5;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_FIFO_CTRL_REG,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t l3gd20_watermk_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client,L3G_ADDR_FIFO_CTRL_REG,
				&data, 0, 5) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_watermk_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	clearbits = L3G_REG_FIFO_WATWRMK_BITS;
	setbits = data & 0x1f;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_FIFO_CTRL_REG,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t l3gd20_watermk_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_FIFO_SRC_REG,
				&data, 7, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_fifoovrn_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_FIFO_SRC_REG,
				&data, 6, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_fifoempty_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_FIFO_SRC_REG,
				&data, 5, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_fifolevel_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_FIFO_SRC_REG,
				&data, 0, 5) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_ble_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	if (l3gd20_smbus_read_bits(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG4,
				&data, 6, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t l3gd20_ble_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_data *l3gd20 = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	clearbits = L3G_REG4_BLE_BITS;
	setbits = (data & 0x01) << 6;

	if (modify_reg(l3gd20->l3gd20_client, L3G_ADDR_CTRL_REG4,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_reg_show, l3gd20_reg_store);
static DEVICE_ATTR(gyro, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_gyro_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_delay_show, l3gd20_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_enable_show, l3gd20_enable_store);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_position_show, l3gd20_position_store);
static DEVICE_ATTR(axis, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_axis_show, l3gd20_axis_store);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_range_show, l3gd20_range_store);
static DEVICE_ATTR(sample, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_sample_show, l3gd20_sample_store);
static DEVICE_ATTR(bandwth, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_bandwth_show, l3gd20_bandwth_store);
static DEVICE_ATTR(highpass_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_highpass_mode_show, l3gd20_highpass_mode_store);
static DEVICE_ATTR(highpass_cutoff_fre, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_highpass_cutoff_fre_show, l3gd20_highpass_cutoff_fre_store);
static DEVICE_ATTR(bdu, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_bdu_show, l3gd20_bdu_store);
static DEVICE_ATTR(temperature, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_temperature_show, NULL);
static DEVICE_ATTR(hpen, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_hpen_show, l3gd20_hpen_store);
static DEVICE_ATTR(fifo_en, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_fifo_en_show, l3gd20_fifo_en_store);
static DEVICE_ATTR(zyxor, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_zyxor_show, NULL);
static DEVICE_ATTR(zor, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_zor_show, NULL);
static DEVICE_ATTR(yor, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_yor_show, NULL);
static DEVICE_ATTR(xor, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_xor_show, NULL);
static DEVICE_ATTR(zyxda, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_zyxda_show, NULL);
static DEVICE_ATTR(zda, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_zda_show, NULL);
static DEVICE_ATTR(yda, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_yda_show, NULL);
static DEVICE_ATTR(xda, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_xda_show, NULL);
static DEVICE_ATTR(fifomode, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_fifomode_show, l3gd20_fifomode_store);
static DEVICE_ATTR(watermk, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_watermk_show, l3gd20_watermk_store);
static DEVICE_ATTR(watermk_status, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_watermk_status_show, NULL);
static DEVICE_ATTR(fifoovrn, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_fifoovrn_show, NULL);
static DEVICE_ATTR(fifoempty, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_fifoempty_show, NULL);
static DEVICE_ATTR(fifolevel, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_fifolevel_show, NULL);
static DEVICE_ATTR(ble, S_IRUGO|S_IWUSR|S_IWGRP,
		l3gd20_ble_show, l3gd20_ble_store);

static struct attribute *l3gd20_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_gyro.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_position.attr,
	&dev_attr_axis.attr,
	&dev_attr_range.attr,
	&dev_attr_sample.attr,
	&dev_attr_bandwth.attr,
	&dev_attr_highpass_mode.attr,
	&dev_attr_highpass_cutoff_fre.attr,
	&dev_attr_bdu.attr,
	&dev_attr_temperature.attr,
	&dev_attr_hpen.attr,
	&dev_attr_fifo_en.attr,
	&dev_attr_zyxor.attr,
	&dev_attr_zor.attr,
	&dev_attr_yor.attr,
	&dev_attr_xor.attr,
	&dev_attr_zyxda.attr,
	&dev_attr_zda.attr,
	&dev_attr_yda.attr,
	&dev_attr_xda.attr,
	&dev_attr_fifomode.attr,
	&dev_attr_watermk.attr,
	&dev_attr_watermk_status.attr,
	&dev_attr_fifoovrn.attr,
	&dev_attr_fifoempty.attr,
	&dev_attr_fifolevel.attr,
	&dev_attr_ble.attr,
	NULL
};

static struct attribute_group l3gd20_attribute_group = {
	.attrs = l3gd20_attributes
};

/* periodically read the gyro data and reported. */
static void l3gd20_work_func(struct work_struct *work)
{
	int result;
	struct l3gd20_data *l3gd20 = container_of((struct delayed_work *)work,
			struct l3gd20_data, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&l3gd20->delay));
	static struct l3gd20_gyro gyro;

	result = l3gd20_read_data(l3gd20->l3gd20_client, &gyro);
	if (result == 0) {
		input_report_abs(l3gd20->input, ABS_X, gyro.x);
		input_report_abs(l3gd20->input, ABS_Y, gyro.y);
		input_report_abs(l3gd20->input, ABS_Z, gyro.z);
		input_sync(l3gd20->input);
	}

	schedule_delayed_work(&l3gd20->work, delay);
}

static int l3gd20_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err, cfg_position;
	int cfg_calibration[3];
	struct l3gd20_data *data;
	struct input_dev *dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "i2c_check_functionality error\n");
		err = -EINVAL;
		goto exit;
	}

	data = kzalloc(sizeof(struct l3gd20_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->l3gd20_client = client;
	mutex_init(&data->enable_mutex);

	INIT_DELAYED_WORK(&data->work, l3gd20_work_func);
	atomic_set(&data->delay, L3G_MAX_DELAY);
	atomic_set(&data->enable, 0);

	cfg_position = -3;
	atomic_set(&data->position, cfg_position);
	atomic_set(&data->calibrated, 0);
	atomic_set(&data->fuzz, L3G_FUZZ);

	err = l3gd20_hw_init(data->l3gd20_client);
	if (err < 0) {
		printk(KERN_ERR "l3gd20 hw_init fail! err:%d\n", err);
		goto kfree_exit;
	}

	dev = input_allocate_device();
	if (!dev) {
		printk(KERN_ERR "l3gd20 input_allocate_device fail!\n");
		err = -ENOMEM;
		goto kfree_exit;
	}
	dev->name = L3G_SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, L3G_ABSMIN, L3G_ABSMAX, L3G_FUZZ, 0);
	input_set_abs_params(dev, ABS_Y, L3G_ABSMIN, L3G_ABSMAX, L3G_FUZZ, 0);
	input_set_abs_params(dev, ABS_Z, L3G_ABSMIN, L3G_ABSMAX, L3G_FUZZ, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		printk(KERN_ERR "l3gd20 input_register_device fail!\n");
		input_free_device(dev);
		goto kfree_exit;
	}

	data->input = dev;
	err = sysfs_create_group(&data->input->dev.kobj,
			&l3gd20_attribute_group);
	if (err < 0)
		goto error_sysfs;

	memset(cfg_calibration, 0, sizeof(cfg_calibration));

	data->offset.x = (signed short)cfg_calibration[0];
	data->offset.y = (signed short)cfg_calibration[1];
	data->offset.z = (signed short)cfg_calibration[2];

	l3gd20_do_enable(&client->dev, 1);
	dev_info(&data->l3gd20_client->dev,
			"Successfully initialized l3gd20!\n");
	return 0;

error_sysfs:
	input_unregister_device(data->input);
kfree_exit:
	kfree(data);
exit:
	return err;
}

static int __devexit l3gd20_remove(struct i2c_client *client)
{
	struct l3gd20_data *data = i2c_get_clientdata(client);

	l3gd20_set_enable(&client->dev, 0);
	sysfs_remove_group(&data->input->dev.kobj, &l3gd20_attribute_group);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static int l3gd20_suspend(struct i2c_client *client, pm_message_t state)
{
	l3gd20_do_enable(&client->dev, 0);

	return 0;
}

static int l3gd20_resume(struct i2c_client *client)
{
	struct l3gd20_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	l3gd20_hw_init(data->l3gd20_client);
	l3gd20_do_enable(dev, atomic_read(&data->enable));

	return 0;
}

static const struct i2c_device_id l3gd20_id[] = {
	{L3G_SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, l3gd20_id);

static struct i2c_driver l3gd20_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= L3G_SENSOR_NAME,
	},
	.class			= I2C_CLASS_HWMON,
	.id_table		= l3gd20_id,
	.probe			= l3gd20_probe,
	.remove			= __devexit_p(l3gd20_remove),
	.suspend 		= l3gd20_suspend,
	.resume  		= l3gd20_resume,
};

static int __init l3gd20_init(void)
{
	return i2c_add_driver(&l3gd20_driver);
}

static void __exit l3gd20_exit(void)
{
	i2c_del_driver(&l3gd20_driver);
}

MODULE_AUTHOR("Sarlor <kinsleer@outlook.com>");
MODULE_DESCRIPTION("L3GD20 Driver");
MODULE_LICENSE("GPL");

module_init(l3gd20_init);
module_exit(l3gd20_exit);
