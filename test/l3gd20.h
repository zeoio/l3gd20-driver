#ifndef _SENSOR_L3GD20_H
#define _SENSOR_L3GD20_H

/* L3GD20 three-axis digital output gyroscope
 *
 * Interface			RW status		Description	 			I/O farmat
 * reg					RW				Register interface		"0x%02x:0x%02x"
 * gyro					Read only		Gyroscope data			"%d %d %d"
 * delay				RW				Delay interface			"%d"
 * enable				RW				Power mode enable		"%d"
 * position				RW				Don't known				"%d"
 * axis					RW				Z/Y/X axis enable		"%d"
 * range				RW				Full scale selection	"%d"
 * sample				RW				Output date rate		"%d"
 * bandwth				RW				Bandwidth selection		"%d"
 * highpass_mode		RW				High-pass mode			"%d"
 * highpass_cutoff_fre	RW				High-pass cutoff freq	output: "%d:%d", input: "%d"
 * bdu					RW				Block data update		"%d"
 * temperature			Read only		Temperature data		"%d"
 * hpen					RW				High-pass enable		"%d"
 * fifo_en				RW				FIFO enable				"%d"
 * zyxor				Read only		X/Y/Z axis data overrun	"%d"
 * zor					Read only		Z axis data overrun		"%d"
 * yor					Read only		Y axis data overrun		"%d"
 * xor					Read only		X axis data overrun		"%d"
 * zyxda				Read only		X/Y/Z axis has new data "%d"
 * zda					Read only		Z axis has new data		"%d"
 * yda					Read only		Y axis has new data		"%d"
 * xda					Read only		X axis has new data		"%d"
 * fifomode				RW				FIFO mode selection		"%d"
 * watermk				RW				FIFO watermark level	"%d"
 * watermk_status		RW				Watermark status		"%d"
 * fifoovrn				RW				FIFO overrun status		"%d"
 * fifoempty			RW				FIFO empty status		"%d"
 * fifolevel			RW				FIFO stored data level	"%d"
 *
 *  */
#define L3G_PATH_BASE			"/sys/devices/virtual/input/input1/"

/* Output data rate selection */
#define L3G_RATE_95HZ			0x00
#define L3G_RATE_190HZ			0x01
#define L3G_RATE_380HZ			0x02
#define L3G_RATE_760HZ			0x03

/* Bandwidth selection */
#define L3G_BANDWTH_0			0x00
#define L3G_BANDWTH_1			0x01
#define L3G_BANDWTH_2			0x02
#define L3G_BANDWTH_3			0x03

/* Rate and Bandwidth config setting */
#define L3G_RATE_95HZ_BANDWTH_12_5	((L3G_RATE_95HZ << 2) | L3G_BANDWTH_0)
#define L3G_RATE_95HZ_BANDWTH_25	((L3G_RATE_95HZ << 2) | L3G_BANDWTH_1)
#define L3G_RATE_95HZ_BANDWTH_25	((L3G_RATE_95HZ << 2) | L3G_BANDWTH_2)
#define L3G_RATE_95HZ_BANDWTH_25	((L3G_RATE_95HZ << 2) | L3G_BANDWTH_3)

#define L3G_RATE_190HZ_BANDWTH_12_5	((L3G_RATE_190HZ << 2) | L3G_BANDWTH_0)
#define L3G_RATE_190HZ_BANDWTH_25	((L3G_RATE_190HZ << 2) | L3G_BANDWTH_1)
#define L3G_RATE_190HZ_BANDWTH_50	((L3G_RATE_190HZ << 2) | L3G_BANDWTH_2)
#define L3G_RATE_190HZ_BANDWTH_70	((L3G_RATE_190HZ << 2) | L3G_BANDWTH_3)

#define L3G_RATE_380HZ_BANDWTH_20	((L3G_RATE_380HZ << 2) | L3G_BANDWTH_0)
#define L3G_RATE_380HZ_BANDWTH_25	((L3G_RATE_380HZ << 2) | L3G_BANDWTH_1)
#define L3G_RATE_380HZ_BANDWTH_50	((L3G_RATE_380HZ << 2) | L3G_BANDWTH_2)
#define L3G_RATE_380HZ_BANDWTH_100	((L3G_RATE_380HZ << 2) | L3G_BANDWTH_3)

#define L3G_RATE_760HZ_BANDWTH_30	((L3G_RATE_760HZ << 2) | L3G_BANDWTH_0)
#define L3G_RATE_760HZ_BANDWTH_35	((L3G_RATE_760HZ << 2) | L3G_BANDWTH_1)
#define L3G_RATE_760HZ_BANDWTH_50	((L3G_RATE_760HZ << 2) | L3G_BANDWTH_2)
#define L3G_RATE_760HZ_BANDWTH_100	((L3G_RATE_760HZ << 2) | L3G_BANDWTH_3)

/* Power-down mode enable */
#define L3G_POWER_DOWN			0
#define L3G_POWER_NORMAL		1

/* Z/Y/X axis enable */
#define L3G_ZYX_AXIS_DIS		0		
#define L3G_X_AXIS_EN			(1 << 0)
#define L3G_Y_AXIS_EN			(1 << 1)
#define L3G_Z_AXIS_EN			(1 << 2)

/* High-pass filter mode */
#define L3G_HIGHPASS_RESET_NORMAL	0x00	/* Normal mode (reset) */
#define L3G_HIGHPASS_REF		0x01	/* Reference signal for filtering*/
#define L3G_HIGHPASS_NORMAL		0x02	/* Normal mode */
#define L3G_HIGHPASS_AUTORESET		0x03	/* Autoreset on interrupt event */

/* High-pass filter cutoff frequency selection
 * Example: L3G_HIGHPASS_CUTOFF_0 && L3G_RATE_95HZ == 7.2
 * Config				value Rate: 95    190   380   760   */
#define L3G_HIGHPASS_CUTOFF_0		0X00 /*     7.2   13.5  27    51.4  */
#define L3G_HIGHPASS_CUTOFF_1		0X01 /*     3.5   7.2   13.5  27    */
#define L3G_HIGHPASS_CUTOFF_2		0X02 /*     1.8   3.5   7.2   13.5  */
#define L3G_HIGHPASS_CUTOFF_3		0X03 /*     0.9   1.8   3.5   7.2   */
#define L3G_HIGHPASS_CUTOFF_4		0X04 /*     0.45  0.9   1.8   3.5   */
#define L3G_HIGHPASS_CUTOFF_5		0X05 /*     0.18  0.45  0.9   1.8   */
#define L3G_HIGHPASS_CUTOFF_6		0X06 /*     0.09  0.18  0.45  0.9   */
#define L3G_HIGHPASS_CUTOFF_7		0X07 /*     0.045 0.09  0.18  0.45  */
#define L3G_HIGHPASS_CUTOFF_8		0X08 /*     0.018 0.045 0.09  0.18  */
#define L3G_HIGHPASS_CUTOFF_9		0X09 /*     0.009 0.018 0.045 0.09  */

/* Block data update */
#define L3G_BDU_CONTN_UPDATA		0	/* Continuos update */
#define L3G_BDU_NOT_UPDATE		1	/* Output registers not update */

/* Full scale selection */
#define L3G_RANGE_250DPS		0x00
#define L3G_RANGE_500DPS		0x01
#define L3G_RANGE_2000DPS		0x02
#define L3G_RANGE_2000DPS		0x03

/* FIFO enable */
#define L3G_FIFO_DIS			0
#define L3G_FIFO_EN			1

/* High-pass filter enable */
#define L3G_HIGHPASS_DIS		0
#define L3G_HIGHPASS_EN			1

/* Status for X/Y/Z axis data overrun and new data available
 * no overrun has occurred and new data isn't available is 0, otherwise is 1. */
#define L3G_STATUS_NOT_HAVE		0
#define L3G_STATUS_HAVE			1

/* FIFO mode selection */
#define L3G_FIFO_MODE_BYPASS		0x00
#define L3G_FIFO_MODE_FIFO		0x01
#define L3G_FIFO_MODE_STREAM		0x02
#define L3G_FIFO_MODE_STREAM_FIFO	0X03
#define L3G_FIFO_MODE_BYPASS_STREAM	0X04

/* FIFO status for watermark overrun empty.
 * When FIFO filling is lower than WTM level, FIFO is not completely filled,
 * FIFO not empty is 0, otherwise is 1. */
#define L3G_FIFO_STATUS_NOT_HAVA	0
#define L3G_FIFO_STATUS_HAVE		1

#endif	/* _SENSOR_L3GD20_H */
