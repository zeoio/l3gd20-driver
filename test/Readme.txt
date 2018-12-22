默认配置:
	1.  电源模式：正常
	2.  Z/Y/X轴启动
	3.  高通滤波模式: 一般模式(重置), 禁止启动
	4.  高通滤波截止频率: 51.4HZ
	5.  开启数据收发准备状态在INT2
	6.  BDU为1 (输出数据不更新, 直到MSB和LSB被读)
	7.  重启内存目录, 正常模式
	8.  启用FIFO, FIFO Bypass模式
	11. 设置Rate为760，Bandwidth为50
	12. 设置Range为2000DPS
	
配置宏:
	包含 #include "l3gd20.h"
	这些宏的值都为数字
	
	L3G_GET_PATH()				得到某个接口的路径， 宏值为路径字符串
								L3G_GET_PATH(reg):	/sys/devices/virtual/input/input1/reg
	
	L3G_RATE_95HZ				输出频率95HZ
	L3G_RATE_190HZ				输出频率190HZ
	L3G_RATE_380HZ				输出频率380HZ
	L3G_RATE_760HZ				输出频率760HZ
	
	L3G_BANDWTH_0				波宽0	(和Rate一起使用确定cutoff的值， 不单独使用)
	L3G_BANDWTH_1				波宽1
	L3G_BANDWTH_2				波宽2
	L3G_BANDWTH_3				波宽3
	
	L3G_RATE_95HZ_BANDWTH_12_5	输出频率95HZ, cutoff为12.5
	L3G_RATE_95HZ_BANDWTH_25	输出频率95HZ, cutoff为25
	L3G_RATE_95HZ_BANDWTH_25	输出频率95HZ, cutoff为25
	L3G_RATE_95HZ_BANDWTH_25	输出频率95HZ, cutoff为25

	L3G_RATE_190HZ_BANDWTH_12_5	输出频率190HZ, cutoff为12.5
	L3G_RATE_190HZ_BANDWTH_25	输出频率190HZ, cutoff为25
	L3G_RATE_190HZ_BANDWTH_50	输出频率190HZ, cutoff为50
	L3G_RATE_190HZ_BANDWTH_70	输出频率190HZ, cutoff为70

	L3G_RATE_380HZ_BANDWTH_20	输出频率380HZ, cutoff为20
	L3G_RATE_380HZ_BANDWTH_25	输出频率380HZ, cutoff为25
	L3G_RATE_380HZ_BANDWTH_50	输出频率380HZ, cutoff为50
	L3G_RATE_380HZ_BANDWTH_100	输出频率380HZ, cutoff为100

	L3G_RATE_760HZ_BANDWTH_30	输出频率760HZ, cutoff为30
	L3G_RATE_760HZ_BANDWTH_35	输出频率760HZ, cutoff为35
	L3G_RATE_760HZ_BANDWTH_50	输出频率760HZ, cutoff为50
	L3G_RATE_760HZ_BANDWTH_100	输出频率760HZ, cutoff为100
	
	L3G_POWER_DOWN				关闭电源
	L3G_POWER_NORMAL			一般模式(当X/Y/Z模式为disable时， 为休眠模式)

	L3G_ZYX_AXIS_DIS			关闭X/Y/Z轴
	L3G_ZYX_AXIS_EN				启动X/Y/Z轴

	L3G_HIGHPASS_RESET_NORMAL	高通滤波一般模式(重置)
	L3G_HIGHPASS_REF			高通滤波参考滤波信号模式
	L3G_HIGHPASS_NORMAL			高通滤波普通模式
	L3G_HIGHPASS_AUTORESET		高通滤波中断事件自动重启
	
	L3G_HIGHPASS_CUTOFF_0		高通滤波截止频率选择0, 和Rate一起确定截止频率(see l3gd20.h)
	L3G_HIGHPASS_CUTOFF_1		
	L3G_HIGHPASS_CUTOFF_2		
	L3G_HIGHPASS_CUTOFF_3		
	L3G_HIGHPASS_CUTOFF_4		
	L3G_HIGHPASS_CUTOFF_5		
	L3G_HIGHPASS_CUTOFF_6		
	L3G_HIGHPASS_CUTOFF_7		
	L3G_HIGHPASS_CUTOFF_8		
	L3G_HIGHPASS_CUTOFF_9		
	
	L3G_BDU_CONTN_UPDATA		块数据更新: 连续更新
	L3G_BDU_NOT_UPDATE			块数据更新: 输出寄存器不更新

	L3G_RANGE_250DPS			范围选择:	250DPS
	L3G_RANGE_500DPS			范围选择:	500DPS
	L3G_RANGE_2000DPS			范围选择:	2000DPS
	L3G_RANGE_2000DPS			范围选择:	2000DPS

	L3G_FIFO_DIS				FIFO禁用
	L3G_FIFO_EN					FIFO使能

	L3G_HIGHPASS_DIS			高通滤波禁止
	L3G_HIGHPASS_EN				高通滤波使能

	L3G_STATUS_NOT_HAVE			X/Y/Z没有数据溢出  (作用于zyxor, zor, yor, xor接口时)
								X/Y/Z没有可用的数据(作用于zyxda, zda, yda, xda接口时)
	L3G_STATUS_HAVE				X/Y/Z有数据溢出    (作用于zyxor, zor, yor, xor接口时)
								X/Y/Z有可用的数据  (作用于zyxda, zda, yda, xda接口时)

	L3G_FIFO_MODE_BYPASS		FIFO模式: Bypass mode
	L3G_FIFO_MODE_FIFO			FIFO模式: FIFO mode
	L3G_FIFO_MODE_STREAM		FIFO模式: Stream mode
	L3G_FIFO_MODE_STREAM_FIFO	FIFO模式: Stream-to-FIFO mode
	L3G_FIFO_MODE_BYPASS_STREAM	FIFO模式: Bypass-to-Stream mode

	L3G_FIFO_STATUS_NOT_HAVA	FIFO填充低于WTM级, FIFO未完全填充,   FIFO非空，时使用此宏
	L3G_FIFO_STATUS_HAVE		FIFO填充大于等于WTM级, FIFO完全填充, FIFO为空，时使用此宏
	
操作接口:
	reg:				寄存器接口, 可查看寄存器的值和修改寄存器的值
						输入输出格式: "0x%02x:0x%02x" 				(例如: "0x02:0xef")
						read(L3G_GET_PATH(reg), buf, 100);			读取全部寄存器的值
																	各个值之间以空格分割(注意: buf不能太小)
						write(L3G_GET_PATH(reg), "0x02:0xef", 10);	修改寄存器0x02值为0xef 
			
	gyro:				陀螺仪数据值
						输出格式: "%d %d %d"						(例如: "12 -11 123")
						read(L3G_GET_PATH(gyro), buf, 10);			读取陀螺仪数据值, 输出的值为字符串数组
	
	delay:				延迟接口				
						输入输出格式: "%d"							(例如: "50")
						read(L3G_GET_PATH(delay), buf, 5);			读取值
						write(L3G_GET_PATH(reg), "12", 3);			修改值为12	
						
	enable				陀螺仪电源管理
						输入输出格式: "%d", 输入输出值为(0-1)
						0: 断电, 1: 启用
						
	position			未知
	axis				启动/禁用X/Y/Z轴
						输入输出格式： "%d", 输入输出值为(0-6)
						0: X/Y/Z全部禁止, 1: X轴使能, 2: Y轴使能, 6: Z轴使能 (可以使用或运算)
						
	range				获取/设置量程选择
						输入输出格式： "%d", 输入输出值为(250DPS, 500DPS, 2000DPS)
						
	sample				获取/设置输出数据速率
						输入输出格式: "%d", 输入输出值为(95HZ, 190HZ, 380HZ, 760HZ)
						
	bandwth				获取/设置波宽选择
						输入输出格式： "%d", 输入输出值为(0-3) 
						bandwth需要和rate一同使用， 才能设置odr和cutoff
						rate		bandwth		odr		cutoff
						00 			00 			95 		12.5
						00 			01 			95 		25
						00 			10 			95 		25
						00 			11 			95 		25
						01 			00 			190 	12.5
						01 			01 			190 	25
						01 			10 			190 	50
						01 			11 			190 	70
						10 			00 			380 	20
						10 			01 			380 	25
						10 			10 			380 	50
						10 			11 			380 	100
						11 			00 			760 	30
						11 			01 			760 	35
						11 			10 			760 	50
						11 			11 			760 	100
	
	highpass_mode		获取/设置高通滤波模式选择
						输入输出格式: "%d",  输入输出值为(0-3) 
						0: 一般模式(重置), 1: 参考滤波信号, 2: 普通模式, 3: 中断事件自动重启
						
	highpass_cutoff_fre	获取/设置高通滤波截止频率选择
						输入输出格式: "%d", 输入输出值为(0-9)
						此设置值， 和rate一同作用
						config		odr=95HZ	odr=190HZ	odr=380HZ	odr=760HZ
						0000 		7.2 		13.5 		27 			51.4
						0001 		3.5 		7.2 		13.5 		27
						0010 		1.8 		3.5 		7.2 		13.5
						0011 		0.9 		1.8 		3.5 		7.2
						0100 		0.45 		0.9 		1.8 		3.5
						0101 		0.18 		0.45 		0.9 		1.8
						0110 		0.09 		0.18 		0.45 		0.9
						0111 		0.045 		0.09 		0.18 		0.45
						1000 		0.018 		0.045 		0.09 		0.18
						1001 		0.009 		0.018 		0.045 		0.09
	
	bdu					获取/设置快数据更新
						输入输出格式: "%d",  输入输出值为(0-1)
						0： 连续更新, 1: 输出寄存器不更新
						
	temp				获取温度值
						输出格式: "%d"
	
	hpen				获得/设置高通滤波使能
						输入输出格式: "%d",  输入输出值为(0-1)
						0: 禁止, 1: 使能
						
	fifo_en				获得/设置FIFO使能
						输入输出格式: "%d",  输入输出值为(0-1)
						0: 禁止, 1: 使能
						
	zyxor				获取X/Y/Z轴数据是否溢出
						输出格式: "%d",  输出值为(0-1)
						0: 没有数据溢出, 1: 溢出
	
	zor					获取Z轴数据是否溢出
						输出格式: "%d",  输出值为(0-1)
						0: 没有数据溢出, 1: 溢出
	
	yor					获取Y轴数据是否溢出
						输出格式: "%d",  输出值为(0-1)
						0: 没有数据溢出, 1: 溢出
						
	xor					获取X轴数据是否溢出
						输出格式: "%d",  输出值为(0-1)
						0: 没有数据溢出, 1: 溢出
						
	zyxda				获取X/Y/Z轴数据是否有新数据可用
						输出格式: "%d",  输出值为(0-1)
						0: 没有可用数据, 1: 有
						
	zda					获取Z轴数据是否有新数据可用
						输出格式: "%d",  输出值为(0-1)
						0: 没有可用数据, 1: 有
						
	yda					获取Y轴数据是否有新数据可用
						输出格式: "%d",  输出值为(0-1)
						0: 没有可用数据, 1: 有
						
	xda					获取X轴数据是否有新数据可用
						输出格式: "%d",  输出值为(0-1)
						0: 没有可用数据, 1: 有
						
	fifomode			获取/设置FIFO模式
						输入输出格式: "%d", 输入输出值为(0-4)
						0: Bypass mode, 1: FIFO mode, 2: Stream mode
						3: Stream-to-FIFO mode, 4: Bypass-to-Stream mode
						
	watermk				获取/设置FIFO水位等级
						输入输出格式: "%d"
						
	watermk_status		获取水位等级的状态
						输出格式: "%d", 输出值为(0-1)
						0: FIFO填充低于WTM级, 1: FIFO填充大于等于WTM级
						
	fifoovrn			获取FIFO是否溢出
						输出格式: "%d", 输出值为(0-1)
						0: FIFO未完全填充, 1: FIFO完全填充
						
	fifoempty			获取FIFO是否为空
						输出格式: "%d", 输出值为(0-1)
						0: FIFO不空, 1: FIFO为空
						
	fifolevel			获取FIFO存储的数据等级
						输出格式: "%d"