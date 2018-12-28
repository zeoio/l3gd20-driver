### L3GD20 MEMS Motion Sensor Driver

L3GD20 MEMS Motion Sensor, Three-axis Digital Output Gyroscope.

#### Building the source

1. Clone the driver code
```bash
https://github.com/Sarlor/l3gd20-driver.git
cd  l3gd20-driver/src
```

2. Edit the 'Makefile' file, set the correct kernel directory 'KERN_DIR'
```bash
vim Makefile
```

3. Copy the driver files to the 'hwmon' directory
```bash
make cp
```

4. Add board-level support code to the appropriate place
```bash
vim arch/arm/plat-s5p4418/realarm/device.c
```

 ```c
                 #if defined(CONFIG_SENSORS_L3GD20) || defined(CONFIG_SENSORS_L3GD20_MODULE)
                #include <linux/i2c.h>

                /* CODEC */
                static struct i2c_board_info __initdata l3gd20_i2c_bdi = {
                        .type   = "l3gd20",
                        .addr   = 0x6A
                };

                #endif

                #if defined(CONFIG_SENSORS_L3GD20) || defined(CONFIG_SENSORS_L3GD20_MODULE)
                        printk("plat: add gyroscope l3gd20\n");
                        i2c_register_board_info(1, &l3gd20_i2c_bdi, 1);
                #endif
 ```

5. Modify the kernel configuration file for l3gd20, and add the code to the appropriate place
	```bash
	vim drivers/hwmon/Kconfig
	```

	```c
                config SENSORS_L3GD20
                        tristate "Three axis digital output gyroscope"
                        depends on I2C
                        default n
                        help
                                Say Y here to enable the gyroscope.

                                If unsure, say N.
	```

6. Edit Makefile for add l3gd20 driver module
	```bash
	vim drivers/hwmon/Makefile
	```

	```makefile
                obj-$(CONFIG_SENSORS_L3GD20)    += l3gd20.o
	```

7. Configure the kernel for required submodule, then save config
	```bash
	make menuconfig
	```

	```makefile
    Device Drivers -> Hardware Monitoring support -> Three axis digital output gyroscope
	```
