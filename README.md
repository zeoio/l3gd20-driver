L3GD20 MEMS Motion Sensor, Three-axis Digital Output Gyroscope.

Directory introduction:
        doc:    development documentation
        src:    driver source code
        test:   test program source code
        tmp:    temporary intermediate files during development

L3GD20 three-axis digital output gyroscope Develop documentation. Development
this driver is divided into three parts. First write the device driver file,
Then add the board support code. Third, modify the kernel configuration.

1. Frist enter the 'src' directory.
        ===> $: cd src
2. Edit the 'Makefile' file, set the correct kernel directory 'KERN_DIR'.
        ===> $: vim Makefile
3. Copy the driver files to the 'hwmon' directory.
        ===> $: make cp
4. Add board-level support code.
        ===> $: vim arch/arm/plat-s5p4418/realarm/device.c
5. Add the code in the appropriate place.
        CODE 1:

                #if defined(CONFIG_SENSORS_L3GD20) || defined(CONFIG_SENSORS_L3GD20_MODULE)
                #include <linux/i2c.h>

                /* CODEC */
                static struct i2c_board_info __initdata l3gd20_i2c_bdi = {
                        .type   = "l3gd20",
                        .addr   = 0x6A
                };

                #endif

        CODE 2:

                #if defined(CONFIG_SENSORS_L3GD20) || defined(CONFIG_SENSORS_L3GD20_MODULE)
                        printk("plat: add gyroscope l3gd20\n");
                        i2c_register_board_info(1, &l3gd20_i2c_bdi, 1);
                #endif
7. Modify the kernel configuration file for l3gd20.
        ===> $: vim drivers/hwmon/Kconfig
        ADD CODE:

                config SENSORS_L3GD20
                        tristate "Three axis digital output gyroscope"
                        depends on I2C
                        default n
                        help
                                Say Y here to enable the gyroscope.

                                If unsure, say N.

        ===> $: vim drivers/hwmon/Makefile
        ADD CODE:

                obj-$(CONFIG_SENSORS_L3GD20)    += l3gd20.o

8. Configure the kernel to use this driver.
        ===> $: make menuconfig
                Device Drivers -> Hardware Monitoring support -> Three axis digital output gyroscope

        Check, save.
