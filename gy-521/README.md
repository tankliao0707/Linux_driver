# GY-521 board 
GY-521 was uesing MPU6050 chip. This project create a kernel module driver.
The project develop on raspberry pi 3 that using i2c to write and read GY-521 board.
That read accelerometer, Gyroscope and temperature. 


# Usage 
1. How to complie driver.
    ```
    $ cd  ~/gy-521/driver
    $ make
    ```
2. How to complie test app.
    ```
    $ cd ~/gy-521/test_app
    $ make
    ```
3. Run app.sh shell script.
    ```
    $ sudo ./app.sh
    ``` 
    
# Driver Command list
 That is kinds of cmd which select cmd from user space. 
```
#define	READ_ALL 	0 		// Read Accelerometer, Gyroscope and Temperature.
#define	READ_ACCEL 	1 		// only Read Accelerometer.
#define	READ_GYRO 	2 		// only Read Gyroscope.
#define	READ_TEMP 	3 		// only Read Temperature.
#define READ_REG 	4		// Read any register.
#define	WRITE_REG 	5		// Read any register.
```
