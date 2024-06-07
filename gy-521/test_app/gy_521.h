#ifndef DRIVER_GY_521_H_
#define DRIVER_GY_521_H_

/**
 * divider from the gyroscope output rate
 * used to generate the Sample Rate
 * formula:
 *    Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) 
 */
#define REG_SMPRT_DIV 		0x19    
//|----------------------------------------------------------------------------|
//|          |        Accelerometer      |                Gyroscope            |
//| DLPF_CFG |-----------------------------------------------------------------|
//|          | Bandwidth(Hz) | Delay(ms) | Bandwidth(Hz) | Delay(ms) | Fs(kHz) |
//|----------------------------------------------------------------------------|
//|    0     |      260      |    0      |      256      |    0.98   |    8    |
//|    1     |      184      |    2.0    |      188      |    1.9    |    1    |
//|    2     |      94       |    3.0    |      98       |    2.8    |    1    |
//|    3     |      44       |    4.9    |      42       |    4.8    |    1    |
//|    4     |      21       |    8.5    |      20       |    8.3    |    1    |
//|    5     |      10       |    13.8   |      10       |    13.4   |    1    |
//|    6     |      5        |    19.0   |      5        |    18.6   |    1    |
//|    7     |    RESERVED   | RESERVED  |    RESERVED   | RESERVED  |    1    |
//|----------------------------------------------------------------------------|
#define REG_CONFIG 			0x1A
#define DLPF_CFG_0 			0	
#define DLPF_CFG_1 			1	
#define DLPF_CFG_2 			2	
#define DLPF_CFG_3 			3	
#define DLPF_CFG_4 			4	
#define DLPF_CFG_5 			5	
#define DLPF_CFG_6 			6	
#define DLPF_CFG_7 			7	

// Gyroscope Configuration (FS_SEL)
#define REG_GYRO_CONFIG		  0x1B
#define GYRO_SCALE_RANGE_250  0
#define GYRO_SCALE_RANGE_500  1
#define GYRO_SCALE_RANGE_1000 2
#define GYRO_SCALE_RANGE_2000 3
#define FS_SEL(val)			(val<<3)

// Accelerometer Configuration (AFSEL)
#define REG_ACCEL_CONFIG		0x1C 
#define ACCEL_SCALE_RANGE_2G  0
#define ACCEL_SCALE_RANGE_4G  1
#define ACCEL_SCALE_RANGE_8G  2
#define ACCEL_SCALE_RANGE_16G 3
#define AFS_SEL(val)		(val<<3)

// Which sensor measurements are loaded
// into the FIFO buffer.
#define REG_FIFO_EN			0x23 		
#define ACCEL_FIFO_EN		(1<<3)
#define ZG_FIFO_EN			(1<<4)
#define YG_FIFO_EN			(1<<5)
#define XG_FIFO_EN			(1<<6)
#define TEMP_FIFO_EN		(1<<7)
									
#define REG_ACCEL_XOUT_H	0x3B	//Register for Accelerometer X 
#define REG_ACCEL_XOUT_L	0x3C	
#define REG_ACCEL_YOUT_H	0x3D	//Register for Accelerometer Y 
#define REG_ACCEL_YOUT_L	0x3E	
#define REG_ACCEL_ZOUT_H	0x3F	//Register for Accelerometer Z 
#define REG_ACCEL_ZOUT_L	0x40	

#define REG_TEMP_OUT_H		0x41	//Register for Temperature
#define REG_TEMP_OUT_L		0x42

#define REG_GYRO_XOUT_H		0x43	//Register for Accelerometer X 
#define REG_GYRO_XOUT_L		0x44	
#define REG_GYRO_YOUT_H		0x45	//Register for Accelerometer Y 
#define REG_GYRO_YOUT_L		0x46	
#define REG_GYRO_ZOUT_H		0x47	//Register for Accelerometer Z 
#define REG_GYRO_ZOUT_L		0x48	

#define REG_SIGNAL_PATH_RESET 	0x68
#define TEMP_RESET	 	  	(1<<0)
#define ACCEL_RESET			(1<<1)
#define GYRO_RESET			(1<<2)

#define REG_USER_CTRL 			0x6A	// User control Register
#define SIG_COND_RESET		(1<<0)
#define I2C_MST_RESET		(1<<1)
#define FIFO_RESET			(1<<2)
#define I2C_IF_DIS			(1<<4)
#define I2C_MST_EN			(1<<5)
#define FIFO_EN				(1<<6)

#define REG_PWR_MGMT_1 			0x6B	// Power Management Register
#define INT_8M_HZ_CLK			0
#define PLL_X_AXIS_GYRO_REF		1
#define PLL_Y_AXIS_GYRO_REF		2
#define PLL_Z_AXIS_GYRO_REF		3
#define PLL_EXT_32K768_HZ_REF	4
#define PLL_EXT_19M2_HZ_REF		5
#define STOP_AND_KEEP_IN_RESET	7
#define TEMP_DIS				(1<<3)
#define CYCLE					(1<<5)
#define SLEEP					(1<<6)
#define DEVICE_RESET			(1<<7)

#define REG_PWR_MGMT_2 		0x6C
#define STBY_ZG			(1<<0)		
#define STBY_YG			(1<<1)
#define STBY_XG			(1<<2)
#define STBY_ZA			(1<<3)
#define STBY_YA			(1<<4)
#define STBY_XA			(1<<5)
#define WK_1_25_HZ		0
#define WK_5_HZ			1
#define WK_20_HZ		2
#define WK_40_HZ		3
#define LP_WAKE_CTRL(val) (val<<6)

// FIFO Count Register 15:8 (0x73 for 7:0)
#define REG_FIFO_COUNT_H 		0x72          
#define REG_FIFO_COUNT_L 		0x73          
#define REG_FIFO_R_W 			0x74          // FIFO buffer
										  
#define WHO_AM_I_ADDR 		0x75
#define ID(val)				(0x7E&val)	

typedef struct accel_xyz_data {
	int x;
	int y;
	int z;
}accel_xyz_data;

typedef struct gyro_xyz_data {
	int x;
	int y;
	int z;
}gyro_xyz_data;

typedef struct mpu_data {
	unsigned char gyro_sens;
	unsigned char accl_sens;
	struct accel_xyz_data accel_data;
	struct gyro_xyz_data gyro_data;
	int temp;
}mpu_data;

typedef struct mpu_data_reg {
	unsigned char reg;
	unsigned char val;
}mpu_data_reg;



// That is kinds of cmd which select cmd from user space. 
#define	READ_ALL 	0 		// Read Accelerometer, Gyroscope and Temperature.
#define	READ_ACCEL 	1 		// only Read Accelerometer.
#define	READ_GYRO 	2 		// only Read Gyroscope.
#define	READ_TEMP 	3 		// only Read Temperature.
#define READ_REG 	4		// Read any register.
#define	WRITE_REG 	5		// Read any register.

#endif /* DRIVER_GY_521_H_ */
