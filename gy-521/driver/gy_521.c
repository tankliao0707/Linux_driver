#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include "gy_521.h"

#define I2C_BUS_NUMBER 		1
#define DEVICE_COUNT		1
#define MPU_SLAVE_ADDR 		0x68
#define SLAVE_DEVICE_BOARD 	"CY-521"
#define MPU_DRIVER_NAME		"mpu6050"
#define DRIVER_NAME 		"gy_521"
#define DRIVER_CLASS 		"mpu6050_Class"						  

static struct i2c_adapter *mpu_i2c_adapter = NULL;
static struct i2c_client *mpu_i2c_client = NULL;

static dev_t mpu_dev_t;
static struct class *mpu_dev_class = NULL;
static struct cdev mpu_cdev;
static mpu_data mpuData; 
static mpu_data_reg mpuDataReg; 

static const struct i2c_device_id mpu_i2c_device_id[] = {
		{ SLAVE_DEVICE_BOARD, 0 }, 
		{ }
};
MODULE_DEVICE_TABLE(i2c, mpu_i2c_device_id);

static struct i2c_board_info mpu_i2c_board_info = {
	I2C_BOARD_INFO(SLAVE_DEVICE_BOARD, MPU_SLAVE_ADDR)
};

/**
 * Write_Register() - Writes to a register.
 * @reg		: Register will be written.
 * @value	: Value will be written to the register.
 *
 * Return: Number of bytes written.
 */
static int Write_Register(unsigned char reg, unsigned int value) {
  unsigned char send_buffer[2] = {reg, value};
  int ret;

  ret = i2c_master_send(mpu_i2c_client, send_buffer, 2);
  if (ret != 2) {
    printk(KERN_DEBUG "Err writing register failed");
  }
  return ret;
}

/**
 * Write_Reg() - Writes to a register.
 * @mpu_data_reg: Pointer to a struct object of mpu_data structure.
 *                putting wants write register and val on reg and val, 
				  respectively
 */
static void Write_Reg(struct mpu_data_reg *data){
	Write_Register(data->reg, data->val );
}	

/**
 * Read_Regiter() - Read a single register.
 * @reg			: Register will be read.
 * @rec_buffer	: Pointer to a buffer that will store the read data.
 * Return		: bytes read.
 */
static int Read_Regiter(unsigned char reg, unsigned char *rec_buf) {
  struct i2c_msg msg[2];
  int ret =-1;

  msg[0].addr = mpu_i2c_client->addr;
  msg[0].flags = 0;
  msg[0].len = 1;
  msg[0].buf = &reg;
  msg[1].addr = mpu_i2c_client->addr;
  msg[1].flags = I2C_M_RD;
  msg[1].len = 1;
  msg[1].buf = rec_buf;

  ret = i2c_transfer(mpu_i2c_client->adapter, msg, 2);
  if (ret < 0) {
    printk(KERN_DEBUG "Err reading register 0x%X", reg);
  }
  return ret;
}

/**
 * Read_multiple_Regiter() - Read multiple registers.
 * @reg			: First register of the reading sequence.
 * @length		: Number of registers to be read.
 * @rec_buffer	: Pointer to a buffer that will store the read data.
 *
 * Return: bytes sent and received
 */
static int Read_Multiple_Regiter(unsigned char start_reg, unsigned int length,
                          unsigned char *rec_buffer) {
  int ret;
  struct i2c_msg msg[2];

  msg[0].addr = mpu_i2c_client->addr;
  msg[0].flags = 0;
  msg[0].len = 1;
  msg[0].buf = &start_reg;
  msg[1].addr = mpu_i2c_client->addr;
  msg[1].flags = I2C_M_RD;
  msg[1].len = length;
  msg[1].buf = rec_buffer;
  ret = i2c_transfer(mpu_i2c_client->adapter, msg, 2);
  if (ret < 0) {
    printk(KERN_DEBUG "Err reading multiple regiter 0x%X\n", start_reg);
  }
  return ret;
}

/**
 * Read_all() - Read Accelerometer, temperature, Gyroscope data.
 * @data : Pointer to a struct object of mpu_data structure.
 */
void Read_all(struct mpu_data *data) {
	uint8_t buf[14];
	Read_Multiple_Regiter(REG_ACCEL_XOUT_H, sizeof(buf), buf);

	data->accel_data.x = buf[0] << 8 | buf[1];
	data->accel_data.y = buf[2] << 8 | buf[3];
	data->accel_data.z = buf[4] << 8 | buf[5];

	data->temp = (buf[6] << 8) | buf[7];

	data->gyro_data.x = buf[8] << 8 | buf[9];
	data->gyro_data.y = buf[10] << 8 | buf[11];
	data->gyro_data.z = buf[13] << 8 | buf[13];

}

/**
 * Read_acc_axis() - Read Accelerometer data.
 * @data : Pointer to a struct object of mpu_data structure.
 */
void Read_acc_axis(struct mpu_data *data) {
	uint8_t buf[6];
	Read_Multiple_Regiter(REG_ACCEL_XOUT_H, sizeof(buf), buf);
	data->accel_data.x = buf[0] << 8 | buf[1];
	data->accel_data.y = buf[2] << 8 | buf[3];
	data->accel_data.z = buf[4] << 8 | buf[5];
}

/**
 * Read_Temp() - Read temperature data.
 * @data : Pointer to a struct object of mpu_data structure.
 */
void Read_Temp(struct mpu_data *data) {
	uint8_t buf[2]={0,0};
	Read_Multiple_Regiter(REG_TEMP_OUT_H, sizeof(buf), buf);
	data->temp = (buf[0] << 8) | buf[1];
}

/**
 * Read_gyro_axis() - Read Gyroscope data.
 * @data : Pointer to a struct object of mpu_data structure.
 */
void Read_gyro_axis(struct mpu_data *data) {
	uint8_t buf[6];
	Read_Multiple_Regiter(REG_GYRO_XOUT_H, sizeof(buf), buf);
	data->gyro_data.x = buf[0] << 8 | buf[1];
	data->gyro_data.y = buf[2] << 8 | buf[3];
	data->gyro_data.z = buf[4] << 8 | buf[5];
}

/**
 * Read_Reg() - Read Gyroscope data.
 * @data : Pointer to a struct object of mpu_data structure. 
 */
static void Read_Reg(struct mpu_data_reg *data){	
	Read_Regiter(data->reg, &data->val);
}

/**
 * Mpu_ioctl() - Controling cmd selected and return date to user.
 * @file  	: Pointer to file description.
 * @cmd		: Select which case to do.
 * Return	: this is struct mpu_data data object.
 *            putting wants read register on reg then 
 *            return register value on val. 
 */
static long Mpu_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	
	switch (cmd) {
	case READ_ALL:
		Read_all(&mpuData);
		if (copy_to_user((struct mpu_data*)arg, 
							&mpuData, sizeof(mpu_data)) != 0) {
			printk(KERN_DEBUG "READ_ALL: mpu_data copy to user fail");
		}
	break;
	case READ_ACCEL:
		Read_acc_axis(&mpuData);
		if (copy_to_user((struct mpu_data*)arg,
							&mpuData, sizeof(mpu_data)) != 0) {
			printk(KERN_DEBUG "READ_ACCEL: mpu_data copy to user fail");
		}
	break;
	case READ_TEMP:
		Read_Temp(&mpuData);
		if (copy_to_user((struct mpu_data*)arg,
							&mpuData, sizeof(mpu_data)) != 0) {
			printk(KERN_DEBUG "READ_TEMP: mpu_data copy to user fail");
		} 
	break;
	case READ_GYRO:
		Read_gyro_axis(&mpuData);
		if (copy_to_user((struct mpu_data*)arg,
							&mpuData, sizeof(mpu_data)) != 0) {
			printk(KERN_DEBUG "READ_GYRO: mpu_data copy to user fail");
		}
	break;
	case READ_REG:
		if( copy_from_user(&mpuDataReg ,
						(struct mpu_data_reg*) arg, sizeof(mpuDataReg)) ){
			printk(KERN_DEBUG "READ_REG: mpuDataReg copy from user fail\n");
        }
		Read_Reg(&mpuDataReg);
		if (copy_to_user((struct mpu_data_reg*)arg,
							&mpuDataReg, sizeof(mpuDataReg)) != 0){
			printk(KERN_DEBUG "READ_REG: mpu_data_reg copy to user fail");
        }
	break;
	case WRITE_REG:
		if( copy_from_user(&mpuDataReg ,
							(struct mpu_data_reg*) arg, sizeof(mpuDataReg)) ){
			printk(KERN_DEBUG "WRITE_REG: mpuDataReg copy from user fail\n");
        }
		Write_Reg(&mpuDataReg);
		if(mpuDataReg.reg == REG_GYRO_CONFIG)
			mpuData.gyro_sens = mpuDataReg.val;
		if(mpuDataReg.reg == REG_ACCEL_CONFIG)
			mpuData.accl_sens = mpuDataReg.val;
	break;
	default:
		printk(KERN_DEBUG "IOCTL command not found");
    break;
  }	
  return 0;
}

/**
 * Mpu6050_init() - initialize MPU 6050 register.
 * 	                set Gyroscope Scale Range on 2000.
 *                  set Accelerometer Scale Range on 2G.
 *					sample rate on 50Hz.
 * 					Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV).
 */
static void Mpu6050_init(void){
	uint8_t buf = 0;
	// The reset value is 0x00 for all registers other than the registers below.
	// Register 107: 0x40.
	// Register 117: 0x68.
	Write_Register(REG_PWR_MGMT_1, DEVICE_RESET);
	Read_Regiter(REG_PWR_MGMT_1, &buf);
	printk(KERN_DEBUG "REG_PWR_MGMT_1: write=0x%x read=0x%x\n", DEVICE_RESET, buf);
	
	Write_Register(REG_PWR_MGMT_1, PLL_X_AXIS_GYRO_REF);
	Read_Regiter(REG_PWR_MGMT_1, &buf);
	printk(KERN_DEBUG "REG_PWR_MGMT_1: write=0x%x read=0x%x\n", PLL_X_AXIS_GYRO_REF, buf);
	
	Write_Register(REG_GYRO_CONFIG, FS_SEL(GYRO_SCALE_RANGE_2000));
	Read_Regiter(REG_GYRO_CONFIG, &buf);
	printk(KERN_DEBUG "REG_GYRO_CONFIG: write=0x%x read=0x%x\n",
			FS_SEL(GYRO_SCALE_RANGE_2000), buf);			
	mpuData.gyro_sens = GYRO_SCALE_RANGE_2000;	
	
	Write_Register(REG_ACCEL_CONFIG, AFS_SEL(ACCEL_SCALE_RANGE_2G));
	Read_Regiter(REG_ACCEL_CONFIG, &buf);
	printk(KERN_DEBUG "REG_ACCEL_CONFIG: write=0x%x read=0x%x\n",
			AFS_SEL(ACCEL_SCALE_RANGE_2G), buf);			
	mpuData.accl_sens = ACCEL_SCALE_RANGE_2G;	
	
	Write_Register(REG_FIFO_EN, 0);
	Read_Regiter(REG_FIFO_EN, &buf);
	printk(KERN_DEBUG "REG_FIFO_EN: write=0x%x read=0x%x\n", 0, buf);
	
	Write_Register(REG_SMPRT_DIV, 0x13);
	Read_Regiter(REG_SMPRT_DIV, &buf);
	printk(KERN_DEBUG "REG_SMPRT_DIV: write=0x%x read=0x%x\n", 0, buf);
	
	Write_Register(REG_CONFIG, DLPF_CFG_6);
	Read_Regiter(REG_CONFIG, &buf);
	printk(KERN_DEBUG "REG_CONFIG: write=0x%x read=0x%x\n", 0, buf);
}

static int mpu_probe(struct i2c_client *client) {
	printk("Initializing driver");
	/* Read Chip ID */
	uint8_t id = i2c_smbus_read_byte_data(client, WHO_AM_I_ADDR);
	if(ID(id) != MPU_SLAVE_ADDR){
		printk(KERN_DEBUG "Device no found");
	}
	else{
		printk(KERN_DEBUG "Device found ID: 0x%x\n", ID(id));
	}
	Mpu6050_init();

	printk(KERN_DEBUG "Done probing");
  return 0;
}

static void mpu_remove(struct i2c_client *client) {
	printk(KERN_DEBUG "Removing driver\n");
}

/**
 * @brief This function is called, when the device file is opened
 */
static int mpu_open(struct inode *deviceFile, struct file *instance) {
	printk(KERN_DEBUG "MPU-6050 Driver - Open was called\n");
	return 0;
}

/**
 * @brief This function is called, when the device file is close
 */
static int mpu_release(struct inode *deviceFile, struct file *instance) {
	printk(KERN_DEBUG "MPU-6050 Driver -  Close was called\n");
	return 0;
}

// static ssize_t mpu_read(
		// struct file *filp, char __user *buf, size_t len, loff_t *off) {
  // return 0;
// }

// static ssize_t mpu_write(struct file *filp, const char *buf, size_t len,
                         // loff_t *off) {
  // return 0;
// }

static struct i2c_driver mpu_i2c_driver = {
	.driver = {
		.name = SLAVE_DEVICE_BOARD,
		.owner = THIS_MODULE,
	},
	.probe = mpu_probe,
	.remove = mpu_remove,
	.id_table = mpu_i2c_device_id,
};


/* Map the file operations */
static struct file_operations mpu_file_ops = {
	.owner = THIS_MODULE,
	.open = mpu_open,
	// .read = mpu_read,
	// .write = mpu_write,
	.release = mpu_release,
	.unlocked_ioctl = Mpu_ioctl,
};

/**
 * @brief function, which is called after loading module to kernel, 
		  do initialization there
 */
static int __init ModuleInit(void) {
	printk(KERN_DEBUG "MPU-6050 Driver init\n");
	
	/* Allocate Device number */
	if (alloc_chrdev_region(&mpu_dev_t, 0, DEVICE_COUNT, DRIVER_NAME) < 0) {
		printk(KERN_ERR "Device Number. could not be allocated!\n");
		return -1;
	}
	printk("MPU-6050 Driver - Major=%d  minor=%d\n",
			MAJOR(mpu_dev_t), MINOR(mpu_dev_t));
			
	/* Initialize Device file */
	cdev_init(&mpu_cdev, &mpu_file_ops);
	
	/* Register device to kernel */
	if (cdev_add(&mpu_cdev, mpu_dev_t, 1) == -1) {
		printk(KERN_ERR"Registering of device to kernel failed!\n");
		goto Err_Dev_Class;
	}
			
	/* Create Device Class */
	if ((mpu_dev_class = class_create(DRIVER_CLASS)) == NULL) {
		printk(KERN_ERR "Device Class can not be created!\n");
		goto Err_Dev_Class;
	}
	/* Create Device file */
	if (device_create(
			mpu_dev_class, NULL, mpu_dev_t, NULL, DRIVER_NAME) == NULL) {
		printk(KERN_ERR "Can not create device file!\n");
		goto Err_Dev_File;
	}

	/* Get adapter of i2c bus number */
	mpu_i2c_adapter = i2c_get_adapter(I2C_BUS_NUMBER);

	if(mpu_i2c_adapter != NULL) {
		/*Create i2c client device */
		mpu_i2c_client = i2c_new_client_device (
										mpu_i2c_adapter, &mpu_i2c_board_info);
		if(mpu_i2c_client != NULL) {
			if(i2c_add_driver(&mpu_i2c_driver) == -1) {
				printk(KERN_ERR "Can't add i2c driver...\n");
				goto Err_Register_Kernel;
			}
		}
		i2c_put_adapter(mpu_i2c_adapter);
	}
	printk(KERN_DEBUG "MPU6050 Driver added!\n");
    return 0;
	
Err_Register_Kernel:
	device_destroy(mpu_dev_class, mpu_dev_t);
Err_Dev_File:
	class_destroy(mpu_dev_class);
Err_Dev_Class:
	unregister_chrdev(mpu_dev_t, DRIVER_NAME);
	return (-1);

}

/**
 * @brief function, which is called when removing module from kernel
 * free alocated resources
 */
static void __exit ModuleExit(void) {
	printk("MPU-6050 Driver eixt\n");
	i2c_unregister_device(mpu_i2c_client);
	i2c_del_driver(&mpu_i2c_driver);
	cdev_del(&mpu_cdev);
	device_destroy(mpu_dev_class, mpu_dev_t);
    class_destroy(mpu_dev_class);
    unregister_chrdev_region(mpu_dev_t, DEVICE_COUNT);
}

module_init(ModuleInit);
module_exit(ModuleExit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tank_Liao");
MODULE_DESCRIPTION("GY-521 driver from i2c interface");
MODULE_VERSION("0.0.1");