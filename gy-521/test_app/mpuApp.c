#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "gy_521.h"

#define mSEC(val) val*1000

char *device = "/dev/gy_521";
double acc_x, acc_y, acc_z, a_sens;
double gyro_x, gyro_y, gyro_z, g_sens;

double acc_Sensitivity[] = {16384, 8192, 4096, 2048};
double gyro_Sensitivity[] = {131, 65.5, 32.8, 16.4};

double temp;
int main(int argc, char *argv[]) {
	mpu_data mpuData;
	mpu_data_reg mpuData_reg;
	int times ;
	int fd = open(device, O_RDWR);
	printf("fd= %d\n", fd);
	if (fd < 0) {
		printf("Failed opening device: %s\n", strerror(errno));
		return -1;
	}
	if(argc < 2){
		times = 1;
		printf("print mpu6050 data default times = %d\n", times);
	}
	else{
		//times = atoi();
		sscanf(argv[1], "%d", &times);
		printf("print mpu6050 date set times = %d\n", times);
	}

	/* Test opening the device */

	mpuData_reg.reg = REG_SMPRT_DIV;
	mpuData_reg.val = 0x09;
	ioctl(fd, WRITE_REG, &mpuData_reg);
	mpuData_reg.val=0;
	ioctl(fd, READ_REG, &mpuData_reg);
	printf("REG_SMPRT_DIV: 0x%x\n", mpuData_reg.val);
	
  for (int i = 0; i < times; i++) {
    ioctl(fd, READ_ALL, &mpuData);
	acc_x = (double)mpuData.accel_data.x/acc_Sensitivity[mpuData.accl_sens];
    acc_y = (double)mpuData.accel_data.y/acc_Sensitivity[mpuData.accl_sens];
    acc_z = (double)mpuData.accel_data.z/acc_Sensitivity[mpuData.accl_sens];
    printf("AccX: %f, AccY: %f, AccZ: %f ", acc_x, acc_y, acc_z);
    gyro_x = (double)mpuData.gyro_data.x/gyro_Sensitivity[mpuData.gyro_sens];
    gyro_y = (double)mpuData.gyro_data.y/gyro_Sensitivity[mpuData.gyro_sens];
    gyro_z = (double)mpuData.gyro_data.z/gyro_Sensitivity[mpuData.gyro_sens];
    printf("GyroX: %f, GyroY: %f, GyroZ: %f ", gyro_x,  gyro_y, gyro_z);
	temp = (((double)mpuData.temp)/340)+ 36.53;
    printf("Temp= %f\n", temp);
	
    usleep(mSEC(1000));

  }
  close(fd);
  return 0;
}
