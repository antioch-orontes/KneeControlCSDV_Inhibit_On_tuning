//#include <stdio.h>
#include "MPU6050.h"
#include "i2c_emem.h"
#include <stdio.h>
#include "uart1.h"
#include <math.h>
// Read IMU data through I2C
// Instantiate Drive and Data objects
I2CEMEM_DRV i2cmem = I2CSEMEM_DRV_DEFAULTS;
I2CEMEM_DATA wData;
I2CEMEM_DATA rData;

uint16_t wBuff[10], rBuff[20];
uint16_t enable;
extern uint16_t jDone;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

float FS_SEL = 131.0; // Sensitivity Scale factor 131 LSB/(0/S)

struct imu_data getImuData( void )
{
        struct imu_data imuData1; /* Declare imuData1 of type imu_data */

        rData.addr =MPU6050_RA_ACCEL_XOUT_H; // 0x3B (ACCEL_XOUT_H)
        rData.n = 14;
        i2cmem.oData = &rData;

        i2cmem.cmd = I2C_READ;  // comment this line to omit the read phase
        while( i2cmem.cmd != I2C_IDLE )
        {
                i2cmem.tick( &i2cmem );
        }

        // combine the high bytes with the low bytes
        imuData1.AcX1 = rBuff [0] <<8 | rBuff [1];
        imuData1.AcY1 = rBuff [2] <<8 | rBuff [3];
        imuData1.AcZ1 = rBuff [4] <<8 | rBuff [5];
        imuData1.Tmp1 = rBuff [6] <<8 | rBuff [7];
        imuData1.GyX1 = rBuff [8] <<8 | rBuff [9];
        imuData1.GyY1 = rBuff [10] <<8 | rBuff [11];
        imuData1.GyZ1 =  rBuff [12] <<8 | rBuff [13];

        return imuData1;
}

int16_t getAccelX( void )
{
        int16_t AcX1;

        rData.addr =MPU6050_RA_ACCEL_XOUT_H; // 0x3B (ACCEL_XOUT_H)
        rData.n = 2;
        i2cmem.oData = &rData;

        i2cmem.cmd = I2C_READ;  // comment this line to omit the read phase
        int count_state = 0;

        while( i2cmem.cmd != I2C_IDLE )
        {
                i2cmem.tick( &i2cmem );
                count_state++;
                //printf("ST%d,",count_state);
                if (count_state > 16) // err  restart IMU
                {
                        break;
                }
                //printf("tick");
        }
        //printf("ST%d,",count_state);
        // combine the high bytes with the low bytes
        AcX1 = rBuff [0] <<8 | rBuff [1];
        return AcX1;
}

void printImuData( struct imu_data imuData )
{
        printf("\n%d %d %d %d %d %d\n",imuData.AcX1,imuData.AcY1,imuData.AcZ1,imuData.GyX1,imuData.GyY1,imuData.GyZ1);
}

struct imu_data calibrateImu( void )
{
        int i;
        int numReading = 10;
        struct imu_data imuData1; /* Declare imuData1 of type imu_data */
        struct imu_data imuOffset; /* Declare imuOffest of type imu_data */

        // zero the the imuOffest
        imuOffset.AcX1 = 0;
        imuOffset.AcY1 = 0;
        imuOffset.AcX1 = 0;
        imuOffset.AcY1 = 0;
        imuOffset.AcZ1 = 0;
        imuOffset.GyX1 = 0;
        imuOffset.GyY1 = 0;
        imuOffset.GyZ1 = 0;

        for( i = 0; i < numReading; i++ )
        {
                //get imu data starting with register ox3B (ACCEL_XOUT_H)

                imuData1 = getImuData(  );

                // compute the average of each set of data.

                imuOffset.AcX1 += imuData1.AcX1/numReading;
                imuOffset.AcY1 += imuData1.AcY1/numReading;
                imuOffset.AcZ1 += imuData1.AcZ1/numReading;
                imuOffset.GyX1 += imuData1.GyX1/numReading;
                imuOffset.GyY1 += imuData1.GyY1/numReading;
                imuOffset.GyZ1 += imuData1.GyZ1/numReading;
        }

        return imuOffset;
}

void initImu ( void )
{
        int i;

        // Initialise Data to be written to serial EEPROM
        for( i = 0; i < 10; i++ )
        {
                wBuff[i] = 0;
                rBuff[i] = 0;
        }

        // Initialise I2C Data object for Write operation
        wData.buff = wBuff;
        wData.n = 1;
        wData.addr = MPU6050_RA_PWR_MGMT_1; //PWR_MGMT_1
        wData.csel = MPU6050_ADDRESS;

        // Initialise I2C Data Object for Read operation
        rData.buff = rBuff;
        rData.n = 1;
        rData.addr = MPU6050_RA_WHO_AM_I; // read register WHO AM I (default value of the register is 0x68)
        rData.csel = MPU6050_ADDRESS; // This is the i2c address of MPU-6050 when AD0 = 0.

        // enable the i2c communication
        enable = 1;

        // read who-am-i register
        i2cmem.oData = &rData;
        i2cmem.cmd = I2C_READ;
        while( i2cmem.cmd != I2C_IDLE )
        {
                i2cmem.tick( &i2cmem );
        }

        // report error
        if (rBuff[0] == 0x68)
        {
                printf("\nI2C Read Test , MPU6050 Address: 0x%x", rBuff[0]);
        }
        else
        {
                printf("\nError: I2C is not correctly set up.\n");
        }
        // write to pwr_mgmt_1 to wake up the imu.

        wBuff[0] = 0x0; // write 0
        i2cmem.oData = &wData;
        i2cmem.cmd = I2C_WRITE;

        while( i2cmem.cmd != I2C_IDLE )
        {
                i2cmem.tick( &i2cmem );
        }

        printf("\nSensor is waked up!\n");
}

struct imu_angle processImuRaw ( struct imu_data imuMyData, struct imu_data imuBase, struct imu_angle last_angle, float dt_us )
{
        struct imu_angle imu_angle1, gyro, accel, accel_angle, gyro_angle;
        float RADIANS_TO_DEGREES = 180/3.14159;
        float alpha = 0.986; // t/(t+dt) dt is 0.014 ms t is 1 sec alpha = 0.986
        // Convert gyro values to degrees/sec
        gyro.x = (imuMyData.GyX1 - imuBase.GyX1)/FS_SEL;
        gyro.y = (imuMyData.GyY1 - imuBase.GyY1)/FS_SEL;
        gyro.z = (imuMyData.GyZ1 - imuBase.GyZ1)/FS_SEL;
        //printf("\ngyro_x = %f",gyro_x);
        //printf("\ngyro_y = %f",gyro_y);
        //printf("\ngyro_z = %f",gyro_z);

        // Get raw acceleration values
        //float G_CONVERT = 16384;

        accel.x = imuMyData.AcX1;
        accel.y = imuMyData.AcY1;
        accel.z = imuMyData.AcZ1;

        //printf("\nAcX = %f",accel_x);
        //printf("\nAcY = %f",accel_y);
        //printf("\nAcZ = %f",accel_z);

        // Get angle values from accelerometer
        accel_angle.y = atan(-1*accel.x/sqrt(pow(accel.y,2) + pow(accel.z,2)))*RADIANS_TO_DEGREES;
        accel_angle.x = atan(accel.y/sqrt(pow(accel.x,2) + pow(accel.z,2)))*RADIANS_TO_DEGREES;
        accel_angle.z = 0;
        //printf("\naccel_angle_y = %f",accel_angle_y);
        //printf("\naccel_angle_x = %f",accel_angle_x);
        //printf("\naccel_angle_z = %f",accel_angle_z);

        // Compute the (filtered) gyro angles
        //dt = (t_now-last_read_time);    //us
        gyro_angle.x = gyro.x*dt_us/1000000.0 + last_angle.x;
        gyro_angle.y = gyro.y*dt_us/1000000.0 + last_angle.y;
        gyro_angle.z = gyro.z*dt_us/1000000.0 + last_angle.z;
        //printf("\ngyro_angle_x = %f",gyro_angle_x);
        //printf("\ngyro_angle_y = %f",gyro_angle_y);
        //printf("\ngyro_angle_z = %f",gyro_angle_z);

        imu_angle1.x = alpha*gyro_angle.x + (1.0-alpha)*accel_angle.x;
        imu_angle1.y = alpha*gyro_angle.y + (1.0-alpha)*accel_angle.y;
        imu_angle1.z = gyro_angle.z;//Accelerometer doesn't give z-angle

        return (imu_angle1);
}


/*
 #define FCY     40000000UL
 #include <libpic30.h>
 #include <p33Fj128GP802.h>
 #include "common.h"
 #include <math.h>
 #include "I2C.h"
 #define gyro_xsensitivity 66.5 //66.5 Dead on at last check
 #define gyro_ysensitivity 66.5 //72.7 Dead on at last check
 #define gyro_zsensitivity 65.5
 #define a 0.01
 */
