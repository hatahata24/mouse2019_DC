
#include "global.h"

SPI_HandleTypeDef hspi3;


void gyro_init(void){
  uint8_t who_am_i;

  HAL_Delay(100); 					// wait start up
  who_am_i = read_byte(WHO_AM_I); 	// 1. read who am i
  printf("0x%x\r\n",who_am_i); 		// 2. check who am i value

  // 2. error check
  if (who_am_i != 0x98){
    while(1){
      printf("gyro_error\r");
    }
  }

  HAL_Delay(50); // wait
  write_byte(PWR_MGMT_1, 0x00); 	// 3. set pwr_might

  HAL_Delay(50);
  write_byte(CONFIG, 0x00); 		// 4. set config

  HAL_Delay(50);
  write_byte(GYRO_CONFIG, 0x18); 	// 5. set gyro config

  HAL_Delay(50);
}


uint8_t read_byte(uint8_t reg){
  uint8_t ret,val;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET ); 	//cs = Low;
  ret = reg | 0x80;  // MSB = 1
  HAL_SPI_Transmit(&hspi3, &ret,1,100); 					// sent 1byte(address)
  HAL_SPI_Receive(&hspi3,&val,1,100); 						// read 1byte(read data)
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET );  	//cs = High;
  return val;
}


void write_byte(uint8_t reg, uint8_t val){
  uint8_t ret;
  ret = reg & 0x7F ; // MSB = 0
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); 	// cs = Low;
  HAL_SPI_Transmit(&hspi3, &ret,1,100); 					// sent 1byte(address)
  HAL_SPI_Transmit(&hspi3, &val,1,100); 					// read 1byte(write data)
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); 		// cs = High;
}


float accel_read_x(void){
  int16_t accel_x;
  float accel;

  // H:8bit shift, Link h and l
  accel_x = (int16_t)((int16_t)(read_byte(ACCEL_XOUT_H) << 8) | read_byte(ACCEL_XOUT_L));

  accel = (float)(accel_x / ACCEL_FACTOR); // dps to accel
  return accel;
}


float accel_read_y(void){
  int16_t accel_y;
  float accel;

  // H:8bit shift, Link h and l
  accel_y = (int16_t)((int16_t)(read_byte(ACCEL_YOUT_H) << 8) | read_byte(ACCEL_YOUT_L));

  accel = (float)(accel_y / ACCEL_FACTOR); // dps to accel
  return accel;
}


float accel_read_z(void){
  int16_t accel_z;
  float accel;

  // H:8bit shift, Link h and l
  accel_z = (int16_t)((int16_t)(read_byte(ACCEL_ZOUT_H) << 8) | read_byte(ACCEL_ZOUT_L));

  accel = (float)(accel_z / ACCEL_FACTOR); // dps to accel
  return accel;
}


float gyro_read_x(void){
  int16_t gyro_x;
  float omega;

  // H:8bit shift, Link h and l
  gyro_x = (int16_t)((int16_t)(read_byte(GYRO_XOUT_H) << 8) | read_byte(GYRO_XOUT_L));

  omega = (float)(gyro_x / GYRO_FACTOR); // dps to deg/sec
  return omega;
}


float gyro_read_y(void){
  int16_t gyro_y;
  float omega;

  // H:8bit shift, Link h and l
  gyro_y = (int16_t)((int16_t)(read_byte(GYRO_YOUT_H) << 8) | read_byte(GYRO_YOUT_L));

  omega = (float)(gyro_y / GYRO_FACTOR); // dps to deg/sec
  return omega;
}


float gyro_read_z(void){
  int16_t gyro_z;
  float omega;

  // H:8bit shift, Link h and l
  gyro_z = (int16_t)((int16_t)(read_byte(GYRO_ZOUT_H) << 8) | read_byte(GYRO_ZOUT_L));

  omega = (float)(gyro_z / GYRO_FACTOR+1.15); // dps to deg/sec
  return omega;
}

