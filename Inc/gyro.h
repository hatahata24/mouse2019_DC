
#ifndef GYRO_H_
#define GYRO_H_

//====a変数====
#ifdef MAIN_C_										//main.cからこのファイルが呼ばれている場合
	/*aグローバル変数の定義*/

	volatile float degree_x, degree_y, degree_z;
	volatile float target_degaccel_z;
	volatile float target_omega_z;
	volatile float target_degree_z;
	volatile float omega_min, omega_max;
	volatile int16_t dg, dgl, dgr;								//a比例制御量
	volatile float dif_omega_z, old_omega_z, gyro_drift_value;	//a偏差と積分
	volatile uint8_t gyro_drift_flag;							//ジャイロドリフト計算開始フラグ
#else												//main.c以外からこのファイルが呼ばれている場合
	/*aグローバル変数の宣言*/

	extern volatile float degree_x, degree_y, degree_z;
	extern volatile float target_degaccel_z;
	extern volatile float target_omega_z;
	extern volatile float target_degree_z;
	extern volatile float omega_min, omega_max;
	extern volatile int16_t dg, dgl, dgr;							//a比例制御量
	extern int16_t dif_g;
	extern volatile float dif_omega_z, old_omega_z, gyro_drift_value;	//a偏差と積分
	extern volatile uint8_t gyro_drift_flag;						//ジャイロドリフト計算開始フラグ
#endif


#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define ACCEL_FACTOR 2048
#define GYRO_FACTOR 16.4


/*============================================================
		関数プロトタイプ宣言
============================================================*/
void gyro_init(void);
uint8_t read_byte(uint8_t reg);
void write_byte(uint8_t reg, uint8_t val);

float accel_read_x(void);
float accel_read_y(void);
float accel_read_z(void);
float gyro_read_x(void);
float gyro_read_y(void);
float gyro_read_z(void);

//====a走行系====
//----base関数----


//----top関数----

#endif /* GYRO_H_ */
