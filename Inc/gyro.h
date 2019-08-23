
#ifndef GYRO_H_
#define GYRO_H_

//====a変数====
#ifdef MAIN_C_										//main.cからこのファイルが呼ばれている場合
	/*aグローバル変数の定義*/

	volatile float degree_z;
	volatile float target_degaccel_z;
	volatile float target_omega_z;
	volatile float target_degree_z;
	volatile float omega_min, omega_max;
#else												//main.c以外からこのファイルが呼ばれている場合
	/*aグローバル変数の宣言*/

	extern volatile float degree_z;
	extern volatile float target_degaccel_z;
	extern volatile float target_omega_z;
	extern volatile float target_degree_z;
	extern volatile float omega_min, omega_max;
#endif


#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define GYRO_FACTOR 16.4


/*============================================================
		関数プロトタイプ宣言
============================================================*/
void gyro_init(void);
uint8_t read_byte(uint8_t reg);
void write_byte(uint8_t reg, uint8_t val);
float gyro_read_z(void);

//====a走行系====
//----base関数----


//----top関数----

#endif /* GYRO_H_ */
