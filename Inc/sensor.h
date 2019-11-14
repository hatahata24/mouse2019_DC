#ifndef SENSOR_H_
#define SENSOR_H_

#ifdef MAIN_C_												//main.cからこのファイルが呼ばれている場合
	/*define gloval voratile*/
	//----other----
	uint8_t tp;												//task pointer
	uint32_t ad_r, ad_fr, ad_fl, ad_l;						//aAD変換値格納用変数
	uint16_t base_l, base_r;								//a基準格納用変数
	int16_t dif_l, dif_r;									//a基準値とAD変換値の偏差
	volatile int16_t dwl, dwr;								//a比例制御量

#else														//main.c以外からこのファイルが呼ばれている場合
	extern uint8_t tp;
	extern uint32_t ad_r, ad_fr, ad_fl, ad_l;
	extern uint16_t base_l,  base_r;
	extern int16_t dif_l, dif_r;
	extern volatile int16_t	dl, dr;

#endif

void sensor_init(void);

uint8_t get_base();					//a両壁基準値取得
void get_wall_info();				//a壁情報を迷路マップに書き込み

void led_write(uint8_t, uint8_t, uint8_t);
void full_led_write(uint8_t);


void sensor_test();					//a壁センサーとジャイロセンサーの値確認

#endif /* SENSOR_H_ */
