#ifndef SENSOR_H_
#define SENSOR_H_

#ifdef MAIN_C_												//main.cからこのファイルが呼ばれている場合
	/*define gloval voratile*/
	//----other----
	uint8_t tp;												//task pointer
	uint32_t ad_r, ad_fr, ad_fl, ad_l;						//variable for ADC
	uint16_t base_l, base_r;								//kijun variable
	int16_t dif_l, dif_r;									//dif between kijun & ADC

	uint8_t wall_info;										//壁情報格納変数

#else														//main.c以外からこのファイルが呼ばれている場合
	extern uint8_t tp;
	extern uint32_t ad_r, ad_fr, ad_fl, ad_l;
	extern uint16_t base_l,  base_r;
	extern int16_t dif_l, dif_r;

	extern uint8_t wall_info;								//壁情報格納変数

	#endif

void sensor_init(void);
//int get_adc_value(int);

uint8_t get_base();					//
void get_wall_info();				//

#endif /* SENSOR_H_ */
