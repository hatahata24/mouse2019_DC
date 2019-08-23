
#include "global.h"


void sensor_init(void){

	tp = 0;
	ad_l = ad_r = ad_fr = ad_fl = 0;
	base_l = base_r = 0;
}


uint8_t get_base(){
	uint8_t res = 1;									//for return

	base_l = ad_l;										//sensor value base L
	base_r = ad_r;										//sensor value base R

	return res;											//
}


void get_wall_info(){

	//----reset----
	wall_info = 0x00;									//wall
	//----look forward----
	if(ad_fr > WALL_BASE_FR || ad_fl > WALL_BASE_FL){
		wall_info |= 0x88;								//forward check
	}
	//----look right----
	if(ad_r > WALL_BASE_R){
		wall_info |= 0x44;								//right check
	}
	//----look left----
	if(ad_l > WALL_BASE_L){
		wall_info |= 0x11;								//light check
	}
}
