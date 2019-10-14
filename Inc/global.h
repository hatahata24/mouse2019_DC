#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_


#include "main.h"

#include "params.h"
#include "sensor.h"
#include "search.h"
#include "buzzer.h"
#include "drive.h"
#include "gyro.h"
#include "eeprom.h"
#include <stdio.h>


typedef union {					//
	uint16_t FLAGS;
	struct ms_flags{			//
		uint16_t RSV0:1;		//yobi bit(B0)		(:1は1ビット分の意味，ビットフィールド)
		uint16_t DRV:1;			//drive flag(B1)
		uint16_t SPD:1;			//speed flag(B2)
		uint16_t WCTRL:1;		//wall control flag(B3)
		uint16_t GCTRL:1;		//gyro control flag(B4)
		uint16_t SCND:1;		//niji soukou flag(B5)
		uint16_t LOG:1;			//log flag(B6)
		uint16_t ENKAI:1;		//enkaigei flag(B7)
		uint16_t GYRO:1;		//gyro flag(B8)
		uint16_t ACCL2:1;		//accel ++ flag(B9)
		uint16_t STRAIGHT:1;	//straight flag(B10)
		uint16_t WEDGE:1;		//壁切れフラグ(B11)
		uint16_t XDIR:1;		//8方向移動フラグ(B12)
		uint16_t RSV13:1;		//予備ビット(B13)
		uint16_t RSV14:1;		//予備ビット(B14)
		uint16_t RSV15:1;		//予備ビット(B15)
	}FLAG;
} mouse_flags;

#ifdef MAIN_C_							//main.cからこのファイルが呼ばれている場合
	/*グローバル変数の定義*/
	volatile mouse_flags MF;			//mouse kyoyou kouzoutai
#else									//main.c以外からこのファイルが呼ばれている場合
	/*グローバル変数の宣言*/
	extern volatile mouse_flags MF;
#endif

#endif /* INC_GLOBAL_H_ */
