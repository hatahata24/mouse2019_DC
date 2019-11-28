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


typedef union {					//aタイマ割り込みや走行に影響を与えるフラグ。このフラグたちは基本的に初期状態でのみ変更が加えられる
	uint16_t FLAGS;
	struct ms_flags{			//
		uint16_t RSV0:1;		//a予備 bit(B0)		(:1は1ビット分の意味，ビットフィールド)
		uint16_t DRV:1;			//aモータ駆動 flag(B1)
		uint16_t SPD:1;			//a速度計算 flag(B2)
		uint16_t WCTRL:1;		//a壁制御 flag(B3)
		uint16_t GCTRL:1;		//aジャイロ制御 flag(B4)
		uint16_t SCND:1;		//a二次走行 flag(B5)
		uint16_t SRC2:1;		//a重ね探索 flag(B6)
		uint16_t FWALL:1;		//a停止時前壁補正 flag(B7)
		uint16_t GYRO:1;		//a旋回角速度計算 flag(B8)
		uint16_t ACCL2:1;		//a既知区間加速 flag(B9)
		uint16_t STRAIGHT:1;	//a直線優先 flag(B10)
		uint16_t WEDGE:1;		//a壁切れフラグ(B11)
		uint16_t XDIR:1;		//8方向移動フラグ(B12)
		uint16_t RSV13:1;		//予備ビット(B13)
		uint16_t RSV14:1;		//予備ビット(B14)
		uint16_t RSV15:1;		//予備ビット(B15)
	}FLAG;
} mouse_flags;


typedef union {					//aプログラム実行中や走行中に自己状態を示すためのフラグ
	uint16_t FLAGS2;
	struct ms_flags2{			//
		uint16_t RSV0:1;		//a予備 bit(B0)		(:1は1ビット分の意味，ビットフィールド)
		uint16_t HACCEL:1;		//a既知区間加速状態 flag(B1)
		uint16_t ALLMAP:1;		//a全面探索完了 flag(B2)
		uint16_t V:1;			//a斜め状態 flag(B3)
		uint16_t WG:1;			//a壁とジャイロの姿勢制御 flag(B4)
		uint16_t ENKAI:1;		//a宴会芸 flag(B5)
		uint16_t LOG:1;			//aログ flag(B6)
		uint16_t GDRIFT:1;		//aジャイロドリフト flag(B7)
		uint16_t GOAL:1;		//goal flag(B8)
		uint16_t TEMP:1;		//a予備ビット(B9)
		uint16_t RSV10:1;		//a予備ビット(B10)
		uint16_t RSV11:1;		//a予備ビット(B11)
		uint16_t RSV12:1;		//a予備ビット(B12)
		uint16_t RSV13:1;		//a予備ビット(B13)
		uint16_t RSV14:1;		//a予備ビット(B14)
		uint16_t RSV15:1;		//a予備ビット(B15)
	}FLAG;
} mouse_flags2;

#ifdef MAIN_C_							//main.cからこのファイルが呼ばれている場合
	/*aグローバル変数の定義*/
	volatile mouse_flags MF;			//mouse 共用構造体
	volatile mouse_flags2 MF2;			//mouse 共用構造体
#else									//main.c以外からこのファイルが呼ばれている場合
	/*aグローバル変数の宣言*/
	extern volatile mouse_flags MF;
	extern volatile mouse_flags2 MF2;			//mouse 共用構造体
#endif

#endif /* INC_GLOBAL_H_ */
