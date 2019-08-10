#include "global.h"

void search_init(void){

	//----探索系----
	goal_x = GOAL_X;        		//GOAL_Xはglobal.hにマクロ定義あり
	goal_y = GOAL_Y;        		//GOAL_Yはglobal.hにマクロ定義あり
	map_Init();						//マップの初期化
	mouse.x = 0;
	mouse.y = 0;					//現在地の初期化
	mouse.dir = 0;					//マウスの向きの初期化
}
