#ifndef SEARCH_H_
#define SEARCH_H_

struct coordinate_and_direction{
	uint8_t x;
	uint8_t y;
	uint8_t dir;
};

#ifdef MAIN_C_												//main.cからこのファイルが呼ばれている場合
	/*define gloval voratile*/
	//----other----
	uint8_t wall_info;										//
	volatile struct coordinate_and_direction mouse;

#else														//main.c以外からこのファイルが呼ばれている場合
	extern uint8_t wall_info;								//
	extern volatile struct coordinate_and_direction mouse;

#endif

//---direction change constant
#define DIR_TURN_R90	0x01	//right 90度回転
#define DIR_TURN_L90	0xff	//left 90度回転
#define DIR_TURN_180	0x02	//180度回転

//====変数====
#ifdef MAIN_C_											//main.cからこのファイルが呼ばれている場合
	/*グローバル変数の定義*/
	uint8_t map[16][16];								//aマップ格納配列
	//uint8_t smap[16][16];								//a歩数マップ格納配列
	uint16_t smap[16][16];								//a歩数マップ格納配列
	uint8_t wall_info;									//a壁情報格納変数
	uint8_t goal_x, goal_y;								//aゴール座標
	uint8_t route[256];									//a最短経路格納配列
	uint8_t r_cnt;										//a経路カウンタ

	uint8_t H_accel_flag;
	uint8_t run_dir;

	uint16_t m_step;
#else													//main.c以外からこのファイルが呼ばれている場合
	/*グローバル変数の宣言*/
	extern uint8_t map[16][16];							//aマップ格納配列
	//extern uint8_t smap[16][16];						//a歩数マップ格納配列
	extern uint16_t smap[16][16];						//a歩数マップ格納配列
	extern uint8_t wall_info;							//a壁情報格納変数
	extern uint8_t goal_x, goal_y;						//aゴール座標
	extern uint8_t route[256];							//a最短経路格納配列
	extern uint8_t r_cnt;								//a経路カウンタ

	extern uint8_t H_accel_flag;
	extern uint8_t run_dir;

	extern uint16_t m_step;
#endif


/*============================================================
		関数プロトタイプ宣言
============================================================*/
//====探索系====

void search_init(void);

void searchA();											//1区画停止型探索走行
void searchB();											//a連続探索走行
void searchC();											//aスラローム探索走行
void searchC2();											//aスラローム+既知区間加速探索走行
void searchD();											//a全面探索走行

void adv_pos();											//aマウスの位置情報を前進
void conf_route();										//a次ルートの確認
void map_Init();										//aマップデータ初期化
void write_map();										//aマップ書き込み
void turn_dir(uint8_t);									//a自機方向情報変更
void make_smap();										//a歩数マップ作成
void make_route();										//a最短経路検索


void store_map_in_eeprom(void);
void load_map_from_eeprom(void);

#endif /* SEARCH_H_ */
