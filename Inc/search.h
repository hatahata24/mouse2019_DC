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
#define DIR_TURN_R90	0x01	//右90度回転
#define DIR_TURN_L90	0xff	//左90度回転
#define DIR_TURN_180	0x02	//180度回転

#define DIR_TURN_R45_8	0x01	//右90度回転
#define DIR_TURN_R90_8	0x02	//左90度回転
#define DIR_TURN_R135_8	0x03	//180度回転
#define DIR_TURN_R180_8	0x04	//右90度回転
#define DIR_TURN_L45_8	0xff	//右90度回転
#define DIR_TURN_L90_8	0xfe	//左90度回転
#define DIR_TURN_L135_8	0xfd	//180度回転
#define DIR_TURN_L180_8	0xfc	//右90度回転

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
	uint16_t m_step2;
	uint8_t pregoal_x, pregoal_y;
	uint8_t allmap_comp_flag;

	int8_t pass[256];									//pass圧縮後のルート保存用配列
	uint8_t p_cnt;										//pass経路カウンタ
	uint8_t pass_mode;									//pass圧縮モード選択

	uint8_t v_flag;										//V斜め状態フラグ

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
	extern uint16_t m_step2;
	extern uint8_t pregoal_x, pregoal_y;
	extern uint8_t allmap_comp_flag;

	extern int8_t pass[256];							//pass圧縮後のルート保存用配列
	extern uint8_t p_cnt;								//pass経路カウンタ
	extern uint8_t pass_mode;							//pass圧縮モード選択

	extern uint8_t v_flag;								//V斜め状態フラグ

#endif


/*============================================================
		関数プロトタイプ宣言
============================================================*/
//====探索系====

void search_init(void);

void searchA();											//1区画停止型探索走行
void searchB();											//a連続探索走行
void searchC();											//aスラローム走行
void searchD();											//aスラローム+既知区間加速探索走行
void searchE();											//aスラローム全面探索走行
void searchF();											//aスラローム+既知区間加速+pass圧縮
void searchF2();										//aスラローム+既知区間加速+pass圧縮+機体方向 &位置未更新
void searchF3();										//aスラローム+既知区間加速探索走行+pass圧縮+機体方向&位置未更新+半区画ベース
void searchF4();										//aスラローム+既知区間加速探索走行+pass圧縮+機体方向&位置未更新+半区画ベース+斜め


void adv_pos();											//aマウスの位置情報を前進
void adv_pos2(int8_t);										//aマウスの位置情報を前進(pass圧縮対応版)
void conf_route();										//a次ルートの確認
void map_Init();										//aマップデータ初期化
void write_map();										//aマップ書き込み
void turn_dir(uint8_t, uint8_t);						//a自機方向情報変更
void make_smap();										//a歩数マップ作成
void make_route();										//a最短経路検索

void find_pregoal();									//a仮goalの検索
void make_smap2();										//a仮goalまでの歩数マップ作成

void pass_route();										//pass圧縮関数
void pass_route2();										//pass圧縮関数2(半区画ベース)
void pass_route3();										//pass圧縮関数2(半区画ベース+斜めあり)

void store_map_in_eeprom(void);
void load_map_from_eeprom(void);



#endif /* SEARCH_H_ */
