
#ifndef DRIVE_H_
#define DRIVE_H_

//====変数====
#ifdef MAIN_C_										//main.cからこのファイルが呼ばれている場合
	/*グローバル変数の定義*/

	//====物理量走行関連====
	volatile float cnt_l, cnt_r;
	volatile float dist_l, dist_r;
	volatile float accel_l, accel_r;
	volatile float speed_l, speed_r, speed_G;

	volatile float target_speed_l, target_speed_r;
	volatile float speed_min_l, speed_max_l;
	volatile float speed_min_r, speed_max_r;
	volatile float target_dist;
	volatile float pulse_l, pulse_r;

	volatile float epsilon_l, epsilon_r;	//dif between target & current
	volatile float epsilon_sum; 	//dif sum
	volatile float old_epsilon; 	//
	volatile float epsilon_dif;	//dif of dif
#else												//main.c以外からこのファイルが呼ばれている場合
	/*グローバル変数の宣言*/
	//====物理量走行関連====
	extern volatile float cnt_l, cnt_r;
	extern volatile float dist_l, dist_r;
	extern volatile float accel_l, accel_r;
	extern volatile float speed_l, speed_r, speed_G;

	extern volatile float target_speed_l, target_speed_r;
	extern volatile float speed_min_l, speed_max_l;
	extern volatile float speed_min_r, speed_max_r;
	extern volatile float target_dist;
	extern volatile float pulse_l, pulse_r;

	extern volatile float epsilon_l, epsilon_r;	//dif between target & current
	extern volatile float epsilon_sum; 	//dif sum
	extern volatile float old_epsilon; 	//
	extern volatile float epsilon_dif;	//dif of dif
#endif


#define drive_wait()	HAL_Delay(50)


/*============================================================
		関数プロトタイプ宣言
============================================================*/
void drive_init(void);
void drive_dir(uint8_t, uint8_t);
void drive_start(void);
void drive_stop(void);

//====a走行系====
//----a基幹関数----
void driveA(uint16_t, uint16_t, uint16_t, uint16_t);		//加速走行
void driveD(int16_t, uint16_t, uint16_t, uint16_t);		//減速走行
void driveU(uint16_t);			//等速走行（前の速度を維持）
void driveC(uint16_t);			//デフォルトインターバルで走行
void slalomU12(uint16_t);
void slalomR12(uint16_t, uint16_t, uint16_t, uint16_t);
void slalomR22(uint16_t);
void slalomR32(int32_t, uint16_t, uint16_t, uint16_t, uint16_t);
void slalomU22(uint16_t);


//----a上位関数----
void half_sectionA(void);		//加速半区画
void half_sectionD(void);		//減速半区画
void one_section(void);			//加減速一区画
void one_sectionU(void);		//等速一区画
void rotate_R90(void);			//右90回転
void rotate_L90(void);			//左90回転
void rotate_180(void);			//180度回転
void slalom_R90(void);			//スラローム右90回転
void slalom_L90(void);			//スラローム左90回転
void set_position(uint8_t);		//上下位置合わせ
void set_positionX(uint8_t);	//上下左右位置合わせ


//----a走行関数----
void test_select(void);			//aテスト走行選択

void init_test(void);			//a初期基幹関数走行テスト

void defo_test(void);			//初期状態でのtableによるテスト走行
void physic_test(void);			//物理量ベースによるテスト走行
void accel_test(void);			//既知区間加速によるテスト走行
void slalom_test(void);			//スラロームによるテスト走行
void search_test(void);			//直線優先や全面探索のテスト走行


void simple_run(void);			//a超新地走行
void slalom_run(void);			//aスラローム走行


void sample_course_run(void);	//試験コース走行
void perfect_run(void);			//本番用走行
void perfect_slalom(void);		//本番用スラローム走行

#endif /* DRIVE_H_ */
