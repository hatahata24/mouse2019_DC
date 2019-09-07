
#ifndef DRIVE_H_
#define DRIVE_H_

//====変数====
#ifdef MAIN_C_										//main.cからこのファイルが呼ばれている場合
	/*aグローバル変数の定義*/
	//====a物理量走行関連====
	volatile float cnt_l, cnt_r;
	volatile float dist_l, dist_r;
	volatile float accel_l, accel_r;
	volatile float speed_l, speed_r, speed_G;

	volatile float target_speed_l, target_speed_r;
	volatile float speed_min_l, speed_max_l;
	volatile float speed_min_r, speed_max_r;
	volatile float target_dist;
	volatile float pulse_l, pulse_r;

	volatile float epsilon_l, epsilon_r;
	volatile float epsilon_sum;
	volatile float old_epsilon;
	volatile float epsilon_dif;
	volatile int16_t accel_hs, speed_max_hs;		//a既知区間加速時使用

	volatile uint8_t start_flag;

	volatile int get_speed_l[log_allay];
	volatile int get_speed_r[log_allay];
#else												//main.c以外からこのファイルが呼ばれている場合
	/*aグローバル変数の宣言*/
	//====a物理量走行関連====
	extern volatile float cnt_l, cnt_r;
	extern volatile float dist_l, dist_r;
	extern volatile float accel_l, accel_r;
	extern volatile float speed_l, speed_r, speed_G;

	extern volatile float target_speed_l, target_speed_r;
	extern volatile float speed_min_l, speed_max_l;
	extern volatile float speed_min_r, speed_max_r;
	extern volatile float target_dist;
	extern volatile float pulse_l, pulse_r;

	extern volatile float epsilon_l, epsilon_r;
	extern volatile float epsilon_sum;
	extern volatile float old_epsilon;
	extern volatile float epsilon_dif;
	extern volatile int16_t accel_hs, speed_max_hs;	//a既知区間加速時使用

	extern volatile uint8_t start_flag;

	extern volatile int get_speed_l[log_allay];
	extern volatile int get_speed_r[log_allay];
#endif


#define drive_wait()	HAL_Delay(50)


/*============================================================
		関数プロトタイプ宣言
============================================================*/
void drive_init(void);
void drive_dir(uint8_t, uint8_t);
void drive_start(void);
void drive_stop(void);
void control_start(void);
void control_stop(void);


//====a走行系====
//----a基幹関数----
void driveA(uint16_t, uint16_t, uint16_t, uint16_t);		//a加速走行
void driveD(int16_t, uint16_t, uint16_t, uint16_t);			//a減速走行
void driveU(uint16_t);			//a等速走行（前の速度を維持）
void driveC(uint16_t);			//aデフォルトインターバルで走行
void driveC2(uint16_t);			//aデフォルトインターバルで逆方向走行
void slalomU12(uint16_t);
void slalomR12(uint16_t, uint16_t, uint16_t, uint16_t);
void slalomR22(uint16_t);
void slalomR32(int32_t, uint16_t, uint16_t, uint16_t, uint16_t);
void slalomU22(uint16_t);


//----a上位関数----
void set_position(void);		//a上下位置合わせ
void set_positionX(uint8_t);	//a上下左右位置合わせ
void start_sectionA(void);		//a加速スタート区画
void start_sectionA2(void);		//a加速スタート区画
void half_sectionA(void);		//a加速半区画
void half_sectionA2(void);		//a加速半区画
void half_sectionD(void);		//a減速半区画
void half_sectionD2(void);		//a減速半区画
void one_section(void);			//a加減速一区画
void one_sectionA(void);		//a加減速一区画
void one_sectionA2(void);		//a加減速一区画
void one_sectionD(void);		//a加減速一区画
void one_sectionD2(void);		//a加減速一区画
void one_sectionU(void);		//a等速一区画
void rotate_R90(void);			//a右90回転
void rotate_L90(void);			//a左90回転
void rotate_180(void);			//180度回転
void slalom_R90(void);			//aスラローム右90回転
void slalom_L90(void);			//aスラローム左90回転
void slalom_R902(void);			//aスラローム右90回転
void slalom_L902(void);			//aスラローム左90回転

void v_R45(void);				//V45右
void v_L45(void);				//V45左
void v_R90(void);				//V90右
void v_L90(void);				//V90左
void v_R135(void);				//V135右
void v_L135(void);				//V135左


//----a走行関数----
void test_select(void);			//aテスト走行選択

void init_test(void);			//a初期基幹関数走行テスト
void slalom_test(void);			//aスラロームによるテスト走行
void v_test(void);			//a直線優先や全面探索のテスト走行

void simple_run(void);			//a超信地走行
void slalom_run(void);			//aスラローム走行
void sample_course_run(void);	//a試験コース走行


void perfect_run(void);			//a本番用走行
void perfect_slalom(void);		//a本番用スラローム走行

#endif /* DRIVE_H_ */
