
#ifndef DRIVE_H_
#define DRIVE_H_

//====変数====
#ifdef MAIN_C_										//main.cからこのファイルが呼ばれている場合
	/*グローバル変数の定義*/

	//====物理量走行関連====
	volatile float speedL, speedR;
	volatile int16_t accel, speed_min, speed_max;
	volatile float widthR, widthL;

	volatile uint16_t target, target_flag;			//スラローム走行時使用
	volatile int16_t accel_hs, speed_max_hs;		//既知区間加速時使用
#else												//main.c以外からこのファイルが呼ばれている場合
	/*グローバル変数の宣言*/
	//====物理量走行関連====
	extern volatile float speedL, speedR;
	extern volatile int16_t accel, speed_min, speed_max;
	extern volatile float widthR, widthL;

	extern volatile uint16_t target, target_flag;	//スラローム走行時使用
	extern volatile int16_t accel_hs, speed_max_hs;	//既知区間加速時使用
#endif


#define drive_wait()	ms_wait(50)


/*============================================================
		関数プロトタイプ宣言
============================================================*/
void drive_init(void);
void drive_enable_motor(void);
void drive_disable_motor(void);
void drive_start(void);
void drive_start2(void);
void drive_stop(void);
void drive_stop2(void);
void drive_set_dir(uint8_t);	//進む方向の設定
float dist_pulse(uint16_t);		//距離(mm)をパルス数に変換

//====走行系====
//----基幹関数----
void driveA(uint16_t);			//加速走行
void driveD(uint16_t);			//減速走行
void driveU(uint16_t);			//等速走行（前の速度を維持）
void driveC(uint16_t);			//デフォルトインターバルで走行
void slalomU1(uint16_t);
void slalomR1(uint16_t);
void slalomR2(uint16_t);
void slalomR3(uint16_t);
void slalomU2(uint16_t);

void driveA2(uint16_t, uint16_t, uint16_t, uint16_t);		//加速走行
void driveD2(int16_t, uint16_t, uint16_t, uint16_t);		//減速走行
void driveU2(uint16_t);			//等速走行（前の速度を維持）
void driveC2(uint16_t);			//デフォルトインターバルで走行
void slalomU12(uint16_t);
void slalomR12(uint16_t, uint16_t, uint16_t, uint16_t);
void slalomR22(uint16_t);
void slalomR32(int32_t, uint16_t, uint16_t, uint16_t, uint16_t);
void slalomU22(uint16_t);


//----上位関数----
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

void half_sectionA2(void);		//加速半区画
void half_sectionD2(void);		//減速半区画
void one_section2(void);		//加減速一区画
void one_sectionA2(void);		//加速一区画　既知区間加速時の加速用
void one_sectionD2(void);		//減速一区画　既知区間加速時の減速用
void one_sectionU2(void);		//等速一区画
void rotate_R902(void);			//右90回転
void rotate_L902(void);			//左90回転
void rotate_1802(void);			//180度回転
void slalom_R902(void);			//スラローム右90回転	右旋回時に右輪が減速しない原因不明の事態に陥ったため物理量スラロームは不使用中
void slalom_L902(void);			//スラローム左90回転　上記理由のため物理量スラロームは不使用中
void set_position2(uint8_t);	//上下位置合わせ
void set_positionX2(uint8_t);	//上下左右位置合わせ　初めのケツあて時のモータ音に違和感


//----走行関数----
void defo_test(void);			//初期状態でのtableによるテスト走行
void physic_test(void);			//物理量ベースによるテスト走行
void accel_test(void);			//既知区間加速によるテスト走行
void slalom_test(void);			//スラロームによるテスト走行
void search_test(void);			//直線優先や全面探索のテスト走行

void test_select(void);			//テスト走行選択

void simple_run(void);			//超新地走行
void slalom_run(void);			//スラローム走行
void sample_course_run(void);	//試験コース走行
void perfect_run(void);			//本番用走行
void perfect_slalom(void);		//本番用スラローム走行

#endif /* DRIVE_H_ */
