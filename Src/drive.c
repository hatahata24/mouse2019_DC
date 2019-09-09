
#include "global.h"
#include "math.h"

//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_init
//a走行系の変数の初期化
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_init(void){
	MF.FLAGS = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_start
//a走行開始前に走行距離と機体角度を初期化
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_start(void){
	dist_l = dist_r = 0;		//a走行距離の初期化
	degree_z = 0;				//a機体角度の初期化
	if(H_accel_flag == 0) pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	MF.FLAG.SPD = 1;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_stop
//a走行を終了する
//a（タイマを止めてタイマカウント値を0にリセットする）
//a引数1：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_stop(void){
	dist_l = dist_r = 0;		//a走行距離の初期化
	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 0;
	MF.FLAG.SPD = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//control_start
// wallとgyroの姿勢制御を開始する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void control_start(){
	MF.FLAG.WCTRL = 1;										//wall制御を有効にする
	MF.FLAG.GCTRL = 1;										//gyro制御を有効にする

	target_omega_z = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//control_stop
// wallとgyroの姿勢制御を停止する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void control_stop(){
	MF.FLAG.WCTRL = 0;										//wall制御を無効にする
	MF.FLAG.GCTRL = 0;										//gyro制御を無効にする

	target_omega_z = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_dir
// wheel turn dir for each wheel
//a引数:1車輪選択(0=>L, 1=>R), 2回転方向選択(0=>CW, 1=>CWW, 2=>ShortBrake, 3=>free)
//a戻り値: nothing
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_dir(uint8_t wheel, uint8_t dir){
	if(wheel == 0){
		if(dir == 0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//L_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else if(dir == 1){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);	//L_CW
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//L_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else if(dir == 2){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//L_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);	//STBY
		}
	}else{
		if(dir == 0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	//R_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else if(dir == 1){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);	//R_CW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	//R_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else if(dir == 2){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	//R_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);	//STBY
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveA
// a指定距離、指定加速度で加速走行する
// a引数1：accel_p 加速度, 引数2：speed_min_p 最低速度, 引数3：speed_max_p 最高速度, 引数4：dist 走行距離
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveA(uint16_t accel_p, uint16_t speed_min_p, uint16_t speed_max_p, uint16_t dist){

	speed_min_l = speed_min_r = speed_min_p;
	speed_max_l = speed_max_r = speed_max_p;
	accel_l = accel_r = accel_p;							//a引数の各パラメータをグローバル変数化
	if(H_accel_flag == 0)target_speed_l = target_speed_r = speed_min_p;

	//if(MF.FLAG.STRT == 0) speed_l = speed_r = 100;		//a最初の加速の際だけspeedを定義
	drive_start();											//a走行開始

	//----a走行----
	while((dist_l < dist) || (dist_r < dist));				//a左右のモータが指定距離以上進むまで待機

	drive_stop();											//a走行停止
	//MF.FLAG.STRT = 1;										//2回目以降の加速の際はspeedは既存のスピードを用いる
	//get_wall_info();										//a壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveD
//a指定距離、指定減速度で減速走行する
//a引数1：accel_p 加速度, 引数2：speed_min_p 最低速度, 引数3：speed_max_p 最高速度, 引数4：dist 走行距離
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveD(int16_t accel_p, uint16_t speed_min_p, uint16_t speed_max_p, uint16_t dist){

	float speed_0 = speed_l;								//直線パルス数を計算するためにTIM15より参照
	speed_min_l = speed_min_r = speed_min_p;
	speed_max_l = speed_max_r = speed_max_p;
	accel_l = accel_r = accel_p;										//引数の各パラメータをグローバル変数化

	drive_start();											//走行開始

	int16_t c_dist = dist - (speed_min_l*speed_min_l  - speed_0*speed_0)/(2*accel_l);			//等速走行距離 = 総距離 - 減速に必要な距離

	accel_l = accel_r = 0;
	if(c_dist > 0){
		//----等速走行----
		while((dist_l < c_dist) || (dist_r < c_dist));	//a左右のモータが等速分の距離以上進むまで待機
	}
	accel_l = accel_r = accel_p;
	//dist_l = 0;
	//dist_r = 0;
	//----減速走行----
	while((dist_l < dist) || (dist_r < dist));			//a左右のモータが減速分の距離以上進むまで待機

	//MF.FLAG.STRT = 0;
	drive_stop();											//走行停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveU
// 指定パルス分等速走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveU(uint16_t dist){

	accel_l = accel_r = 0;												//等速走行のため加速度は0
	drive_start();											//走行開始

	//----走行----
	while((dist_l < dist) || (dist_r < dist)){				//左右のモータが指定パルス以上進むまで待機
		if(MF.FLAG.WEDGE == 1){
			if(ad_l < WALL_BASE_L-30 || ad_r < WALL_BASE_R-10){
				while((dist_l < W_DIST) || (dist_r < W_DIST));	//左右のモータが壁切れ用指定パルス以上進むまで待機
			break;
			}
		}
	}

	drive_stop();											//走行停止
//	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveC
// 指定パルス分デフォルト速度で走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC(uint16_t dist){

	speed_min_l = speed_min_r = 150;
	speed_max_l = speed_max_r = 150;
	accel_l = accel_r = 0;												//等速走行のため加速度は0

	drive_start();											//走行開始
//	MF.FLAG.LOG = 1;
	//====回転====
	while((dist_l < dist) || (dist_r < dist));			//左右のモータが定速分のパルス以上進むまで待機

	drive_stop();											//走行停止
//	MF.FLAG.LOG = 0;

/*	while(ad_l <= 100);

	for(int i=0; i<log_allay; i++){
		printf("l:	%d\n", get_speed_l[i]);
		HAL_Delay(5);
	}
	for(int i=0; i<log_allay; i++){
		printf("r:	%d\n", get_speed_r[i]);
		HAL_Delay(5);
	}
*/
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveC2
// 指定パルス分デフォルト速度で走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC2(uint16_t dist){

	speed_min_l = speed_min_r = -250;
	speed_max_l = speed_max_r = -250;
	accel_l = accel_r = 0;												//等速走行のため加速度は0

	drive_start();											//走行開始
//	MF.FLAG.LOG = 1;
	//====回転====
	while((dist_l > (-1*dist)) || (dist_r > (-1*dist)));			//左右のモータが定速分のパルス以上進むまで待機

	drive_stop();											//走行停止
//	MF.FLAG.LOG = 0;

/*	while(ad_l <= 100);

	for(int i=0; i<log_allay; i++){
		printf("l:	%d\n", get_speed_l[i]);
		HAL_Delay(5);
	}
	for(int i=0; i<log_allay; i++){
		printf("r:	%d\n", get_speed_r[i]);
		HAL_Delay(5);
	}
*/
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//set_position
// 機体の尻を壁に当てて場所を区画中央に合わせる
// 引数：sw …… 0以外ならget_base()する
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_position(){

  driveC2(SETPOS_BACK);          //尻を当てる程度に後退。回転後に停止する
  driveC(SETPOS_SET);           //デフォルトインターバルで指定パルス分回転。回転後に停止する
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//start_sectionA
// aスタート区画分加速しながら走行する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void start_sectionA(void){

	control_start();
	if(start_flag == 0){
		driveA(4000, SPEED_MIN, SPEED_RUN, SEC_START);					//半区画のパルス分加速しながら走行。走行後は停止しない
	}else{
		driveA(4000, SPEED_MIN, SPEED_RUN, SEC_HALF);					//半区画のパルス分加速しながら走行。走行後は停止しない
	}
	start_flag = 1;
	if(MF.FLAG.SCND == 0)get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionA
// a半区画分加速しながら走行する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA(void){

	control_start();
	driveA(4000, SPEED_MIN, SPEED_RUN, SEC_HALF);					//半区画のパルス分加速しながら走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionD
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD(void){

	control_start();
	driveD(-4000, SPEED_MIN, SPEED_RUN, SEC_HALF);				//指定パルス分指定減速度で減速走行。走行後は停止する
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//start_sectionA2
// aスタート区画分加速しながら走行する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void start_sectionA2(void){

	control_start();
	if(start_flag == 0){
		driveA(8000, SPEED_MIN, SPEED_HIGH, SEC_START);					//半区画のパルス分加速しながら走行。走行後は停止しない
	}else{
		driveA(8000, SPEED_MIN, SPEED_HIGH, SEC_HALF);					//半区画のパルス分加速しながら走行。走行後は停止しない
	}
	start_flag = 1;
	if(MF.FLAG.SCND == 0)get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionA2
// a半区画分加速しながら走行する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA2(void){
	full_led_write(1);
	control_start();
	driveA(8000, SPEED_MIN, SPEED_HIGH, SEC_HALF);				//半区画のパルス分加速しながら走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();											//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionD2
//a半区画分減速しながら走行し停止する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD2(void){
	full_led_write(3);
	control_start();
	driveD(-8000, SPEED_MIN, SPEED_HIGH, SEC_HALF);				//指定パルス分指定減速度で減速走行。走行後は停止する
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_section
//a1区画分進んで停止する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_section(void){

	half_sectionA();										//半区画分加速走行
	half_sectionD();										//半区画分減速走行のち停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionA
//a1区画分加速する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionA(void){
	full_led_write(4);
	control_start();
	driveA(accel_hs, SPEED_RUN, speed_max_hs, SEC_HALF*2);			//1区画のパルス分加速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();												//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionD
//a1区画分減速する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionD(void){
	full_led_write(2);
	control_start();
	driveD(-1*accel_hs, SPEED_RUN, speed_max_hs, SEC_HALF*2);		//1区画のパルス分減速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();												//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionA2
//a1区画分加速する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionA2(void){
	full_led_write(4);
	control_start();
	driveA(accel_hs, SPEED_HIGH, speed_max_hs, SEC_HALF*2);			//1区画のパルス分加速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();												//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionD2
//a1区画分減速する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionD2(void){
	full_led_write(2);
	control_start();
	driveD(-1*accel_hs, SPEED_HIGH, speed_max_hs, SEC_HALF*2);		//1区画のパルス分減速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();												//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionU
//a等速で1区画分進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionU(void){
	full_led_write(7);
	control_start();
	driveU(SEC_HALF*2);										//半区画のパルス分等速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();											//壁情報を取得
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_R90
//a右に90度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_R90(void){
	target_omega_z = 800;
	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = target_omega_z/180*M_PI * TREAD/2;
	speed_min_r = -1*target_omega_z/180*M_PI * TREAD/2;

	drive_start();											//走行開始
	control_stop();
	while(degree_z > -80.3);
	drive_stop();

	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = 100;
	speed_min_r = -100;

	drive_start();											//走行開始
	while(degree_z > -90+80);

	turn_dir(DIR_TURN_R90);									//マイクロマウス内部位置情報でも左回転処理
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_L90
//a左に90度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_L90(void){
	target_omega_z = 800;
	accel_l = -3000;
	accel_r = 3000;
	speed_min_l = -1*target_omega_z/180*M_PI * TREAD/2;
	speed_max_r = target_omega_z/180*M_PI * TREAD/2;

	drive_start();											//走行開始
	control_stop();
	while(degree_z < 80.3);
	drive_stop();

	accel_l = -3000;
	accel_r = 3000;
	speed_min_l = -100;
	speed_max_r = 100;

	drive_start();											//走行開始
	while(degree_z < 90-80);

	turn_dir(DIR_TURN_L90);									//マイクロマウス内部位置情報でも左回転処理
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_180
//a180度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_180(void){
	target_omega_z = 800;
	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = target_omega_z/180*M_PI * TREAD/2;
	speed_min_r = -1*target_omega_z/180*M_PI * TREAD/2;

	drive_start();											//走行開始
	control_stop();
	while(degree_z > -170.6);
	drive_stop();

	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = 100;
	speed_min_r = -100;

	drive_start();											//走行開始
	while(degree_z > -180+170);

	turn_dir(DIR_TURN_180);									//マイクロマウス内部位置情報でも180度回転処理
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_R90
//aスラロームで左に90度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_R90(void){
	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = -10000;
	accel_r = -10000;
	speed_min_l = 400;
	speed_min_r = 400;

	drive_start();											//走行開始
	control_start();
	while(dist_l < 18 && dist_r < 18);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = 4000;
	omega_max = 550;
	speed_G = 400;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -38.087);

	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -22);

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -4000;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -28);

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 18 && dist_r < 18);
	turn_dir(DIR_TURN_R90);									//マイクロマウス内部位置情報でも左回転処理
	if(MF.FLAG.SCND == 0)get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_L90
//aスラロームで右に90度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_L90(void){
	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 3000;
	accel_r = 3000;
	speed_max_l = 400;
	speed_max_r = 400;

	drive_start();											//走行開始
	control_start();
	while(dist_l < 18 && dist_r < 18);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -4000;
	omega_min = -550;
	speed_G = 400;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 38.087);
	drive_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 22);
	drive_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = 4000;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 28);
	drive_stop();

	MF.FLAG.GYRO = 0;

	accel_l = 3000;
	accel_r = 3000;
	speed_max_l = 400;
	speed_max_r = 400;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 18 && dist_r < 18);
	turn_dir(DIR_TURN_L90);									//マイクロマウス内部位置情報でも左回転処理
	if(MF.FLAG.SCND == 0)get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_R90
//aスラロームで左に90度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_R902(void){
	//MF.FLAG.LOG = 1;

	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 800;
	speed_max_r = 800;

	drive_start();											//走行開始
	control_start();
//	while(dist_l < 10 && dist_r < 10);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = 20000;
	omega_max = 800;
	speed_G = 800;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -32);

	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -34);

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -20000;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -14);

	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 800;
	speed_max_r = 800;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 34 && dist_r < 34);
	turn_dir(DIR_TURN_R90);									//マイクロマウス内部位置情報でも左回転処理
	if(MF.FLAG.SCND == 0)get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_L90
//aスラロームで右に90度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_L902(void){
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 800;
	speed_max_r = 800;

	drive_start();											//走行開始
	control_start();
//	while(dist_l < 18.5 && dist_r < 18.5);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -20000;
	omega_min = -800;
	speed_G = 800;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 32);
	drive_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 34.3);
	drive_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = 20000;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 14);
	drive_stop();

	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 800;
	speed_max_r = 800;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 34 && dist_r < 34);
	turn_dir(DIR_TURN_L90);									//マイクロマウス内部位置情報でも左回転処理
	if(MF.FLAG.SCND == 0)get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R45
//a区画中心から左に45度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R45(void){
	//MF.FLAG.LOG = 1;

	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;

	drive_start();											//走行開始
//	control_start();
	while(dist_l < 25 && dist_r < 25);
	drive_stop();
	control_stop();

	full_led_write(3);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 6000;
	omega_max = 300;
	speed_G = 600;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -7.5);

	full_led_write(4);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -37.5);

	full_led_write(5);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = -6000;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -45);

	full_led_write(6);
	MF.FLAG.GYRO = 0;

/*	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 800;
	speed_max_r = 800;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 34 && dist_r < 34);
	turn_dir(DIR_TURN_R90);									//マイクロマウス内部位置情報でも左回転処理
	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
*/	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L45
//a区画中心から右に45度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L45(void){
	//MF.FLAG.LOG = 1;

	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;

	drive_start();											//走行開始
//	control_start();
	while(dist_l < 25 && dist_r < 25);
	drive_stop();
	control_stop();

	full_led_write(3);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = -6000;
	omega_max = -300;
	speed_G = 600;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 7.5);

	full_led_write(4);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 37.5);

	full_led_write(5);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 6000;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > 45);

	full_led_write(6);
	MF.FLAG.GYRO = 0;

/*	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 800;
	speed_max_r = 800;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 34 && dist_r < 34);
	turn_dir(DIR_TURN_R90);									//マイクロマウス内部位置情報でも左回転処理
	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
*/	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R90
//a柱中心から左に90度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R90(void){
	//MF.FLAG.LOG = 1;

	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;

	drive_start();											//走行開始
//	control_start();
	while(dist_l < 10 && dist_r < 10);
	drive_stop();
	control_stop();

	full_led_write(3);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 4000;
	omega_max = 400;
	speed_G = 600;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -20);

	full_led_write(4);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -73);

	full_led_write(5);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = -6000;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -90);

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 10 && dist_r < 10);
//	turn_dir(DIR_TURN_R90);									//マイクロマウス内部位置情報でも左回転処理
//	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L90
//a柱中心から右に90度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L90(void){
	//MF.FLAG.LOG = 1;

	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;

	drive_start();											//走行開始
//	control_start();
	while(dist_l < 10 && dist_r < 10);
	drive_stop();
	control_stop();

	full_led_write(3);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = -4000;
	omega_max = -400;
	speed_G = 600;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 20);

	full_led_write(4);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 73);

	full_led_write(5);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = -6000;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 90);

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 10 && dist_r < 10);
//	turn_dir(DIR_TURN_R90);									//マイクロマウス内部位置情報でも左回転処理
//	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R135
//a区画中心から左に135度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R135(void){
	//MF.FLAG.LOG = 1;

	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;

	drive_start();											//走行開始
//	control_start();
	while(dist_l < 50 && dist_r < 50);
	drive_stop();
	control_stop();

	full_led_write(3);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 10000;
	omega_max = 400;
	speed_G = 600;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -8);

	full_led_write(4);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -130);

	full_led_write(5);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = -10000;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z > -135);

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 35 && dist_r < 35);
//	turn_dir(DIR_TURN_R90);									//マイクロマウス内部位置情報でも左回転処理
//	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L135
//a区画中心から右に135度回転する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L135(void){
	//MF.FLAG.LOG = 1;

	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;

	drive_start();											//走行開始
//	control_start();
	while(dist_l < 50 && dist_r < 50);
	drive_stop();
	control_stop();

	full_led_write(3);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = -10000;
	omega_max = -400;
	speed_G = 600;

	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 8);

	full_led_write(4);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 0;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 130);

	full_led_write(5);
	MF.FLAG.GYRO = 1;

	target_degaccel_z = 10000;

//	degree_z = 0;				//a機体角度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	while(degree_z < 135);

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 5000;
	accel_r = 5000;
	speed_max_l = 600;
	speed_max_r = 600;
	drive_start();											//走行開始
	control_start();
	while(dist_l < 35 && dist_r < 35);
//	turn_dir(DIR_TURN_R90);									//マイクロマウス内部位置情報でも左回転処理
//	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
	drive_stop();
}


/*----------------------------------------------------------
		テスト関数
----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//init_test
//a初期基幹関数走行テスト
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void init_test(void){

	int mode = 0;
	printf("Test Init Run, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  switch(mode){
				case 0:
					get_base();
					break;
				case 1:
					//----4区画等速走行----
					printf("4 Section, Forward, Constant Speed.\n");
					for(int i = 0; i < 1; i++){
						driveC(SEC_HALF*2);	//一区画のパルス分デフォルトインターバルで走行
					}
					break;
				case 2:
					//----right90度回転----
					printf("Rotate R90.\n");
					for(int i = 0; i < 32; i++){
						rotate_R90();				//16回右90度回転、つまり4周回転
					}
					break;
				case 3:
					//----left90度回転----
					printf("Rotate L90.\n");
					for(int i = 0; i < 32; i++){
						rotate_L90();				//16回左90度回転、つまり4周回転
					}
					break;
				case 4:
					//----180度回転----
					printf("Rotate 180.\n");
					for(int i = 0; i < 16; i++){
						rotate_180();				//8回右180度回転、つまり4周回転
					}
					break;
				case 5:
					//----4区画連続走行----
					printf("4 Section, Forward, Continuous.\n");
					half_sectionA();				//半区画のパルス分加速しながら走行
					for(int i = 0; i < 6-1; i++){
						one_sectionU();			//一区画のパルス分等速走行
					}
					half_sectionD();				//半区画のパルス分減速しながら走行。走行後は停止する
					break;
				case 6:
					set_position();
					break;
				case 7:
					target_degree_z = degree_z;
					accel_l = 5000;

					MF.FLAG.ENKAI = 1;

					while(1);
					break;
			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_test
//aスラローム走行テスト
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_test(void){

	int mode = 0;
	printf("Test Slalom Run, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  while(ad_fl <= WALL_BASE_FL){
				  led_write(1, 1, 1);
				  HAL_Delay(200);
				  led_write(0, 0, 0);
				  HAL_Delay(200);
			  }
			  HAL_Delay(2000);

			  switch(mode){
				case 0:
					get_base();
					break;
				case 1:
					//----slalom右折----
					printf("slalom turn right .\n");
					half_sectionA();
					for(int i = 0; i < 32; i++){
						slalom_R90();	//一区画のパルス分デフォルトインターバルで走行
						one_sectionU();
					}
					half_sectionD();
					break;
				case 2:
					//----slalom左折----
					printf("slalom turn left .\n");
					half_sectionA();
					for(int i = 0; i < 32; i++){
						slalom_L90();				//16回右90度回転、つまり4周回転
						one_sectionU();
					}
					half_sectionD();
					break;
				case 3:
					//----slalom右折----
					printf("slalom turn right .\n");
					for(int i = 0; i < 1; i++){
						half_sectionA();
						slalom_R90();	//一区画のパルス分デフォルトインターバルで走行
						half_sectionD();
					}
				//log print
					while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);

					for(int i=0; i<log_allay; i++){
						printf("l:	%d\n", get_speed_l[i]);
						HAL_Delay(5);
					}
					for(int i=0; i<log_allay; i++){
						printf("r:	%d\n", get_speed_r[i]);
						HAL_Delay(5);
					}
					break;
				case 4:
					//----slalom左折----
					printf("slalom turn left .\n");
					for(int i = 0; i < 8; i++){
						half_sectionA();
						slalom_L90();				//16回右90度回転、つまり4周回転
						half_sectionD();
					}
					break;
				case 5:
					//----slalom2右折----
					printf("slalom turn right .\n");
					half_sectionA2();
					for(int i = 0; i < 1; i++){
						full_led_write(1);
						slalom_R902();	//一区画のパルス分デフォルトインターバルで走行
						full_led_write(2);
						one_sectionU();
					}
					full_led_write(3);
					half_sectionD2();
					break;
				case 6:
					//----slalom2右折----
					printf("slalom turn right .\n");
					half_sectionA2();
					for(int i = 0; i < 16; i++){
						full_led_write(1);
						slalom_R902();	//一区画のパルス分デフォルトインターバルで走行
						full_led_write(2);
						one_sectionU();
					}
					full_led_write(3);
					half_sectionD2();
					break;
				case 7:
					//----slalom2左折----
					printf("slalom turn left .\n");
					half_sectionA2();
					for(int i = 0; i < 16; i++){
						full_led_write(1);
						slalom_L902();				//16回右90度回転、つまり4周回転
						full_led_write(2);
						one_sectionU();
					}
					full_led_write(3);
					half_sectionD2();
					break;
			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_test
//aスラローム走行テスト
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_test(void){

	int mode = 0;
	printf("Test V Run, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  while(ad_fl <= WALL_BASE_FL){
				  led_write(1, 1, 1);
				  HAL_Delay(200);
				  led_write(0, 0, 0);
				  HAL_Delay(200);
			  }

			  switch(mode){
				case 0:
					get_base();
					break;
				case 1:
					//----V左45----
					printf("V 45 right .\n");
					full_led_write(7);
					half_sectionA();
					for(int i = 0; i < 1; i++){
						v_R45();
						v_R45();
					}
					full_led_write(7);
					half_sectionD();
					break;
				case 2:
					break;
				case 3:
					break;
				case 4:
					break;
				case 5:
					break;
				case 6:
					break;
				case 7:
					break;
			}
		}
	}
}


void test_select(void){
	int mode = 0;
	printf("Test Select, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  switch(mode){
				case 1:
					init_test();
					break;

				case 2:
					slalom_test();
					break;

				case 3:
					sample_course_run();
					break;
				case 4:
					v_test();
			  }
		  }
	}
}


/*----------------------------------------------------------
		走行モード選択関数
----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//simple_run
//a超新地走行モード
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void simple_run(void){

	int mode = 0;
	printf("Simple Run, Mode : %d\n", mode);

	while(1){

		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  switch(mode){

				case 0:
					break;
				case 1:
					//----a一次探索走行----
					printf("First Run.\n");

					MF.FLAG.SCND = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchA();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchA();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 2:
					//----a一次探索連続走行----
					printf("First Run. (Continuous)\n");

					MF.FLAG.SCND = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchB();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchB();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 3:
					//----a二次探索走行----
					printf("Second Run. (Continuous)\n");

					MF.FLAG.SCND = 1;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchB();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchB();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 4:
					break;

				case 5:
					break;

				case 6:
					break;

				case 7:
					break;
			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_run
//aスラローム走行モード
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_run(void){

	int mode = 0;
	printf("Slalom Run, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  switch(mode){

			  case 0:
					//----a一次探索スラローム走行----
					printf("First Run. (Slalom)\n");

					MF.FLAG.SCND = 0;
					start_flag = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 1:
					//---a二次探索スラローム走行----
					printf("Second Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					start_flag = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 2:
					//----a二次探索スラローム走行+既知区間加速----
					printf("Second Run. (Slalom+accel)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 600;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchD();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 3:
					//----a二次探索スラローム走行+既知区間加速----
					printf("Second Run. (Slalom+accel)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					accel_hs = 5000;
					speed_max_hs = 1000;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchD();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 4:
					//----a二次探索スラローム走行+既知区間加速----
					printf("Second Run. (Slalom+accel)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					accel_hs = 5000;
					speed_max_hs = 1500;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchD();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 5:
					//----a二次走行+直線優先----
					printf("High Speed Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					HAL_Delay(5000);
					searchC();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 6:
					//----a二次走行+直線優先----
					printf("High Speed Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
//					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					HAL_Delay(5000);
					searchC2();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchC2();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 7:
					break;

			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//sample_course_run
//a試験走行モード
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void sample_course_run(void){

	int mode = 0;
	printf("Sample Course Run, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  switch(mode){
				case 0:
					get_base();
					break;

				case 1:
					//----aサンプルコース1　超信地----
					half_sectionA();
					half_sectionD();
					rotate_R90();
					half_sectionA();
					half_sectionD();
					rotate_R90();
					half_sectionA();
					half_sectionD();
					break;

				case 2:
					//----aサンプルコース1　超信地----
					half_sectionA();
					half_sectionD();
					rotate_L90();
					half_sectionA();
					half_sectionD();
					rotate_L90();
					half_sectionA();
					half_sectionD();
					break;

				case 3:
					//---aサンプルコース2　スラローム----
					half_sectionA();
					slalom_R90();
					slalom_R90();
					half_sectionD();
					break;

				case 4:
					//----a二次探索スラローム走行+既知区間加速----
/*					printf("Second Run. (Slalom+accel)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					accel_hs = 3000;
					speed_max_hs = 1000;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
*/					break;

				case 5:
					//----aスラローム走行&全面探索スラローム走行----
					printf("Slalom Run & All Map Run.\n");

					MF.FLAG.SCND = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchE();

					searchC();
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 6:
					//----aスラローム走行&全面探索スラローム走行----
					printf("Slalom Run & All Map Run.\n");

					MF.FLAG.SCND = 0;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchE();

					searchC();
					goal_x = 7;
					goal_y = 7;

					break;

				case 7:
					break;

			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//perfect_run
//a本番用走行モード
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void perfect_run(void){

	int mode = 0;
	printf("Perfect Run, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  switch(mode){

				case 0:
					break;

				case 1:
					//----a一次探索連続走行----
					printf("First Run. (Continuous)\n");

					MF.FLAG.SCND = 0;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchB();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchB();

					goal_x = 7;
					goal_y = 7;

					break;

				case 2:
					//----a二次探索走行----
					printf("Second Run. (Continuous)\n");

					MF.FLAG.SCND = 1;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchB();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchB();

					goal_x = 7;
					goal_y = 7;

					break;

				case 3:
					//----a一次探索スラローム走行----
					printf("First Run. (Slalom)\n");

					MF.FLAG.SCND = 0;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchC();

					goal_x = 7;
					goal_y = 7;

					break;

				case 4:
					//---a二次探索スラローム走行----
					printf("Second Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchC();

					goal_x = 7;
					goal_y = 7;

					break;

				case 5:
					break;

				case 6:
					break;
				case 7:
			  		  for(int i=0; i<m_select; i++){
			  			  buzzer(mario_select[i][0], mario_select[i][1]);
			  		  }
					perfect_slalom();
					break;
			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//perfect_slalom
//a本番用スラローム走行モード
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void perfect_slalom(void){

	int mode = 0;
	printf("Perfect Slalom, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
	  		  for(int i=0; i<m_select; i++){
	  			  buzzer(mario_select[i][0], mario_select[i][1]);
	  		  }
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
	  		  for(int i=0; i<m_select; i++){
	  			  buzzer(mario_select[i][0], mario_select[i][1]);
	  		  }
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  for(int i=0; i<m_ok; i++){
	  			  buzzer(mario_ok[i][0], mario_ok[i][1]);
	  		  }

			  while(ad_fl <= WALL_BASE_FL){
				  led_write(1, 1, 1);
				  HAL_Delay(200);
				  led_write(0, 0, 0);
				  HAL_Delay(200);
			  }
	  		  for(int i=0; i<m_start; i++){
	  			  buzzer(mario_start[i][0], mario_start[i][1]);
	  			  full_led_write(1);
	  		  }

			  switch(mode){
				case 0:
					break;

				case 1:
					//----a一次探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 0;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchC();

					goal_x = 7;
					goal_y = 7;
					break;

				case 2:
					//----a二次走行スラローム+既知区間加速走行 speed1----
					printf("First Run. (Continuous)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;

					accel_hs = 3000;
					speed_max_hs = 600;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchD();

					goal_x = 7;
					goal_y = 7;
					break;

				case 3:
					//----a二次探索スラローム+既知区間加速走行 speed2----
					printf("Second Run. (Continuous)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;

					accel_hs = 3000;
					speed_max_hs = 1000;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchD();

					goal_x = 7;
					goal_y = 7;
					break;

				case 4:
					//----a二次探索スラローム+既知区間加速走行 speed3----
					printf("First Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;

					accel_hs = 3000;
					speed_max_hs = 1200;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchD();

					goal_x = 7;
					goal_y = 7;
					break;

				case 5:
					//----a二次探索スラロームHigh Speed----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 0;
					MF.FLAG.STRAIGHT = 1;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC2();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchC2();

					goal_x = 7;
					goal_y = 7;
					break;

				case 6:
					//----a二次探索スラロームHigh Speed----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;

					accel_hs = 3000;
					speed_max_hs = 1200;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD2();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchD2();

					goal_x = 7;
					goal_y = 7;
					break;

				case 7:
					//----a二次探索スラロームHigh Speed + 既知区間加速----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;

					accel_hs = 3000;
					speed_max_hs = 1600;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD2();
					HAL_Delay(500);

					goal_x = goal_y = 0;
					searchD2();

					goal_x = 7;
					goal_y = 7;
					break;
			}
		}
	}
}
