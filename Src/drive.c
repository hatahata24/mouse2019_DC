
#include "global.h"
#include "math.h"

//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_init
// 走行系の変数の初期化
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_init(void){
	MF.FLAGS = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_ready
// 走行前のLED点滅&ジャイロのドリフト計算
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_ready(void){
	  while(ad_fl <= WALL_BASE_FL){
		  led_write(1, 1, 1);
		  HAL_Delay(200);
		  led_write(0, 0, 0);
		  HAL_Delay(200);
	  }
	  gyro_drift_flag = 1;
	  HAL_Delay(2000);
	  degree_z = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_start
// 走行開始前に走行距離と機体角度を初期化
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_start(void){
	dist_l = dist_r = 0;		//走行距離の初期化
	if(H_accel_flag == 0) target_speed_l = target_speed_r = 0;		//モータ出力の初期化
	MF.FLAG.DRV = 1;
	MF.FLAG.SPD = 1;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_stop
// 走行を終了する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_stop(void){
	dist_l = dist_r = 0;		//a走行距離の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 0;
	MF.FLAG.SPD = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//control_start
// wallとgyroの姿勢制御を開始する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void control_start(){
	MF.FLAG.WCTRL = 1;										//wall制御を有効にする
	MF.FLAG.GCTRL = 1;										//gyro制御を有効にする
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//control_stop
// wallとgyroの姿勢制御を停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void control_stop(){
	MF.FLAG.WCTRL = 0;										//wall制御を無効にする
	MF.FLAG.GCTRL = 0;										//gyro制御を無効にする
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_dir
// wheel turn dir for each wheel
// 引数:1車輪選択(0=>L, 1=>R), 2回転方向選択(0=>CW, 1=>CWW, 2=>ShortBrake, 3=>free)
// 戻り値: なし
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
// 指定距離、指定加速度で加速走行する
// 引数1：accel_p 加速度, 引数2：speed_min_p 最低速度, 引数3：speed_max_p 最高速度, 引数4：dist 走行距離
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveA(uint16_t accel_p, uint16_t speed_min_p, uint16_t speed_max_p, uint16_t dist){

	speed_min_l = speed_min_r = speed_min_p;
	speed_max_l = speed_max_r = speed_max_p;
	accel_l = accel_r = accel_p;							//引数の各パラメータをグローバル変数化
	if(H_accel_flag == 1)target_speed_l = target_speed_r = speed_min_p;

	drive_start();											//走行開始

	//----走行----
	while((dist_l < dist) || (dist_r < dist));				//左右のモータが指定距離以上進むまで待機

//	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveD
// 指定距離、指定減速度で減速走行する
// 引数1：accel_p 加速度, 引数2：speed_min_p 最低速度, 引数3：speed_max_p 最高速度, 引数4：dist 走行距離
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveD(int16_t accel_p, uint16_t speed_min_p, uint16_t speed_max_p, uint16_t dist){

	float speed_0 = (speed_l + speed_r) / 2;								//等速走行距離を計算するためにmain.cより参照
	speed_min_l = speed_min_r = speed_min_p;
	speed_max_l = speed_max_r = speed_max_p;
	accel_l = accel_r = accel_p;							//引数の各パラメータをグローバル変数化

	int16_t c_dist = dist - (speed_min_l*speed_min_l  - speed_0*speed_0)/(2*accel_l);			//等速走行距離 = 総距離 - 減速に必要な距離

	accel_l = accel_r = 0;
	dist_l = dist_r = 0;
	if(c_dist > 0){
		//----等速走行----
		while((dist_l < c_dist) || (dist_r < c_dist));	//a左右のモータが等速分の距離以上進むまで待機
	}
	accel_l = accel_r = accel_p;
	//----減速走行----
	while((dist_l < dist) || (dist_r < dist));			//a左右のモータが減速分の距離以上進むまで待機

	if(H_accel_flag != 1)drive_stop();											//走行停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveU
// 指定距離分等速走行して停止する
// 引数1：dist …… 走行距離
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveU(uint16_t dist){

	accel_l = accel_r = 0;									//等速走行のため加速度は0
	dist_l = dist_r = 0;

	//----走行----
	while((dist_l < dist) || (dist_r < dist)){				//左右のモータが指定パルス以上進むまで待機
		if(MF.FLAG.WEDGE == 1){
			if(ad_l < WALL_BASE_L-30 || ad_r < WALL_BASE_R-10){
				while((dist_l < W_DIST) || (dist_r < W_DIST));	//左右のモータが壁切れ用指定距離以上進むまで待機
			break;
			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveC
// 指定距離分デフォルト速度で走行して停止する
// 引数1：dist …… 走行距離
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC(uint16_t dist){

	speed_min_l = speed_min_r = 150;
	speed_max_l = speed_max_r = 150;
	accel_l = accel_r = 0;												//等速走行のため加速度は0

	drive_start();											//走行開始
	//====回転====
	while((dist_l < dist) || (dist_r < dist));			//左右のモータが定速分の距離以上進むまで待機

	drive_stop();											//走行停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveC2
//a指定距離分デフォルト逆回転速度で走行して停止する
//a引数1：dist …… 走行距離
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC2(uint16_t dist){

	speed_min_l = speed_min_r = -250;
	speed_max_l = speed_max_r = -250;
	accel_l = accel_r = 0;									//a等速走行のため加速度は0

	drive_start();											//a走行開始
	//====a回転====
	while((dist_l > (-1*dist)) || (dist_r > (-1*dist)));	//a左右のモータが定速分の逆走距離以上進むまで待機

	drive_stop();											//a走行停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//set_position
// 機体の尻を壁に当てて場所を区画中央に合わせる
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_position(){

  driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
  degree_z = target_degree_z;
  start_flag = 0;
  start_sectionA();
  //driveC(SETPOS_SET);           //aデフォルト速度で区画中心になる分回転。回転後に停止する
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//start_sectionA
// スタート区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void start_sectionA(void){

	control_start();
	if(start_flag == 0){
		driveA(4000, SPEED_MIN, SPEED_RUN, SEC_START);					//aスタート区画分加速しながら走行。走行後は停止しない
	}else if(start_flag == 1){
		driveA(4000, SPEED_MIN, SPEED_RUN, SEC_HALF);					//a半区画分加速しながら走行。走行後は停止しない
	}else if(start_flag == 2){
		driveA(4000, SPEED_MIN, SPEED_RUN, SEC_START_HALF);				//aスタート半区画分加速しながら走行。走行後は停止しない
	}
	start_flag = 1;
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//start_sectionA2
// スタート区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void start_sectionA2(void){

	control_start();
	if(start_flag == 0){
		driveA(8000, SPEED_MIN, SPEED_HIGH, SEC_START);					//aスタート区画分加速しながら走行。走行後は停止しない
	}else if(start_flag == 1){
		driveA(8000, SPEED_MIN, SPEED_HIGH, SEC_HALF);					//a半区画分加速しながら走行。走行後は停止しない
	}else if(start_flag == 2){
		driveA(8000, SPEED_MIN, SPEED_HIGH, SEC_START_HALF);			//aスタート半区画分加速しながら走行。走行後は停止しない
	}
	start_flag = 1;
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionA
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA(void){

	control_start();
	driveA(4000, SPEED_MIN, SPEED_RUN, SEC_HALF);						//半区画分加速しながら走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionD
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD(void){

	control_start();
	driveD(-4000, SPEED_MIN, SPEED_RUN, SEC_HALF);						//半区画分指定減速度で減速走行。走行後は停止する
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionA2
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA2(void){
	control_start();
	driveA(8000, SPEED_MIN, SPEED_HIGH, SEC_HALF);						//半区画分加速しながら走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionD2
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD2(void){
	control_start();
	driveD(-8000, SPEED_MIN, SPEED_HIGH, SEC_HALF);						//半区画分指定減速度で減速走行。走行後は停止する
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionA3
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA3(void){
	control_start();
	driveA(10000, SPEED_MIN, SPEED_HIGH_HIGH, SEC_HALF);				//半区画分加速しながら走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionD3
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD3(void){
	control_start();
	driveD(-10000, SPEED_MIN, SPEED_HIGH_HIGH, SEC_HALF);						//半区画分指定減速度で減速走行。走行後は停止する
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionU
// 等速で半区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionU(void){
	full_led_write(7);
	control_start();
	driveU(SEC_HALF);													//半区画分等速走行。走行後は停止しない
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionV
// 等速で斜め半区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionV(void){
	full_led_write(7);
	control_start();
	driveU(SEC_HALF_V);													//半区画分等速走行。走行後は停止しない
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_section
// 1区画分進んで停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_section(void){

	half_sectionA();													//半区画分加速走行
	half_sectionD();													//半区画分減速走行のち停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionA
// 1区画分加速する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionA(void){
	full_led_write(4);
	control_start();
	driveA(accel_hs, SPEED_RUN, speed_max_hs, SEC_HALF*2);				//1区画分加速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionD
// 1区画分減速する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionD(void){
	full_led_write(3);
	control_start();
	driveD(-1*accel_hs, SPEED_RUN, speed_max_hs, SEC_HALF*2);			//1区画分減速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionA2
// 1区画分加速する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionA2(void){
	full_led_write(4);
	control_start();
	driveA(accel_hs, SPEED_HIGH, speed_max_hs, SEC_HALF*2);				//1区画分加速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionD2
// 1区画分減速する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionD2(void){
	full_led_write(3);
	control_start();
	driveD(-1*accel_hs, SPEED_HIGH, speed_max_hs, SEC_HALF*2);			//1区画分減速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionU
// 等速で1区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionU(void){
	full_led_write(7);
	control_start();
	driveU(SEC_HALF*2);													//1区画分等速走行。走行後は停止しない
	if(MF.FLAG.SCND == 0)get_wall_info();								//壁情報を取得
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_R90
// 右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_R90(void){
	target_omega_z = 800;
	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = target_omega_z/180*M_PI * TREAD/2;
	speed_min_r = -1*target_omega_z/180*M_PI * TREAD/2;

	drive_start();											//走行開始
	control_stop();
	while(degree_z > target_degree_z-80);

	accel_l = -30000;
	accel_r = 30000;
	speed_min_l = 100;
	speed_max_r = -100;

	while(degree_z > target_degree_z-90);

	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度左90度
	}else{
		turn_dir(DIR_TURN_R90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度左90度
	}
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_L90
// 左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_L90(void){
	target_omega_z = 800;
	accel_l = -3000;
	accel_r = 3000;
	speed_min_l = -1*target_omega_z/180*M_PI * TREAD/2;
	speed_max_r = target_omega_z/180*M_PI * TREAD/2;

	drive_start();											//走行開始
	control_stop();
	while(degree_z < target_degree_z+80);

	accel_l = 30000;
	accel_r = -30000;
	speed_max_l = -100;
	speed_min_r = 100;

	while(degree_z < target_degree_z+90);

	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも右回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_L90_8, 3);									//マイクロマウス内部位置情報でも右回転処理&目標角度右90度
	}
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_180
// 180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_180(void){

	full_led_write(2);
	target_omega_z = 800;
	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = target_omega_z/180*M_PI * TREAD/2;
	speed_min_r = -1*target_omega_z/180*M_PI * TREAD/2;

	drive_start();											//走行開始
	control_stop();
	while(degree_z > target_degree_z-170);

	accel_l = -30000;
	accel_r = 30000;
	speed_min_l = 100;
	speed_max_r = -100;

	while(degree_z > target_degree_z-180);

	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_180, 1);									//マイクロマウス内部位置情報でも180度回転処理&目標角度左180度
	}else{
		turn_dir(DIR_TURN_R180_8, 3);									//マイクロマウス内部位置情報でも180度回転処理&目標角度左180度
	}
	drive_stop();

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_R90
// スラロームで左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_R90(void){
	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = -10000;
	accel_r = -10000;
	speed_min_l = SPEED_RUN;
	speed_min_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < SLALOM_OFFSET && dist_r < SLALOM_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = SLALOM_DEGACCEL;
	target_omega_z = 0;
	omega_max = SLALOM_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-38);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-65);

	target_degaccel_z = -SLALOM_DEGACCEL;

	while(degree_z > target_degree_z-90);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度左90度
	}else{
		turn_dir(DIR_TURN_R90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度左90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;		//走行距離の初期化
	MF.FLAG.SPD = 1;

	control_start();
	while(dist_l < SLALOM_OFFSET && dist_r < SLALOM_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_L90
// スラロームで右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_L90(void){
	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = -10000;
	accel_r = -10000;
	speed_min_l = SPEED_RUN;
	speed_min_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < SLALOM_OFFSET && dist_r < SLALOM_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -SLALOM_DEGACCEL;
	target_omega_z = 0;
	omega_min = -SLALOM_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+38);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+65);

	target_degaccel_z = SLALOM_DEGACCEL;

	while(degree_z < target_degree_z+90);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_L90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;		//走行距離の初期化
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < SLALOM_OFFSET && dist_r < SLALOM_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_R902
// スラロームで左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_R902(void){
	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < SLALOM_H_OFFSET-34 && dist_r < SLALOM_H_OFFSET-34);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = SLALOM_H_DEGACCEL;
	target_omega_z = 0;
	omega_max = SLALOM_H_OMEGA;
	speed_G = SPEED_HIGH;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-32);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-66);

	target_degaccel_z = -SLALOM_H_DEGACCEL;

	while(degree_z > target_degree_z-80);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度左90度
	}else{
		turn_dir(DIR_TURN_R90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度左90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < SLALOM_H_OFFSET && dist_r < SLALOM_H_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_L90
// スラロームで右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_L902(void){
	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < SLALOM_H_OFFSET-34 && dist_r < SLALOM_H_OFFSET-34);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -SLALOM_H_DEGACCEL;
	target_omega_z = 0;
	omega_min = -SLALOM_H_OMEGA;
	speed_G = SPEED_HIGH;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+32);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+66.3);

	target_degaccel_z = SLALOM_H_DEGACCEL;

	while(degree_z < target_degree_z+80);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_L90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < SLALOM_H_OFFSET && dist_r < SLALOM_H_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_R90
// スラロームで右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_R90(void){
	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LSLALOM_OFFSET && dist_r < LSLALOM_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = LSLALOM_DEGACCEL;
	target_omega_z = 0;
	omega_max = LSLALOM_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-25);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-75);

	target_degaccel_z = -LSLALOM_DEGACCEL;

	while(degree_z > target_degree_z-90);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_R90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LSLALOM_OFFSET && dist_r < LSLALOM_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_L90
// 大回りスラロームで右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_L90(void){
	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LSLALOM_OFFSET && dist_r < LSLALOM_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -LSLALOM_DEGACCEL;
	target_omega_z = 0;
	omega_min = -LSLALOM_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+15);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+85);

	target_degaccel_z = LSLALOM_DEGACCEL;

	while(degree_z < target_degree_z+90);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_L90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LSLALOM_OFFSET && dist_r < LSLALOM_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_R902
// スラロームで右に90度回転する High Speed
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_R902(void){
	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LSLALOM_H_OFFSET && dist_r < LSLALOM_H_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = LSLALOM_H_DEGACCEL;
	target_omega_z = 0;
	omega_max = LSLALOM_H_OMEGA;
	speed_G = SPEED_HIGH;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-45);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-55);

	target_degaccel_z = -LSLALOM_H_DEGACCEL;

	while(degree_z > target_degree_z-90);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_R90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LSLALOM_H_OFFSET && dist_r < LSLALOM_H_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_L902
// 大回りスラロームで右に90度回転する High Speed
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_L902(void){
	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LSLALOM_H_OFFSET && dist_r < LSLALOM_H_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -LSLALOM_H_DEGACCEL;
	target_omega_z = 0;
	omega_min = -LSLALOM_H_OMEGA;
	speed_G = SPEED_HIGH;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+45);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+55);

	target_degaccel_z = LSLALOM_H_DEGACCEL;

	while(degree_z < target_degree_z+90);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_L90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LSLALOM_H_OFFSET && dist_r < LSLALOM_H_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_R903
// スラロームで右に90度回転する High High Speed
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_R903(void){
	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH_HIGH;
	speed_max_r = SPEED_HIGH_HIGH;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LSLALOM_H_H_OFFSET && dist_r < LSLALOM_H_H_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = LSLALOM_H_H_DEGACCEL;
	target_omega_z = 0;
	omega_max = LSLALOM_H_H_OMEGA;
	speed_G = SPEED_HIGH_HIGH;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-40);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-50);

	target_degaccel_z = -LSLALOM_H_H_DEGACCEL;

	while(degree_z > target_degree_z-80);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_R90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH_HIGH;
	speed_max_r = SPEED_HIGH_HIGH;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LSLALOM_H_H_OFFSET && dist_r < LSLALOM_H_H_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_L903
// 大回りスラロームで右に90度回転する High High Speed
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_L903(void){
	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH_HIGH;
	speed_max_r = SPEED_HIGH_HIGH;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LSLALOM_H_H_OFFSET && dist_r < LSLALOM_H_H_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -LSLALOM_H_H_DEGACCEL;
	target_omega_z = 0;
	omega_min = -LSLALOM_H_H_OMEGA;
	speed_G = SPEED_HIGH_HIGH;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+40);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+50);

	target_degaccel_z = LSLALOM_H_H_DEGACCEL;

	while(degree_z < target_degree_z+80);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_L90_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH_HIGH;
	speed_max_r = SPEED_HIGH_HIGH;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LSLALOM_H_H_OFFSET && dist_r < LSLALOM_H_H_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_R180
// スラロームで右に180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_R180(void){
	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LROTATE_OFFSET && dist_r < LROTATE_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = LROTATE_DEGACCEL;
	target_omega_z = 0;
	omega_max = LROTATE_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-40);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-160);

	target_degaccel_z = -LROTATE_DEGACCEL;

	while(degree_z > target_degree_z-180);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_R180_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LROTATE_OFFSET && dist_r < LROTATE_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_L180
// 大回りスラロームで右に180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_L180(void){
	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LROTATE_OFFSET && dist_r < LROTATE_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -LROTATE_DEGACCEL;
	target_omega_z = 0;
	omega_min = -LROTATE_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+40);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+160);

	target_degaccel_z = LROTATE_DEGACCEL;

	while(degree_z < target_degree_z+180);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_L180_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LROTATE_OFFSET && dist_r < LROTATE_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_R1802
// スラロームで右に180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_R1802(void){
	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LROTATE_H_OFFSET && dist_r < LROTATE_H_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = LROTATE_H_DEGACCEL;
	target_omega_z = 0;
	omega_max = LROTATE_H_OMEGA;
	speed_G = SPEED_HIGH;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-70);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-130);

	target_degaccel_z = -LROTATE_H_DEGACCEL;

	while(degree_z > target_degree_z-180);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
		turn_dir(DIR_TURN_R90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_R180_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LROTATE_H_OFFSET && dist_r < LROTATE_H_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_L1802
// 大回りスラロームで右に180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_L1802(void){
	full_led_write(2);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < LROTATE_H_OFFSET && dist_r < LROTATE_H_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -LROTATE_H_DEGACCEL;
	target_omega_z = 0;
	omega_min = -LROTATE_H_OMEGA;
	speed_G = SPEED_HIGH;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+70);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+130);

	target_degaccel_z = LROTATE_H_DEGACCEL;

	while(degree_z < target_degree_z+180);
	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
		turn_dir(DIR_TURN_L90, 1);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_L180_8, 3);									//マイクロマウス内部位置情報でも左回転処理&目標角度右90度
	}

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < LROTATE_H_OFFSET && dist_r < LROTATE_H_OFFSET);
	if(MF.FLAG.SCND == 0)get_wall_info();					//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R45
// 区画中心から右に45度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R45(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < V_OFFSET && dist_r < V_OFFSET);
	}else{
		while(dist_l < V_OFFSET+35 && dist_r < V_OFFSET+35);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = V_DEGACCEL;
	target_omega_z = 0;
	omega_max = V_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-20);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-40);

	target_degaccel_z = -V_DEGACCEL;

	while(degree_z > target_degree_z-45);
	turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < V_OFFSET && dist_r < V_OFFSET);
	}else{
		while(dist_l < V_OFFSET+35 && dist_r < V_OFFSET+35);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L45
// 区画中心から左に45度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L45(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < V_OFFSET && dist_r < V_OFFSET);
	}else{
		while(dist_l < V_OFFSET+43 && dist_r < V_OFFSET+43);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -V_DEGACCEL;
	target_omega_z = 0;
	omega_min = -V_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+20);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+40);

	target_degaccel_z = V_DEGACCEL;

	while(degree_z > target_degree_z+45);
	turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < V_OFFSET && dist_r < V_OFFSET);
	}else{
		while(dist_l < V_OFFSET+43 && dist_r < V_OFFSET+43);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R45D
//a区画中心から右に45度回転する　ゴール用
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R45D(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < V_OFFSET && dist_r < V_OFFSET);
	}else{
		while(dist_l < V_OFFSET+35 && dist_r < V_OFFSET+35);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = V_DEGACCEL;
	target_omega_z = 0;
	omega_max = V_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-20);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-40);

	target_degaccel_z = -V_DEGACCEL;

	while(degree_z > target_degree_z-45);
	turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < V_OFFSET-10 && dist_r < V_OFFSET-10);
	}else{
		while(dist_l < V_OFFSET+25 && dist_r < V_OFFSET+25);
	}

	control_start();
	driveD(-10000, SPEED_MIN, SPEED_RUN, 10);

/*	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < 75 && dist_r < 75);
*/}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L45D
//a区画中心から左に45度回転する　ゴール用
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L45D(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < V_OFFSET && dist_r < V_OFFSET);
	}else{
		while(dist_l < V_OFFSET+43 && dist_r < V_OFFSET+43);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -V_DEGACCEL;
	target_omega_z = 0;
	omega_min = -V_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+20);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+40);

	target_degaccel_z = V_DEGACCEL;

	while(degree_z > target_degree_z+45);
	turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < V_OFFSET-10 && dist_r < V_OFFSET-10);
	}else{
		while(dist_l < V_OFFSET+33 && dist_r < V_OFFSET+33);
	}

	control_start();
	driveD(-10000, SPEED_MIN, SPEED_RUN, 10);

/*	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < 75 && dist_r < 75);
*/}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R452
//a区画中心から右に45度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R452(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < V_H_OFFSET && dist_r < V_H_OFFSET);
	}else{
		while(dist_l < V_H_OFFSET+37 && dist_r < V_H_OFFSET+37);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = V_H_DEGACCEL;
	target_omega_z = 0;
	omega_max = V_H_OMEGA;
	speed_G = SPEED_HIGH;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-15);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-40);

	target_degaccel_z = -V_H_DEGACCEL;

	while(degree_z > target_degree_z-45);
	turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_HIGH;
	speed_max_r = SPEED_HIGH;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < V_H_OFFSET && dist_r < V_H_OFFSET);
	}else{
		while(dist_l < V_H_OFFSET+37 && dist_r < V_H_OFFSET+37);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L452
// 区画中心から左に45度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L452(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < V_H_OFFSET && dist_r < V_H_OFFSET);
	}else{
		while(dist_l < V_H_OFFSET+37 && dist_r < V_H_OFFSET+37);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -V_H_DEGACCEL;
	target_omega_z = 0;
	omega_min = -V_H_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+20);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+40);

	target_degaccel_z = V_H_DEGACCEL;

	while(degree_z > target_degree_z+45);
	turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < V_H_OFFSET && dist_r < V_H_OFFSET);
	}else{
		while(dist_l < V_H_OFFSET+37 && dist_r < V_H_OFFSET+37);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R45D2
//a区画中心から右に45度回転する　ゴール用
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R45D2(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < V_OFFSET && dist_r < V_OFFSET);
	}else{
		while(dist_l < V_OFFSET+37 && dist_r < V_OFFSET+37);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = V_DEGACCEL;
	target_omega_z = 0;
	omega_max = V_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-20);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-40);

	target_degaccel_z = -V_DEGACCEL;

	while(degree_z > target_degree_z-45);
	turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < V_OFFSET-10 && dist_r < V_OFFSET-10);
	}else{
		while(dist_l < V_OFFSET+27 && dist_r < V_OFFSET+27);
	}

	control_start();
	driveD(-10000, SPEED_MIN, SPEED_RUN, 10);

/*	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < 75 && dist_r < 75);
*/}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L45D2
//a区画中心から左に45度回転する　ゴール用
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L45D2(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < V_OFFSET && dist_r < V_OFFSET);
	}else{
		while(dist_l < V_OFFSET+35 && dist_r < V_OFFSET+35);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -V_DEGACCEL;
	target_omega_z = 0;
	omega_min = -V_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+20);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+40);

	target_degaccel_z = V_DEGACCEL;

	while(degree_z > target_degree_z+45);
	turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < V_OFFSET-10 && dist_r < V_OFFSET-10);
	}else{
		while(dist_l < V_OFFSET+25 && dist_r < V_OFFSET+25);
	}

	control_start();
	driveD(-10000, SPEED_MIN, SPEED_RUN, 10);

/*	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < 75 && dist_r < 75);
*/}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R90
// 柱中心から右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R90(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < VV_OFFSET && dist_r < VV_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = VV_DEGACCEL;
	target_omega_z = 0;
	omega_max = VV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-25);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-75);

	target_degaccel_z = -VV_DEGACCEL;

	while(degree_z > target_degree_z-90);
	turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < VV_OFFSET && dist_r < VV_OFFSET);
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L90
// 柱中心から左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L90(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < VV_OFFSET && dist_r < VV_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -VV_DEGACCEL;
	target_omega_z = 0;
	omega_min = -VV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+25);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+75);

	target_degaccel_z = VV_DEGACCEL;

	while(degree_z < target_degree_z+90);
	turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < VV_OFFSET && dist_r < VV_OFFSET);
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R902
// 柱中心から右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R902(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < VV_H_OFFSET && dist_r < VV_H_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = VV_DEGACCEL;
	target_omega_z = 0;
	omega_max = VV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-25);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-75);

	target_degaccel_z = -VV_DEGACCEL;

	while(degree_z > target_degree_z-90);
	turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < VV_H_OFFSET && dist_r < VV_H_OFFSET+10);
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L902
// 柱中心から左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L902(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	while(dist_l < VV_H_OFFSET && dist_r < VV_H_OFFSET);
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -VV_DEGACCEL;
	target_omega_z = 0;
	omega_min = -VV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+25);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+75);

	target_degaccel_z = VV_DEGACCEL;

	while(degree_z < target_degree_z+90);
	turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < VV_H_OFFSET && dist_r < VV_H_OFFSET);
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R135
// 区画中心から右に135度回転する　ゴール用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R135(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+30 && dist_r < VVV_OFFSET+30);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = VVV_DEGACCEL;
	target_omega_z = 0;
	omega_max = VVV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-25);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-120);

	target_degaccel_z = -VVV_DEGACCEL;

	while(degree_z > target_degree_z-135);
	turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+30 && dist_r < VVV_OFFSET+30);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L135
// 区画中心から左に135度回転する　ゴール用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L135(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+25 && dist_r < VVV_OFFSET+25);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -VVV_DEGACCEL;
	target_omega_z = 0;
	omega_min = -VVV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+25);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+120);

	target_degaccel_z = VVV_DEGACCEL;

	while(degree_z < target_degree_z+135);
	turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+25 && dist_r < VVV_OFFSET+25);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R135D
// 区画中心から右に135度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R135D(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+30 && dist_r < VVV_OFFSET+30);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = VVV_DEGACCEL;
	target_omega_z = 0;
	omega_max = VVV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-25);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-120);

	target_degaccel_z = -VVV_DEGACCEL;

	while(degree_z > target_degree_z-135);
	turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+20 && dist_r < VVV_OFFSET+20);
	}else{
		while(dist_l < VVV_OFFSET-10 && dist_r < VVV_OFFSET-10);
	}

	control_start();
	driveD(-10000, SPEED_MIN, SPEED_RUN, 10);

/*	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < 35 && dist_r < 35);
*/}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L135D
// 区画中心から左に135度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L135D(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+25 && dist_r < VVV_OFFSET+25);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -VVV_DEGACCEL;
	target_omega_z = 0;
	omega_min = -VVV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+25);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+120);

	target_degaccel_z = VVV_DEGACCEL;

	while(degree_z < target_degree_z+135);
	turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+15 && dist_r < VVV_OFFSET+15);
	}else{
		while(dist_l < VVV_OFFSET-10 && dist_r < VVV_OFFSET-10);
	}

	control_start();
	driveD(-10000, SPEED_MIN, SPEED_RUN, 10);

/*	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < 35 && dist_r < 35);
*/}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R1352
// 区画中心から右に135度回転する　ゴール用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R1352(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < VVV_H_OFFSET+15 && dist_r < VVV_H_OFFSET+15);
	}else{
		while(dist_l < VVV_H_OFFSET && dist_r < VVV_H_OFFSET);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = VVV_DEGACCEL;
	target_omega_z = 0;
	omega_max = VVV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-25);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-120);

	target_degaccel_z = -VVV_DEGACCEL;

	while(degree_z > target_degree_z-135);
	turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+15 && dist_r < VVV_OFFSET+15);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L1352
// 区画中心から左に135度回転する　ゴール用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L1352(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+15 && dist_r < VVV_OFFSET+15);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -VVV_DEGACCEL;
	target_omega_z = 0;
	omega_min = -VVV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+25);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+120);

	target_degaccel_z = VVV_DEGACCEL;

	while(degree_z < target_degree_z+135);
	turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+15 && dist_r < VVV_OFFSET+15);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R135D2
// 区画中心から右に135度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R135D2(void){

	full_led_write(5);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+15 && dist_r < VVV_OFFSET+15);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = VVV_DEGACCEL;
	target_omega_z = 0;
	omega_max = VVV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z > target_degree_z-25);

	target_degaccel_z = 0;

	while(degree_z > target_degree_z-120);

	target_degaccel_z = -VVV_DEGACCEL;

	while(degree_z > target_degree_z-135);
	turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+5 && dist_r < VVV_OFFSET+5);
	}else{
		while(dist_l < VVV_OFFSET-10 && dist_r < VVV_OFFSET-10);
	}

	control_start();
	driveD(-10000, SPEED_MIN, SPEED_RUN, 10);

/*	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < 35 && dist_r < 35);
*/}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L135D2
// 区画中心から左に135度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L135D2(void){

	full_led_write(6);
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;

	control_start();
	dist_l = dist_r = 0;
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+15 && dist_r < VVV_OFFSET+15);
	}else{
		while(dist_l < VVV_OFFSET && dist_r < VVV_OFFSET);
	}
	drive_stop();
	control_stop();

	MF.FLAG.GYRO = 1;

	target_degaccel_z = -VVV_DEGACCEL;
	target_omega_z = 0;
	omega_min = -VVV_OMEGA;
	speed_G = SPEED_RUN;

	MF.FLAG.DRV = 1;
	while(degree_z < target_degree_z+25);

	target_degaccel_z = 0;

	while(degree_z < target_degree_z+120);

	target_degaccel_z = VVV_DEGACCEL;

	while(degree_z < target_degree_z+135);
	turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

	v_flag = (v_flag + 1) % 2;
	MF.FLAG.GYRO = 0;

	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	if(v_flag == 0){
		while(dist_l < VVV_OFFSET+5 && dist_r < VVV_OFFSET+5);
	}else{
		while(dist_l < VVV_OFFSET-10 && dist_r < VVV_OFFSET-10);
	}

	control_start();
	driveD(-10000, SPEED_MIN, SPEED_RUN, 10);

/*	accel_l = 10000;
	accel_r = 10000;
	speed_max_l = SPEED_RUN;
	speed_max_r = SPEED_RUN;
	dist_l = dist_r = 0;
	MF.FLAG.SPD = 1;
	control_start();
	while(dist_l < 35 && dist_r < 35);
*/}


/*----------------------------------------------------------
		テスト関数
----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//test_select
// 走行系テスト選択
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
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


//+++++++++++++++++++++++++++++++++++++++++++++++
//init_test
// 初期基幹関数走行テスト
// 引数：なし
// 戻り値：なし
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
			  drive_ready();
			  get_base();

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
					get_base();
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
// スラローム走行テスト
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_test(void){

	int mode = 0;
	printf("Test Slalom Run, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode < 8){
				  full_led_write(7);
			  }else if(mode < 16){
				  full_led_write(4);
			  }else{
				  full_led_write(5);
			  }
			  if(mode > 23){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 8){
				  full_led_write(7);
			  }else if(mode < 16){
				  full_led_write(4);
			  }else{
				  full_led_write(5);
			  }
			  if(mode < 0){
				  mode = 23;
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  drive_ready();
			  get_base();

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
					half_sectionA();
					for(int i = 0; i < 1; i++){
						slalom_R90();	//一区画のパルス分デフォルトインターバルで走行
					}
					half_sectionD();
					break;
				case 4:
					//----slalom左折----
					printf("slalom turn left .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						slalom_L90();				//16回右90度回転、つまり4周回転
					}
					half_sectionD();
					break;
				case 5:
					//----slalom2右折 High Speed----
					printf("slalom turn right High Speed .\n");
					half_sectionA2();
					for(int i = 0; i < 16; i++){
						full_led_write(1);
						slalom_R902();				//16回右90度回転、つまり4周回転
						full_led_write(2);
						one_sectionU();
					}
					full_led_write(3);
					half_sectionD2();
					break;
				case 6:
					//----slalom2左折 High Speed----
					printf("slalom turn left High Speed .\n");
					half_sectionA2();
					for(int i = 0; i < 16; i++){
						full_led_write(1);
						slalom_L902();				//16回左90度回転、つまり4周回転
						full_led_write(2);
						one_sectionU();
					}
					full_led_write(3);
					half_sectionD2();
					break;
				case 7:
					//----Lslalom右折----
					printf("Lslalom turn right .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_R90();				//16回右90度回転、つまり4周回転
					}
					full_led_write(3);
					half_sectionD();
					break;
				case 8:
					//----Lslalom左折----
					printf("Lslalom turn left .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_L90();				//16回左90度回転、つまり4周回転
					}
					full_led_write(3);
					half_sectionD();
					break;
				case 9:
					//----Lslalom2右折 High Speed----
					printf("Lslalom turn right High Speed .\n");
					half_sectionA2();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_R902();				//16回右90度回転、つまり4周回転
					}
					full_led_write(3);
					half_sectionD2();
					break;
				case 10:
					//----Lslalom2左折 High Speed----
					printf("Lslalom turn left High Speed .\n");
					half_sectionA2();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_L902();				//16回左90度回転、つまり4周回転
					}
					full_led_write(3);
					half_sectionD2();
					break;
				case 11:
					//----Lslalom3右折 High High Speed----
					printf("Lslalom turn right High High Speed .\n");
					half_sectionA3();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_R903();				//16回右90度回転、つまり4周回転
					}
					full_led_write(3);
					half_sectionD3();
					break;
				case 12:
					//----Lslalom3左折 High High Speed----
					printf("Lslalom turn left High High Speed .\n");
					half_sectionA3();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_L903();				//16回左90度回転、つまり4周回転
					}
					full_led_write(3);
					half_sectionD3();
					break;
				case 13:
					//----Lslalom右180----
					printf("Lslalom turn right & right .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_R180();				//16回右180度回転、つまり4周回転
					}
					full_led_write(3);
					half_sectionD();
					break;
				case 14:
					//----Lslalom左180----
					printf("Lslalom turn left & left .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_L180();				//16回左180度回転、つまり4周回転
					}
					full_led_write(3);
					half_sectionD();
					break;
				case 15:
					//----Lslalom右180 High Speed----
					printf("Lslalom turn right & right High Speed .\n");
					half_sectionA2();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_R1802();				//16回右180度回転、つまり4周回転
					}
					full_led_write(3);
					half_sectionD2();
					break;
				case 16:
					//----Lslalom左180 High Speed----
					printf("Lslalom turn left & left High Speed .\n");
					half_sectionA2();
					for(int i = 0; i < 1; i++){
						full_led_write(2);
						Lslalom_L1802();				//16回左180度回転、つまり4周回転
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
// 斜め走行テスト
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_test(void){

	int mode = 0;
	printf("Test V Run, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 15){
				  mode = 0;
			  }
			  if(mode < 8){
				  full_led_write(7);
			  }else if(mode < 16){
				  full_led_write(4);
			  }else{
				  full_led_write(5);
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 15;
			  }
			  if(mode < 8){
				  full_led_write(7);
			  }else if(mode < 16){
				  full_led_write(4);
			  }else{
				  full_led_write(5);
			  }
			  printf("Mode : %d\n", mode);
			  //buzzer(pitagola2[mode-1][0], pitagola2[mode-1][1]);
			  //buzzer(pitagola[2][0], pitagola[2][1]);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
			  drive_ready();

			  MF.FLAG.XDIR = 1;
			  v_flag = 0;
			  get_base();

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
					}
					full_led_write(3);
					half_sectionD();
					break;
				case 2:
					//----V右45----
					printf("V 45 left .\n");
					full_led_write(7);
					half_sectionA();
					for(int i = 0; i < 1; i++){
						v_L45();
					}
					full_led_write(3);
					half_sectionD();
					break;
				case 3:
					//----V左90----
					printf("V 90 right .\n");
					full_led_write(7);
					half_sectionA();
					v_R45();
					for(int i = 0; i < 1; i++){
						v_R90();
					}
					v_R45();
					full_led_write(3);
					half_sectionD();
					break;
				case 4:
					//----V右90----
					printf("V 90 left .\n");
					full_led_write(7);
					half_sectionA();
					v_L45();
					for(int i = 0; i < 1; i++){
						v_L90();
					}
					v_L45();
					full_led_write(3);
					half_sectionD();
					break;
				case 5:
					//----V左135----
					printf("V 135 right .\n");
					full_led_write(7);
					half_sectionA();
					v_R45();
					for(int i = 0; i < 1; i++){
						v_R135();
					}
					v_R45();
					full_led_write(3);
					half_sectionD();
					break;
				case 6:
					//----V右135----
					printf("V 135 left .\n");
					full_led_write(7);
					half_sectionA();
					v_L45();
					for(int i = 0; i < 1; i++){
						v_L135();
					}
					v_L45();
					full_led_write(3);
					half_sectionD();
					break;
				case 7:
					//----V左45D----
					printf("V 45 right .\n");
					full_led_write(7);
					half_sectionA();
					for(int i = 0; i < 1; i++){
						v_R45D();
					}
					full_led_write(3);
					break;
				case 8:
					//----V右45D----
					printf("V 45 left .\n");
					full_led_write(7);
					half_sectionA();
					for(int i = 0; i < 1; i++){
						v_L45D();
					}
					full_led_write(3);
					break;
				case 9:
					//----V左135D----
					printf("V 135 right .\n");
					full_led_write(7);
					half_sectionA();
					v_R45();
					for(int i = 0; i < 1; i++){
						v_R135D();
					}
					full_led_write(3);
					break;
				case 10:
					//----V右135D----
					printf("V 135 left .\n");
					full_led_write(7);
					half_sectionA();
					v_L45();
					for(int i = 0; i < 1; i++){
						v_L135D();
					}
					full_led_write(3);
					break;
			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//pass_test
// pass圧縮走行テスト
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void pass_test(void){

	int mode = 0;
	printf("Test pass Run, Mode : %d\n", mode);

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
			  drive_ready();
			  v_flag = 0;

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
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 1:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース----
					printf("pass press 3.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 800;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 2:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF32();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF32();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 3:
					//----a直線と大回り圧縮と斜めｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 800;

					pass_mode = 4;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

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


/*----------------------------------------------------------
		走行モード選択関数
----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//simple_run
// 超信地走行モード
// 引数：なし
// 戻り値：なし
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
			  drive_ready();

			  switch(mode){

				case 0:
					break;
				case 1:
					//----一次探索走行----
					printf("First Run.\n");

					MF.FLAG.SCND = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchA();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchA();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 2:
					//----一次探索連続走行----
					printf("First Run. (Continuous)\n");

					MF.FLAG.SCND = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchB();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchB();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 3:
					//----二次探索走行----
					printf("Second Run. (Continuous)\n");

					MF.FLAG.SCND = 1;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchB();
					HAL_Delay(2000);

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
// スラローム走行モード
// 引数：なし
// 戻り値：なし
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
			  drive_ready();

			  switch(mode){

			  case 0:
					//----一次探索スラローム走行----
					printf("First Run. (Slalom)\n");

					MF.FLAG.SCND = 0;
					start_flag = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 1:
					//----二次探索スラローム走行----
					printf("Second Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					start_flag = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 2:
					//----二次探索スラローム走行+既知区間加速----
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
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 3:
					//----二次探索スラローム走行+既知区間加速----
					printf("Second Run. (Slalom+accel)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 1000;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 4:
					//----二次探索スラローム走行+既知区間加速----
					printf("Second Run. (Slalom+accel)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 1500;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 5:
					//----二次走行スラローム+直線優先----
					printf("High Speed Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					HAL_Delay(5000);
					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 6:
					//----二次走行スラロームHigh Speed+直線優先+既知区間加速----
					printf("High Speed Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.STRAIGHT = 1;
					MF.FLAG.ACCL2 = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					HAL_Delay(5000);
					searchD2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD2();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 7:
					//----二次走行スラロームHigh Speed+直線優先+既知区間加速----
					printf("High Speed Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.STRAIGHT = 1;
					MF.FLAG.ACCL2 = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 1600;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					HAL_Delay(5000);
					searchD2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD2();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//sample_course_run
// 試験走行モード
// 引数：なし
// 戻り値：なし
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
			  drive_ready();

			  switch(mode){
				case 0:
					get_base();
					break;

				case 1:
					//----サンプルコース1　超信地----
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
					//----サンプルコース1　超信地----
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
					//---サンプルコース2　スラローム----
					half_sectionA();
					slalom_R90();
					slalom_R90();
					half_sectionD();
					break;

				case 4:
					//----二次探索スラローム走行+既知区間加速----
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
					//----スラローム走行&全面探索スラローム走行----
					printf("Slalom Run & All Map Run.\n");

					MF.FLAG.SCND = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchE();

					searchC();
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 6:
					//----スラローム走行&全面探索スラローム走行----
					printf("Slalom Run & All Map Run.\n");

					MF.FLAG.SCND = 0;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

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
// 本番用走行モード
// 引数：なし
// 戻り値：なし
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
					start_flag = 0;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchB();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchB();

					goal_x = 7;
					goal_y = 7;

					break;

				case 2:
					//----a二次探索走行----
					printf("Second Run. (Continuous)\n");

					MF.FLAG.SCND = 1;
					start_flag = 0;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchB();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchB();

					goal_x = 7;
					goal_y = 7;

					break;

				case 3:
					//----a一次探索スラローム走行----
					printf("First Run. (Slalom)\n");

					MF.FLAG.SCND = 0;
					start_flag = 0;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = 7;
					goal_y = 7;

					break;

				case 4:
					//---a二次探索スラローム走行----
					printf("Second Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					start_flag = 0;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

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
/*			  		  for(int i=0; i<m_select; i++){
			  			  buzzer(mario_select[i][0], mario_select[i][1]);
			  		  }
*/					perfect_slalom();
					break;
			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//perfect_slalom
// 本番用スラローム走行モード
// 引数：なし
// 戻り値：なし
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
/*	  		  for(int i=0; i<m_select; i++){
	  			  buzzer(mario_select[i][0], mario_select[i][1]);
	  		  }
*/		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
/*	  		  for(int i=0; i<m_select; i++){
	  			  buzzer(mario_select[i][0], mario_select[i][1]);
	  		  }
*/		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
/*			  for(int i=0; i<m_ok; i++){
	  			  buzzer(mario_ok[i][0], mario_ok[i][1]);
	  		  }
*/
			  drive_ready();
/*	  		  for(int i=0; i<m_start; i++){
	  			  buzzer(mario_start[i][0], mario_start[i][1]);
	  			  full_led_write(1);
	  		  }
*/
			  switch(mode){
				case 0:
					break;

				case 1:
					//----a一次探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 0;
					start_flag = 0;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

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
					start_flag = 0;

					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = 7;
					goal_y = 7;
					break;

				case 3:
					//----a二次探索スラロームHigh Speed----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 0;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC2();

					goal_x = 7;
					goal_y = 7;
					break;

				case 4:
					//----a二次探索スラロームHigh Speed + 既知区間加速----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;

					accel_hs = 3000;
					speed_max_hs = 1200;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD2();

					goal_x = 7;
					goal_y = 7;
					break;

				case 5:
					//----a二次探索スラロームHigh Speed + 既知区間加速----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;

					accel_hs = 10000;
					speed_max_hs = 1600;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD2();

					goal_x = 7;
					goal_y = 7;
					break;

				case 6:
					//----a二次探索スラロームHigh Speed + 既知区間加速----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;

					accel_hs = 10000;
					speed_max_hs = 2000;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD2();

					goal_x = 7;
					goal_y = 7;
					break;

				case 7:
					perfect_pass();
					break;
			}
		}
	}
}
//+++++++++++++++++++++++++++++++++++++++++++++++
//perfect_pass
// a本番用pass圧縮走行モード
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void perfect_pass(void){

	int mode = 0;
	printf("Perfect Pass Press, Mode : %d\n", mode);

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 7){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
/*	  		  for(int i=0; i<m_select; i++){
	  			  buzzer(mario_select[i][0], mario_select[i][1]);
	  		  }
*/		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 7;
			  }
			  printf("Mode : %d\n", mode);
/*	  		  for(int i=0; i<m_select; i++){
	  			  buzzer(mario_select[i][0], mario_select[i][1]);
	  		  }
*/		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
			  HAL_Delay(50);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
/*			  for(int i=0; i<m_ok; i++){
	  			  buzzer(mario_ok[i][0], mario_ok[i][1]);
	  		  }
*/
			  drive_ready();
/*	  		  for(int i=0; i<m_start; i++){
	  			  buzzer(mario_start[i][0], mario_start[i][1]);
	  			  full_led_write(1);
	  		  }
*/
			  switch(mode){
				case 0:
					break;

				case 1:
					//----a一次探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 0;
					start_flag = 0;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = 7;
					goal_y = 7;
					break;

				case 2:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース----
					printf("pass press 3.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 800;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = 7;
					goal_y = 7;
					break;
				case 3:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchF32();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF32();

					goal_x = 7;
					goal_y = 7;
					break;
				case 4:
					//----a直線と大回り圧縮と斜めｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 800;

					pass_mode = 4;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					goal_x = 7;
					goal_y = 7;
					break;

				case 5:
					break;
				case 6:
					break;
				case 7:
					//----a一次探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 0;
					start_flag = 0;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					HAL_Delay(2000);



					//----a二次探索スラローム+既知区間加速走行 speed2----
					printf("Second Run. (Continuous)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;

					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = 7;
					goal_y = 7;

//					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					HAL_Delay(2000);



/*					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース----
					printf("pass press 3.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = 7;
					goal_y = 7;

//					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					HAL_Delay(2000);

*/					//----a二次探索スラロームHigh Speed + 既知区間加速----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;

					accel_hs = 10000;
					speed_max_hs = 1600;
					goal_x = 7;
					goal_y = 7;

//					get_base();

					searchD2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD2();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					HAL_Delay(2000);



					//----a二次探索スラロームHigh Speed + 既知区間加速 Speed2----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;

					accel_hs = 10000;
					speed_max_hs = 2000;
					goal_x = 7;
					goal_y = 7;

//					get_base();

					searchD2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD2();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					HAL_Delay(2000);



					//----a二次探索スラロームHigh Speed + 既知区間加速 Speed3----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;

					accel_hs = 20000;
					speed_max_hs = 2500;
					goal_x = 7;
					goal_y = 7;

//					get_base();

					searchD2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD2();


					for(int i=0; i<m_start; i++){
					  buzzer(mario_start[i][0], mario_start[i][1]);
					  full_led_write(1);
					}
					//----a直線と大回り圧縮と斜めｰｰｰｰ
/*					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_flag = 0;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 4;

					goal_x = 7;
					goal_y = 7;

//					get_base();

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					goal_x = 7;
					goal_y = 7;

*/					break;
			}
		}
	}
}
