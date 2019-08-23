
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
	MF.FLAG.DRV = 1;
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
	MF.FLAG.DRV = 0;
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
	accel_l = accel_r = accel_p;										//引数の各パラメータをグローバル変数化

	//if(MF.FLAG.STRT == 0) speed_l = speed_r = 100;						//最初の加速の際だけspeedを定義
	drive_start();											//走行開始

	//----a走行----
	while((dist_l < dist) || (dist_r < dist));			//左右のモータが指定パルス以上進むまで待機

	drive_stop();											//a走行停止
	//MF.FLAG.STRT = 1;										//2回目以降の加速の際はspeedは既存のスピードを用いる
	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
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
	while((dist_l < dist) || (dist_r < dist));			//左右のモータが指定パルス以上進むまで待機

	drive_stop();											//走行停止
	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveC
// 指定パルス分デフォルト速度で走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC(uint16_t dist){

	speed_min_l = speed_min_r = 150;
	speed_max_l = speed_max_r = 300;
	accel_l = accel_r = 0;												//等速走行のため加速度は0

	drive_start();											//走行開始

	//====回転====
	while((dist_l < dist) || (dist_r < dist));			//左右のモータが定速分のパルス以上進むまで待機

	drive_stop();											//走行停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionA
// a半区画分加速しながら走行する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA(void){

	MF.FLAG.WCTRL = 1;										//wall制御を有効にする
	MF.FLAG.GCTRL = 1;										//gyro制御を有効にする

	target_omega_z = 0;
	driveA(1000, 10, 400, SEC_HALF);					//半区画のパルス分加速しながら走行。走行後は停止しない
	get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionD
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD(void){

	MF.FLAG.WCTRL = 1;										//wall制御を有効にする
	MF.FLAG.GCTRL = 1;										//gyro制御を有効にする

	target_omega_z = 0;
	driveD(-1000, 10, 400, SEC_HALF);				//指定パルス分指定減速度で減速走行。走行後は停止する
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
//one_sectionU
//a等速で1区画分進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionU(void){

	MF.FLAG.WCTRL = 1;										//wall制御を有効にする
	MF.FLAG.GCTRL = 1;										//gyro制御を有効にする

	driveU(SEC_HALF*2);										//半区画のパルス分等速走行。走行後は停止しない
	get_wall_info();										//壁情報を取得
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
	while(degree_z > -80);
	drive_stop();

	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = 100;
	speed_min_r = -100;

	drive_start();											//走行開始
	while(degree_z > -90+80);

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
	while(degree_z < 80);
	drive_stop();

	accel_l = -3000;
	accel_r = 3000;
	speed_min_l = -100;
	speed_max_r = 100;

	drive_start();											//走行開始
	while(degree_z < 90-80);

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
	while(degree_z > -170);
	drive_stop();

	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = 100;
	speed_min_r = -100;

	drive_start();											//走行開始
	while(degree_z > -180+170);

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
					//----a尻当て----
					printf("Set Position.\n");
					//set_position(0);
					break;
				case 1:
					//----4区画等速走行----
					printf("4 Section, Forward, Constant Speed.\n");
					for(int i = 0; i < 4; i++){
						driveC(SEC_HALF*2);	//一区画のパルス分デフォルトインターバルで走行
					}
					break;
				case 2:
					//----right90度回転----
					printf("Rotate R90.\n");
					for(int i = 0; i < 16; i++){
						rotate_R90();				//16回右90度回転、つまり4周回転
					}
					break;
				case 3:
					//----left90度回転----
					printf("Rotate L90.\n");
					for(int i = 0; i < 16; i++){
						rotate_L90();				//16回左90度回転、つまり4周回転
					}
					break;
				case 4:
					//----180度回転----
					printf("Rotate 180.\n");
					for(int i = 0; i < 8; i++){
						rotate_180();				//8回右180度回転、つまり4周回転
					}
					break;
				case 5:
					//----4区画連続走行----
					printf("4 Section, Forward, Continuous.\n");
					half_sectionA();				//半区画のパルス分加速しながら走行
					for(int i = 0; i < 4-1; i++){
						one_sectionU();			//一区画のパルス分等速走行
					}
					half_sectionD();				//半区画のパルス分減速しながら走行。走行後は停止する
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
					//init test mode
					init_test();
					break;
			  }
		  }
	}
}
