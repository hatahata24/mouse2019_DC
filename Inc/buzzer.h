#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_


#define D2 2349.3/32
#define E2 2637/32
#define F2 2793.8/32
#define G2 3136/32
#define A2 3520/32
#define B2 3951/32
#define C3 4186/32		//DO
#define D3 2349.3/16
#define E3 2637/16
#define F3 2793.8/16
#define G3 3136/16
#define A3 3520/16
#define B3 3951/16
#define C4 4186/16		//DO
#define D4 2349.3/8
#define E4 2637/8
#define F4 2793.8/8
#define G4 3136/8
#define A4 3520/8
#define B4 3951/8
#define C5 4186/8		//DO
#define D5 2349.3/4
#define E5 2637/4
#define F5 2793.8/4
#define G5 3136/4
#define A5 3520/4
#define B5 3951/4
#define C6 4186/4		//DO
#define D6 2349.3/2
#define E6 2637/2
#define F6 2793.8/2
#define G6 3136/2
#define A6 3520/2
#define B6 3951/2
#define C7 4186/2		//DO
#define D7 2349.3
#define E7 2637
#define F7 2793.8
#define G7 3136
#define A7 3520
#define B7 3951
#define C8 4186		//DO
#define RST 5

#define pita 10
#define pita2 7
#define m_start 7
#define m_coin 2
#define m_select 2
#define m_ok 7
#define m_goal 15

#ifdef MAIN_C_              //main.cからこのファイルが呼ばれている場合

	  volatile int hz;                  //
	  volatile int pitagola[pita][2] = {{D6, 200},
			  {E6, 200},
			  {RST, 50},
			  {D6, 200},
			  {E6, 200},
			  {RST, 50},
			  {C7, 200},
			  {B6, 200},
			  {RST, 200},
			  {G6, 250}};
	  volatile int pitagola2[pita][2] = {{D6, 200},
			  {E6, 200},
			  {D6, 200},
			  {E6, 200},
			  {C7, 200},
			  {B6, 200},
			  {G6, 250}};
	  volatile int mario_start[m_start][2] = {{A5, 500},
			  {RST, 500},
			  {A5, 500},
			  {RST, 500},
			  {A5, 500},
			  {RST, 500},
			  {A6, 1000}};
	  volatile int mario_coin[m_coin][2] = {{A6, 80},
			  {C7, 250}};
	  volatile int mario_select[m_select][2] = {{F5, 80},
			  {F6, 250}};
	  volatile int mario_ok[m_ok][2] = {{D6, 120},
			  {E6, 120},
			  {F6, 120},
			  {G6, 120},
			  {A6, 120},
			  {B6, 120},
			  {C7, 320}};
	  volatile int mario_goal[m_goal][2] = {{E6, 250},
			  {RST, 120},
			  {E6, 150},
			  {C6, 400},
			  {D6, 250},
			  {RST, 120},
			  {D6, 150},
			  {B5, 400},
	  	  	  {G6, 800},
			  {A6, 150},
			  {RST, 200},
			  {G6, 150},
			  {A6, 150},
			  {RST, 150},
			  {B6, 500}};


#else                       //main.c以外からこのファイルが呼ばれている場合
  /*グローバル変数の宣言*/
	  extern volatile int hz;
	  extern volatile int pitagola[pita][2];
	  extern volatile int pitagola2[pita2][2];
	  extern volatile int mario_start[m_start][2];
	  extern volatile int mario_coin[m_coin][2];
	  extern volatile int mario_select[m_select][2];
	  extern volatile int mario_ok[m_ok][2];
	  extern volatile int mario_goal[m_goal][2];
#endif


#endif /* INC_BUZZER_H_ */
