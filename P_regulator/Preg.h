#ifndef Preg_h
#define Preg_h
//*************************************
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>


#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
/*
#define TOK_RABOTY    1000
#define TOK_POKOJA    400
#define TOK_ZAPUSKA   1700
*/
#define TOK_RABOTY    		540  //509-573
#define TOK_RABOTY_MAX    770  //639
#define TOK_POKOJA    		500  //372
#define TOK_ZAPUSKA   		1000 //1200


//************надо инициализировать в основном коде *****************
// свой выбранный пин как выход PoolPush активный сигнал 1.
//++++++++++++включение обмотки S ***********************************
#define  Start  GPIO_SetBits(GPIOA, GPIO_Pin_6);    // менять на свой пин
#define  Stop   GPIO_ResetBits(GPIOA, GPIO_Pin_6);  // менять на свой пин
#define  MotorStop   GPIO_ResetBits(GPIOA, GPIO_Pin_7); 
#define  MotorStart  GPIO_SetBits(GPIOA, GPIO_Pin_7); 

#define  DELAY300 delay_ms(300);                    // менять только на свою функцию
#define  DELAY200 delay_ms(200); 
#define  DELAY500 delay_ms(500); 
#define  DELAY1000 delay_ms(1000);                  // менять только на свою функцию
extern struct menu {
	 int IdMenu;
	u32 SetTemp;
	float TemperStop;
	double SetPWR;
	double SetHisteresis;
//	void(*menu1)(int);  // Функция	 

}Menu;
//******************************************************

volatile double max_min[4][2];
extern int cnt_run;
extern volatile int cnt_polka,  cnt_perehod,cnt_second, str_cnt_second,cnt_adjust;

extern volatile double diapazon,raznost_polka_tcar,delta,prev_delta;
double target_t;
double Limit_delt=0,Polka,prvTc;
int cnt_target_polka,preCnt;

extern  int starter_second,starter_sec,iTime_hold_target ;

extern volatile int cnt_polka;

extern volatile uint16_t adc_pa0,adc_pa1;
extern _Bool Enbl_Starter;
double T_min, delta_min,delta_max,Delta_stop_before,raznost,prey_temper;
_Bool SW0,SW1,SW2,SW3,SW4,SW5,SW6,SW7,SW8, First_in ,OneTime ;
extern double Del_stop_before;
int cmd_quiui,cnt_SW0;
double prv_difrent,vect_direction,raznost;
//double Polka;
volatile int cnt_auto_search=0; 
//*******************************************************
int cntaverage=4;  // анализ мин. мах. точек от целевой температуры
double ADC_Work_current = 1000; // ток мотора в работе >1000, но меньше <1500, при простое около 970
#define ADC_CURRENTMOTOR =1000
int EnableMotor(_Bool St);
_Bool pointup=FALSE;// условие при первом вхождении в функцию в Mode 0. 
int aver_count =4;                     // анализ, когда температура выше установленной, повышается
int Starter(uint16_t ADC,int status);
extern int cnt_work_pump;
volatile double correct_temp;
//*************************************

double pre_Tcur,pre_Tcur2;
int avegage_count=3;
int Cnt_points=0;

int  TestStarter(uint16_t ADC);
int search_minmax(double *Tc,double *Tst,double *Tmax,double *Tmin);

void Stabilizacij(double *Tcurent,double *Tstop,double *Limit_delt);
int adjust( int TimingWork,int TimigPause, double p, int que);
double Auto_search(_Bool Sw,int cnt ,double T_stop,double T_curent, double Delta_stop_before);
int Stat_Starter(uint16_t ADC);
int Starter2(uint16_t ADC, int status);
int Enbl_Motor(_Bool St);
double Limit_polka;
int polka(double *Tc, double *Ts, double *Plka, double *Toff,int *Thold,_Bool *On);
int search_polka(double *Tc, double *Ts, double *Plka, double *Toff,int *Thold,_Bool *On);
int search_polka2(double *Tc, double *Ts, double *Plka, double *Toff,int *Thold,_Bool *On);

#endif