/*
инж.программист Андрей М.
8-903-140-7333
UPG 22.03.23

*/

//--------------------------------------
#include "main.h"

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
 
#include "onewire.h"
#include "onewire2.h"

//#include "encoder.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "HidDev_Config.h"
#include "SysTimer.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

//#include "ADS1220_r.h"
#include "spi2_new.h"
#include "Display_SSD1306_I2C.h"
//#include "ADS_1220.h"
#include "ssd1306.h"
//#include "fonts.h"
//#include "max31865short.h" 
//#include "encoderTIM2_PA0_PA1.h"
 
//#include "OLED_Fonts.h"

#define UWORD unsigned short //65536                     16bit
#define UBYTE unsigned char  //255                        8bit
#define ULONG unsigned long long //18446744073709551615  64bit

//--------------------------------------------------
//костыль DS18B20 вовращает то 0 но норм темп.
//#define  DS18B20_ZERO_BLANK


#define  DS1306    //включить отображение на дисплеи
#define  DS1306_BLACK
#define  SERVIS_DEBUG	
//#define  ENCODER_ENBL 
#define  A    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#define  B    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)

#define  Start  GPIO_SetBits(GPIOA, GPIO_Pin_6);
#define  Stop   GPIO_ResetBits(GPIOA, GPIO_Pin_6);

#define  Start_PA11 GPIO_SetBits(GPIOA, GPIO_Pin_11);
#define  StopPA11   GPIO_ResetBits(GPIOA, GPIO_Pin_11);

#define GreenON  GPIO_ResetBits(GPIOA, GPIO_Pin_10);//green
#define GreenOff GPIO_SetBits(GPIOA, GPIO_Pin_10);

#define RedON  GPIO_ResetBits(GPIOA, GPIO_Pin_9);//red
#define RedOff GPIO_SetBits(GPIOA, GPIO_Pin_9);

#define BlueON  GPIO_ResetBits(GPIOA, GPIO_Pin_8);//blue
#define BlueOff GPIO_SetBits(GPIOA, GPIO_Pin_8);

//ENCODER_IRQ_DISANBL BUTTUN_IRQ_DISABEL ENCODER_IRQ_ENABL BUTTUN_IRQ_ENABL FAZE_IRQ_DISABEL FAZE_IRQ_ENABEL

#define  ENCODER_IRQ_DISANBL      EXTI->PR &= ~EXTI_IMR_MR0;  //PA0
#define	 BUTTUN_IRQ_DISABEL				EXTI->PR &= ~EXTI_IMR_MR12; //PB12

#define  ENCODER_IRQ_ENABL 		    EXTI->IMR  |=	EXTI_IMR_MR0;		
#define	 BUTTUN_IRQ_ENABL					EXTI->IMR  |=	EXTI_IMR_MR12;

#define  FAZE_IRQ_DISABEL         EXTI->IMR  &=	~EXTI_IMR_MR2;//PB2
#define  FAZE_IRQ_ENABEL 	        EXTI->IMR  |=	EXTI_IMR_MR2;		

#define  RELE_START     GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define  RELE_STOP      GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define  PB12_PUSH      GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)





//_Bool S=TRUE;
extern int cnt_SW0;
int cnt_Led;
extern _Bool SW0,SW1,SW6,SW7,SW8;
extern _Bool First_in ,OneTime; 
extern double delta_max;
int out;
double diapazon,raznost_polka_tcar;
volatile _Bool Enbl_Starter=TRUE;	
//double tm;	
int Switch_Programm=0;
volatile double prev_Tcurrent,
	delta, prev_delta;/*prv_slope, 
	delta_after_stop,delt,
pre_value2,
pre_value3,
pre_value,

prev_direction,
vector_direction,
prev_difrent,
prev_delta,
  Del_stop_before;
*/
volatile int Str;//iQueue,,Twork,Tpause;
/*volatile int Cnt_points=0, 
	cnt_perehod=0, 
Deltime=0,
before_cnt1,
before_cnt2,
before_cnt3,
before_cnt4,
before_cnt5,

before_correct;*/
extern double Limit_delt ,Polka;//,prvTc;
extern int cnt_target_polka,preCnt;

volatile double prev_value1,prev_value2;
volatile double maxmin[5][2];//points max min  5ть строк 2-а столбца
//volatile double T_min;
//volatile double T_max;
//volatile	double Kf=1;

volatile	double _min; 
volatile	double _max; 
 
 //volatile	double K_down,K_down;
//volatile	double  K_min,K_max;

volatile	double  Calculated_Tstop;
//volatile int preCnt;
volatile int cnt_polka,
	starter_second,
iTime_hold_target;

volatile int
cnt_work_pump,cnt_adjust,
cnt_work_pump_tstop,
prev_cnt_work_pump_tstop,
cnt_run, 
prev_cnt_run,
cnt_second,starter_sec,str_cnt_second; 

volatile double min,max;
double modul,Kf_up=1;;
_Bool sign_minus =FALSE;
_Bool stabilizacija =FALSE;
_Bool start = FALSE;	
_Bool FirstTime=TRUE;
int encoder;
int32_t nVol=0;	
//bool Phaza=FALSE;
_Bool FirstEner=FALSE;
_Bool activ_menu=FALSE;
double CurrentTenmper=0,CurrentPrev=0;
double number32,firs_number32;
struct menu {
	 int IdMenu;
	u32 SetTemp;
	double /*float*/ TemperStop;
	double SetPWR;
	double SetHisteresis;
//	void(*menu1)(int);  // Функция	 

}Menu;
//double TemperStop=0;
//int ex=0;
//int indx;
//char buffer_set_t[3];
int32_t encoder_read_previous=0 ,encoder_real=0,val;
int start_convert=0;
int flag_eprom_slop_intercept=0;
volatile char buffer_temp[15]; //for sd1306
volatile char buffer_temp2[15]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
volatile char buffer_temp3[20]; //for sd1306
//volatile char buffer_text[10] ="test";


_Bool Starter_motor(_Bool r, uint16_t adc_curent );
u32 enc_read(u32 offset);
u16 Get_Adc(u8 ch);
int menu1(int indx, int m);
void submenu1(int encodert,int indx);
void Delay_us (uint32_t __IO us) ;
void MonitorTEMPER_DS(void);
void displey(int X, int Y,double volue,int sizefont);
double WorkingPomp( double dCurrentTenmper,double dTenmperTarget, double dHister);	
void IWDG_Feed(void); 
void IWDG_Init(u8 prer,u16 rlr);

void _SPI2_Init(void);
void spi2(void);
void sendrecivtest(uint8_t data);
double gettemp(uint8_t* buf1, int num);
uint8_t Prepare_Buf2WritEpr[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static float pid_output = 0.0;

typedef struct {
    int   mount_ds1;   // amount working sensor ds18b20
	  int   mount_ds2;   //get adr for save
	volatile  char Tmpr1[8];
	volatile  char Adr_ds1[8];  // address first  ds18b20
	volatile  char PrevTmpr1[8];
	volatile u16   power_ten; 
	volatile u16  Set_temp;
	volatile u16 current_cnt;
} ds;
ds DS18b20;

struct Calibr{
					double dLoad_Slope;
          double dIntercept;
          double dF1;
          double dF2;
          double dLoad_Ceel;
          double Load_offset;
          int iADC_L;
          int iADC_H;
};

volatile int flag_wait=0;	
volatile int main_menu=0;	
//u16 Get_Adc(u8 ch); //ПОСТОЯННО БРАТЬ ЗНАЧЕНИЕ В ТАЙМЕРЕ.
char GetMessureDS1820_adr2(char *adr,char *tmp,int num);
volatile uint16_t adc_pa0,adc_pa1;
uint32_t temper_hex1,temper_hex2,temper_hex_l,temper_hex_h,temper_dec,temper_hex_half_gradus=0;
char get_buf[20];
//char get_buf2[20];
//char adr[25];
 TIM_TimeBaseInitTypeDef timer;
int eprcnt=0;
//-----------------------------------------------------
//char Buff_Res[1];
void  RCC_Configuration_internal(void);
void HSI_SetSysClk( uint32_t RCC_PLLMul_x );
void HSE_SetSysClk( uint32_t RCC_PLLMul_x );
void HSE_SetSysClk_4mhz( uint32_t RCC_PLLMul_x );
uint32_t System_CoreClock,preveus_CoreClock,Time_delay;
int tr, stm_st1;
char t[1];
void TIM3_PWM_Init(u16 arr,u16 psc);
void PWM_TIM3_Init2(void);
void TIM3_PWM_Init22(u16 arr,u16 psc);
//--------------------------------------- PA1 --------------------------------------
void ADC1_2_IRQHandler(void)
{

	if (ADC_GetITStatus(ADC1, ADC_IT_EOC)) /* ждем окончания цикла (13,5 достаточно для 3-х значного значения),
	                                      	если надо более точное, поменяйте в ADC_setup() -> ADC_SampleTime_13Cycles5 
	                                        на другое значение согласно документации на этот контроллер*/
 {     
   ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);  // сброс флага преобразования его в принципе можно поставить и в конце функции
  //  result_1 = ADC_GetConversionValue(ADC1); // преобразованное значение        
	              // фильтр помехи, если у вас значения преобразования будут меньше, тогда закоменируйте эту строку
	 


}
}
//================================================================
//================================================================
//================================================================
//================================================================
#define TARGET_TEMP    -1.5f       	// Целевая температура
#define FILTER_SIZE    30          	// Размер окна усреднения
#define TREND_HISTORY_SIZE 30  // последние усреднённые значения
#define D_FILTER_SIZE  15
#define HYSTERESIS     0.2f
#define FAST_COOL_DELTA 2.5f // Диапазон форсированного охлаждения, градусы
//#define KFF 120.0f  // если дельта температуры ~-0.01 — коррекция -1.2%
#define FEEDFORWARD_ZONE 1.05f 
#define MIN_COMPRESSOR_OFF_TIME 35 // например, 1 минута
#define CYCLE_TIME 90 // 2 минуты
#define MIN_ON_TIME 5
#define TREND_SIGN_BUF_SIZE 12
#define Kp2	300.0f   // пропорциональный для «второго» регулятора
static float Kd2 = 20.0f;  // дифференциальный (т. е. «чувствительность» к deltaForTresholds)
#define Kp3 50.0f;

static uint32_t cycle_start = 0;
static int32_t compressor_on_time = 0;
static int32_t compressor_on_time_result = 0;
static uint8_t compressor_state = 0; // 0=OFF, 1=ON
static uint32_t last_switch_time = 0;
static int32_t elapsed_time = 0;
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float output;
} PID_TypeDef;
static PID_TypeDef pid = { .Kp = 50.5f, .Ki = 0.000f, .Kd = 0.0f, .integral = 0.0f, .prev_error = 0.0f, .output = 0.0f };
float setpoint = -1.5f; // Целевая температура
static float ff_correction = 0.0f;
static float deltaForTresholds = 0.0f;
static float last_delta = 0.0f;       // храним предыдущий deltaForTresholds
static float derivative = 0.0f;
static float delta_for_delta_sub_delta = 0.0f;
static float pd_correction_pid2 = 0.0f;
static float koeffMultiplicator = 0.0f;
static float filtered_d = 0.0f;
static float d_buffer[D_FILTER_SIZE] = {0};
static int d_buffer_index = 0;
static int d_buffer_count = 0;
static float fine_p3 = 0.0f;
static float fine_d = 0.0f;
static float fine_p = 0.0f;
static int32_t compressor_time_offset = 15;
static float err_sum_for_offset = 0;
static int n = 0;
static float avg_error_for_offset = 0.0f;

static int stable_time = 0;      // Сколько секунд держимся выше цели
static int was_below = 0;   // Флаг — опускались ли ниже setpoint
#define STABLE_PERIOD  300 // Время ожидания (например, 1 час = 3600 сек)

void addToDBuffer(float value) {
    d_buffer[d_buffer_index] = value;
    d_buffer_index = (d_buffer_index + 1) % D_FILTER_SIZE;
    if (d_buffer_count < D_FILTER_SIZE) d_buffer_count++;
}

float getDAverage() {
    float sum = 0;
    for (int i = 0; i < d_buffer_count; ++i) sum += d_buffer[i];
    return (d_buffer_count > 0) ? (sum / d_buffer_count) : 0;
}


void TurnCompressorOnSafe(uint32_t now) {
    // Включаем только если выдержали минимальную паузу
    if (!compressor_state) {
        motorStart_();
        compressor_state = 1;
        last_switch_time = now;
    }
}

void TurnCompressorOffSafe(uint32_t now) {
    if (compressor_state) {
        motorStop_();
        compressor_state = 0;
        last_switch_time = now;
    }
}

//void adapt_offset(float tempAvr) {
//	
//    n++;
//	
//		if (n >= (CYCLE_TIME * 4)) {
//				if (tempAvr <= (setpoint - (HYSTERESIS / 2.0)))
//						compressor_time_offset += 1;
//				n = 0;
//		} else {
//				if (n >= CYCLE_TIME) {
//						if (tempAvr <= (setpoint - (HYSTERESIS / 2.0)))
//								compressor_time_offset -= 1;
//						n = 0;
//				}		
//		}


//		//if (n >= (CYCLE_TIME*2)) {
//    //    avg_error_for_offset = err_sum_for_offset / n;
//    //    if (avg_error_for_offset > 0.1f) compressor_time_offset += 1;
//    //    else if (avg_error_for_offset < -0.1f) compressor_time_offset -= 1;
//    //    if (compressor_time_offset < 10) compressor_time_offset = 10;
//    //    if (compressor_time_offset > 60) compressor_time_offset = 60;
//    //    err_sum_for_offset = 0;
//    //    n = 0;
//    //}
//}

void adapt_offset(float tempAvr) {
    float error = tempAvr - setpoint;
    err_sum_for_offset += error;
    n++;

    // Быстрая корректировка, если ошибка вышла за пределы гистерезиса
    //if (fabsf(error) > 0.2f) { // за пределами хистерезиса
    //    if (error > 0) compressor_time_offset += 2;
    //    else compressor_time_offset -= 2;
    //    err_sum_for_offset = 0;
    //    n = 0;
    //}
    // Обычная плавная корректировка раз в N циклов
    //else 
		if (n >= CYCLE_TIME) {
        avg_error_for_offset = err_sum_for_offset / n;
        if (avg_error_for_offset > 0.09f) compressor_time_offset += 1;
        else if (avg_error_for_offset < -0.09f) compressor_time_offset -= 2;
        if (compressor_time_offset < 10) compressor_time_offset = 10;
        if (compressor_time_offset > 60) compressor_time_offset = 60;
        err_sum_for_offset = 0;
        n = 0;
    }
}

void UpdateCompressor(float pid_output, uint32_t now, float tempAvr) {
		float delta = deltaForTresholds;
		addToDBuffer(delta - last_delta);
		delta_for_delta_sub_delta = getDAverage() * 7500.0;
		//fine_p = CYCLE_TIME * (pid_output / 100.0f);
    compressor_on_time = (int32_t)(CYCLE_TIME * (pid_output / 100.0f)) + compressor_time_offset;

	
    if ((tempAvr - setpoint) > FAST_COOL_DELTA) {
        TurnCompressorOnSafe(now);
				last_delta = delta;
        return;
    }
		
    if ((setpoint - tempAvr) > FAST_COOL_DELTA) {
        TurnCompressorOffSafe(now);
        last_delta = delta;
				return;
    }
		
		if (setpoint - tempAvr > FEEDFORWARD_ZONE) {
				last_delta = delta;
				return;
		}
		
		adapt_offset(tempAvr);		

		
		if (!compressor_state && (now - last_switch_time) < MIN_COMPRESSOR_OFF_TIME) {
        last_delta = delta;
        return;
    }
		
	
		// обновляем по завершению цикла
    if ((now - cycle_start) >= CYCLE_TIME) {
        // Начинаем новый цикл: сразу обнуляем elapsed
        cycle_start = now;
//        compressor_on_time = (uint32_t)(CYCLE_TIME * (pid_output / 100.0f));
        //compressor_on_time = (uint32_t)fine_p;
        //if (compressor_on_time < MIN_ON_TIME) {
        //    compressor_on_time = MIN_ON_TIME;
        //}			
        TurnCompressorOffSafe(now);

        elapsed_time = 0;  // Жёстко обнуляем
    } else {
        // В старом цикле — считаем, сколько уже прошло
        elapsed_time = now - cycle_start;
    }
		
		if (fabsf(tempAvr - setpoint) < FEEDFORWARD_ZONE) {
				//const float ALPHA = 0.25f; // можно подбирать от 0.2 до 0.5
				//filtered_d = (1.0f - ALPHA) * filtered_d + ALPHA * (delta - last_delta);
			
				//fine_p = 0.0 * (tempAvr - setpoint);  // мягкий подгон
				//float fine_d = Kp2 * delta + delta_for_delta_sub_delta; // Kd2 * filtered_d
				//if (deltaForTresholds > 0.008 && delta_for_delta_sub_delta < 0 || 
				//		deltaForTresholds < -0.008 && delta_for_delta_sub_delta > 0) {
				//		fine_d = Kd2 * deltaForTresholds;
				//} else {
				//		fine_d = Kd2 * deltaForTresholds + delta_for_delta_sub_delta; 
				//}
				fine_d = Kd2 * deltaForTresholds; 
				pd_correction_pid2 = fine_d;
			
			
				//derivative = (delta - last_delta); 
				//pd_correction_pid2 = Kp2 * delta + Kd2 * derivative;
				last_delta = delta;	
				int32_t  adj = (int32_t)pd_correction_pid2;
				int32_t  new_on  = (int32_t)compressor_on_time + adj;
			
				//if (new_on < (int32_t)MIN_ON_TIME)        new_on = MIN_ON_TIME;
				
				// - 1 для того чтоб при при завершении цикла при обнулении elapsed завершалась работа компрессора
				//Если время увеличивается до максимума elapsed обнуляется и снова идёт следующий цикл, 
				//интресно посмотреть как поведёт себя система если дельта сильно изменится 
				//по идее compressor_on_time_result должен будет резко стремиться к нулю и завершить следующий цикл
				compressor_on_time_result = (int32_t)new_on;
		} else {
        last_delta = delta;
				compressor_on_time_result = compressor_on_time;
		}	

		if (compressor_on_time_result >= (int32_t)CYCLE_TIME)         
				compressor_on_time_result = CYCLE_TIME - 1; // - 1;

    // А теперь уже включаем/выключаем по ровному elapsed
    if (elapsed_time < compressor_on_time_result) {
        TurnCompressorOnSafe(now);
    } else {
        TurnCompressorOffSafe(now);
    }
}

float PID_Compute(PID_TypeDef *pid, float setpoint, float measured, float dt) {
    float error = measured - setpoint;

    // --- Anti-windup (интеграл только если выход не на нуле или ошибка > 0)
    if (!(pid->output <= 0.0f && error < 0)) {
        pid->integral += error * dt;
    }

    // --- Жёсткое ограничение интеграла
    if (pid->integral > 50.0f) pid->integral = 50.0f;
    if (pid->integral < -50.0f) pid->integral = -50.0f;

    // --- (Опционально) Сброс интеграла при пересечении цели
    static float last_error = 0;
    if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
        pid->integral = 0;
    }
    last_error = error;

    float derivative = (error - pid->prev_error) / dt;
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;

    return pid->output;
}



// === Фильтр температуры ===
static float tempBuffer[FILTER_SIZE] = {0};
static int bufferIndex = 0;
static int bufferCount = 0;
static float tempSum = 0.0f;
static float tempAvr = 0.0;
static int lastTrend = 0;
static int stableTrendCounter = 0;
const int TREND_CONFIRM_COUNT = 3; // сколько раз подряд тренд должен совпасть
//const float TREND_THRESHOLD = 0.03f; // чувствительность (в градусах)  можешь понизить до 0.0625f, но не ниже
static int8_t trendSignBuf[TREND_SIGN_BUF_SIZE] = {0};
static int trendSignIdx = 0;
static int reversal = 0;
static int trendAvr = 0;
static float tempTrendHistory[TREND_HISTORY_SIZE] = {0};
static int trendIndex = 0;
static int trendCount = 0;
static int counterTick = 1;
static int switchCoolingOn = 1;

void addToTrendHistory(float avgTemp) {
    tempTrendHistory[trendIndex] = avgTemp;
    trendIndex = (trendIndex + 1) % TREND_HISTORY_SIZE;
    if (trendCount < TREND_HISTORY_SIZE) trendCount++;
}

float averageN(const float* buf, int start, int count) {
    float sum = 0;
    for (int i = 0; i < count; ++i) {
        sum += buf[(start + i) % TREND_HISTORY_SIZE];
    }
    return sum / count;
}
// Вызов при каждом обновлении температуры
int calculateTrend() {
    if (trendCount < TREND_HISTORY_SIZE) return 0;

    // Сравниваем среднее старых и новых значений
		int window = TREND_HISTORY_SIZE / 3;
    int oldStart = (trendIndex + TREND_HISTORY_SIZE - 2 * window) % TREND_HISTORY_SIZE;
    int newStart = (trendIndex + TREND_HISTORY_SIZE - window) % TREND_HISTORY_SIZE;

    float avgOld = averageN(tempTrendHistory, oldStart, window);
    float avgNew = averageN(tempTrendHistory, newStart, window);

    deltaForTresholds = avgNew - avgOld;

    const float TREND_THRESHOLD = 0.010f;

    if (deltaForTresholds > TREND_THRESHOLD)
        return +1; // растёт
    else if (deltaForTresholds < -TREND_THRESHOLD)
        return -1; // падает
    else
        return 0; // стабильно
}

void updateTrendSignBuffer(int8_t trendSign) {
    trendSignBuf[trendSignIdx] = trendSign;
    trendSignIdx = (trendSignIdx + 1) % TREND_SIGN_BUF_SIZE;
}

// Проверка разворота тренда: смена знака между текущим и хотя бы одним предыдущим значением
bool isTrendReversalImproved() {
    int lastNonZero = 0;
    for (int i = 1; i < TREND_SIGN_BUF_SIZE; ++i) {
        int idx = (trendSignIdx - 1 - i + TREND_SIGN_BUF_SIZE) % TREND_SIGN_BUF_SIZE;
        if (trendSignBuf[idx] != 0) {
            lastNonZero = trendSignBuf[idx];
            break;
        }
    }
    int current = trendSignBuf[(trendSignIdx - 1 + TREND_SIGN_BUF_SIZE) % TREND_SIGN_BUF_SIZE];
    if (current != 0 && lastNonZero != 0 && current * lastNonZero < 0) {
        return 1;
    }
    return 0;
}

void tempFilter_add(float value) {
    tempSum -= tempBuffer[bufferIndex];
    tempBuffer[bufferIndex] = value;
    tempSum += value;

    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
    if (bufferCount < FILTER_SIZE) bufferCount++;
}

float tempFilter_average(void) {
    return (bufferCount > 0) ? (tempSum / bufferCount) : 0.0f;
}

void updateControl(void) {
	
    float rawTemp = CurrentTenmper;
    tempFilter_add(rawTemp);
    tempAvr = tempFilter_average();

		addToTrendHistory(tempAvr);
		trendAvr = calculateTrend();

		pid_output = PID_Compute(&pid, setpoint, tempAvr, 1.0f);
		
    //if (pid_output > 100.0f) pid_output = 100.0f;
    //if (pid_output < 0.0f) pid_output = 0.0f;
		UpdateCompressor(pid_output, counterTick, tempAvr);			
			
		counterTick++;
		adc_pa1_SW6_SMTH_FUNC();	
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//================================================================
//================================================================
//================================================================
//================================================================



//------------ фильтр-------------------
// период дискретизации (измерений), process variation, noise variation
float dt = 0.01;
float sigma_process = 5.0;
float sigma_noise = 0.9;
float ABfilter(float newVal) {
  static float xk_1, vk_1, a, b;
  static float xk, vk, rk;
  static float xm;
  float lambda = (float)sigma_process * dt * dt / sigma_noise;
  float r = (4 + lambda - (float)sqrt(8 * lambda + lambda * lambda)) / 4;
  a = (float)1 - r * r;
  b = (float)2 * (2 - a) - 4 * (float)sqrt(1 - a);
  xm = newVal;
  xk = xk_1 + ((float) vk_1 * dt );
  vk = vk_1;
  rk = xm - xk;
  xk += (float)a * rk;
  vk += (float)( b * rk ) / dt;
  xk_1 = xk;
  vk_1 = vk;
  return xk_1;
}
//-----------------------------------
 
double convert_2_double(char buffer[])
{

   // Определяем указатель на символ endptr для использования с функцией strtod [4](https://www.geeksforgeeks.org/convert-a-char-array-to-double-in-c/)
    char* endptr;
    // Конвертируем строку в число double и сохраняем его в переменную num с помощью функции strtod [4](https://www.geeksforgeeks.org/convert-a-char-array-to-double-in-c/)
     firs_number32 = strtod(buffer, &endptr);
    firs_number32 = atof(buffer);

											
return firs_number32;
}

// Поиск мин. макс. точек тока
int measureCurrent(){// in working 150-225 , in stop 30-75
    int i;
    const int cnt = 1000;
    int minv = 2060;
    int maxv = -1;
    for(i = 0; i < cnt; i++) {
      int value =  	Get_Adc(ADC_Channel_3); //  analogRead(A0);
      if (value > maxv) {
        maxv = value;
      }
      if (value < minv) {
        minv = value;
      }
    //  delay_ms(2);
    }
    return maxv - minv;
}

double dif,limit,curr,k;
int cnt_points=0;
void displ(void)
{
 	
#ifdef DS1306_BLACK
SSD1306_Fill(SSD1306_COLOR_BLACK);
			
//++++++++++++++++  Setting threate temperature +++++++++++++++++++++++++			
							SSD1306_GotoXY(0, 10); 
					 	   sprintf((char*)buffer_temp,"%g",Menu.TemperStop);
						  	SSD1306_Puts(buffer_temp, &Font_16x26, SSD1306_COLOR_WHITE);
//*************  Get real temperature *****************************	
			if((DS18b20.Tmpr1[0] != 0xff))  /*&& (DS18b20.Tmpr1[1] != 0xff))&& (DS18b20.Tmpr1[2] != 0xff)) &&  (DS18b20.Tmpr1[3] != 0xff))*/
							{memcpy(&DS18b20.PrevTmpr1,&DS18b20.Tmpr1,8);	
								//	 gettemp(DS18b20.Tmpr1,1);buffer_temp	
								 gettemp(DS18b20.Tmpr1,2);//buffer_temp2
									memcpy(&DS18b20.Tmpr1,&DS18b20.PrevTmpr1,8);	// иначе глюк с отправкой с перевернутым минусом через раз 
							}
			else
				 	memcpy(&DS18b20.Tmpr1,&DS18b20.PrevTmpr1,8);	
			
			
//***************************************************************
	SSD1306_DrawLine(0, 34,118,34,SSD1306_COLOR_WHITE);	
	SSD1306_DrawLine(80,0,80,32,SSD1306_COLOR_WHITE);			
//*****************************************************************
	SSD1306_GotoXY(0, 36);// Current T
			// snprintf(buffer_temp2, sizeof buffer_temp2, "",buffer_temp2);
	SSD1306_Puts(buffer_temp2, &Font_16x26, SSD1306_COLOR_WHITE);	
		SSD1306_DrawLine(0, 64,118,64,SSD1306_COLOR_WHITE);		
	CurrentTenmper=	convert_2_double(buffer_temp2);
	
			
			SSD1306_GotoXY(82, 1);//power
	 sprintf((char*)buffer_temp,"R %d",adc_pa1);
	 	SSD1306_Puts(buffer_temp, &Font_7x10, SSD1306_COLOR_WHITE);
		
		 SSD1306_GotoXY(82,10);
					  snprintf(buffer_temp, sizeof buffer_temp, "p %g",(double)Menu.SetPWR);
 						SSD1306_Puts(buffer_temp, &Font_7x10, SSD1306_COLOR_WHITE);
		
 SSD1306_GotoXY(82,20);
					  snprintf(buffer_temp, sizeof buffer_temp, "h %g",(double)	Menu.SetHisteresis);
 						SSD1306_Puts(buffer_temp, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_UpdateScreen();					
	 
			
	#else
 SSD1306_Fill(SSD1306_COLOR_WHITE);
//++++++++++++++++  Setting threate temperature +++++++++++++++++++++++++			
							SSD1306_GotoXY(10, 10); 
					 	   sprintf((char*)buffer_temp,"%g",Menu.TemperStop);
						  	SSD1306_Puts(buffer_temp, &Font_16x26, SSD1306_COLOR_BLACK);
//*************  Get real temperature *****************************	
 memcpy(&DS18b20.PrevTmpr1,&DS18b20.Tmpr1,8);	
           gettemp(DS18b20.Tmpr1,1);	
	memcpy(&DS18b20.Tmpr1,&DS18b20.PrevTmpr1,8);	// иначе глюк с отправкой с перевернутым минусом через раз 

//*****************************************************************
	SSD1306_GotoXY(10, 36);// Current T
	SSD1306_Puts(buffer_temp, &Font_16x26, SSD1306_COLOR_BLACK);	
	CurrentTenmper=	convert_2_double(buffer_temp);
	
			
			SSD1306_GotoXY(90, 10);//power
	 sprintf((char*)buffer_temp,"%d",adc_pa1);
	 	SSD1306_Puts(buffer_temp, &Font_7x10, SSD1306_COLOR_BLACK);
		
		 SSD1306_GotoXY(90,20);
					  snprintf(buffer_temp, sizeof buffer_temp, "%g",(double)Menu.SetPWR);
 						SSD1306_Puts(buffer_temp, &Font_7x10, SSD1306_COLOR_BLACK);
		
 SSD1306_GotoXY(90,30);
					  snprintf(buffer_temp, sizeof buffer_temp, "h %g",(double)	Menu.SetHisteresis);
 						SSD1306_Puts(buffer_temp, &Font_7x10, SSD1306_COLOR_BLACK);
			SSD1306_UpdateScreen();					
	 
		
		
		
	#endif 	
 	
  
	
	

		/*
if(FirstTime==TRUE)
{  
		  Menu.SetPWR=2000;   
        delay_ms(1);	
	if(adc_pa1>=2000)
			{
					Start; 
				delay_ms(100);
				Stop;
				delay_ms(1000);
							if((adc_pa1>=1000) && (adc_pa1<=2000))
							    FirstTime=FALSE;
				start=FALSE;
			}
			else
				Stop;

}
 //************************************				
 	dif=CurrentTenmper-Menu.TemperStop;
		
	 if((dif>0.8))// && (start == FALSE)) 
			{// start = FALSE;	
			  Menu.SetPWR=1997;
				if(adc_pa1< 1000)
					  FirstTime=TRUE; 
			}
		else if (dif <=0)
			{
			start = FALSE;	
							Menu.SetPWR =0;  //stop
			}
   else if(FirstTime==FALSE)
				{ start = TRUE;	
							Menu.SetPWR =0;  //stop
				}
	//else if((dif == 0))// || (dif <= 0.1))
	//	  Menu.SetPWR =0;  //stop
//***********************************************					 
		if((dif <= 0.1) &&(dif >= 0.0) && (start == TRUE))		
					{ //if(Menu.SetPWR==0) 
						  Menu.SetPWR=1998;
						  FirstTime=TRUE;               
						//	start=	Starter_motor(start,adc_pa1);	 
					}
	  

		*/
 
}
//********************************************
void displ2(void)
{
 
//#ifdef DS1306_BLACK
_1306_Fill(SSD1306_COLOR_BLACK);
//++++++++++++++++  Setting threate temperature +++++++++++++++++++++++++			
						//	SSD1306_GotoXY(0, 10); 
					 	   sprintf((char*)buffer_temp,"%2.1f",Menu.TemperStop);
		          FontSet16X26_DEC();		
	            OLED_DrawStr(buffer_temp, 0, 10 , 1);	
//	OLED_DrawNum(buffer_temp, 0, 10 , 1);
//*************  Get real temperature *****************************	
		/*	if((DS18b20.Tmpr1[0] != 0xff))  
							{memcpy(&DS18b20.PrevTmpr1,&DS18b20.Tmpr1,8);	
										 gettemp(DS18b20.Tmpr1,2);//buffer_temp2
									memcpy(&DS18b20.Tmpr1,&DS18b20.PrevTmpr1,8);	// иначе глюк с отправкой с перевернутым минусом через раз 
							}
			else
				 	memcpy(&DS18b20.Tmpr1,&DS18b20.PrevTmpr1,8);	
			*/
			
//***************************************************************
OLED_DrawHLine(0, 34,118, 1);
OLED_DrawVLine(75, 0, 80,1);
//*****************************************************************
	 FontSet16X26_DEC();		
			 OLED_DrawStr(buffer_temp2, 1, 38 , 1);	
	//CurrentTenmper=	convert_2_double(buffer_temp2);
 
	sprintf((char*)buffer_temp,"F% d",adc_pa1);//A
	FontSetArial();	
	OLED_DrawStr(buffer_temp, 82, 1 , 1);	
	
	snprintf(buffer_temp, sizeof buffer_temp, "V% g",(double)Menu.SetPWR);//M
		FontSetArial();		
	   OLED_DrawStr(buffer_temp, 82, 10 , 1);	
	 OLED_DrawStr("Ghjuhtcc", 78, 40 , 1);	
			OLED_DrawStr("  Utj", 78, 47 , 1);	
			OLED_DrawStr("dthc 1-0", 78, 55 , 1);	
/*		snprintf(buffer_temp, sizeof buffer_temp, "U% g",(double)	Menu.SetHisteresis);
		for(int uu=0;uu<=sizeof(buffer_temp);uu++)
		      {
					 if (buffer_temp[uu]== 0x2e) 
						    buffer_temp[uu] = 0x7e;
					
					}
		
				FontSetArial();		
	  OLED_DrawStr(buffer_temp, 82, 20 , 1);	
	 */
				   OLED_UpdateScreen();	
 
}


//****************************************************************************************
double WorkingPomp(double dCurrentTenmper,double dTenmperTarget, double dHister)
{
double diff=0;//,modul=0;

	
//********** Starter **************************	
if(FirstTime==TRUE)
{ 
	if( Menu.SetPWR==0)	  Menu.SetPWR=2000;   
        delay_ms(1);	
	if(adc_pa1>=2000)//starter
			{
					Start; 
				delay_ms(300);
				Stop;
				delay_ms(1000);
							if((adc_pa1>=1000) && (adc_pa1<=2000))
							    FirstTime=FALSE;
				start=FALSE;
			}
			else
				Stop;

}
//************************************	
if(stabilizacija==FALSE)
{ 

	
diff=CurrentTenmper-Menu.TemperStop;
modul=	fabs(CurrentTenmper-Menu.TemperStop);	
	
	if(CurrentTenmper>prev_value1)// + maximum
	{
		if(CurrentTenmper > maxmin[cnt_points][0])//cmd 0x74
		{ maxmin[cnt_points][0]=CurrentTenmper;
			max=maxmin[cnt_points][0];
		}
	}
  
	if(CurrentTenmper<prev_value1)// - minimum
	{if(CurrentTenmper < maxmin[cnt_points][1]) //cmd 0x73
	    {
				 maxmin[cnt_points][1]=CurrentTenmper;
				// Kf_up=(maxmin[cnt_points][1] - Menu.TemperStop);	
				min=maxmin[cnt_points][1];
				
			}
	}
	// maxmin[cnt_points][1]=CurrentTenmper;
				
	
prev_value1=CurrentTenmper ;	
	
k=(double)1.0;// + abs(Kf_up);
	
	//       Kf_up = Kf_up+modul;
	 if(diff>=0.8)/* && (start == FALSE))	*/
			{// start = FALSE;	
			  Menu.SetPWR=2001;
				if(adc_pa1< 1000)
					  FirstTime=TRUE; 
				sign_minus =FALSE;
				stabilizacija=FALSE;
			}
			
	/*	else if (CurrentTenmper < Menu.TemperStop)
			{ Kf_up=(maxmin[cnt_points][1] - Menu.TemperStop);	
			//	Menu.SetPWR =0;  //stop
			} */
   else if(FirstTime==FALSE)
				{ start = TRUE;	
							Menu.SetPWR =0;  //stop
				}
			else;
				
				
if((dif < 0.5))// || (dif <= 0.1))
{ sign_minus=TRUE;
	
	start = FALSE;	
	Menu.SetPWR =0;  //stop
	
}
else
//	sign_minus =FALSE;
//***********************************************					 
	if(sign_minus==TRUE)
	   { 
    if((diff <= 0.2) &&(diff >= -0.4))// && (start == TRUE))		
					{ //if(Menu.SetPWR==0) 
						  Menu.SetPWR=2002;
						  FirstTime=TRUE;               
					//	stabilizacija=TRUE;
					}
	  

		
			}	
}
				
				
				
				
return diff;

}


//---------------INTERUPT USB------
void HID_Receive(uint8_t *Buff, uint8_t Size) // Получен новый пакет данных по USB.
{   uint8_t out_data[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};//Read and store
 uint8_t out_data8[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//Read and store
  uint16_t out_data16[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//Read and store
	uint32_t out_data32[8];
	uint8_t STATUS;
	uint8_t send[8];
		uint8_t send2[8];
		uint8_t send3[8];
	
	
	uint8_t resp[8] = {0};
	if (Size < 5) return;  // safety

	uint8_t command = Buff[0];  // команда
	float value;

	// Собираем float из 4 байт (предполагаем little-endian)
	union {
			float f;
		  int32_t u;
			int8_t b[4];
	} converter;



	switch (command) {
			case 0x10:		// set
					converter.b[0] = Buff[1];
					converter.b[1] = Buff[2];
					converter.b[2] = Buff[3];
					converter.b[3] = Buff[4];
					setpoint = converter.f;
					break;
			case 0x11:		// set
					converter.b[0] = Buff[1];
					converter.b[1] = Buff[2];
					converter.b[2] = Buff[3];
					converter.b[3] = Buff[4];
					pid.Kp = converter.f;
					break;			
			case 0x12:		// set
					converter.b[0] = Buff[1];
					converter.b[1] = Buff[2];
					converter.b[2] = Buff[3];
					converter.b[3] = Buff[4];
					Kd2 = converter.f;
					break;			
			case 0x13:		// set
					converter.b[0] = Buff[1];
					converter.b[1] = Buff[2];
					converter.b[2] = Buff[3];
					converter.b[3] = Buff[4];
					compressor_time_offset = converter.u;			
					break;			
			
			// GETTER

			case 0x20:		// get tempAvr
					TIM_Cmd(TIM4, DISABLE);
					converter.u = command;
					resp[0] = converter.b[0];
					resp[1] = converter.b[1];
					resp[2] = converter.b[2];
					resp[3] = converter.b[3];
			
					converter.f = tempAvr;
					resp[4] = converter.b[0];
					resp[5] = converter.b[1];
					resp[6] = converter.b[2];
					resp[7] = converter.b[3];
			
					HID_SendBuff(resp, 8);
					TIM_Cmd(TIM4, ENABLE);
					break;
			case 0x21:		// get pid.Kp
					TIM_Cmd(TIM4, DISABLE);
					converter.u = command;
					resp[0] = converter.b[0];
					resp[1] = converter.b[1];
					resp[2] = converter.b[2];
					resp[3] = converter.b[3];	
			
					converter.f = pid.Kp;
					resp[4] = converter.b[0];
					resp[5] = converter.b[1];
					resp[6] = converter.b[2];
					resp[7] = converter.b[3];
			
					HID_SendBuff(resp, 8);
					TIM_Cmd(TIM4, ENABLE);
					break;
			case 0x22:		// get Kd2
					TIM_Cmd(TIM4, DISABLE);
					converter.u = command;
					resp[0] = converter.b[0];
					resp[1] = converter.b[1];
					resp[2] = converter.b[2];
					resp[3] = converter.b[3];	
			
					converter.f = Kd2;
					resp[4] = converter.b[0];
					resp[5] = converter.b[1];
					resp[6] = converter.b[2];
					resp[7] = converter.b[3];
					HID_SendBuff(resp, 8);
					TIM_Cmd(TIM4, ENABLE);
					break;
			case 0x23:		// get compressor_time_offset
					TIM_Cmd(TIM4, DISABLE);
					converter.u = command;
					resp[0] = converter.b[0];
					resp[1] = converter.b[1];
					resp[2] = converter.b[2];
					resp[3] = converter.b[3];	
			
					converter.u = compressor_time_offset;
					resp[4] = converter.b[0];
					resp[5] = converter.b[1];
					resp[6] = converter.b[2];
					resp[7] = converter.b[3];
					HID_SendBuff(resp, 8);
					TIM_Cmd(TIM4, ENABLE);
					break;
			case 0x24:		// getCycleTime
					TIM_Cmd(TIM4, DISABLE);
					converter.u = command;
					resp[0] = converter.b[0];
					resp[1] = converter.b[1];
					resp[2] = converter.b[2];
					resp[3] = converter.b[3];	
			
					converter.u = CYCLE_TIME;
					resp[4] = converter.b[0];
					resp[5] = converter.b[1];
					resp[6] = converter.b[2];
					resp[7] = converter.b[3];
					HID_SendBuff(resp, 8);
					TIM_Cmd(TIM4, ENABLE);
					break;
			case 0x25:		// get setpoint
					TIM_Cmd(TIM4, DISABLE);
					converter.u = command;
					resp[0] = converter.b[0];
					resp[1] = converter.b[1];
					resp[2] = converter.b[2];
					resp[3] = converter.b[3];
			
					converter.f = setpoint;
					resp[4] = converter.b[0];
					resp[5] = converter.b[1];
					resp[6] = converter.b[2];
					resp[7] = converter.b[3];
			
					HID_SendBuff(resp, 8);
					TIM_Cmd(TIM4, ENABLE);
					break;			
			default:
					setpoint = -1.0f;
					break;
	}
	return;

//GPIO_SetBits(OUT_C,LED);
 
if(flag_eprom_slop_intercept==0) 
 {
	if (Buff[0]== 0x99) //test usb cmd
			{
				TIM_Cmd(TIM4, DISABLE);
		send[0]	=0x12;
		send[1]	=0x34;
		send[2]	=0x56;
		send[3]	=0x78;
		send[4]	=0x9a;
		send[5]	=0xbc;
		send[6]	=0xde;
		send[7]	=0xf0;		
				
			HID_SendBuff(send, 8);
				
			TIM_Cmd(TIM4, ENABLE);	
			
			}
if (Buff[0]== 0xAA) //spi get data
			{
				TIM_Cmd(TIM4, DISABLE);
				
	//	send[0]	=SPI2_Read();
				
	//	send[0]	=0x12;
		send[1]	=0x34;
		send[2]	=0x56;
		send[3]	=0x78;
		send[4]	=0x9a;
		send[5]	=0xbc;
		send[6]	=0xde;
		send[7]	=0xf0;		
				
			HID_SendBuff(send, 8);
				
			TIM_Cmd(TIM4, ENABLE);	
			
			}			
			
//code bisy for adcbto timn4
   if (Buff[0]== 0x60) //read adc1  датчик силы
			{
	/*			TIM_Cmd(TIM4, DISABLE);
		send[0]	=adc_pa1>>8;
		send[1]	=adc_pa1;		
				*/	
			HID_SendBuff(send, 8);
			
			TIM_Cmd(TIM4, ENABLE);	
			
			}
//-------------------------
		if (Buff[0]== 0x61) //read adc1 датчик перемещения
			{	
				TIM_Cmd(TIM4, DISABLE);
		send[0]	=adc_pa0>>8;
		send[1]	=adc_pa0;		
				
			HID_SendBuff(send, 8);
				
				TIM_Cmd(TIM4, ENABLE);		
			
			}
//------------------------------
if (Buff[0]== 0x62) //read adc 24
	{//uint32_t t =Read_ADC(0);
	/*	send[0]	=iV_out;//iV_out >>24;
		send[1]	=iV_out>>8;;//iV_out>>16;
		send[2]	=iV_out>>16;;//iV_out>>8;
		send[3]	=iV_out>>24;//iV_out;
	*/
		send[4]	=0;
		send[5]	=0;
		send[6]	=0;
		send[7]	=0;
			
				HID_SendBuff(send , 8);
			
	}

//-------------------------------
  if (Buff[0]== 0x70) // adc1  датчик силы записать мин. значенеие
			{
		int t=	Buff[1];
			t=	  Buff[2];
			t=		Buff[3];
			t=		Buff[4];	
			t=		Buff[5];		
				
				TIM_Cmd(TIM4, DISABLE);
	 
		send[0]	=adc_pa1>>8;
		send[1]	=adc_pa1;		
	//	 DS18b20.dLoad_Slope = (DS18b20.dF2 - DS18b20.dF1) / (Convert.ToDouble(DS18b20.iADC_H) - Convert.ToDouble(DS18b20.iADC_L));
 //    DS18b20.dIntercept = DS18b20.dF1 - (DS18b20.dLoad_Slope * Convert.ToDouble(DS18b20.iADC_L));
  int dValue1 = send[0];// & 0xf8;
  int dValue2 = send[1];// &0xfe;
	HID_SendBuff(send , 8);
    dValue1 = dValue2 + (dValue1 * 256);        



				
			TIM_Cmd(TIM4, ENABLE);	
			
			}

 if (Buff[0]== 0x71) // adc1  датчик силы записать мин. значенеие
			{
	//uint8_t send[8];	 
				
	TIM_Cmd(TIM4, DISABLE);
 //Menu.TemperStop =Menu.TemperStop;
				
	 sprintf((char*)buffer_temp,"%f",Menu.TemperStop);			
				
	HID_SendBuff(buffer_temp, 8);

				
			TIM_Cmd(TIM4, ENABLE);	
			
			}

 if (Buff[0]== 0x72) // k
			{
	//uint8_t send[8];	 
				
	TIM_Cmd(TIM4, DISABLE);
 //Menu.TemperStop =Menu.TemperStop;
				
	 sprintf((char*)buffer_temp,"%f",Kf_up);			
				
	HID_SendBuff(buffer_temp, 8);

				
			TIM_Cmd(TIM4, ENABLE);	
			
			}
//***********************************			
		 if (Buff[0]== 0x73) // min
			{
	//uint8_t send[8]; 
				
	TIM_Cmd(TIM4, DISABLE);
 //Menu.TemperStop =Menu.TemperStop;
				
	 sprintf((char*)buffer_temp,"%f",min);			
				
	HID_SendBuff(buffer_temp, 8);

				
			TIM_Cmd(TIM4, ENABLE);	
			
			}	
//***********************************			
		 if (Buff[0]== 0x75) // max
			{
	//uint8_t send[8];	 
				
	TIM_Cmd(TIM4, DISABLE);
 //Menu.TemperStop =Menu.TemperStop;
				
	 sprintf((char*)buffer_temp,"%f",max);			
				
	HID_SendBuff(buffer_temp, 8);

				
			TIM_Cmd(TIM4, ENABLE);	
			
			}	
			
			
//-------------------------
	if (Buff[0]== 0x20) //write eeprom intercept
			{
    //  flag_eprom_slop_intercept=9; // собрать сообшение по байтам из программы
			HID_SendBuff(Buff, 8);
      } 
	
//**************************************
if (Buff[0]== 0x22) //write eeprom slop & intercept
		{
		flag_eprom_slop_intercept=1; // собрать сообшение по байтам из программы
		HID_SendBuff(Buff, 8);
	 		
		}		
		
if (Buff[0]== 0x21) //read eeprom intercept
			{
		   Flash_Read(2,out_data16,8);// address in word
				
			out_data[0]=out_data16[0];
			out_data[1]=out_data16[0]>>8;
			out_data[2]=out_data16[1];
			out_data[3]=out_data16[1]>>8;
			out_data[4]=out_data16[2];
			out_data[5]=out_data16[2]>>8;
			out_data[6]=out_data16[3];
			out_data[7]=out_data16[3]>>8;
				
			 HID_SendBuff(out_data,8);
			} 
if (Buff[0]== 0x23) //read eeprom slop
			{
		   Flash_Read(0,out_data16,8);// address in word
	uint8_t *ptr =	&out_data[0];		
			out_data[0]=out_data16[0];
			out_data[1]=out_data16[0]>>8;
			out_data[2]=out_data16[1];
			out_data[3]=out_data16[1]>>8;
			out_data[4]=out_data16[2];
			out_data[5]=out_data16[2]>>8;
			out_data[6]=out_data16[3];
			out_data[7]=out_data16[3]>>8;

 
double ddouble;
Menu.TemperStop=*(double*) out_data;
//printf(" %f   \n:", ddouble);
				
				
	HID_SendBuff(out_data,8);
				
			} 
if (Buff[0]== 0x24) // запись перемещения
{
flag_eprom_slop_intercept=17; // собрать сообшение по байтам из программы
		HID_SendBuff(Buff, 8);
	 
}		

if (Buff[0]== 0x26) //read eeprom intercept
			{
		   Flash_Read(6,out_data16,8);// address in word
				
			out_data[0]=out_data16[0];
			out_data[1]=out_data16[0]>>8;
			out_data[2]=out_data16[1];
			out_data[3]=out_data16[1]>>8;
			out_data[4]=out_data16[2];
			out_data[5]=out_data16[2]>>8;
			out_data[6]=out_data16[3];
			out_data[7]=out_data16[3]>>8;
				
			 HID_SendBuff(out_data,8);
			} 
if (Buff[0]== 0x25) //read eeprom slop
			{
		   Flash_Read(4,out_data16,8);// address in word
				
			out_data[0]=out_data16[0];
			out_data[1]=out_data16[0]>>8;
			out_data[2]=out_data16[1];
			out_data[3]=out_data16[1]>>8;
			out_data[4]=out_data16[2];
			out_data[5]=out_data16[2]>>8;
			out_data[6]=out_data16[3];
			out_data[7]=out_data16[3]>>8;
			
			 HID_SendBuff(out_data,8);
			} 



//-------------- Clear ---------------
if (Buff[0]== 0xcc) //clear
			{
	flag_eprom_slop_intercept=0;
				HID_SendBuff(Buff, 8);

			} 	



//------------ Power Hiter -----------------------------
if (Buff[0]== 0x50) //490ma
{
DS18b20.power_ten=880;
HID_SendBuff("0", 8);


} 	
if (Buff[0]== 0x51)//200ma
{

DS18b20.power_ten=440;
HID_SendBuff("0", 8);



} 	
if (Buff[0]== 0x52)
{

DS18b20.power_ten=380;
HID_SendBuff("0", 8);


} 
if (Buff[0]== 0x53)
{

DS18b20.power_ten=320;
HID_SendBuff("0", 8);



} 

if (Buff[0]== 0x54)
{

DS18b20.power_ten=290;
HID_SendBuff("0", 8);



} 
if (Buff[0]== 0x55)
{

DS18b20.power_ten=210;
HID_SendBuff("0", 8);



}
if (Buff[0]== 0x56)
{

DS18b20.power_ten=100;
HID_SendBuff("0", 8);



}
if (Buff[0]== 0x57)
{

DS18b20.power_ten=0;
HID_SendBuff("0", 8);



}

//-----------------------------------------	
	 if (Buff[0]== 0x40) // set 2^12
    {//	X4 X6 X5 
			//----------------X6----------------------
					  OW_Send(USART2,OW_SEND_RESET,  "\xcc\x4e\x56\x44\x7f\x10\xff\xff",8,0,0,0);// x7f =2^12
			delay(50);
			      OW_Send(USART2,OW_SEND_RESET,  "\xcc\x48",2,0,0,0); // recorg to eprom
				delay(50);
			 OW_Send(USART2,OW_SEND_RESET,  "\xcc\xb8\xff\xff\xff",5,0,0,0);// x7f =2^12
			    delay(10);
			 OW_Reset(USART2); //X6
			//-------------X5----------------------------
		     OW_Send2(USART1,OW_SEND_RESET,  "\xcc\x4e\x50\x40\x7f",5,0,0,0);// x7f =2^12
			delay(50);
			    OW_Send2(USART1,OW_SEND_RESET,  "\xcc\x48",2,0,0,0); // recorg to eprom
      delay(50);
			 OW_Send2(USART1,OW_SEND_RESET,  "\xcc\xb8\xff\xff\xff",5,0,0,0);// x7f =2^12
 OW_Reset2(USART1);
			//--------------X4------------
//		   OW_Send3(USART3,OW_SEND_RESET,  "\xcc\x4e\x45\x30\x7f",5,0,0,0);// x7f =2^12
//			delay(50);
		//	    OW_Send3(USART3,OW_SEND_RESET,  "\xcc\x48",2,0,0,0); // recorg to eprom
   //   delay(50);
//			 OW_Send3(USART3,OW_SEND_RESET,  "\xcc\xb8\xff\xff\xff",5,0,0,0);// x7f =2^12
 //OW_Reset3(USART3);/*	*/
			
			
			 HID_SendBuff("0", 8);
		
    }
	
	if (Buff[0]== 0x455) // recorg to eprom
    {//	GPIO_SetBits(OUT_C,LED);
					  OW_Send(USART2,OW_SEND_RESET,  "\xcc\xb8\xff\xff\xff",5,0,0,0);// x7f =2^12
			    delay(100);
        		OW_Send(USART2,OW_SEND_RESET,  "\xcc\x48",2,0,0,0); // recorg to eprom
			 OW_Reset(USART2);
			
			 HID_SendBuff("0", 8);
	//-------------------------------		
			  OW_Send2(USART1,OW_SEND_RESET,  "\xcc\xb8\xff\xff\xff",5,0,0,0);// x7f =2^12
			 delay(100);
		   OW_Send2(USART1,OW_SEND_RESET,  "\xcc\x48",2,0,0,0); // recorg to eprom
			 OW_Reset2(USART1);
	
			 HID_SendBuff("0", 8);
		
    }
		if (Buff[0]== 0x46) {
	//	GPIO_SetBits(OUT_C,LED);
		/*	OW_Send(USART2,OW_NO_RESET,  "\xcc\x44\xff\xff\xff\xff\xff\xff\xff\xff",10,0, 0, 0);
	 delay(50);
			OW_Send2(USART1,OW_NO_RESET,  "\xcc\x44\xff\xff\xff\xff\xff\xff\xff\xff",10,0, 0, 0);  
	 delay(50);
			OW_Send3(USART3,OW_NO_RESET,  "\xcc\x44\xff\xff\xff\xff\xff\xff\xff\xff",10,0, 0, 0);  			
	 delay(50);
			*/
				OW_Send(USART2,OW_NO_RESET,  "\xcc\x44",2,0, 0, 0);
	 delay(50);
			OW_Send2(USART1,OW_NO_RESET,  "\xcc\x44",2,0, 0, 0);  
	 delay(50);
	//		OW_Send3(USART3,OW_NO_RESET,  "\xcc\x44",2,0, 0, 0);  			
	// delay(50);	
			
			HID_SendBuff("0", 8);
 /*
			У термометра DS18B20 значение температуры по умолчанию - 85 градусов. 
			Т.е. когда он включается в памяти стоит занчение 85град. 
			Затем термометру по интерфайсу MicroLan выдаются команды 
			((сброс,выбор устройства,преобразование температуры);
			(сброс,выбор устройства,чтение памяти)).

Возможно:

1) термометру программой не была выдана  первая серия комад;
2) интервал времени между первой серией и второй слишком мал;
3) ему просто не хватает питания во время преобразования;
			
			*/
		
		}
		
		if (Buff[0]== 0x47) {
					OW_Send(USART2,OW_SEND_RESET,  "\xcc\x33",2,0, 0, 0);
	 delay(50);
					OW_Send(USART2,OW_NO_RESET,  "\xcc\x44",2,0, 0, 0);
	 delay(50);
			
			
			
		}
//------------41 get temperature  DS1 ---------------------------
	 if (Buff[0]== 0x41) // запрос ACD
    {	
			HID_SendBuff(DS18b20.Tmpr1, 8);
	
    }
	//------------42 get temperature  DS2 ---------------------------
	// if (Buff[0]== 0x42) // запрос ACD
  //  {	//GPIO_SetBits(OUT_C,LED);
	//		HID_SendBuff(DS18b20.Tmpr2, 8);
 //   }	
//------------43 get temperature  DS3 ---------------------------
//	 if (Buff[0]== 0x43) // запрос ACD
 //   {//	GPIO_SetBits(OUT_C,LED);
	//		HID_SendBuff(DS18b20.Tmpr3, 8);
 //   }	
	/*	else if (Buff[0]== 0x44) // запрос 
    {	GPIO_SetBits(OUT_C,LED);
			HID_SendBuff(DS18b20.Tmpr4, 8);
  }	*/

		
	//----------31 get address and  save in eprom-----------------------------
	    if (Buff[0]== 0x31) // запрос  
    {	//GPIO_SetBits(OUT_C,LED);
			GetAddressDS1820(DS18b20.Adr_ds1);
		
			 HID_SendBuff(DS18b20.Adr_ds1, 8);
			//GPIO_ResetBits(OUT_C,LED);
		
				
	 }
	 
	/*	  if (Buff[0]== 0x32) // запрос  
    {	//GPIO_SetBits(OUT_C,LED);
			GetAddressDS1820_2(DS18b20.Adr_ds2);
			
			HID_SendBuff(DS18b20.Adr_ds2, 8);
		//	GPIO_ResetBits(OUT_C,LED);
		
    }
		  if (Buff[0]== 0x33) // запрос  
    {//	GPIO_SetBits(OUT_C,LED);
		//	GetAddressDS1820_3(DS18b20.Adr_ds3);

			HID_SendBuff(DS18b20.Adr_ds3, 8);
		//	GPIO_ResetBits(OUT_C,LED);
		
    }
		  if (Buff[0]== 0x34) // запрос  
    {	//GPIO_SetBits(OUT_C,LED);
			GetAddressDS1820(DS18b20.Adr_ds4);
			//	delay(200);
			HID_SendBuff(DS18b20.Adr_ds4, 8);
	//		GPIO_ResetBits(OUT_C,LED);
		
    }*/
		
		//---------------------------------------
		  if (Buff[0]== 0x35) // record to eprom
    {	//GPIO_SetBits(OUT_C,LED);
	start_convert=1;
			eprcnt=1;
		//	send_cmd_convert_to_all();
		 		 OW_Reset(USART2); // отдать линию на приём
		 	OW_Send(USART2,OW_NO_RESET,  "\xcc\x44",2,0, 0, 0);// все датчики выполнят конвертация температуры
			
		
			 HID_SendBuff("0", 8);

		//	GPIO_ResetBits(OUT_C,LED);
		
    }
		if (Buff[0]== 0x36)
		{
	//	OW_Reset(USART2); // отдать линию на приём
 //     OW_Send(USART2,OW_NO_RESET,  "\xcc\x44",2,0, 0, 0);
//	delay_ms(2);
   OW_Reset(USART2); // отдать линию на приём
   OW_Send(USART2,OW_NO_RESET,  "\x55",1,0,0,0);// адресация конкретного датчи

	//  OW_Send(USART2,OW_NO_RESET,  give_adr,18,  tmp, 9,9); 
   OW_Send(USART2,OW_NO_RESET,  "\x28\x61\xb4\xb6\x0d\x00\x00\x32\xbe\xff\xff\xff\xff\xff\xff\xff\xff\xff",18,DS18b20.Tmpr1, 9,9);  //улица
	OW_Reset(USART2);
	   OW_Send(USART2,OW_NO_RESET,  "\xcc\x44",2,0, 0, 0);
	//	OW_Reset(USART2);
			
			
			HID_SendBuff(DS18b20.Tmpr1, 8);
		
		}
//** PT100 *******************
if (Buff[0]== 0xA1)	//midlle
	 {
	char ch[8];
  //  sprintf(ch,"%2.2f",PT100_temp1); 	
	 	HID_SendBuff(ch, 8);
	 }
 if (Buff[0]== 0xA2)	//top
	 {
	 char ch2[8];
 //   sprintf(ch2,"%2.2f",PT100_temp2); 	
	 	HID_SendBuff(ch2, 8);
	 }	 
if (Buff[0]== 0xA3)	//deown
	 {
	 char ch3[8];
//    sprintf(ch3,"%2.2f",PT100_temp3); 	
	 	HID_SendBuff(ch3, 8);
		 for(int t=0;t<=8;t++)
		 {
		 ch3[t]=0x00;
		 
		 }
	 }	 
	 

//**********************************		
}	
		
else if(flag_eprom_slop_intercept==1)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=2;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[0]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}
	
		
	else if(flag_eprom_slop_intercept==2)// for slop
			{	flag_eprom_slop_intercept=3;
					Prepare_Buf2WritEpr[1]=Buff[0]; // второе значение
					HID_SendBuff(Buff, 8);
			
			}		
		else if(flag_eprom_slop_intercept==3)// for slop
			{	flag_eprom_slop_intercept=4;
					Prepare_Buf2WritEpr[2]=Buff[0]; // третье значение
					HID_SendBuff(Buff, 8);
			
			}		
			else if(flag_eprom_slop_intercept==4)// for slop
			{	flag_eprom_slop_intercept=5;
					Prepare_Buf2WritEpr[3]=Buff[0]; // четвертое значение
				HID_SendBuff(Buff, 8);
			
			}			
				else if(flag_eprom_slop_intercept==5)// for slop
			{	flag_eprom_slop_intercept=6;
					Prepare_Buf2WritEpr[4]=Buff[0]; // четвертое значение
				HID_SendBuff(Buff, 8);
			
			}		
	else if(flag_eprom_slop_intercept==6)// for slop
			{	flag_eprom_slop_intercept=7;
					Prepare_Buf2WritEpr[5]=Buff[0]; // четвертое значение
				HID_SendBuff(Buff, 8);
			
			}			
			else if(flag_eprom_slop_intercept==7)// for slop
			{	flag_eprom_slop_intercept=8;
					Prepare_Buf2WritEpr[6]=Buff[0]; // четвертое значение
				HID_SendBuff(Buff, 8);
			
			}				
			 else if(flag_eprom_slop_intercept==8)// for slop
			{	flag_eprom_slop_intercept=9;
					Prepare_Buf2WritEpr[7]=Buff[0]; // четвертое значение
		//		Flash_Write(0,Prepare_Buf2WritEpr,8);	//0x0800F800
				HID_SendBuff(Buff, 8);
			
			}			
			//********************* intercept ****************
	   else if(flag_eprom_slop_intercept==9)//  
			{	flag_eprom_slop_intercept=10;
					Prepare_Buf2WritEpr[8]=Buff[0]; //  
					HID_SendBuff(Buff, 8);
			
			}	
			 else if(flag_eprom_slop_intercept==10)//  
			{	flag_eprom_slop_intercept=11;
					Prepare_Buf2WritEpr[9]=Buff[0]; //  
					HID_SendBuff(Buff, 8);
			
			}	
			else if(flag_eprom_slop_intercept==11)//  
			{	flag_eprom_slop_intercept=12;
					Prepare_Buf2WritEpr[10]=Buff[0]; // 
					HID_SendBuff(Buff, 8);
			
			}	
			else if(flag_eprom_slop_intercept==12)//  
			{	flag_eprom_slop_intercept=13;
					Prepare_Buf2WritEpr[11]=Buff[0]; // 
					HID_SendBuff(Buff, 8);
			
			}	
			else if(flag_eprom_slop_intercept==13)//  
			{	flag_eprom_slop_intercept=14;
					Prepare_Buf2WritEpr[12]=Buff[0]; // 
					HID_SendBuff(Buff, 8);
			
			}	
			else if(flag_eprom_slop_intercept==14)//  
			{	flag_eprom_slop_intercept=15;
					Prepare_Buf2WritEpr[13]=Buff[0]; // 
					HID_SendBuff(Buff, 8);
			
			}	
			else if(flag_eprom_slop_intercept==15)//  
			{	flag_eprom_slop_intercept=16;
					Prepare_Buf2WritEpr[14]=Buff[0]; // 
					HID_SendBuff(Buff, 8);
			
			}	
			else if(flag_eprom_slop_intercept==16)//  
			{
					Prepare_Buf2WritEpr[15]=Buff[0]; // 
					Flash_Write(0,Prepare_Buf2WritEpr,16);	//0x0800F800
				//Buff=0xcc;
				flag_eprom_slop_intercept=0;
					HID_SendBuff(Buff, 8);
				
			}	
			
	//***************************************************		
		else if(flag_eprom_slop_intercept==17)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=18;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[0]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
		else if(flag_eprom_slop_intercept==18)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=19;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[1]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}		
		else if(flag_eprom_slop_intercept==19)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=20;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[2]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}		
		else if(flag_eprom_slop_intercept==20)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=21;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[3]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
		else if(flag_eprom_slop_intercept==21)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=22;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[4]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
		else if(flag_eprom_slop_intercept==22)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=23;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[5]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
		else if(flag_eprom_slop_intercept==23)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=24;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[6]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
			else if(flag_eprom_slop_intercept==24)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=25;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[7]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}			
			//-------------------------------------------
	  	else if(flag_eprom_slop_intercept==25)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=26;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[8]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
	  	else if(flag_eprom_slop_intercept==26)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=27;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[9]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}				
	  	else if(flag_eprom_slop_intercept==27)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=28;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[10]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
			else if(flag_eprom_slop_intercept==28)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=29;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[11]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
			else if(flag_eprom_slop_intercept==29)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=30;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[12]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
			else if(flag_eprom_slop_intercept==30)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=31;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[13]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
			else if(flag_eprom_slop_intercept==31)//for slop if was cdm 0x22
			{			
				flag_eprom_slop_intercept=32;//	TIM_Cmd(TIM4, DISABLE);
				Prepare_Buf2WritEpr[14]=Buff[0];// перое значение
				HID_SendBuff(Buff, 8);	
			
			}	
			else if(flag_eprom_slop_intercept==32)//for slop if was cdm 0x22
			{			
			
				Prepare_Buf2WritEpr[15]=Buff[0];// перое значение
					Flash_Write(4,Prepare_Buf2WritEpr,16);	//0x0800F800 +4
				flag_eprom_slop_intercept=0; 
				HID_SendBuff(Buff, 8);	
			
			}	
		 
			
	//*****************************************************		
			else 
				{	HID_SendBuff(Buff, 8);
			flag_eprom_slop_intercept=0;
			}
			
			
			
	
}
//-----------------------------------------------------------------------------------------------
//************************************************************************************************
//****************************  MAIN *************************************************************
//************************************************************************************************
//************************************************************************************************
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
int main(void)
{
	 HSE_SetSysClk( RCC_PLLMul_9 );
	 InitTim4();
 // TIM3_PWM_Init22(1999,719);//(100,719)=100*0,000010= 1 ms
  DS18b20.power_ten= 1990;	
//	TIM_SetCompare2(TIM3,DS18b20.power_ten);
//	TIM_SetCompare1(TIM3,DS18b20.power_ten/2);
	 DS18b20.power_ten= 1999;	

	init_adc_PA3();

	OW_Init(USART2);// one-wire для ds18b20  PA2

	SystemCoreClockUpdate();
			SysTim_Init(10);
			Set_System();
	
			USB_Interrupts_Config();
			Set_USBClock();
			USB_Init();/**/
//	USB_Init();/**/
	delay(5000);
	
	  spi2_init();
  gpio_init();	
//PWM_TIM3_Init222();
/* WDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
IWDG_SetPrescaler(IWDG_Prescaler_32); //делитель на 32 =1кгц
IWDG_SetReload(1000); // период
IWDG_ReloadCounter(); // сброс счетчика
IWDG_Enable(); //вкл WatchDog

//сброс счетчика в программе

 */
 	
//*************************************************************
 
volatile char buffer[50] = {'\0'};
 
//----------------------------------------
start_convert=1;
 
 
//******************* 240524 ******************
unsigned char reg0 ,reg1, reg2,reg3  ;
  
	volatile float Rtd_Value;
    volatile float temp;
 
#ifdef DS1306	 
//BlueON
    	ssd1306_I2C_Init();
      ssd1306Init();
//BlueOff
#endif 

int volatile cnt=0;
/*
Например, если задать 720 – 1 = 719, то частота после предделителя будет 72 000 000 / 720 = 100 000 Гц, или период 10 мкс.
Если в регистр перезагрузки задать значение 50 000, то получим требуемый период 0,5 секунд.
*/
encoder_real=0;
 


//DS18b20.power_ten= 1990;	
// 	TIM3_PWM_Init2(1999,719);
//		PWM_TIM3_Init222();
//delay_ms(10000);
Menu.TemperStop=-1.0;	//test
Menu.SetHisteresis=0.1;
Menu.SetPWR=2000;
activ_menu=TRUE;
//flag_wait=0;
main_menu=0;//in interup PA4
maxmin[cnt_points][0]=-33;
maxmin[cnt_points][1]=33;
//****************************************************************************
//****************************************************************************
//****************************************************************************
//
//IWDG_Init(IWDG_Prescaler_32,5000);
// IWDG_ReloadCounter();
//IWDG_Enable();
uint32_t offSet=0;
int mode=0;
int resultat;
	
	_Bool reset =FALSE;
	_Bool Run=TRUE,  start;
	int iStart;
	
extern uint8_t Rotate;
extern int Counter;
Calculated_Tstop =	Menu.TemperStop;
flag_wait=5;
 	_min=-10;
 	_max=-10;
	

	

	starter_second=0;
	Enbl_Starter=TRUE;
	Switch_Programm=2;
	SW0=TRUE;SW1=FALSE;
	SW8=FALSE;
	SW6=TRUE;
	SW7=FALSE;
	Limit_delt=0.1;
cnt_SW0=0;
BlueON;
while(1)
	  { //	IWDG_ReloadCounter();
if(	main_menu==0)	
	             //****************************
							 if(activ_menu)
										 {flag_wait++;
											 
		        					  /*	if(SW1==TRUE)
															{  Str=  Starter(adc_pa1,Str);
																Stabilizacij(&CurrentTenmper,&Menu.TemperStop,&Limit_delt);
														   
															}
														 else if(SW0==TRUE)	
															*/	{ 
																	// Str=  Starter(adc_pa1,Str);
															//	if(SW7==FALSE)
	                              //   	MonitorTEMPER_DS();
													//       search_polka2(&CurrentTenmper, &Menu.TemperStop, &Polka, &Limit_delt,&iTime_hold_target,&SW0);
//****************************//amp(&CurrentTenmper, &Menu.TemperStop,  &Limit_delt);
						    
															    }/*	*/
												//	TestStarter(adc_pa1);			
														
	                  	 /*		if(flag_wait>= 30)
												 {
												 	flag_wait=0;	
												 
												 
												 }			
										
										  if(SW0==1)
													 {
														 OLED_DrawStr("gjbcr N",85,15,1);
															 OLED_DrawRectangleFill(80,25,85+flag_wait,30,1  );
														  OLED_UpdateScreen(); 
														 	
													 }
												else if((SW1==1)&& (SW0==0))
													 { 
														  OLED_DrawStr("wtkm yfqltyf",80,15,1);
														 OLED_DrawCircleFill(100,27,4);
													  OLED_UpdateScreen(); 
													 
													 }
													*/ 
													 
												
											 
											  
											 activ_menu==1;
											// activ_menu=FALSE;
										 
										}   
							//****************************************************************			 
							 else
											 { //  flag_wait++;//задержка для изменения температуры
										//	 displ2();  
						
							 					 
										//	 displey(90, 50,dif,7)	;
										//		  displey(90, 40,modul,7)	;
									//	   SSD1306_UpdateScreen();		/**/	

						 
												 activ_menu=TRUE;
										 
						        	 }
											 
								/*			 if(cnt_Led >=4)
											  cnt_Led=0;
									 if(cnt_Led== 3) 
											 { RedON
												 GreenON
												 BlueON
											 }		 
											 if(cnt_Led== 2) 
											 { RedON
												 GreenOff
												 BlueOff
											 }
							   if(cnt_Led==1)
											 {
											 GreenON
												 RedOff
												 BlueOff
											 }
									if(cnt_Led==0)
											 {BlueON
											 GreenOff
												 RedOff
											 }*/
              //*******************************************
if(main_menu>0){
	main_menu=0;// TEST
	start=TRUE;
   menu11(main_menu,encoder);
	start=FALSE;/**/
	}
}


}
 
void displey(int X, int Y,double volue,int sizefont)
{
	char buffer[10];
	 sprintf((char*)buffer,"%g",volue);	
/*	if(volue==0)
	 snprintf(buffer, sizeof buffer, "%c",'@');	
		else
   snprintf(buffer, sizeof buffer, "%d", volue);*/
	
//		SSD1306_Fill(SSD1306_COLOR_BLACK);	
	SSD1306_GotoXY(X, Y);
	if(sizefont==7)
SSD1306_Puts((char*)buffer, &Font_7x10, SSD1306_COLOR_WHITE);
	else	if(sizefont==11)
SSD1306_Puts((char*)buffer, &Font_11x18, SSD1306_COLOR_WHITE);
	else		if(sizefont==16)
SSD1306_Puts((char*)buffer, &Font_16x26, SSD1306_COLOR_WHITE);
	else;
	
SSD1306_UpdateScreen();	
}
 

 // ch: значение канала 0 ~ 3100
u16 Get_Adc(u8 ch)
{
  	   // Устанавливаем обычный групповой канал указанного АЦП, последовательность и время выборки
	
//if(ch==0)
	ADC_RegularChannelConfig (ADC1, ch, 1, ADC_SampleTime_28Cycles5); // ADC1, канал ADC, время выборки 239,5 цикла	  			    
//if(ch==1)
//ADC_RegularChannelConfig (ADC1, ch, 2, ADC_SampleTime_28Cycles5);//ADC_SampleTime_239Cycles5
 delay(10);
	ADC_SoftwareStartConvCmd (ADC1, ENABLE); // Включить функцию запуска программного преобразования указанного ADC1	
 delay(10);	
	// while (! ADC_GetFlagStatus (ADC1, ADC_FLAG_EOC)); // Ожидаем окончания преобразования
 
	 return ADC_GetConversionValue (ADC1); // Возвращаем последний результат преобразования группы правил ADC1
}

double ds18b20_read_temp(char ds_addr[8]) {
    uint8_t matchrom[9];
    matchrom[0] = 0x55;
    for (int i = 0; i < 8; ++i)
        matchrom[i + 1] = ds_addr[i];

	   	  OW_Send(USART2,OW_SEND_RESET,  "\xcc\x4e\x56\x44\x7f\x10\xff\xff",8,0,0,0);// x7f =2^12
			delay(50);
			      OW_Send(USART2,OW_SEND_RESET,  "\xcc\x48",2,0,0,0); // recorg to eprom
				delay(50);
			 OW_Send(USART2,OW_SEND_RESET,  "\xcc\xb8\xff\xff\xff",5,0,0,0);// x7f =2^12
			    delay(10);
	
    OW_Reset(USART2);
    OW_Send(USART2, OW_NO_RESET, matchrom, 9, 0, 0, 0);
    uint8_t convT = 0x44;
    OW_Send(USART2, OW_NO_RESET, &convT, 1, 0, 0, 0);

    delay(750);

    // 2. Reset, MatchROM, Read Scratchpad
    OW_Reset(USART2);
    OW_Send(USART2, OW_NO_RESET, matchrom, 9, 0, 0, 0);
    uint8_t read_scratchpad = 0xBE;
    uint8_t scratchpad[9];
    OW_Send(USART2, OW_NO_RESET, &read_scratchpad, 1, scratchpad, 9, 9);

    // Проверка связи: если весь буфер 0xFF, значит нет ответа
    int valid = 0;
    for (int i = 0; i < 9; ++i) if (scratchpad[i] != 0xFF) valid = 1;
    if (!valid) return -1000.0; // или другое спец. значение ошибки

    // Переводим результат (2 байта температуры) в градусы
    int16_t raw = (scratchpad[1] << 8) | scratchpad[0];
    double temp = raw / 16.0;
    return temp;
}

//void MonitorTEMPER_DS(void) {
//	double temp = ds18b20_read_temp(DS18b20.Adr_ds1);
//	if (temp > -100.0 && temp < 125.0)
//	{
//		CurrentTenmper = temp; // CurrentTenmper это переменная использовалась для вывода на дисплей
//			//DS18b20.current_temp = temp;
//	} else {
//		volatile int fake = 0;
//		(void)fake;
//	}
//}

double getTemperature(uint8_t* buf) {
    int16_t raw = ((int16_t)buf[1] << 8) | buf[0]; // склеиваем два байта
    return raw * 0.0625; // каждый шаг — 1/16 градуса
}

void MonitorTEMPER_DS(void)
{
		
			GetAddressDS1820(DS18b20.Adr_ds1);
   	  OW_Send(USART2,OW_SEND_RESET,  "\xcc\x4e\x56\x44\x7f\x10\xff\xff",8,0,0,0);// x7f =2^12
			delay(50);
			OW_Send(USART2,OW_SEND_RESET,  "\xcc\x48",2,0,0,0); // recorg to eprom
			delay(50);
			OW_Send(USART2,OW_SEND_RESET,  "\xcc\xb8\xff\xff\xff",5,0,0,0);// x7f =2^12
			delay(10);
			OW_Reset(USART2); //X6
			GetMessureDS1820_adr2(DS18b20.Adr_ds1,DS18b20.Tmpr1,1);
	
			double currentTemperature = getTemperature(DS18b20.Tmpr1);
			CurrentTenmper = currentTemperature;
			updateControl();
			//if((DS18b20.Tmpr1[0] != 0xff))  /*&& (DS18b20.Tmpr1[1] != 0xff))&& (DS18b20.Tmpr1[2] != 0xff)) &&  (DS18b20.Tmpr1[3] != 0xff))*/
			//{
					
				
				//memcpy(&DS18b20.PrevTmpr1,&DS18b20.Tmpr1,8);	
				//gettemp(DS18b20.Tmpr1,2);//buffer_temp2
				//memcpy(&DS18b20.Tmpr1,&DS18b20.PrevTmpr1,8);	// иначе глюк с отправкой с перевернутым минусом через раз 
			//}
			//else
				//memcpy(&DS18b20.Tmpr1,&DS18b20.PrevTmpr1,8);			
			//CurrentTenmper=	convert_2_double(buffer_temp2);			
	// сделать три датчика ds2 ds3
}	
/*int CompareRealPrevius(char prev[], char real[])
{
	 int16_t pr,rl,dif;
	pr = prev[0]<<8;
	pr |= prev[1];
	
	rl = real[0]<<8;
	rl |= real[1];
	
	if(rl==0)
		return -1;
	else 
		return 1;	
 
	
}*/
//*******************************************
void HSI_SetSysClk( uint32_t RCC_PLLMul_x )
{
	__IO uint32_t HSIStatus = 0;
	
	 // Сбрасываем регистр RCC на значение сброса
	RCC_DeInit();	
 
	 // включить HSI 
	RCC_HSICmd(ENABLE);
//	RCC_HSECmd(ENABLE);
	
	HSIStatus = RCC->CR & RCC_CR_HSIRDY;

	
	if( HSIStatus == RCC_CR_HSIRDY )
	{
		 // включить предварительную выборку
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);
		
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		
		 // Настройка PLLCLK = HSE * RCC_PLLMul_x
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_x); //8/2 *12 =4*12 =48mhz
		
         // включить PLL
		RCC_PLLCmd(ENABLE);
		
		 // Ожидание стабилизации PLL
		while( RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );
		
         // выбираем системные часы
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		
    while( RCC_GetSYSCLKSource() != 0x08 );
	}
	else
  {
		 /* Если HSI не запускается, пользователь может добавить код обработки ошибок здесь */
	}
}
//--------------------------------------------------------------
// HSE настроить системные часы
void HSE_SetSysClk( uint32_t RCC_PLLMul_x )
{
	ErrorStatus HSEStatus;
	
	 // Сбрасываем регистр RCC на значение сброса
	RCC_DeInit();	
 
	 // включить HSE 
	RCC_HSEConfig(RCC_HSE_ON);
	
	HSEStatus = RCC_WaitForHSEStartUp();
	
	if( HSEStatus == SUCCESS )
	{
		 // включить предварительную выборку
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);
		
		RCC_HCLKConfig(RCC_SYSCLK_Div2);//8/2
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		
		 // Настройка PLLCLK = HSE * RCC_PLLMul_x
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_x);
		
         // включить PLL
		RCC_PLLCmd(ENABLE);
		
		 // Ожидание стабилизации PLL
		while( RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );
		
         // выбираем системные часы
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		
    while( RCC_GetSYSCLKSource() != 0x08 );
	}
	else
  {
		 /* Если HSE не запускается, пользователь может добавить код обработки ошибок здесь */
	}
}
void HSE_SetSysClk_4mhz( uint32_t RCC_PLLMul_x )
{
	ErrorStatus HSEStatus;
	
	 // Сбрасываем регистр RCC на значение сброса
	RCC_DeInit();	
 
	 // включить HSE 
	RCC_HSEConfig(RCC_HSE_ON);
	
	HSEStatus = RCC_WaitForHSEStartUp();
	
	if( HSEStatus == SUCCESS )
	{
		 // включить предварительную выборку
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);
		
		RCC_HCLKConfig(RCC_SYSCLK_Div1);//4mhz
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		
		 // Настройка PLLCLK = HSE * RCC_PLLMul_x
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_x);
		
         // включить PLL
		RCC_PLLCmd(ENABLE);
		
		 // Ожидание стабилизации PLL
		while( RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );
		
         // выбираем системные часы
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		
    while( RCC_GetSYSCLKSource() != 0x08 );
	}
	else
  {
		 /* Если HSE не запускается, пользователь может добавить код обработки ошибок здесь */
	}
}
//----------------- настройка частоты контроллера и тактирования ---------------------
void RCC_Configuration_internal(void)
{
/*    ErrorStatus HSEStartUpStatus;
   // RCC system reset(for debug purpose) 
    RCC_DeInit();

RCC_HSICmd(ENABLE);
RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
//RCC_PLLConfig(RCC_PLLSource_HSI,8,192,6,8); //ABP1 ABP2 64MHZ SYSTEMCLOCK=64MHZ
RCC_PLLConfig(RCC_PLLSource_HSI,8,192,8,8); //ABP1 ABP2 48MHZ SYSTEMCLOCK=48MHZ
RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
RCC_PLLCmd(ENABLE);*/
/*RCC_HSEConfig(RCC_HSE_ON);//Включаем HSE
RCC_WaitForHSEStartUp();//Ждем пока HSE будет готов
RCC_HCLKConfig(RCC_SYSCLK_Div1);
// Общий делитель(AHB Prescaler) периферии равен 1, на всякий случай
RCC_PCLK2Config( RCC_HCLK_Div1);//APB2 делитель равен 1, на всякий случай
RCC_PCLK1Config( RCC_HCLK_Div2);//APB1 делитель равен 2, максимальная частота APB1 36Мгерц
FLASH_SetLatency(FLASH_Latency_2);//Пропускаем два такта, для Flash
FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);// Доступен предварительный буфер
//RCC_PREDIV1Config( RCC_PREDIV1_Source_HSE, RCC_PREDIV1_Div1);//HSE источник, делитель 1

	RCC_PLLConfig( RCC_SYSCLKSource_PLLCLK, RCC_PLLMul_6); //SYSCLK=8Мгерц*6=48Мгерц
RCC_PLLCmd(ENABLE);//Доступ к PLL разрешен

//while(RCC_GetFlagStatus( RCC_FLAG_PLLRDY) == RESET);//Ждем пока PLL не будет готов
RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK); // Выбираем PLL как источник частоты
while(RCC_GetSYSCLKSource() != 0x08);// Ждем пока PLL не станет источником
RCC_HSICmd(DISABLE);// Выключаем HSI


*/

RCC_DeInit(); //    сброс настроек тактового генератора
RCC_HSEConfig(RCC_HSE_OFF); //  отключение внешнего тактового генератора
 // включить HSI 
	RCC_HSICmd(ENABLE);
RCC_PCLK2Config( RCC_HCLK_Div1);//APB2 делитель равен 1, на всякий случай 48
RCC_PCLK1Config( RCC_HCLK_Div1);//APB1 делитель равен 2, максимальная частота APB1 24Мгерц

//RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_4); //  тактирование от HSI с делителем 2: 8 / 2 * 4 = 16МГц
RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_12); //  тактирование от HSI с делителем 2: 8 / 2 * 12 = 48МГц

RCC_PLLCmd(ENABLE); //  Включаем PLL
RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //   На системную шину подаем тактирование с множителя PLLMUL
  
}
//------------------------------------
void TIM4_IRQHandler()
{
//	if(SW7==FALSE)
		MonitorTEMPER_DS();
	
	cnt_adjust++;
// cnt_work_pump++;
	cnt_run++;
	cnt_polka++;
//	cnt_target_polka++;
//	cnt_Led++;
////	cnt_work_pump_tstop++;
	starter_sec++;
//if(cnt_second>1000) cnt_second=0;
//	cnt_second++;
  //-------------------------------
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);


}
/*void TIM2_IRQHandler()
{

	if(TIM2->SR & TIM_SR_TIF)
		{
	//	TIM12->CCR1 = 25 + TIM2->CNT;
//		encoder=TIM2->CNT;
		
	}

//	TIM2->SR &= ~TIM_SR_TIF;		
  //-------------------------------
//	  EXTI_ClearITPendingBit(TIM2_IRQn);
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);


}*/
/*
//----------------------------------------------------------
//TIM3 PWM partial initialization 83.3kHz
 //PWM output initialization
 //arr: automatic reload value
 //psc: clock prescaler number
/*void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Enable timer 3 clock
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //Enable GPIO peripheral and AFIO multiplexing function module clock
 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
	 GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;//GPIO_Mode_AF_OD; //Multiplex push-pull output
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);//Initialize GPIO
	 
			 
	 // период между импульсами 1000=10ms
	 TIM_TimeBaseStructure.TIM_Period = arr; //Set the value of the automatic reload register period that is loaded in the next update event

uint16_t PrescalerValue = 0;
PrescalerValue = (uint16_t) (SystemCoreClock / 72000000) - 1;


   TIM_TimeBaseStructure.TIM_Prescaler =psc; //PrescalerValue;Set the prescaler value used as the divisor of the TIMx clock frequency 

 
TIM_TimeBaseStructure.TIM_ClockDivision = 0; //Set clock division: TDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM up counting mode
	 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //Initialize the time base unit of TIMx according to the parameters specified in TIM_TimeBaseInitStruct
	
	 //Initialize TIM3 Channel2 PWM mode	 
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;// TIM_OCMode_PWM2; //Select timer mode: TIM pulse width modulation mode 2
 	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Comparison output enable
		TIM_OCInitStructure.TIM_Pulse = 0;//arr/3;
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_High; //Output polarity: TIM output is relatively high
	//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;


    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //Enable the preload register of TIM3 on CCR2
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); //Initialize the peripheral TIM3 OC2 according to the parameters specified by T
  
 
	 TIM_Cmd(TIM3, ENABLE); //Enable TIM3
	
 
}
*/ 



void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
 

 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //Enable GPIO peripheral and AFIO multiplexing function module clock
		 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Enable timer 3 clock
	 GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3 partial remap TIM3_CH2->PB5    
 
       //Set this pin as a multiplexed output function and output the PWM pulse waveform of TIM3 CH2 GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
	 GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;//GPIO_Mode_AF_OD; //Multiplex push-pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);//Initialize GPIO
 
       //Initialize TIM3
	 TIM_TimeBaseStructure.TIM_Period = arr; //Set the value of the automatic reload register period that is loaded in the next update event
	 TIM_TimeBaseStructure.TIM_Prescaler =psc; //Set the prescaler value used as the divisor of the TIMx clock frequency 
	 TIM_TimeBaseStructure.TIM_ClockDivision = 0; //Set clock division: TDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM up counting mode
	 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //Initialize the time base unit of TIMx according to the parameters specified in TIM_TimeBaseInitStruct
	
	 //Initialize TIM3 Channel2 PWM mode	 
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //Select timer mode: TIM pulse width modulation mode 2
 	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Comparison output enable
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //Output polarity: TIM output is relatively high
	 TIM_OC2Init(TIM3, &TIM_OCInitStructure); //Initialize the peripheral TIM3 OC2 according to the parameters specified by T
 
	 TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //Enable the preload register of TIM3 on CCR2
 
	 TIM_Cmd(TIM3, ENABLE); //Enable TIM3
	
 
}

void TIM3_PWM_Init2(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
 
	
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //Enable GPIO peripheral and AFIO multiplexing function module clock
	 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Enable timer 3 clock
   
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
	 GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;//GPIO_Mode_AF_OD; //Multiplex push-pull output
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3 partial remap TIM3_CH2->PB5
	 GPIO_Init(GPIOA, &GPIO_InitStructure);//Initialize GPIO

		       //Set this pin as a multiplexed output function and output the PWM pulse waveform of TIM3 CH2 GPIOB.5
 /*	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
	 GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;//GPIO_Mode_AF_OD; //Multiplex push-pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);//Initialize GPIO
		*/
	 // период между импульсами 1000=10ms
	 TIM_TimeBaseStructure.TIM_Period = arr; //Set the value of the automatic reload register period that is loaded in the next update event
   TIM_TimeBaseStructure.TIM_Prescaler =psc; //PrescalerValue;Set the prescaler value used as the divisor of the TIMx clock frequency 
 // период между импульсами 1000=10ms
//	 TIM_TimeBaseStructure.TIM_Period = arr; //Set the value of the automatic reload register period that is loaded in the next update event
// Установка скважности 50%:

 
TIM_TimeBaseStructure.TIM_ClockDivision = 0; //Set clock division: TDTS = Tck_tim
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM up counting mode

TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //Initialize the time base unit of TIMx according to the parameters specified in TIM_TimeBaseInitStruct
  
 
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;// TIM_OCMode_PWM2; //Select timer mode: TIM pulse width modulation mode 2
 	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Comparison output enable
		//TIM_OCInitStructure.TIM_Pulse = 0;//1000-1;//arr/3;
	  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High; //TIM_OCPolarity_Low;//TIM_OCPolarity_Low;////TIM_OCPolarity_Low;//TIM_OCPolarity_High; //Output polarity: TIM output is relatively high
	//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_High;

    TIM_OC2Init(TIM3, &TIM_OCInitStructure); //Initialize the peripheral TIM3 OC2 according to the parameters specified by T
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //Enable the preload register of TIM3 on CCR2
	 TIM_Cmd(TIM3, ENABLE); //Enable TIM3
	
 
}
void TIM3_PWM_Init22(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Enable timer 3 clock
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //Enable GPIO peripheral and AFIO multiplexing function module clock
	

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
	 GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;//GPIO_Mode_AF_OD; //Multiplex push-pull output
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);//Initialize GPIO
	 
			 
	 // период между импульсами 1000=10ms
	 TIM_TimeBaseStructure.TIM_Period = arr; //Set the value of the automatic reload register period that is loaded in the next update event
   TIM_TimeBaseStructure.TIM_Prescaler =psc; //PrescalerValue;Set the prescaler value used as the divisor of the TIMx clock frequency 
 // период между импульсами 1000=10ms
//	 TIM_TimeBaseStructure.TIM_Period = arr; //Set the value of the automatic reload register period that is loaded in the next update event
// Установка скважности 50%:

 
TIM_TimeBaseStructure.TIM_ClockDivision = 0; //Set clock division: TDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM up counting mode
	 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //Initialize the time base unit of TIMx according to the parameters specified in TIM_TimeBaseInitStruct
 
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;// TIM_OCMode_PWM2; //Select timer mode: TIM pulse width modulation mode 2
 	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Comparison output enable
		TIM_OCInitStructure.TIM_Pulse = 0;//1000-1;//arr/3;
	  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_Low;//TIM_OCPolarity_High; ////TIM_OCPolarity_Low;////TIM_OCPolarity_Low;//TIM_OCPolarity_High; //Output polarity: TIM output is relatively high
	  TIM_OCInitStructure.TIM_OCNPolarity =TIM_OCPolarity_High; //TIM_OCPolarity_Low;//
  
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //Enable the preload register of TIM3 on CCR2
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); //Initialize the peripheral TIM3 OC2 according to the parameters specified by T
 
	 TIM_Cmd(TIM3, ENABLE); //Enable TIM3
	
 
}
void TIM3_PWM_Init3(u16 arr,u16 psc)
{ // установка начальных значений шима
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t CCR1_Val = 333;
  uint16_t CCR2_Val = 249;
  uint16_t PrescalerValue = 0;

   // разрешить клокирование ШИМа
  /* System Clocks Configuration */
 /* TIM3 clock enable */ // разрешить клокирование TIM3 и альтернативного ввода-
                          // вывода на порт B
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  // установить кофигурацию штырей ввода-вывода для ШИМа
  /* GPIO Configuration */
 GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;//6|7
  GPIO_Init(GPIOB, &GPIO_InitStructure);	 
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3 partial remap TIM3_CH2->PB5    
  // инициализация TIM3 режиме ШИМ целиком выдрана из примеров к стандартной
  // библиотеке STM32  используются 2 из 4х каналов ШИМ
  /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 72 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM3 is running at  KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 72 MHz / 1800 = 40 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / 72000000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 1799;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 /* TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
*/
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

//--------------------------------------------------------------------------------------
void send_cmd_convert_to_all(void)
{
OW_Init(USART2);
 OW_Send(USART2,OW_SEND_RESET, "\xcc\x44", 2, NULL, NULL, OW_NO_READ);//\xfff\xff\xff\xff\xff\xff\xff\xff
	 OW_Reset(USART2); 

delay(750);

}

int GetAddressDS1820(char TMP[])
{

//ничего не трогать
OW_Init(USART2);
 OW_Send(USART2,OW_SEND_RESET, "\xcc\x44", 2, NULL, NULL, OW_NO_READ);//\xfff\xff\xff\xff\xff\xff\xff\xff
	 OW_Reset(USART2); // отдать линию на приём
               OW_Send(USART2,OW_NO_RESET,  "\x33\xff\xff\xff\xff\xff\xff\xff\xff\xff",10,  TMP,9,1);
	 OW_Reset(USART2); 
return *TMP;
}
 
//--------------------------------------------------------------------------------------
char give_adr[18];
char GetMessureDS1820_adr2(char *adr,char *tmp, int num)
{
char ttmp[10];
int i;
for(i=0;i<8;i++)
{
  give_adr[i] = adr[i];
}	

//give_adr[8] =0xCC;
give_adr[8] =0xBE;
for(i=9;i<=17;i++)
{
  give_adr[i] = 0xff;
}	
if(num==1)//usart2
	{
	
		
OW_Reset(USART2); // отдать линию на приём
    
 delay(1800);
  // отдать линию на приём
   OW_Send(USART2,OW_NO_RESET,  "\x55",1,0,0,0);// адресация конкретного датчи
		 delay(100);
	  OW_Send(USART2,OW_NO_RESET,  give_adr,16,  tmp, 8,9);
	
	   OW_Send(USART2,OW_SEND_RESET,  "\xcc\x44\xff",3,0, 0, 0);
		 delay(800);
  
		
	}
 

if(tmp[0]!=0)
    return *adr;
 else 
	 return 0x30;

// fTemp = (float)(rom[0] + (rom[1]<<8))/16;
}
//-------------------------------------
int Get_Temperature(int type)
{UWORD Temper,Temper_Half;
UWORD TMP, SIGN;
	
		/*
	   [0]   [1]
	0x 8 0   0 1
	   | |   | |
	   | |   | |
	   | |   | |_______________ 0x10 =16       4    <<   0x01
	   |_|___|_________________ 0x08 =8  +     0x80 >> 4 0x08
       |   |                    -----------
       |   |                   0x18 =>	24
	     |   |
	     |   |_________________ sign  0 -> +   f-> -
       |______________________________________0x01=0.0625
			                                        0x02=0.125
																							0x04=0.250
																							0x08=0.5

/////////////////////////////////////////////////////////////




j=	0x0017 =22  +22
    0x1017 =22  -22 -> d[0]
    0x0100 =0.5     -> d[4]
	
/////////////////////////////////////////////////////
	if( j >= 10 && j<=19)            d[1] = 0x31   "1"
	         j2=j-10                 d[3] = j2+0x30
	//------------------------------------
	if(j>=20  && j <=29)             d[1] = 0x32    "2"
	         j2=j-20                 d[3] = j2+0x30
					 
	if(j>=30 && j <=39)              d[1] = 0x33    "3"
	         j2= j-30                d[3] = j2+0x30
												 
//---------------------------------------------------

Sing:
 */
	
	
	
if(type ==1)
{
TMP=get_buf[0]; // принятое значение температуры
SIGN=get_buf[1];// знак температуры
//----------------------------------------------------------------
//-------------- минусовая температура формат 0х0|знак '-'| 0.5C |MLB_t|SLB_t|
if(SIGN == 0xff)
   {
   TMP=TMP-0x01;    // корректировка на пол градуса
   TMP=(~TMP)&0xff; // инверсия выйти в диапазон положительных чисел

        if(0x01==(TMP&0x01))
              { TMP=TMP>>1;// сдвиг в младшею сторону  разделить на 2
                temper_dec= (TMP | 0x1100); // приклеить знак и пол градуса
              }
        else  { TMP=TMP>>1;  temper_dec= (TMP | 0x1000);}
       return temper_dec;
   }
//---------------------------------------------------
//-------------- плюсовая температура формат 0х0|знак '0'| 0.5C |MLB_t|SLB_t|
if(SIGN >= 0x00)
   {
    if(0x01==(TMP&0x01)) // присутствует пол градуса
              { TMP=TMP>>1;// сдвиг в младшею сторону
                temper_dec= TMP | 0x0100;

              }
        else  { TMP=TMP>>1; temper_dec= TMP | 0x0000; }
    return temper_dec;

 }

}
//-------------------------------------------------------
if(type==2)
{
TMP=get_buf[0]; // принятое значение температуры
SIGN=get_buf[1];// знак температуры
Temper=SIGN<<8;
Temper_Half=Temper+TMP;
Temper=Temper_Half>>4;
//----------------------------------------------------------------
//-------------- минусовая температура формат 0х0|знак '-'| 0.5C |MLB_t|SLB_t|
if(SIGN >= 0xf0)
   {
   TMP=TMP-0x01;    // корректировка на пол градуса
   TMP=(~TMP)&0xff; // инверсия выйти в диапазон положительных чисел

        if(0x01==(TMP&0x01))
              { //TMP=TMP>>1;// сдвиг в младшею сторону  разделить на 2
                temper_dec= (Temper | 0x1100); // приклеить знак и пол градуса
              }
        else  { //TMP=TMP>>1;
                 temper_dec= (Temper | 0x1000);}
       return temper_dec;
   }
//---------------------------------------------------
//-------------- плюсовая температура формат 0х0|знак '0'| 0.5C |MLB_t|SLB_t|
else if(SIGN <= 0xe0)
   {
    if(0x01==(Temper_Half&0x01)) // присутствует пол градуса
              { //TMP=TMP>>1;// сдвиг в младшею сторону
                temper_dec= Temper | 0x0100;

              }
        else  { //TMP=TMP>>1;
                   temper_dec= Temper | 0x0000; }
    return temper_dec;

 }
}


return -1;
}
//-------------------------------------------------------------------------------

//*****************************
void spi2(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
	
  // Тактирование модуля SPI и порта 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
 // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  //stm32f407
  // Настраиваем ноги SPI1 для работы в режиме альтернативной функции
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.  GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure .GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13  | GPIO_Pin_15;//  | GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//GPIOB->CRH &= ~(GPIO_CRH_CNF14);//floating
//	GPIOB->CRH |= GPIO_CRH_CNF14_0;
	
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /*	*/
  //Заполняем структуру с параметрами SPI модуля
  SPI_InitStructure.SPI_Direction |= SPI_Direction_2Lines_FullDuplex; //полный дуплекс
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // передаем по 8 бит
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; // Полярность и
  SPI_InitStructure.SPI_CPHA |= 0x001;//SPI_CPHA_1Edge; // фаза тактового сигнала
  SPI_InitStructure.SPI_NSS |= SPI_NSS_Soft;//SPI_NSS_Hard;// // Управлять состоянием сигнала NSS программно
  SPI_InitStructure.SPI_BaudRatePrescaler |= SPI_BaudRatePrescaler_64; // Предделитель SCK
  SPI_InitStructure.SPI_FirstBit |= SPI_FirstBit_MSB; // Первым отправляется старший бит
  SPI_InitStructure.SPI_Mode |= SPI_Mode_Master; // Режим - мастер
  SPI_Init(SPI2, &SPI_InitStructure); //Настраиваем SPI1
  SPI_Cmd(SPI2, ENABLE); // Включаем модуль SPI1....

 SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_RXNE,ENABLE); //Включаем прерывание по приему байта
 NVIC_EnableIRQ(SPI2_IRQn); //Разрешаем прерывания от SPI1
  // Поскольку сигнал NSS контролируется программно, установим его в единицу
  // Если сбросить его в ноль, то наш SPI модуль подумает, что
  // у нас мультимастерная топология и его лишили полномочий мастера.
//  SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);
	
	
	
	
	
}

void SPI2_IRQHandler (void) {
  if (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==SET) 
		{
    // Прерывание вызвано приемом байта ?
    uint8_t data = *(volatile uint8_t *)&SPI2->DR; //Читаем то что пришло
		
		//  while(!(SPI1->SR&SPI_SR_RXNE));	
  //  GPIOC->ODR ^= (GPIO_Pin_9 | GPIO_Pin_8); //Инвертируем состояние светодиодов
   // SPI1->DR;// = data; //И отправляем обратно то что приняли
			SPI_I2S_ClearFlag(SPI2,SPI_I2S_FLAG_RXNE);
	//		SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_TXE);
  }
//	else;
}

//******************************
double gettemp(uint8_t* buf1, int num)
        {	
				  static char prev_buf[8];
        static  int TMP, SIGN, SIGN1 = 0, SIGN2 = 0, SIGN3 = 0, Temper_Half = 0;
      static  double Temper = 0;
      static  double Half_t = 0;
					
					
              SIGN = buf1[1] & 0xF0;  // если больше 0 то отрицательная ;
              //  if (DS18b20._from_ds2 > 0)
               // { DS18b20._from_ds2 = DS18b20._from_ds2; }

                if (SIGN == 0xf0)
                {
                    buf1[0] = (char)(~buf1[0]);
                    buf1[1] = (char)(~buf1[1]);
                    Temper_Half = buf1[0] & 0x0f;
                    Temper = ((buf1[1] << 4) + (buf1[0] >> 4));// *-1;
                    SIGN = -1;
                }
                
                else
                {
                   
                    /*бывает, что присылает температуру без минуса, когда минус на самом деле*/
                  //  DS18b20._from_ds2 = 0;
// 9b.01. ... 9+16 =>
                    Temper_Half = buf1[0] & 0x0f; //b =11 =>0.687
                    Temper = (buf1[1] << 4) + (buf1[0] >> 4); //25
                   //25.687
                }
							
								
             //    buf1[0]= prev_buf[0];
             //   buf1[1] = prev_buf[1]; ;
           //     prev_buf = buf1;
                switch (Temper_Half)
                {//  0x05????? 
                    case 0x0f:
                        Half_t = 0.937;
                        break;

                    case 0x0e:
                        Half_t = 0.874;
                        break;

                    case 0x0d:
                        Half_t = 0.812;
                        break;


                    case 0x0c:
                        Half_t = 0.749;
                        break;

                    case 0x0b:
                        Half_t = 0.687;
                        break;

                    case 0x0A:
                        Half_t = 0.624;
                        break;

                    case 0x09:
                        Half_t = 0.562;
                        break;


                    case 0x08:
                        Half_t = 0.5;
                        break;
                    case 0x07:
                        Half_t = 0.437;
                        break;
                    case 0x06:
                        Half_t = 0.375;
                        break;

                    case 0x05:
                        Half_t = 0.312;
                        break;

                    case 0x04:
                        Half_t = 0.250;
                        break;


                    case 0x03:
                        Half_t = 0.187;
                        break;

                    case 0x02:
                        Half_t = 0.125;
                        break;
                    case 0x01:
                        Half_t = 0.0625;
                        break;
                    case 0x00:
                        Half_t = 0.00;
                        break;

                    default:
                        Half_t = 0.0;
                        break;

                }

                if (num == 1)
                {
                    Temper = Temper + Half_t;
													 sprintf(buffer_temp,"%3.2f\n",Temper); 	
												buffer_temp[4]=0x00;		buffer_temp[5]=0x00;
									
                    if ((SIGN == -1) || (SIGN1 == -1))
										{   Temper = Temper * -1;
										sprintf(buffer_temp,"%3.2f\n",Temper); 	
												buffer_temp[5]=0x00;		buffer_temp[6]=0x00;
										
										}
										
                        return Temper;
                }
                if (num == 2)
                { Temper = Temper + Half_t;
									 sprintf((char*)buffer_temp2,"%2.2f",Temper);
						

                    if ((SIGN == -1) || (SIGN2 == -1))
                       {   Temper = Temper * -1;
												sprintf((char*)buffer_temp2,"%2.2f",Temper);
										
										}
                        return Temper;
                }
                if (num == 3)
                {
                    Temper = Temper + Half_t;
													 sprintf(buffer_temp3,"%3.2f\n",Temper); 	
													buffer_temp3[5]=0x43;


                    if ((SIGN == -1) || (SIGN3 == -1))
                       {   Temper = Temper * -1;
										sprintf(buffer_temp3,"%3.2f\n",Temper); 	
													buffer_temp3[6]=0x43;
										
										}
                        return Temper;
                }
            
         
            return 0;

    
 
}

 

//****************************
// Initialize independent watchdog
// PRER: Division: 0 ~ 7 (only low 3 is effective!)
// Division factor = 4 * 2 ^ prer. But the maximum value can only be 256!
// rlr: Heavy loading register value: Low 11 bits are valid.
// Time Calculation (Probably): TOUT = ((4 * 2 ^ PRER) * RLR) / 40 (MS).
void IWDG_Init(u8 prer,u16 rlr)
{
	// 1, cancel the register write protection write 0x5555
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	// 2, set a stand-alone watchdog pre-frequency coefficient
	IWDG_SetPrescaler(prer);
	// 3, set up a separate watchdog reload value
	IWDG_SetReload(rlr);;
	// 4, heavy duty count value feed dog write 0xAAAA
	IWDG_ReloadCounter();
	// 5, start the watchdog write 0xcccc
	IWDG_Enable();
}
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();
}
//********************************
void faze(void)
{	
FAZE_IRQ_DISABEL//Menu.SetPWR
//	Tim3_init(2000,719);
//	TIM3_PWM_Init22(2000,719);//20 ms скваженость 2
//	TIM3->CCR2 = 	(uint16_t)Menu.SetPWR;
//	TIM3->CCR1 = 	(uint16_t)Menu.SetPWR/2;
//	if(Menu.SetPWR==0) Menu.SetPWR=2;
					TIM_SetCompare2(TIM3,(uint16_t)Menu.SetPWR);//
			  	TIM_SetCompare1(TIM3,Menu.SetPWR/2);//
	FAZE_IRQ_ENABEL/*	*/
			adc_pa1 =	measureCurrent(); 
		 	
	
	/*if((adc_pa1>=1000) && (SW6==TRUE))
									{ SW6=FALSE;
															SW7=TRUE;	
  													     				 Start;	
																					delay_ms(500);  ;
																					Stop;	
									starter_second=0;
									}

else if((adc_pa1>=1000)&& (SW6==FALSE) && (starter_second>1)&& (starter_second<=2))
  {Stop;Menu.SetPWR=0; SW6=FALSE; }
	
else if((adc_pa1 <=500) && (starter_second>15)&& (SW6==FALSE))
{	SW6=TRUE;SW7=FALSE;}
else ;	*/

}
void EXTI9_5_IRQHandler(void) //PA5 faza
{ 
	 if (EXTI_GetITStatus(EXTI_Line5) != RESET)
			{
				EXTI_ClearITPendingBit(EXTI_Line5); 
				ENCODER_IRQ_DISANBL 
				BUTTUN_IRQ_DISABEL 
	 
				faze();
				
			ENCODER_IRQ_ENABL 
			BUTTUN_IRQ_ENABL 
			
				EXTI_ClearITPendingBit(EXTI_Line5);

			}
}
 void Delay_us (uint32_t __IO us) //Р¤СѓРЅРєС†РёСЏ Р·Р°РґРµСЂР¶РєРё РІ РјРёРєСЂРѕСЃРµРєСѓРЅРґР°С… us
{
//us *=(SystemCoreClock/1000000)/5;
	//надо подбирать 72 mhz
	us *=(SystemCoreClock/72000000); //1 uS
	
	while(us--);
}

 
	
void EXTI2_IRQHandler(void) {//начало фазы PB2
   
   if (EXTI_GetITStatus(EXTI_Line2) != RESET)
			{	EXTI_ClearITPendingBit(EXTI_Line2); 
				ENCODER_IRQ_DISANBL 
				BUTTUN_IRQ_DISABEL 
	 
				faze();
				
			ENCODER_IRQ_ENABL 
				BUTTUN_IRQ_ENABL 
		 
					EXTI_ClearITPendingBit(EXTI_Line2);
		 
	 
				
		   } 

}
//***********************************************
void EXTI1_IRQHandler(void) {
#ifdef ENCODER_ENBL 		
 EXTI->IMR  &=	~EXTI_IMR_MR2;//PB
	EXTI->PR &= ~EXTI_IMR_MR0;
 if (EXTI_GetITStatus(EXTI_Line1) != RESET) 
			{ 
 if (A == 1) {
       
                nVol--;
         	
             }
				
			
				EXTI_ClearITPendingBit(EXTI_Line1);		
			} 
			
			EXTI->IMR  |=	EXTI_IMR_MR0;	
			
#else
 if (EXTI_GetITStatus(EXTI_Line1) != RESET)
			{

				encoder--;	
				delay_ms(300);
				EXTI_ClearITPendingBit(EXTI_Line1);

			}
#endif  			
			
			
#ifdef SERVIS_DEBUG	 
//	encoder_real=enc_read(encoder_real);
	/* 				 displey(105, 50,nVol,7)	;
					 displey(105, 40,ex,7)	;
					SSD1306_UpdateScreen();		*/				
#endif 
	//		EXTI->IMR  |=	EXTI_IMR_MR0;		
		//		EXTI->IMR  |=	EXTI_IMR_MR2;		
}
//***********************************************
void EXTI0_IRQHandler(void) {// энкодер PA0
 
//#define VOLUME_MAX_VAL  2000
	
	
#ifdef ENCODER_ENBL 	
 EXTI->IMR  &=	~EXTI_IMR_MR0; 
 EXTI->PR &= ~EXTI_IMR_MR0;
 /* 
	   // Убеждаемся, что флаг прерывания установлен
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
			{ 
         if (B == 1) {
       
                nVol++;
         	
             }
       // else if(B==0){   // Ручку против часовой стрелки
           
       //         nVol--;
       //         }
			//	else;
				
				  EXTI_ClearITPendingBit(EXTI_Line0);
     }
			
	 EXTI->IMR  |=	EXTI_IMR_MR1;	  	 
		EXTI->IMR  |=	EXTI_IMR_MR2;			 
  
 	
 

#ifdef SERVIS_DEBUG	 
//	encoder_real=enc_read(encoder_real);
	 				 displey(105, 50,nVol,7)	;
					 displey(105, 40,ex,7)	;
					SSD1306_UpdateScreen();						
#endif */

Rotate=EC12_1SCAN();
//	EC12_1Handle(); 
Delay_us(20);
	 if (Rotate == 1) // meets clockwise
		{
         Counter++;           // Add your code here 
		}
   else  if (Rotate == 2) // meets counterclockwise
		{
        Counter--;             // Add your code here 
		}
	 else;
	EXTI_ClearITPendingBit(EXTI_Line0);
	 EXTI->IMR  |=	EXTI_IMR_MR0;	  	 
		EXTI->IMR  |=	EXTI_IMR_MR0;		
}	
#else
 if (EXTI_GetITStatus(EXTI_Line0) != RESET)
			{	
				encoder++;
					delay_ms(300);
				EXTI_ClearITPendingBit(EXTI_Line0); 
			}
#endif  

}
//********************************************
void EXTI4_IRQHandler(void) {
#ifdef ENCODER_ENBL 
#else
 if (EXTI_GetITStatus(EXTI_Line4) != RESET)
			{	
				encoder=0;
				if(main_menu==0) 
				   main_menu=1;//menu
				 else if(main_menu==1)
				    	main_menu=2; // submenu
					 else if(main_menu==2)
					        main_menu=3;  //exit
				     //   else if(main_menu==3)
					   //        main_menu=0;
					else;
					
					delay_ms(300);
				EXTI_ClearITPendingBit(EXTI_Line4); 
			}
#endif  	
	
}

	//******************************************

void EXTI15_10_IRQHandler(void) //Button menu PB12
{ 
#ifdef ENCODER_ENBL 	 	
	if (EXTI_GetITStatus(EXTI_Line12) != RESET)
			{ EXTI_ClearITPendingBit(EXTI_Line12);	
			 			     	activ_menu=	FALSE;
				
				//	BUTTUN_IRQ_DISABEL  ENCODER_IRQ_ENABL BUTTUN_IRQ_ENABL  FAZE_IRQ_ENABEL
				
				 ENCODER_IRQ_DISANBL 
				 FAZE_IRQ_DISABEL
							 
				encoder_real=0;				
			 ex++;
       //  if(ex==2) 				
			//	 else 
					 if(ex>=3) ex=0;	
         else;				

/*#ifdef SERVIS_DEBUG	 
	 				 displey(105, 50,encoder_real,7)	;
					 displey(105, 40,ex,7)	;
					SSD1306_UpdateScreen();						
#endif*/
		  EXTI->IMR  |=	EXTI_IMR_MR0;		
			EXTI->IMR  |=	EXTI_IMR_MR2;
	 			 
		 // EXTI_ClearITPendingBit(EXTI_Line12);	
			}
 #else
	if (EXTI_GetITStatus(EXTI_Line12) != RESET)
			{ EXTI_ClearITPendingBit(EXTI_Line12);	
				
			}
#endif  
	
}
/*double set_temper(void)
{
char buffer[3];
	int cc=0,ret=0,size;
	
	SSD1306_Fill(SSD1306_COLOR_BLACK);	
	
  SSD1306_GotoXY(10, 5);
	SSD1306_Puts("SET POWER", &Font_11x18, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	
	
	
	while(1)
	{
		
		//	cc++;	
		delay_ms(30);	
	//DS18b20.Set_temp =	enc_read(DS18b20.Set_temp);
		
		SSD1306_GotoXY(40, 33);						
sprintf(buffer_set_t,"%d",DS18b20.Set_temp); 
 SSD1306_Puts(buffer, &Font_11x18, SSD1306_COLOR_WHITE);
		 SSD1306_UpdateScreen();	
		
		
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0) 
		{	//__disable_irq (); // запретить прерывания
		 	SSD1306_Fill(SSD1306_COLOR_BLACK);
			 SSD1306_UpdateScreen();	
			ret= 1;
			break;
			
		}
		else 
			 ret= 0;
	 
		
	}
	

	
	return ret ;

}
*/
int menu1(int indx, int m)
{

	if(indx ==1)
 		{
	
			if(m<=0) 
			{ m=m*-1;
				m=5-m;
				if(m<0)
				{ 	m=2; 
				    encoder=0; 
				}
			 else	if(m>=5)
			 	{	m=0;
				  encoder=0;
				}
			}
			 if(m >= 5)
			 { m=0;	//else if(m < 1) m=3;
				 encoder=0;
			 }
			
			 

 	
	switch(m)
					{
							
					case 0:
				// 	case 1:
					//case 2:
					//case 3:
						SSD1306_Fill(SSD1306_COLOR_BLACK);	
					SSD1306_GotoXY(10, 1);
					SSD1306_Puts("TEMPER thr.", &Font_11x18, SSD1306_COLOR_BLACK);
					SSD1306_GotoXY(10, 20); //24
					SSD1306_Puts("power", &Font_7x10, SSD1306_COLOR_WHITE);		
					SSD1306_GotoXY(10, 30); //45
					SSD1306_Puts("hister", &Font_7x10, SSD1306_COLOR_WHITE);	
						SSD1306_GotoXY(10, 40); //45
					SSD1306_Puts("save", &Font_7x10, SSD1306_COLOR_WHITE);	
						SSD1306_GotoXY(10, 50); //45
					SSD1306_Puts("exit", &Font_7x10, SSD1306_COLOR_WHITE);	
					SSD1306_UpdateScreen();		
					Menu.IdMenu =0;
				 	FirstEner=TRUE;
					break;
				case 1:
			//	case 3:
	     // case 6:
	    //  case 7:		
					SSD1306_Fill(SSD1306_COLOR_BLACK);	
					SSD1306_GotoXY(10, 1);
					SSD1306_Puts("temper", &Font_7x10, SSD1306_COLOR_WHITE);
					SSD1306_GotoXY(10, 10);
					SSD1306_Puts("POWER motr.", &Font_11x18, SSD1306_COLOR_BLACK);		
				  SSD1306_GotoXY(10, 30); //45
					SSD1306_Puts("hister", &Font_7x10, SSD1306_COLOR_WHITE);	
						SSD1306_GotoXY(10, 40); //45
					SSD1306_Puts("save", &Font_7x10, SSD1306_COLOR_WHITE);	
						SSD1306_GotoXY(10, 50); //45
					SSD1306_Puts("exit", &Font_7x10, SSD1306_COLOR_WHITE);	
				 	
					SSD1306_UpdateScreen();	
					Menu.IdMenu =1;
					FirstEner=TRUE;
					break;
				case 2:
			 
					SSD1306_Fill(SSD1306_COLOR_BLACK);	
				  SSD1306_GotoXY(10, 1);
					SSD1306_Puts("temper", &Font_7x10, SSD1306_COLOR_WHITE);
					SSD1306_GotoXY(10, 10);
					SSD1306_Puts("power", &Font_7x10, SSD1306_COLOR_WHITE);		
				  SSD1306_GotoXY(10, 20); //45
					SSD1306_Puts("HISTERESIS", &Font_11x18, SSD1306_COLOR_BLACK);	
						SSD1306_GotoXY(10, 40); //45
					SSD1306_Puts("save", &Font_7x10, SSD1306_COLOR_WHITE);	
					 
		      SSD1306_GotoXY(10, 50);
					SSD1306_Puts("exit", &Font_7x10, SSD1306_COLOR_WHITE);	
					SSD1306_UpdateScreen();		
	         Menu.IdMenu =2;	
	        FirstEner=TRUE;				
					break;	
				
				
				case 3:
			 
					SSD1306_Fill(SSD1306_COLOR_BLACK);	
				 SSD1306_GotoXY(10, 1);
					SSD1306_Puts("temper", &Font_7x10, SSD1306_COLOR_WHITE);
					SSD1306_GotoXY(10, 10);
					SSD1306_Puts("power", &Font_7x10, SSD1306_COLOR_WHITE);		
				  SSD1306_GotoXY(10, 20); //45
					SSD1306_Puts("hister", &Font_7x10, SSD1306_COLOR_WHITE);	
						SSD1306_GotoXY(10, 30); //45
					SSD1306_Puts("SAVE conf.", &Font_11x18, SSD1306_COLOR_BLACK);	
		      SSD1306_GotoXY(10, 50);
					SSD1306_Puts("exit", &Font_7x10, SSD1306_COLOR_WHITE);	
					SSD1306_UpdateScreen();		
	         Menu.IdMenu =3;	
	        FirstEner=TRUE;				
					break;	
				
				case 4:
			 
					SSD1306_Fill(SSD1306_COLOR_BLACK);	
				 SSD1306_GotoXY(10, 1);
					SSD1306_Puts("temper", &Font_7x10, SSD1306_COLOR_WHITE);
					SSD1306_GotoXY(10, 10);
					SSD1306_Puts("power", &Font_7x10, SSD1306_COLOR_WHITE);		
				  SSD1306_GotoXY(10, 20); //45
					SSD1306_Puts("hister", &Font_7x10, SSD1306_COLOR_WHITE);	
						SSD1306_GotoXY(10, 30); //45
					SSD1306_Puts("save", &Font_7x10, SSD1306_COLOR_WHITE);	
		      SSD1306_GotoXY(10, 40);
					SSD1306_Puts("EXIT", &Font_11x18, SSD1306_COLOR_BLACK);	
					SSD1306_UpdateScreen();		
	         Menu.IdMenu =4;
	        FirstEner=TRUE;				
				
					break;	
				
	
					default:
					SSD1306_GotoXY(10, 5);
					SSD1306_Puts("Power", &Font_11x18, SSD1306_COLOR_WHITE);
					SSD1306_GotoXY(10, 24);
					SSD1306_Puts("Temper", &Font_11x18, SSD1306_COLOR_WHITE);			
					SSD1306_GotoXY(10, 45);
					SSD1306_Puts("Exit", &Font_11x18, SSD1306_COLOR_WHITE);		
						Menu.IdMenu =4;
					break;
		//	SSD1306_Fill(SSD1306_COLOR_BLACK);		
					
		
	}

//	return Menu.IdMenu;
	//	break;

}
  else 	if(indx ==2)
 	 submenu1(m,Menu.IdMenu);
		
	 
	 else if(main_menu==3)
		     main_menu=0;	
		
		
	else;	
	return 0;							
}
volatile int prev_encod;	
volatile int tmp_modul;	
volatile int tmp_encod_znak;	
volatile int tmp_first;	
volatile int itmp_enc;	
double dtmp_enc;
double dtmp_prev;
double dtmp_modul;
double dtmp_first;
void submenu1(int encod,int indx)
{
					SSD1306_Fill(SSD1306_COLOR_BLACK);	
	

//	delay_ms(100);
	char buffer[10];
	
switch(indx)
{     //************** Set Temper: ***************
		    case 0:
				{	 
						SSD1306_GotoXY(10, 5);
				    SSD1306_Puts("Set Temper:", &Font_11x18, SSD1306_COLOR_WHITE);
				 
			 	if(FirstEner==TRUE)
				{ 
					tmp_first=prev_encod =  Menu.TemperStop;
				
				}
				else tmp_first=0;
				
						if(prev_encod !=encoder)
						{ tmp_modul=abs(prev_encod-encoder);
						
							if((encoder -prev_encod )<0)// ||  ((prev_encod - encoder)  <0)  )
								tmp_encod_znak = -1;
							else 
								tmp_encod_znak = 1;
							
								if(FirstEner==TRUE)
								{ 
									if(tmp_first<0)
										tmp_encod_znak=-1;
									
									Menu.TemperStop=Menu.TemperStop +(1 *tmp_encod_znak );//(double)encoder;
									
								}
							 else 
							     	 Menu.TemperStop=Menu.TemperStop +((double)tmp_modul *tmp_encod_znak );//(double)encoder;	
								 FirstEner=FALSE;
						//	prev_encod=0;
							
						}
						prev_encod=encoder;
						
				    SSD1306_GotoXY(20,30);
					  snprintf(buffer, sizeof buffer, "%g",(double) Menu.TemperStop);
 						SSD1306_Puts(buffer, &Font_16x26, SSD1306_COLOR_WHITE);
									SSD1306_UpdateScreen();		
					}
				 break;
				//**********Power Pomp**********
				 case 1:
				 {
						itmp_enc=encoder*100;
					SSD1306_GotoXY(10, 5);
				    SSD1306_Puts("Power Pomp:", &Font_11x18, SSD1306_COLOR_WHITE); 
					 
				if(FirstEner==TRUE)
				{ 
					tmp_first=prev_encod =  Menu.SetPWR;
				
				}
				else tmp_first=0;
				
			
						if(prev_encod !=itmp_enc)
						{ 
							tmp_modul=abs(prev_encod-itmp_enc);
						
							if((itmp_enc -prev_encod )<0)// ||  ((prev_encod - encoder)  <0)  )
								tmp_encod_znak = -1;
							else 
								tmp_encod_znak = 1;
							
								if(FirstEner==TRUE)
								{ 
									if(tmp_first<0)
										tmp_encod_znak=-1;
									
									Menu.SetPWR=Menu.SetPWR +(1 *tmp_encod_znak );//(double)encoder;
									
								}
							 else 
							     	 Menu.SetPWR=Menu.SetPWR +((double)tmp_modul *tmp_encod_znak );//(double)encoder;	
				 				
						 if(Menu.SetPWR<=0)
							   Menu.SetPWR=100;
	      else if(Menu.SetPWR>2000)
							   Menu.SetPWR=2000;
             else;

							 FirstEner=FALSE;
						//	prev_encod=0;
							
						}
						prev_encod=itmp_enc;
						
				    SSD1306_GotoXY(20,30);
					  snprintf(buffer, sizeof buffer, "%g",(double)Menu.SetPWR);
 						SSD1306_Puts(buffer, &Font_16x26, SSD1306_COLOR_WHITE);
									SSD1306_UpdateScreen();		
					}						
				break;
				 //********Menu.SetHisteresis****************
				    case 2:
							 {
					 
					SSD1306_GotoXY(10, 5);
				    SSD1306_Puts("Histeresis:", &Font_11x18, SSD1306_COLOR_WHITE); 
					 
				if(FirstEner==TRUE)
				{ 
					dtmp_first=dtmp_prev = Menu.SetHisteresis;
				
				}
				else tmp_first=0;
				dtmp_enc= (double)encoder/10;
				
					if(dtmp_enc <= 0)
					{  dtmp_enc=0.1;
						encoder=0;
					}
					else if(dtmp_enc > 3)
					{ dtmp_enc=3;
							encoder=0;
					}
					else;
					
						if(dtmp_prev != dtmp_enc )
						{ dtmp_modul= (dtmp_prev - dtmp_enc);
						   if(dtmp_modul<0)
								  dtmp_modul=dtmp_modul*-1;
							 
							if((dtmp_enc -dtmp_prev )<0)// ||  ((prev_encod - encoder)  <0)  )
								tmp_encod_znak = -1;
							else 
								tmp_encod_znak = 1;
							
								if(FirstEner==TRUE)
								{ 
									if(dtmp_first<0)
										tmp_encod_znak=-1;
									
									Menu.SetHisteresis=Menu.SetHisteresis +(0.1 *tmp_encod_znak );//(double)encoder;
									
								}
							 else 
							     	 Menu.SetHisteresis=Menu.SetHisteresis +((double)dtmp_modul *tmp_encod_znak );//(double)encoder;	
								 FirstEner=FALSE;
						//	prev_encod=0;
							
						}
						dtmp_prev=dtmp_enc;
						
				    SSD1306_GotoXY(20,30);
					  snprintf(buffer, sizeof buffer, "%g",(double)Menu.SetHisteresis);
 						SSD1306_Puts(buffer, &Font_16x26, SSD1306_COLOR_WHITE);
									SSD1306_UpdateScreen();		
					}	
						break;	
				//************ save **********	
				   	case 3:
			 	    Starter_motor(1,adc_pa1);
						
								 {
					 
					SSD1306_GotoXY(10, 5);
				    SSD1306_Puts("SAVE:", &Font_11x18, SSD1306_COLOR_WHITE); 
					 
				if(FirstEner==TRUE)
				{ 
					dtmp_first=dtmp_prev = Menu.SetPWR;
				
				}
				else tmp_first=0;
				dtmp_enc= (double)encoder*100;
				
					/*if(dtmp_enc <= 100)
					{  dtmp_enc=100;
						encoder=0;
					}
					else if(dtmp_enc > 2000)
					{ dtmp_enc=2000;
							encoder=0;
					}
					else;*/
					
						if(dtmp_prev != dtmp_enc )
						{ dtmp_modul= (dtmp_prev - dtmp_enc);
						   if(dtmp_modul<0)
								  dtmp_modul=dtmp_modul*-1;
							 
							if((dtmp_enc -dtmp_prev )<0)// ||  ((prev_encod - encoder)  <0)  )
								tmp_encod_znak = -1;
							else 
								tmp_encod_znak = 1;
							
								if(FirstEner==TRUE)
								{ 
									if(dtmp_first<0)
										tmp_encod_znak=-1;
									
									Menu.SetPWR=Menu.SetPWR +(100 *tmp_encod_znak );//(double)encoder;
									
								}
							 else 
							     	 Menu.SetPWR=Menu.SetPWR +((double)dtmp_modul *tmp_encod_znak );//(double)encoder;	
								 FirstEner=FALSE;
						//	prev_encod=0;
							
						}
						dtmp_prev=dtmp_enc;
						
				    SSD1306_GotoXY(20,30);
					  snprintf(buffer, sizeof buffer, "%g",(double)Menu.SetPWR);
 						SSD1306_Puts(buffer, &Font_16x26, SSD1306_COLOR_WHITE);
									SSD1306_UpdateScreen();		
					}	
						
						
						
						
						
				    break;
				//************* exit******************	
		        case 4:
							Start;
					delay_ms(2);
						Stop;
						start=FALSE;
							//Starter_motor(1,adc_pa1);
							// Starter_motor(0);
						break;	
				default:
			    main_menu=0;
				break;	
		
}
	
	
}

//****************************
void submenu11(int encod,int indx)
{
				_1306_Fill(SSD1306_COLOR_BLACK);	

	char buffer[10];
	
switch(indx)
{     //************** Set Temper: ***************
		    case 0:
				{	 
					 	
					FontSet12();		
	        OLED_DrawStr("Ecnfyjdrf N",10, 0 , 1);	
					OLED_DrawHLine(0, 13, 120, 1);
					
			 	if(FirstEner==TRUE)
				{ 
					tmp_first=prev_encod =  Menu.TemperStop;
				
				}
				else tmp_first=0;
				
						if(prev_encod !=encoder)
						{ tmp_modul=abs(prev_encod-encoder);
						
							if((encoder -prev_encod )<0)// ||  ((prev_encod - encoder)  <0)  )
								tmp_encod_znak = -1;
							else 
								tmp_encod_znak = 1;
							
								if(FirstEner==TRUE)
								{ 
									if(tmp_first<0)
										tmp_encod_znak=-1;
									
									Menu.TemperStop=Menu.TemperStop +(1 *tmp_encod_znak );//(double)encoder;
									
								}
							 else 
							     	 Menu.TemperStop=Menu.TemperStop +((double)tmp_modul *tmp_encod_znak );//(double)encoder;	
								 FirstEner=FALSE;
						//	prev_encod=0;
							
						}
						prev_encod=encoder;
					   snprintf(buffer, sizeof buffer, "%g",(double) Menu.TemperStop);
 					    FontSet16X26_DEC();		
	            OLED_DrawStr(buffer,20, 30 ,1);	
						  OLED_UpdateScreen();
					}
				 break;
				//**********Power Pomp**********
				 case 1:
				 {
						itmp_enc=encoder*100;
					//SSD1306_GotoXY(10, 5);
				   // SSD1306_Puts("Power Pomp:", &Font_11x18, SSD1306_COLOR_WHITE); 
					 	FontSet12();		
	        OLED_DrawStr("Vjomyjcnm rjvghtc.",1, 0 , 1);	
					OLED_DrawHLine(0, 13, 120, 1);
					 
				if(FirstEner==TRUE)
				{ 
					tmp_first=prev_encod =  Menu.SetPWR;
				
				}
				else tmp_first=0;
				
			
						if(prev_encod !=itmp_enc)
						{ 
							tmp_modul=abs(prev_encod-itmp_enc);
						
							if((itmp_enc -prev_encod )<0)// ||  ((prev_encod - encoder)  <0)  )
								tmp_encod_znak = -1;
							else 
								tmp_encod_znak = 1;
							
								if(FirstEner==TRUE)
								{ 
									if(tmp_first<0)
										tmp_encod_znak=-1;
									
									Menu.SetPWR=Menu.SetPWR +(1 *tmp_encod_znak );//(double)encoder;
									
								}
							 else 
							     	 Menu.SetPWR=Menu.SetPWR +((double)tmp_modul *tmp_encod_znak );//(double)encoder;	
				 				
						 if(Menu.SetPWR<=0)
							   Menu.SetPWR=100;
	      else if(Menu.SetPWR>2000)
							   Menu.SetPWR=2000;
             else;

							 FirstEner=FALSE;
						//	prev_encod=0;
							
						}
						prev_encod=itmp_enc;
					   snprintf(buffer, sizeof buffer, "%g",(double)Menu.SetPWR);
						 FontSet16X26_DEC();		
	            OLED_DrawStr(buffer,20, 30 ,1);	
						  OLED_UpdateScreen();
					}						
				break;
				 //********Menu.SetHisteresis****************
				    case 2:
							 {
					 
				//	SSD1306_GotoXY(10, 5);
				 //   SSD1306_Puts("Histeresis:", &Font_11x18, SSD1306_COLOR_WHITE); 
					  	FontSet12();		
	        OLED_DrawStr("Ubcnthtpbc ",1, 0 , 1);	
					OLED_DrawHLine(0, 13, 120, 1);
								 
				if(FirstEner==TRUE)
				{ 
					dtmp_first=dtmp_prev = Menu.SetHisteresis;
				
				}
				else tmp_first=0;
				dtmp_enc= (double)encoder/10;
				
					if(dtmp_enc <= 0)
					{  dtmp_enc=0.1;
						encoder=0;
					}
					else if(dtmp_enc > 3)
					{ dtmp_enc=3;
							encoder=0;
					}
					else;
					
						if(dtmp_prev != dtmp_enc )
						{ dtmp_modul= (dtmp_prev - dtmp_enc);
						   if(dtmp_modul<0)
								  dtmp_modul=dtmp_modul*-1;
							 
							if((dtmp_enc -dtmp_prev )<0)// ||  ((prev_encod - encoder)  <0)  )
								tmp_encod_znak = -1;
							else 
								tmp_encod_znak = 1;
							
								if(FirstEner==TRUE)
								{ 
									if(dtmp_first<0)
										tmp_encod_znak=-1;
									
									Menu.SetHisteresis=Menu.SetHisteresis +(0.1 *tmp_encod_znak );//(double)encoder;
									
								}
							 else 
							     	 Menu.SetHisteresis=Menu.SetHisteresis +((double)dtmp_modul *tmp_encod_znak );//(double)encoder;	
								 FirstEner=FALSE;
						//	prev_encod=0;
							
						}
						dtmp_prev=dtmp_enc;
						
				  //  SSD1306_GotoXY(20,30);
					  snprintf(buffer, sizeof buffer, "%g",(double)Menu.SetHisteresis);
 					//	SSD1306_Puts(buffer, &Font_16x26, SSD1306_COLOR_WHITE);
					//				SSD1306_UpdateScreen();		
						
						 FontSet16X26_DEC();		
	            OLED_DrawStr(buffer,20, 30 ,1);	
						  OLED_UpdateScreen();
						
					}	
						break;	
				//************ save **********	
				   	case 3:
			 	    Starter_motor(1,adc_pa1);
						
				{
					 
					FontSet12();		
	        OLED_DrawStr("Lbcgk'q ",10, 0 , 1);	
					OLED_DrawHLine(0, 13, 120, 1);
					  
				if(FirstEner==TRUE)
				{ 
					dtmp_first=dtmp_prev = Menu.SetPWR;
				
				}
				else tmp_first=0;
				dtmp_enc= (double)encoder*100;
				
					/*if(dtmp_enc <= 100)
					{  dtmp_enc=100;
						encoder=0;
					}
					else if(dtmp_enc > 2000)
					{ dtmp_enc=2000;
							encoder=0;
					}
					else;*/
					
						if(dtmp_prev != dtmp_enc )
						{ dtmp_modul= (dtmp_prev - dtmp_enc);
						   if(dtmp_modul<0)
								  dtmp_modul=dtmp_modul*-1;
							 
							if((dtmp_enc -dtmp_prev )<0)// ||  ((prev_encod - encoder)  <0)  )
								tmp_encod_znak = -1;
							else 
								tmp_encod_znak = 1;
							
								if(FirstEner==TRUE)
								{ 
									if(dtmp_first<0)
										tmp_encod_znak=-1;
									
									Menu.SetPWR=Menu.SetPWR +(100 *tmp_encod_znak );//(double)encoder;
									
								}
							 else 
							     	 Menu.SetPWR=Menu.SetPWR +((double)dtmp_modul *tmp_encod_znak );//(double)encoder;	
								 FirstEner=FALSE;
						//	prev_encod=0;
							
						}
						dtmp_prev=dtmp_enc;
				 
					  snprintf(buffer, sizeof buffer, "%g",(double)Menu.SetPWR);
 					 	  FontSet16X26_DEC();		
	            OLED_DrawStr(buffer,20, 30 ,1);	
						  OLED_UpdateScreen();
					}	
						
						
						
						
						
				    break;
				//************* exit******************	
		        case 4:
							Start;
					delay_ms(2);
						Stop;
						start=FALSE;
							//Starter_motor(1,adc_pa1);
							// Starter_motor(0);
						break;	
				default:
			    main_menu=0;
				break;	
		
}
	
	
}
//***************************
int menu11(int indx, int m)
{

	if(indx ==1)
 		{
	
			if(m<=0) 
			{ m=m*-1;
				m=5-m;
				if(m<0)
				{ 	m=2; 
				    encoder=0; 
				}
			 else	if(m>=5)
			 	{	m=0;
				  encoder=0;
				}
			}
			 if(m >= 5)
			 { m=0;	//else if(m < 1) m=3;
				 encoder=0;
			 }
			
			 

 	
	switch(m)
					{
							
					case 0:
			
				_1306_Fill(SSD1306_COLOR_BLACK);
					
					FontSet12();		
	            OLED_DrawStr("NTVGTHFNEHF",0, 0 , 0);		
					FontSet12();	
					  OLED_DrawStr("Vjomyjcnm ",0, 14 , 1);		
				  FontSet12();						
					  OLED_DrawStr("Ubcnthtpbc",0, 29 , 1);		
					FontSet12();	
					  OLED_DrawStr("Lbcgktq ",0, 43 , 1);		
				  FontSet12();	
			    OLED_DrawStr("Ds[jl",0, 57 , 1);		
					OLED_UpdateScreen();
					Menu.IdMenu =0;
				 	FirstEner=TRUE;
					break;
				case 1:
			 
				_1306_Fill(SSD1306_COLOR_BLACK);
					
					FontSet12();		
	            OLED_DrawStr("Ntvgthfnehf",0, 0 , 1);		
					FontSet12();	
					  OLED_DrawStr("VJOMYJCNM ",0, 14 , 0);		
				  FontSet12();						
					  OLED_DrawStr("Ubcnthtpbc ",0, 27 , 1);		
					FontSet12();	
					  OLED_DrawStr("Lbcgktq  ",0, 39 , 1);		
				  FontSet12();	
			    OLED_DrawStr("Ds[jl",0, 51 , 1);		
					OLED_UpdateScreen();
									
			 
					Menu.IdMenu =1;
					FirstEner=TRUE;
					break;
				case 2:
						_1306_Fill(SSD1306_COLOR_BLACK);
				
			 	  FontSet12();		
	            OLED_DrawStr("Ntvgthfnehf",0, 0 , 1);		
					FontSet12();	
					  OLED_DrawStr("Vjomyjcnm ",0, 14 , 1);		
				  FontSet12();						
					  OLED_DrawStr("UBCNTHTPBC ",0, 27 , 0);		
					FontSet12();	
					  OLED_DrawStr("Lbcgktq  ",0, 39 , 1);		
				  FontSet12();	
			    OLED_DrawStr("Ds[jl ",0, 51 , 1);		
					OLED_UpdateScreen();
					 
	         Menu.IdMenu =2;	
	        FirstEner=TRUE;				
					break;	
				
				
				case 3:
						_1306_Fill(SSD1306_COLOR_BLACK);
			 	FontSet12();		
	            OLED_DrawStr("Ntvgthfnehf",0, 0 , 1);		
					FontSet12();	
					  OLED_DrawStr("Vjomyjcnm ",0, 14 , 1);		
				  FontSet12();						
					  OLED_DrawStr("Ubcnthtpbc ",0, 27 , 1);		
					FontSet12();	
					  OLED_DrawStr("Lbcgktq  ",0, 39 , 0);		
				  FontSet12();	
			    OLED_DrawStr("Ds[jl ",0, 51 , 1);		
					OLED_UpdateScreen();
					 	
	         Menu.IdMenu =3;	
	        FirstEner=TRUE;				
					break;	
				
				case 4:
						_1306_Fill(SSD1306_COLOR_BLACK);
			 	FontSet12();		
	            OLED_DrawStr("Ntvgthfnehf",0, 0 , 1);		
					FontSet12();	
					  OLED_DrawStr("Vjomyjcnm ",0, 14 , 1);		
				  FontSet12();						
					  OLED_DrawStr("Ubcnthtpbc ",0, 27 , 1);		
					FontSet12();	
					  OLED_DrawStr("Lbcgktq  ",0, 39 , 1);		
				  FontSet12();	
			    OLED_DrawStr("DS{JL ",0, 51 , 0);		
					OLED_UpdateScreen();
				 
	         Menu.IdMenu =4;
	        FirstEner=TRUE;				
				
					break;	
				
	
					default:
							_1306_Fill(SSD1306_COLOR_BLACK);
					SSD1306_GotoXY(10, 5);
					SSD1306_Puts("Power", &Font_11x18, SSD1306_COLOR_WHITE);
					SSD1306_GotoXY(10, 24);
					SSD1306_Puts("Temper", &Font_11x18, SSD1306_COLOR_WHITE);			
					SSD1306_GotoXY(10, 45);
					SSD1306_Puts("Exit", &Font_11x18, SSD1306_COLOR_WHITE);		
						Menu.IdMenu =4;
					break;
	}
}
  else 	if(indx ==2)
 	 submenu11(m,Menu.IdMenu);
		
	 
	 else if(main_menu==3)
		     main_menu=0;	
		
		
	else;	
	_1306_Fill(SSD1306_COLOR_BLACK); 
	 
	 
	return 0;							
}


//***********************************
// Обработчик прерывания для энкодера по спаду PB0
/*int adjusting2(int i){// EXTI_0_IRQHandler(void) {
//char buffer[3];	
val=100;
//	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)// If the OUTA is RESET
//	{
	   if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0)// If OUTB is also reset... CCK
				 {
					while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0){};  // wait for the OUTB to go high
					 val++;
					while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0){};  // wait for the OUTA to go high	
						delay_ms(1);
				 }
				 
				 
		 else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1)// If OUTB is also set
				 {
				 while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1){};  // wait for the OUTB to go high
					 val--;	//DS18b20.power_ten= 300;    
					while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1){};  // wait for the OUTB to go high	
					 	delay_ms(1);
				 
				 }
		 	else;	
          // if (val<=0) val = 0;
					//	 if (val>100) val =100;	
	 
//	}
	
	
	return val;
}*/
//******************
u32 enc_read(u32 offset )
{
	// A = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) PA0
	// B = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) PA1
u32 cnt=0;
cnt=cnt+offset;

char buffer[3];	
	// A ____|----|____|----|____       PA0 interup
	// B __|----|____|----|____|-
	// B      __|----|____|----|____|-  PA1 input
 
					
					while (A == 1){};//синронизация//	 	for(int t=0;t<10000;t++) break;
						
				//	while (A == 0){};//синронизация//	 
				                
							if(A == 0)// If OUTB is also reset... CCK
											{ delay_ms(1);
												 if(B == 0)// If OUTB is also reset... CCK
															 {
																while (B == 0){};  // wait for the OUTB to go high
																 cnt++;
																//	delay_ms(1);
																while (A == 0){};  // wait for the OUTA to go high	
																	//delay_ms(1);
															 }
															 
												else if(B == 1)// If OUTB is also reset... CCK
															 {
																 //	while (B == 1){};
																 
																while (A == 1){};  // wait for the OUTB to go high
																 cnt--;
																//		delay_ms(1);
																while (B == 1){};  // wait for the OUTA to go high	
																//	delay_ms(1);
															 }
															 
													else;
											}
															 
 
			#ifdef SERVIS_DEBUG	 
		 		//		 displey(90, 50,cnt,7)	;
			//			 displey(90, 40,ex,7)	;
			//		 	SSD1306_UpdateScreen();						
#endif 		 									
										 					
	//	 }
	
	return cnt;
}
//***************************************
_Bool Starter_motor(_Bool r, uint16_t adc_curent )
{//adc_pa1
	//Start_PA11;
	//	Start;
	
	
		if(r==TRUE)
		{
		//	if((adc_curent>870)&&(adc_curent<=920)) // not work
		if((adc_curent>=2000))
			{
						//	for(int s=0;s<10000;s++)
							{
								Start;
								//	if((adc_curent>=1300) && (adc_curent <= 1500))
                    delay_ms(2);
								Stop; r=FALSE; //break;
								
							}
		 
		  }
		}
return r;
}
 

//***************************************
 
	