#include "P_regulator.h"
#include <math.h>
volatile  _Bool BYSY=FALSE; 	
_Bool S,S2 ;
int iStrart=0,Tmp;
extern double Limit_delt,Polka;
extern int Switch_Programm;
extern volatile int iQueue,Str,Twork,Tpause;
extern volatile int cnt_polka, starter_second,iTime_hold_target, Cnt_points, cnt_perehod,cnt_second, Deltime;
extern volatile uint16_t adc_pa0,adc_pa1;
extern volatile double T_min;
extern volatile double T_max;
extern volatile	double K_down,K_down;
extern volatile	double  K_min,K_max;
extern volatile	double  Calculated_Tstop;	
extern volatile int cnt_work_pump,
	cnt_work_pump_tstop  ,
cnt_run,prev_cnt_run,before_correct, 
before_cnt1,
before_cnt2,
before_cnt3,
before_cnt4,
before_cnt5; 
volatile	double delta_min_afterzero,delta_polka;
volatile	double delta_max;
volatile	 		double delta_min;// = ((T_min-Tstop)/2);
//volatile			double delta_max;// = ((Tstop -T_max)/2);
int comand_queue=0;
extern volatile	double _min,_max,prv_slope;
double Dis,t_slope;

volatile int second;
_Bool First_in = TRUE,OneTime=FALSE;
_Bool Enbl_1= FALSE,Enbl_2= FALSE,Enbl_3= FALSE,Enbl_4= FALSE,Enbl_5= FALSE;
extern _Bool Enbl_Starter;
volatile int cnt_cycle=0;     // не менять
volatile int cnt_equal=0;     // не менять
_Bool Zatychka = FALSE;
int ret=0;


double Dilta,DistanceStop, SpDelta,time_work;	
double	diff;
extern volatile double  pre_value2,
												pre_value3,
												pre_value,
												prev_Tcurrent,
												prev_direction,
												vector_direction,
												prev_difrent,
												delta,delt,
												prev_delta,
Del_stop_before;
volatile double max_min[5][2] = {-9.1,-11.0,-9.2,-11.1,-9.3,-11.2,-8.9,-11.1  };//points max min  5ть строк 2-а столбца
volatile	double up_regulator, dwn_regulator;

//************************************************************
void test(void);
double Autosearch(_Bool Sw ,double T_stop,double T_curent, double Delta_stop_before);
int Status_Starter(uint16_t ADC );
double Time_Slope(double Tcurent, double Tstop,int *Begin_calculate,int time,double *Dis2 );
double Check_moving_temper(double Tcurent, double Tstop, int *Begin_calculate,int time,double *Dis2 );
double Actuator(double Tcurent, double Tstop );
double direct(double Tcurent, double Tstop,  int DeltaTime  );
int  adjast( int TimingWork,int TimigPause, double p, int que);
double	okruglennye(double chislo, long znaki);
volatile int sig=0;
int P_regulator(int Mode,double Tstop, double Tmin, double Tmax,double Tcurent, double Kmin,double Kmax, _Bool reset)
{ int direction;
//*********************************************************
// reset
if(reset ==  TRUE)
	{cnt_cycle=0;
		pre_value2=0;
		pre_value3=0;
		point_up=FALSE;
		reset =  FALSE;
	 
		ret=0;
		Cnt_points=0;
	return -100;
	}	

//**********************************************************	
//Tcurent = okruglennye(Tcurent,4);
//Tstop   = okruglennye(Tstop,4);
	
	if((Tcurent <	Tstop+0.2)&& (comand_queue!=3) && (Mode !=4))//0.4
	{ Menu.SetPWR=0;
	comand_queue=1;
	}
	//if((Mode==0))// && (comand_queue==1))
	if(((Tcurent>=	(Tstop+0.2))&& (comand_queue!=3)) && (Mode !=4)) // -0.1
	 {  
	 Menu.SetPWR=2000; 
	 comand_queue=2;
	 }
//**********************************************************	
diff=Tcurent-Tstop; 	
diff = okruglennye(diff,4);		
//*********************************************************	
// определить точки перехода	
if(diff != pre_value3)
{Zatychka=TRUE;
	pre_value =pre_value3;
 	}	
if((diff == 0)&& (Zatychka==TRUE))	// точка перехода
									{ Zatychka=FALSE;
										cnt_equal++;
										
										 if(cnt_equal==3)// переходсостоялся
											 	{cnt_equal=0;
												 Cnt_points++;
												//	cnt_cycle++; 	
											 	}
							 
									}
		
	 


//*****************************************
//T_min =max_min[Cnt_points][0];
//T_max =max_min[Cnt_points][1];									
//*****************************************
pre_value3=diff;
pre_value3 = okruglennye(pre_value3,2);
//*****************************************
								
									
									
	switch(Mode)
	{
//-------- Mode 0---- поиск мах мин точек  и приближенного Kf------
		case 0:
		{
	// заполнить массив 		
			if(Tcurent>pre_value2)// + maximum
					{ if(max_min[Cnt_points][0]!=0)
									if(Tcurent > max_min[Cnt_points][0])//cmd 0x74
									{ max_min[Cnt_points][0]=Tcurent;
										Tmax=max_min[Cnt_points][0];
									}
									else;
						else
										 max_min[Cnt_points][0]=Tcurent;
					}
					
			else if(Tcurent<pre_value2)// - minimum
					{
						if(max_min[Cnt_points][1]!=0)
						if(Tcurent < max_min[Cnt_points][1]) //cmd 0x73
							{ max_min[Cnt_points][1]=Tcurent;
								Tmin=max_min[Cnt_points][1];		
							}
							else;
							
							else
								 max_min[Cnt_points][1]=Tcurent;	
	        }
			else;
		
		if(Tcurent!=pre_value2)
		pre_value2=Tcurent ;	//prev_value1 ...2 глобальные переменные
	
			// накапливаем данные
			//	Cnt_points=	cnt_cycle;		
		//			cnt_points++;
					
					
  if(Cnt_points==avegage_count) 	// заполнить массив		
		{		
				// найти среднее из массива. c 1 по 3
				//	Tmax
				//	Tmin
	      for(int i=1;i<=Cnt_points-1 ;i++) // 0- не стабильный
					{
					 Tmax=Tmax+max_min[i][0];
					 Tmin=Tmin+max_min[i][1];
					}
				Tmin=Tmin/(Cnt_points-1);
				Tmax=Tmax/(Cnt_points-1);
				//*********************************************************	
				T_min=Tmin; 
				T_max=Tmax;	
		 
						
					cnt_cycle=0;
					Cnt_points=0;
					ret=	100;// цикл анализа закончен.
				 	
		}				
		else ret =101;
		}
			break;
//**********************************************************************
//-------- Mode 1  регулировка  ----------
		case 1:
		{Tstop = Calculated_Tstop; //not send by Tstop;
			 
			_min=Calculated_Tstop;
			_max=Calculated_Tstop;
			//double up_regulator, dwn_regulator;
	 	//	double 
			delta_min = ((T_min-Tstop)/2)/2;
		//	double 
			delta_max = ((Tstop -T_max)/2)/2;
			
			if(delta_min<0)  delta_min= delta_min* -1;
			if(delta_max<0)  delta_max= delta_max* -1;
			
			up_regulator =  Tstop+delta_max ;//0.63  -9.36
	   dwn_regulator  	= Tstop- delta_min;//0.53  -10.53
			
	//**************************************************								
			     
			if(Tcurent > (Tstop-0.2 )) //Tstop+0.5 => 9.1  8.9  9.2
						{  
							if(diff<=0.3)
              adjast( 22,48,1999,0);
							if(diff>0.3)
							adjast( 80,100,1998,0);	
							

						      comand_queue=3;
					
						}
					 
					 
			/*	if((Tcurent <= (Tstop+0.1 ))&& (Tcurent >= (Tstop+0.0 )))
					{comand_queue=4;
					
					
						 adjast( 10,34,1900);
					 
				 }
					else;*/
				 
			/*	if(cnt_work_pump > 10)
					 {  comand_queue=6;
					   Menu.SetPWR=0;
					 }
					 if(cnt_work_pump >= 25)// остывание
					 { cnt_work_pump=0;comand_queue=7;
					 }	
				*/
					 
			/*
				if((diff>= -0.2) && (diff <= 0.1))
					{cnt_work_pump=0;
					  comand_queue=8;
						if(( cnt_work_pump_tstop >0 ) && ( cnt_work_pump_tstop <=10 ) )
						{  Menu.SetPWR=1500;
						comand_queue=9;}
						if(cnt_work_pump_tstop > 10)
					 {  comand_queue=10;
					   Menu.SetPWR=0;
					 }
					 
					}						
					 
			if(cnt_work_pump_tstop > 24)
					      cnt_work_pump_tstop=0;	
			if(_min > Tcurent)
			_min=Tcurent;
			if(_max < Tcurent)
			_max=Tcurent;	
			*/
			
		//	comand_queue=99;
		ret=102;// Kf;
		
		}
			break;
//**********************************************************************	
		case 2:
			if(Tcurent >= (Tstop-0.06 )) //Tstop-0.2 = -10.2  Tstop+0.5 => 9.1  8.9  9.2
						{  
							if(diff<=0.1)
							{if(diff > pre_value)
                 adjast( 100,120,1997,0);
								else
									adjast( 25,50,1998,0);
							}
							if(diff>0.1)
							adjast( 180,200,2001,0);	
							

						      comand_queue=3;
					
						}
		
			ret=103;
		
		
		
		 break;
//**********************************************************************	
		case 3:
			if(Tcurent >= (Tstop+0.0 )) //Tstop-0.2 = -10.2  Tstop+0.5 => 9.5  
						{  
					 
							if(((diff>0.1)) && (diff<0.3))
							{adjast( 55,80,1800,0);	
						      comand_queue=3;
							}
					else	if(diff>= 0.3)
							{comand_queue=99;
							Menu.SetPWR=2222;
							}
				
							
						}
					else 
						{	// Menu.SetPWR=0;
							comand_queue=99;
						}	
						
						
						
					if(((diff==0.0) || (diff>=-0.1)) && (diff <=0.1))
				      {
							adjast( 35,65,2011,0);	
						      comand_queue=3;
							}		
						
						
						
		
			ret=104;
		
		
		
		 break;		
//**********************************************************************	
		case 4:
		{	if(comand_queue!=3) 
				    cnt_work_pump=0;
			

			
			 	if(diff> 0.6)
							{comand_queue=99;
							Menu.SetPWR=2222;
							}
				else	if((diff<=0.6) && (diff>0.4))
							{adjast( 25,60,2000,0);	
						      comand_queue=3;
							}
					// 0.3 .... 0.2  empty		
			 else	if((diff<=0.1) && (diff>-0.1))	
					{
					adjast( 35,60,2001,0);	
						      comand_queue=3;
					
					}						
							
				
				else	if((diff<= 0.4) && (diff >= 0.3))
							{comand_queue=3;
							adjast( 15,40,2002,0);	
								//Menu.SetPWR=0;
							}
				else	if((diff<=  -0.1) && (diff > -0.2))			
							{comand_queue=3;
						//	Menu.SetPWR=1900;
									adjast( 75,90,2003,0);	
						      comand_queue=3;
						//	Menu.SetPWR=1900;
							}
		 
			 	else{Menu.SetPWR=0; comand_queue=95; }
				 
						
	//******************************					
						
			/*		if((diff<=0.6) && (diff>=0.4)) //&& (diff <=0.1))
				      {
							adjast( 20,40,1800);	//30,40
							  
						      comand_queue=3;
							}
				 	else if((diff<0.4)) 		
						      Menu.SetPWR=0;	
						
					*/	
						
		
			ret=104;
		
				}	
		 break;				
//******************************
	case 5:
	{		if(comand_queue!=3) 
				    cnt_work_pump=0;
			
			
			if(Tcurent >= (Tstop-0.1 )) //Tstop-0.2 = -10.2  Tstop+0.5 => 9.5  
				{  
					 
				if((diff<=0.4) && (diff>=0.2))
							{adjast( 55,80,1800,0);	
						      comand_queue=3;
							}
					else	if(diff> 0.4)
							{comand_queue=99;
							Menu.SetPWR=2000;//2004;
							}
				
							
				}
			
				else 
						{	// Menu.SetPWR=0;
							comand_queue=98;
						}	
						
	//******************************					
						
					if((diff<=0.3) && (diff>=-0.06)) //&& (diff <=0.1))
				      {
							adjast( 35,75,2001,0);	
							if(cnt_run>=2)	
									{Menu.SetPWR=2001;
										 cnt_run=0;
									}
						      comand_queue=3;
							}
				 	else if((diff<=0.1)) 		
						      Menu.SetPWR=0;	
						
						
						
		
			ret=104;
		
				}
		 break;		
			
//*************** 0.25 -0.25
	case 6:
	{	
		
		comand_queue=3;
		
		if( Tcurent <	Tstop+0.4 )//-9.6
		{    Menu.SetPWR=0;
	
	
		}
	 
	 if(Tcurent>=	(Tstop-0.06 )) // -0.1
	  {  
	  // Menu.SetPWR=2000; 
	  
	 
		if( (Tcurent-Tstop) > 1)
		{ 	Menu.SetPWR=2011;//adjast( 160,190,2000);	
		cnt_second=0;
			cnt_work_pump=0; First_in=TRUE;
		}
		
		else
			{
				/*	vector_direction=	Actuator(Tcurent, Tstop);//direct(Tcurent, Tstop,Deltime );
					if(vector_direction !=0)
					{ 
						if(vector_direction<10) vector_direction=10;
						
					adjast( vector_direction, 20,2002);	
					cnt_second=0;
					}*/
					if(First_in)
					 {cnt_work_pump=0;First_in=FALSE;}
					else
					{if(cnt_work_pump==0)First_in=TRUE;}
					
						
				
				adjast( 20, 25,2002,0);	
		 }
		}	
	ret=104;
	
		}
		break;
//************************************************
case 7:
	{	int time=10;
 
		
		
		if(Tcurent!=pre_value2)
		pre_value2=Tcurent ;	//prev_value1 ...2 глобальные переменные
		if( fabs(Tcurent-Tstop)==fabs(Tstop))
			Tcurent=pre_value2;
		 
		
	
    delta	=	Tcurent-Tstop;	
		
		
		comand_queue=3;
		
		if( Tcurent <	Tstop-0.1 )// 0.2
		     Menu.SetPWR=0;
 	
	 // -9.6 > Tcur >= -10.06
	 if(Tcurent>	(Tstop-0.1)) //   -10.1
	  { 
	   
						if((Tcurent-Tstop) > 0.8)  
							{ 	Menu.SetPWR=2001; 
							//	adjast( 140, 20,2111,1);
									//cnt_work_pump=0; 
								First_in=TRUE;
								iQueue=0; 
							}
				
					else
						{
							
								if(First_in)  {
								cnt_work_pump=0;
								//BYSY=FALSE;
								First_in=FALSE; 
								DELAY1000}
								else				
									{
								if(cnt_work_pump==0)
								First_in=TRUE;
									}
								
					  // if(iQueue==0) cnt_work_pump=0;
					
						//		if(iQueue==0)
						//		  BYSY=TRUE;
						
           
//*****************************************************************						 						
				//	 if(   ((Tcurent-Tstop) <= 1.5) && ((Tcurent-Tstop) >=  0.94)  )
						 if(   ((Tcurent-Tstop) <= 0.8) && ((Tcurent-Tstop) >=  0.4)  ) 
									 { 
							 
										 Enbl_2=FALSE;
										 Enbl_3=FALSE;
										 Enbl_4=FALSE;
										 Enbl_5=FALSE;
										before_cnt1=20;
										 if(Enbl_1==FALSE)
										 {DELAY500;
										 if(cnt_work_pump> before_cnt1)
												  before_cnt1=cnt_work_pump-before_cnt1;
										// cnt_work_pump=0; 
										 Enbl_1=	TRUE; }
										 
										/*   before_correct= Check_moving_temper( Tcurent,  Tstop, &iStrart,time,&Dis );
                    if(before_correct!=-1)
										 {
										 if(Dis==0)
												 { 
													 Tmp=1;
													 }
											}
										 
										 if(vector_direction< 0 )
										 {	iQueue=			adjast( before_cnt1-Tmp, 30,2002,1);}//30
										
										 else if(vector_direction>0)
										 {	iQueue=			adjast( before_cnt1+Tmp, 25,2022,1); }//30*/
										 
									  iQueue=		adjast( before_cnt1, 20,2011,11); 
									 } 
//*****************************************************************						 						
									 
					/*	 else if(((Tcurent-Tstop) < 0.94) && ((Tcurent-Tstop) >=  0.62)  )
						 {  Enbl_1=FALSE;
							  Enbl_3=FALSE;
							  Enbl_4=FALSE;
							  Enbl_5=FALSE;
							 before_cnt2=30;
							  if(Enbl_2==FALSE)
										 {DELAY500;
											  if(cnt_work_pump> before_cnt2)
												  before_cnt2=cnt_work_pump-before_cnt2;
											else ;
											// cnt_work_pump=0; 
											 Enbl_2=	TRUE; 
										 }*/
								/*  before_correct= Check_moving_temper( Tcurent,  Tstop, &iStrart,time,&Dis );
                   if(before_correct!=-1)
										 {
										 if(Dis==0)
												 { 
													 Tmp=1;
													 }
											}	 
										 
										 
							 if(vector_direction<0)
									 	iQueue=			adjast( before_cnt2-Tmp, 25,2003,2);//25
							
							 else if(vector_direction>0)
								    iQueue=			adjast( before_cnt2+Tmp, 20,2033,2);//25*/
						
							// else 
							/* iQueue=		adjast( before_cnt2, 30,2022,22);
							 
						 }*/ 
//******************************************************						 
					/*	  else if(((Tcurent-Tstop) < 0.56)&& ((Tcurent-Tstop) >=  0.38)  )
						 { 
               Enbl_1=FALSE;
							  Enbl_2=FALSE;
							  Enbl_4=FALSE;
							  Enbl_5=FALSE;
							 before_cnt3=35;
							 
							  if(Enbl_3==FALSE)
										 {DELAY500;
											
                    if(cnt_work_pump> before_cnt3)
												  before_cnt3=cnt_work_pump-before_cnt3;
											else ;
											// cnt_work_pump=0; 
											Enbl_3=	TRUE; 
										 }
										 
									/*	   before_correct= Check_moving_temper( Tcurent,  Tstop, &iStrart,time,&Dis );
                     if(before_correct!=-1)
										 {
										 if(Dis==0)
												 { 
													 Tmp=1;
													 }
											}
										 
										 
											 if(vector_direction<0)
														iQueue=			adjast( before_cnt3-Tmp, 32,2004,3);//20
										
											 else if(vector_direction>0)
														iQueue=			adjast( before_cnt3+Tmp, 30,2044,3);//20
											 */
											
											// else 	
									/*	iQueue=	 adjast( before_cnt3, 20,2033,33);
						      
					 }*/
//*************************************************************						 
					/*		else	if(((Tcurent-Tstop) < 0.31) &&((Tcurent-Tstop) > 0.19) )
							{	 Enbl_1=FALSE;
							  Enbl_2=FALSE;
							  Enbl_3=FALSE;
							  Enbl_5=FALSE;
							 before_cnt4=10;
							  if(Enbl_4==FALSE)
										 {DELAY500;
											 	if(cnt_work_pump> before_cnt4)
												  before_cnt4=cnt_work_pump-before_cnt4;
												 Enbl_4=	TRUE; 
										 }
										 
										/*  before_correct= Check_moving_temper( Tcurent,  Tstop, &iStrart,time,&Dis );
                     if(before_correct!=-1)
										 {
										 if(Dis==0)
												 { 
													 Tmp=1;
													 }
											}
														
											if(vector_direction<0) //
											{	
												
												iQueue=		adjast( before_cnt4-Tmp, 25+Tmp,2005,4);//14
											}
										
											else if(vector_direction>0) //
											 { iQueue=		adjast(   before_cnt4+Tmp, 20-Tmp,2055,4); 	//18
												 
											 }*/
											 // else 	
									/*	iQueue=		 adjast( before_cnt4, 20,2044,44);
											 
											 
											 
							 OneTime=TRUE; 				
						 } */
							//else;
//************************************************							
						else	if(((Tcurent-Tstop) <= 0.3)&&((Tcurent-Tstop) >= 0.0)  )
							{		 Enbl_1=FALSE;
							  Enbl_2=FALSE;
							  Enbl_3=FALSE;
							  Enbl_4=FALSE;
								before_cnt5=14;//14
							 
							  if(Enbl_5==FALSE)
										 {DELAY500; 
										 Menu.SetPWR=0;
											if(cnt_work_pump> before_cnt5)
												  before_cnt5=cnt_work_pump-before_cnt5;
											else ;
										  //  cnt_work_pump=0;
										 Enbl_5=	TRUE; 
										 }
             	if((vector_direction<0)&&((Tcurent-Tstop) >0.06 )) //-0.1
														 iQueue=		adjast( before_cnt5, 26,2006,5);	//14,26
								
								 else if((vector_direction>0)&&((Tcurent-Tstop) >= -0.06)) //+0.1
													 iQueue=		adjast( before_cnt5+6, 20,2066,55);	//18
								 
								 else //if((Tcurent-Tstop) < -0.06)
						   	       Menu.SetPWR=0;//  iQueue=		adjast( before_cnt5, 25,0,55); 
										 /* */
									// 		
										 
										 if(((Tcurent-Tstop) == 0.3) || ((Tcurent-Tstop) == 0.1))
											 cnt_work_pump=0;
									//	iQueue= adjast( before_cnt5, 26,2005,555);	 
							 }
							else
		            	{iQueue=0;		Menu.SetPWR=0;cnt_work_pump=0;}
						 
					 }
		}	
		else
			{iQueue=0;		Menu.SetPWR=0;cnt_work_pump=0;}
		if(delta != prev_difrent)
		{	
				vector_direction =+(	delta-prev_difrent);		
			prev_difrent	=delta;
			
		}
	ret=104;
	
		}
		break;				
//***********************************************************************		
case 8:
{	int out=0;
 //Limit_delt=0.6;
		comand_queue=3;	
		
		if(Tcurent!=pre_value2)
		pre_value2=Tcurent ;	//prev_value1 ...2 глобальные переменные
		if( fabs(Tcurent-Tstop)==fabs(Tstop))
			Tcurent=pre_value2;
		 
		
	
    delta	=	Tcurent-Tstop;	
		if((fabs(delta)<=0.19)&&(out==2))
		{;
			//stabilizacij
			Stabilizacij(&Tcurent,&Tstop,&Limit_delt);
		}
		else
		{//searching target
     out =	 polka(&Tcurent, &Tstop, &Polka, &Limit_delt,&iTime_hold_target);
		if(delta >= Limit_delt)  
							{ Menu.SetPWR=2111;
							} 	
		else 
	  { 	Menu.SetPWR=0;}
	 }
if(out==3)
{
//****************************************	 
	 if(Tcurent>	(Tstop-0.31)) //   -10.31
	  { 
	     
		
			if(delta > Limit_delt)  
							{ Menu.SetPWR=2111; 	
							 Enbl_5=TRUE;
							//	Enbl_4=FALSE;
							delta_min_afterzero=0;
								delta_max=0;
							delta_polka=23;	
								First_in=TRUE;
								iQueue=11; cnt_work_pump=0;
								
							} 
							
							
							
                      //0.19
						else	if((delta <= 0.38)&&(delta >=  -0.31)  )
							{	
								if(delta_polka>Tcurent)
								{ delta_polka = Tcurent;
									cnt_polka=0;
								}
								if((delta_polka ==Tcurent) && (cnt_polka>10)) 
								{ delta_polka = Tstop-delta_polka;
									Limit_delt=Limit_delt+ delta_polka;
								}
								else
									  delta_polka = 0;
								
								if(delta<=0)
								{	if(delta_min_afterzero>Tcurent)  delta_min_afterzero=Tcurent;
									if(delta_max<Tcurent)  delta_max=Tcurent;
									
								}
								
								iQueue=12;	 
							 //----------- target 
							 if((delta<0.07)&&(delta >0))  //for 0.06
												if(First_in==TRUE)//попали в полку первый раз
														 {cnt_work_pump=0;
														 Menu.SetPWR=0;
																	 First_in=FALSE;
															 iQueue=13;
														 }
												else
														 {
															 if(Enbl_5==TRUE)
															 {iQueue=14;
																if(cnt_work_pump>25) // пока не пройдет пауза. нижняя полка
																	{Enbl_5=FALSE;
																	Enbl_1=TRUE;
																		Enbl_2=TRUE;
																		Enbl_3=TRUE;
																	}
															
															 }
											
											 
													 
													if((delta >=0.00) && (delta <=0.4))//0.12
														{ 
																if(Enbl_1==TRUE){cnt_work_pump=36;
																Enbl_1=FALSE;
																Enbl_2=TRUE;
														    Enbl_3=TRUE;
																Enbl_4=TRUE;}
																
																if((delta <=0.14)&&(delta>=0.06))
      															iQueue=	adjast( 20, 22,2057,16);	
																else if((delta<=0.07)&& (delta>=-0.07))
																	 iQueue=	adjast( 14, 26,2057,116);	
															//	else if(delta>0.19)
															//		 iQueue=	adjast( 35, 20,2057,1116);	
														}
																									
												else if((delta_min_afterzero > 0) && (delta>=0.15))//0.19
												{
 													if(Enbl_2==TRUE)
														    {cnt_work_pump=20;
																Enbl_1=TRUE;
																Enbl_2=FALSE;
														    Enbl_3=TRUE;
													      Enbl_4=TRUE;
													        }
												           iQueue=	adjast( 20, 20,2055,17);//20/25
												}
												else if((delta_min_afterzero <0) && (delta>=0.15))//0.19
												{
 													if(Enbl_2==TRUE)
														   {cnt_work_pump=20;
																Enbl_1=TRUE;
																Enbl_2=FALSE;
														    Enbl_3=TRUE;
													      Enbl_4=TRUE;
													        }
												           iQueue=	adjast( 45, 20,2055,-17);//20/25
												}
														
												
												
												
												 else if((delta<= 0.0) && (delta>= -0.27))
												 {
													 if(Enbl_3==TRUE){cnt_work_pump=31;
																Enbl_1=TRUE;
																Enbl_2=TRUE;
														    Enbl_3=FALSE;
													      Enbl_4=TRUE; }
                          if(delta_min_afterzero < -0.12)
														   if((delta >= -0.2) && (delta <=-0.07))
													    iQueue=	adjast( 25, 30,2056,-18);	 
												 }
												 
												 else	if((delta_min_afterzero <0) &&((delta  >=0.3)&& (delta < 0.38)))//0.12
														{ 
																if(Enbl_4==TRUE){cnt_work_pump=30;
																Enbl_4=FALSE;
																Enbl_2=TRUE;
														    Enbl_3=TRUE;
																Enbl_1=TRUE;	
																}
															//	if(delta <=0.12)
      												//			iQueue=	adjast( 14, 26,2057,16);	
														//		else if(delta==0.19)
																	 iQueue=	adjast( 20, 20,2022,-19);	
														}
												 
												 
                          else 
													{ Menu.SetPWR=0;iQueue=19;}
												 
													
													// Enbl_4=FALSE;
												}
												
								   }
              	 	
							else	if(((Tcurent-Tstop) < -0.12)&&( Enbl_4==FALSE )  )
							{cnt_work_pump=0;
						  	Menu.SetPWR=0;
								iQueue=20;
								Enbl_1=TRUE;
								Enbl_2=TRUE;
							  Enbl_3=TRUE;
							}
					else {Menu.SetPWR=0; iQueue=21; }		
		}	
		else
			{iQueue=22;		Menu.SetPWR=0;cnt_work_pump=0;}
		if(delta != prev_difrent)
		{	
				vector_direction =+(	delta-prev_difrent);		
			prev_difrent	=delta;
			
		}
	ret=104;
	}	
		}
break;
		
		default:
			ret =-1;
			break;
		
		
	}
	
	return ret;
	

}
int adjast( int TimingWork,int TimigPause, double p, int que)
{
 
	  //самоблокировка вызова пока не отработает предыдущий вызов
	if(que!=0) 
	{	
        if(( cnt_work_pump >=0 ) && ( cnt_work_pump <  TimingWork ) )
					 { 	Menu.SetPWR=p;  
						 
					 }
				 
			else	if(cnt_work_pump >= TimingWork)
					 {   
						  
					   Menu.SetPWR=0;
					 }
	 
	}
			if(cnt_work_pump>=(TimingWork+TimigPause))
				  {  cnt_work_pump=0;
						 Menu.SetPWR=0;
						Str=0;
					 que=0;
					}
	
return que;
}
//************************************************************

double direct(double Tcurent, double Tstop, volatile int DeltaTime )
{
	double difCurrentPrev,difCurrentStop;	
//************************************	
	difCurrentStop=Tcurent-Tstop; 
//**************************************	
	difCurrentPrev=Tcurent-prev_direction; // -0.1  +0.1
//**************************************	
	if(difCurrentPrev!=0)
		 prev_difrent=difCurrentPrev;
	
	 	if(Tcurent!=prev_direction)
		{	prev_direction=Tcurent ;	//
			if(First_in)
			{
				Deltime=cnt_second;
				First_in = FALSE;	
			}
			else
			{second = cnt_second-Deltime;
				First_in=TRUE;
			  return difCurrentPrev; }  
			
			}
		if(difCurrentPrev==0) 
			difCurrentPrev=prev_difrent;
		
			
	return -1;//difCurrentPrev;
}



double Check_moving_temper(double Tcurent, double Tstop, int *Begin_calculate,int time,double *Dis2 )
{

	if(*Begin_calculate==1)
					{
					//*Dis2 = Tcurent-Tstop; 	
						prv_slope= Tcurent;
						cnt_second=0;
						Deltime=cnt_second;
					*Begin_calculate = 0;	
						
					}
		 else
					{
						if((time-cnt_second)>=0)
						{  
							second = cnt_second-Deltime;
						}	
            else
						{  *Begin_calculate=1;
									// скорость изменения 
							*Dis2=  Tcurent-prv_slope;// -0.1  +0.1
						    	if(*Dis2==0.06)    *Dis2=0.1;
						else	if(*Dis2== -0.06)  *Dis2=-0.1;	
							
							 	return 1;   
						}										
					}
					
return -1;
}

//************************************************************
double Time_Slope(double Tcurent, double Tstop, int *Begin_calculate,int time,double *Dis2 )
{
	

				if(*Begin_calculate==1)
					{prv_slope= Tcurent;
						cnt_second=0;
						Deltime=cnt_second;
					*Begin_calculate = 0;	
						
					}
			 else
					{
						if((time-cnt_second)>=0)
						{  second = cnt_second-Deltime;}
							
						else
									{  *Begin_calculate=1;
									// скорость изменения 
										
								   Dilta=  Tcurent-prv_slope; // -0.1  +0.1
			            if(Dilta <0) Dilta=Dilta*-1;
										
										
							//		SpDelta=Dilta/second;
							//		time_work=	Dis/SpDelta;	
										time_work=	(*Dis2/Dilta)*10;//SpDelta;	
										if(	time_work <0) 	time_work=time_work*-1;		
										//return 	time_work;	
										
									}							
						
					}
if(time_work==0)
return 	-1;	
else return time_work;
 
}
//*************************************************************
double Actuator(double Tcurent, double Tstop )
{
//Dilta,DistanceStop, SpDelta,time_work;	
//************************************	
	DistanceStop=Tcurent-Tstop; 
//**************************************

//************************************	
if(Tcurent != prev_Tcurrent)
			{Dilta=   Tcurent-prev_Tcurrent; // -0.1  +0.1
			if(Dilta <0) Dilta=Dilta*-1;
		 
//**************************************	
	
	if(Dilta !=0)//!=prev_delta)
	  { 
				if(First_in)
					{
						Deltime=cnt_second;
						First_in = FALSE;	
					}
			 else
					{second = cnt_second-Deltime;
						First_in=TRUE;
					//  return difCurrentPrev;
					}  
	

// скорость изменения 
	SpDelta=Dilta/second;
					
time_work=	DistanceStop/SpDelta;			 
if(	time_work <0) 
	  time_work=time_work*-1;				

		}
	}	
		if(Tcurent != prev_Tcurrent)
		prev_Tcurrent=Tcurent;
	prev_delta=Dilta;
return 	time_work;	
//return -1;
}
//*********************************************
double okruglennye(double chislo, long znaki)
{
    return round(chislo * pow(10, znaki)) / pow(10, znaki);
}
//************************************************************


//***********************************************************
int EnableMotor(_Bool St)
{
    if(St)// Включаем таймер
    TIM_Cmd(TIM3, ENABLE);
		else
	   TIM_Cmd(TIM3, DISABLE);

}

int Starter(uint16_t ADC, int status)
{//test();
 
						//	if((status!=0x1)||(status!=0x2))
									 
											if((adc_pa1>=1700))// && (Enbl_Starter==TRUE))  
													{   
																
  													       if(Enbl_Starter==TRUE)
																				{ 
																				Enbl_Starter=FALSE;
																			  Start;	DELAY500;Stop;	
																	 		  starter_second=0;
																				}
														 		
													      	if(starter_second>=1)
																				{Menu.SetPWR=0;
																					Stop;	
																					Enbl_Starter=FALSE;
																					cnt_work_pump=100;
																				}	

																/*if(starter_second>=3)
																				{Enbl_Starter=FALSE;
																					Menu.SetPWR=0;
																					cnt_work_pump=20;
																					Stop;
																				starter_second=0;	
																			//	 	
																				}	*/		
													 
													}
										  	
													
													else
													{	
													Stop;
														
													}
	             
	 
	if((adc_pa1 > 1000) && (adc_pa1 < 1300))	
	{ starter_second=0;
	Enbl_Starter=FALSE;
 
		return 1; 	
	
	}  //работает
	
	
	if(adc_pa1<900)// not working overtemperature hating
	{
		if(starter_second>5)
		{starter_second=0;
		 Enbl_Starter=TRUE;
	 
		}
	}
			
return -1; 			
}

int Status_Starter(uint16_t ADC)
{
 	if((ADC > 1000) && (ADC < 1300))	
	{  return 1; 	}  //работает

	else if(ADC < 1000)
	{ return 2; 	}  //ожидание отключен

	else return -1;
}
//************* autosearch ********************
double Autosearch(_Bool Sw ,double T_stop,double T_curent, double Delta_stop_before)
{
	double dif=0;
delt =	T_curent-T_stop;//+Del_stop_before;
// поиск идёт всегда сверху вниз.
	if(delt>0)
	{ Menu.SetPWR =2222;
	delta_max=0;Delta_stop_before=0;
	Enbl_3=TRUE;	
	}
	else if((delt==0)&& (Enbl_3==TRUE))
	{Enbl_3=FALSE;
	//подсчитать переходы
	Cnt_points++;
	
	}
	else//определить проскок
	{		Menu.SetPWR =0;
	  
	if(delta_max>T_curent)
		{ delta_max=T_curent;
		 Del_stop_before=fabs(delta_max-	T_stop);
	  }
		 
 }
 
return Del_stop_before;
	



}


void test(void)
{
Start;
	DELAY1000;
	
	Stop;
	DELAY1000;

}