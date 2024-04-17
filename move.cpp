#include <Arduino.h>

//match previous STM32 to ESP32 module
#define PIN15 13
#define PIN3  12
#define PIN2  27
#define PIN1  33

#define Y 1
#define X 2

static int ax,ay;	//current x & y
static char statex=0,statey=0;
static char cptaccelx=0;
static char cptaccely=0;
static int act_sens_x=0,act_sens_y=0;
static int desired_sens_x=0,desired_sens_y=0;
//static char previous_edge_x=0;
//static char previous_edge_y=0;

static int CPT_ACCEL_X=100;
static int CPT_ACCEL_Y=0;

int d=2;

extern int STEP_DIVIDER;
extern int INV_Y;

int stby_Y=0;
int stby_X=0;

int STDBY=500;
int STDBX=500;

void init_pos()
{
	ax=0;
	ay=0;
  pinMode(PIN15,OUTPUT);
  pinMode(PIN3,OUTPUT);
  pinMode(PIN2,OUTPUT);
  pinMode(PIN1,OUTPUT);

}

void cycle_down()
{
	static int state=0;
	
	switch (state)
	{
		case 0 : 
		{
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1); // 0  (3)
     digitalWrite(PIN15,1);
		 state=1;
		break;
		}
		
		case 1 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0); // 0  
      digitalWrite(PIN3,0);

			state=2;
			
		break;
		}
		case 2 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1); // 0 1
     digitalWrite(PIN1,1);
			state=3;
		break;
		}
		case 3 : 
		{
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);	// 1 
      digitalWrite(PIN15,0);
			state=4;
		break;
		}
		case 4 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1); // 1 2
      digitalWrite(PIN2,1);
			state=5;
		break;
		}
		case 5 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0); // 2
      digitalWrite(PIN1,0);
			state=6;
		break;
		}
		case 6 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1); // 2 3
      digitalWrite(PIN3,1);
			state=7;
		break;
		}
		case 7 : 
		{
				//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0); // 3
      digitalWrite(PIN2,0);
			state=0;
		break;
		}
	}
	
	

}

void cycle_up(char updown)
{
	static int state=0;
	
	switch (state)
	{
		case 0 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1); // 2 3 
      digitalWrite(PIN2,1);
			//state=1;
		break;
		}
		case 1 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0); // 2 
      digitalWrite(PIN3,0);
			//state=2;
		break;
		}
	case 2 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1); // 1 2 
      digitalWrite(PIN1,1);
      //state=3;
		break;
		}
	case 3 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0); // 1 
      digitalWrite(PIN2,0);
			//state=4;
		  break;
		}
	case 4 : 
		{
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);	// 1  0
       digitalWrite(PIN15,1);
			//state=5;
		break;
		}
	
		case 5 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0); // 0
      digitalWrite(PIN1,0);
			//state=6;
		break;
		}
	
	case 6 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1); // 3 0
      digitalWrite(PIN3,1);
			//state=7;
		break;
		}
	
	case 7 : 
		{
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0); // 3
    digitalWrite(PIN15,0);
		//state=0;
		break;
		}	
	
	}
	
	
	if (updown==1) 
	{
		if (state!=7) state++;
		else state=0;
	}
	
	if (updown==0)
	{
		if (state==0) state=7;
		else state--;
	}
	

}

void Moteur_Y_Off()
{
	
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0); 
      digitalWrite(PIN2,0);
      digitalWrite(PIN3,0);
      digitalWrite(PIN1,0);
      digitalWrite(PIN15,0);

	
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0); 
		
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
	
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);	
		

}



void moteur_X_Off()
{
		
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0); 
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,0); 
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,0); 

}

void cycle_NEMA17(char updown)
{
/*static int state=0;
	
  //pilote la borche de sens
	if (updown) //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);	
	else //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);	
		
	switch (state)
	{
		case 0 : 
		{
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);	
			
			state=1;
		break;
		}
		case 1 : 
		{
			 //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);	
 			state=0;
		break;
		}
	
	}
*/
}




void Move_Step(char voie)
{
	
	
	if (voie==X)
	{
		//un cycle dans le sens sp�cifi�
		//	cycle_NEMA17(act_sens_x);
	}
	
	if (voie==Y)
	{
		//un cycle dans le sens sp�cifi�
			cycle_up(act_sens_y);
	}
	
}

void Move_Sens_Change(char voie,char *state)
{
	if (voie==X)
	{
			*state=CPT_ACCEL_X;
			cptaccelx=CPT_ACCEL_X;
		  act_sens_x=desired_sens_x;
	}
	
	if (voie==Y)
	{
			*state=CPT_ACCEL_Y;
			cptaccely=CPT_ACCEL_Y;
		  act_sens_y=desired_sens_y;
	}
}

void Move_Make_Step_X()
{
						/*if (act_sens_x!=0)
						{
							//Ajuste direction
							//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,1);
							
							act_sens_x=1;
							
							Move_Step(X);
							ax++;
						
						}
						else
						{
							//Ajuste direction
							
							//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,0);
							act_sens_x=0;
							Move_Step(X);
							ax--;
							//sens 2
						}*/          
}

void Move_Make_Step_Y()
{
						if (act_sens_y!=0)
						{
							//Ajuste direction
							//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,1);
							act_sens_y=1;
							Move_Step(Y);
							ay++;
						
						}
						else
						{
							//Ajuste direction
							
							//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,0);
							act_sens_y=0;
							Move_Step(Y);
							ay--;
							//sens 2
						}
}

void Move_Update(int dx,int dy,char Xenable,char Yenable)
{ 
	//s'il faut bouger
	if (Xenable)
	{
		//si consigne differente position
		if (ax!=dx)
		{
				stby_X=STDBX;
					
				if (ax>dx) desired_sens_x=0;
				else desired_sens_x=1;
			
				//sens 1
				if (act_sens_x!=desired_sens_x)	//si chgt sens
				{
					Move_Sens_Change(X,&statex);
				}
				
				//si tempo accel pas fini
				if (statex>0)
				{
						//on decompte de moins en moins longtemps
						if (cptaccelx>0) cptaccelx--; 	//decremente mais n'avance pas
					
							//decremente le compteur du dessus recharge celui du dessous et avance
							else 
							{
								statex--;
								cptaccelx=statex/10;
								
							//fait un pas de temps en temps ...
							Move_Make_Step_X();
							}
						}
				//accel finie
				else
				{
						//fait un pas sans accel
					Move_Make_Step_X();
				}
		}
		else
			//si on est arriv�, on reset les accel et on informe 
		{
	
		//en stop
		act_sens_x=2;
			
			if (stby_X>0)  stby_X--;
			if (stby_X==0) //desactive moteur
			moteur_X_Off();
		}
	}
	
	////////////////Y
	
	//s'il faut bouger
	if (Yenable)
	{
		if (ay!=dy)
		{
				stby_Y=STDBY;
				if (ay>dy) desired_sens_y=0;
				else desired_sens_y=1;
			
				//sens 1
				if (act_sens_y!=desired_sens_y)	//si chgt sens
				{
					Move_Sens_Change(Y,&statey);
				}
				
				//si tempo accel pas fini
				if (statey>0)
				{
						//on decompte de moins en moins longtemps
						if (cptaccely>0) cptaccely--; 	//decremente mais n'avance pas
					
							//decremente le compteur du dessus recharge celui du dessous et avance
							else 
							{
								statey--;
								cptaccely=statey/10;
								
							//fait un pas de temps en temps ...
							Move_Make_Step_Y();
							}
						}
				//accel finie
				else
				{
						//fait un pas sans accel
					Move_Make_Step_Y();
				}
		}
		else
			//si on est arriv�, on reset les accel et on informe 
		{
			//en stop
			act_sens_y=2;
			if (stby_Y>0) stby_Y--;
			//desactive le moteur
			if (stby_Y==0) Moteur_Y_Off();
		}
	}
	
}

int Move_Get_Y()
{
	return (ay);
}
