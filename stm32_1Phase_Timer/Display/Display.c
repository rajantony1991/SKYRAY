/*
 * Display.c
 *
 *  Created on: Dec 21, 2018
 *      Author: karthik1
 */


#include "Display.h"
volatile uint8_t  Segment = 1 ;
extern RTC_DateTypeDef gDate;
extern RTC_TimeTypeDef gTime;
//              0 1 2 3 4 5 6 7 8 9 A B C D E F G H I J K L M N O P Q R S T U V W X Y Z - n h i a
const bool A[]={1,0,1,1,0,1,1,1,1,1,1,0,1,0,1,1,1,0,0,0,0,0,1,0,0,1,1,0,1,0,0,0,0,0,0,1,0,1,0,0,1};
const bool B[]={1,1,1,1,1,0,0,1,1,1,1,0,0,1,0,0,0,1,0,1,1,0,1,0,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,1};
const bool C[]={1,1,0,1,1,1,1,1,1,1,1,1,0,1,0,0,1,1,0,1,0,0,1,1,1,0,1,0,1,0,1,1,0,1,1,0,0,1,1,0,1};
const bool D[]={1,0,1,1,0,1,1,0,1,1,0,1,1,1,1,0,1,0,0,1,1,1,0,0,1,0,0,0,1,1,1,1,1,0,1,1,0,0,0,0,1};
const bool E[]={1,0,1,0,0,0,1,0,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,0,1,0,1,1,1,0,0,0,0,0,1,1,1,1};
const bool F[]={1,0,0,0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,0,0,1,1,0,1,1,1,0,1,1,1,0,0,1,1,1,0};
const bool G[]={0,0,1,1,1,1,1,0,1,1,1,1,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,0,1,0,1};

void Display_event(uint8_t LED1 , uint8_t LED2 , uint8_t LED3 , uint8_t LED4 , uint8_t Blink)
{

	  switch(Segment)
			 	   {
			 	   case 1 :
			 	   {
			 	  	 GPIOA->BSRR  = (uint32_t)(LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin);
			 	  	 GPIOA->BSRR = A_Pin|D_Pin;
			 	     GPIOF->BSRR = B_Pin|C_Pin;
			 	  	 GPIOB->BSRR = E_Pin|F_Pin|G_Pin|DB_Pin;
                     if(Blink==1||Blink==5||Blink==6||Blink==8)
                     {
                    	 if(((gTime.Seconds%2)==0)||(HAL_GPIO_ReadPin(GPIOB,SW_up_Pin)==0)||(HAL_GPIO_ReadPin(GPIOB,SW_down_Pin)==0))
                     {
                    	GPIOA->BRR  = (A[LED1]<<15)|(D[LED1]<<12);
                    	GPIOF->BRR  = (B[LED1]<<7) |(C[LED1]<<6);
                    	GPIOB->BRR  = (E[LED1]<<15)|(F[LED1]<<14)|(G[LED1]<<13);

                     }
                     else
                     {


                     }
                     }
                     else
                     {

                      GPIOA->BRR  = (A[LED1]<<15)|(D[LED1]<<12);
                      GPIOF->BRR  = (B[LED1]<<7) |(C[LED1]<<6);
                      GPIOB->BRR  = (E[LED1]<<15)|(F[LED1]<<14)|(G[LED1]<<13);


                     }

			 	  	 GPIOA->BRR = (uint32_t)LED1_Pin ;


			 	     break;
			 	   }
			 	   case 2 :
			 	   {

			 	  	 GPIOA->BSRR   =(uint32_t)(LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin);
			 	  	 GPIOA->BSRR = A_Pin|D_Pin;
			 	  	 GPIOF->BSRR = B_Pin|C_Pin;
			 	  	 GPIOB->BSRR = E_Pin|F_Pin|G_Pin|DB_Pin;

			 	  	 if(Blink==8||Blink==9||Blink==0)
			 	  	 {
			 	  		 if((Blink==0)&&(gTime.Seconds%2)==0)
			 	  		 {

			 	  		   GPIOB->BSRR  = DB_Pin;

			 	  		 }
			 	  		 else
			 	  		 {
			 	  	     GPIOB->BRR  = DB_Pin;
			 	  		 }
			 	  	 }
			 	  	 if(Blink==2||Blink==5||Blink==6||Blink==8)
			 	  	 {
			 	  		if(((gTime.Seconds%2)==0)||(HAL_GPIO_ReadPin(GPIOB,SW_up_Pin)==0)||(HAL_GPIO_ReadPin(GPIOB,SW_down_Pin)==0))
			 	  	   {
			 	  			GPIOA->BRR  = (A[LED2]<<15)|(D[LED2]<<12);
			 	  			GPIOF->BRR  = (B[LED2]<<7) |(C[LED2]<<6);
			 	  	        GPIOB->BRR  = (E[LED2]<<15)|(F[LED2]<<14)|(G[LED2]<<13);


			 	  	   }
			 	  	   else
			 	  	   {


			 	  	   }

			 	  	 }
			 	  	 else
			 	  	 {

			 	  	 GPIOA->BRR  = (A[LED2]<<15)|(D[LED2]<<12);
			 	  	 GPIOF->BRR  = (B[LED2]<<7) |(C[LED2]<<6);
			 	  	 GPIOB->BRR  = (E[LED2]<<15)|(F[LED2]<<14)|(G[LED2]<<13);

			 	  	 }

			 	     GPIOA->BRR  =(uint32_t)LED2_Pin ;
			 	  	 break;

			 	   }
			 	   case 3 :
			 	   {

			 	  	 GPIOA->BSRR =  (uint32_t)(LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin);
			 	  	 GPIOA->BSRR = A_Pin|D_Pin;
			 	     GPIOF->BSRR = B_Pin|C_Pin;
			 	  	 GPIOB->BSRR = E_Pin|F_Pin|G_Pin|DB_Pin;
			 	  	 if(Blink==8||Blink==9||Blink==0)
			 	  	 {
			 	  		 if((Blink==0)&&(gTime.Seconds%2)==0)
			 	  		 {

			 	  		   GPIOB->BSRR  = G_Pin;

			 	  		 }
			 	  		 else
			 	  		 {
			 	  	     GPIOB->BRR  = G_Pin;
			 	  		 }
			 	  	 }
			 	  	 if(Blink==3||Blink==5||Blink==7||Blink==9)
			 	  	 {
			 	  		if(((gTime.Seconds%2)==0)||(HAL_GPIO_ReadPin(GPIOB,SW_up_Pin)==0)||(HAL_GPIO_ReadPin(GPIOB,SW_down_Pin)==0))
			 	  	   {
			 	  			GPIOA->BRR  = (A[LED3]<<15)|(D[LED3]<<12);
			 	  			GPIOF->BRR  = (B[LED3]<<7) |(C[LED3]<<6);
			 	  			GPIOB->BRR  = (E[LED3]<<15)|(F[LED3]<<14)|(G[LED3]<<12);

			 	  	   }
			 	  	   else
			 	  	   {

			 	  	   }


			 	  	 }
			 	  	 else
			 	  	 {
			 	  	 GPIOA->BRR  = (A[LED3]<<15)|(D[LED3]<<12);
			 	  	 GPIOF->BRR  = (B[LED3]<<7) |(C[LED3]<<6);
			 	     GPIOB->BRR  = (E[LED3]<<15)|(F[LED3]<<14)|(G[LED3]<<12);
			 	  	 }
			 	  	 GPIOA->BRR = (uint32_t)LED3_Pin;
			 	  	 break;
			 	   }
			 	   case 4:
			 	   {
			 	  	 GPIOA->BSRR =(uint32_t)(LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin);
			 	  	 GPIOA->BSRR = A_Pin|D_Pin;
			 	  	 GPIOF->BSRR = B_Pin|C_Pin;
			 	  	 GPIOB->BSRR = E_Pin|F_Pin|G_Pin|DB_Pin;
			 	  	 if(Blink==4||Blink==5||Blink==7||Blink==9)
			 	  	 {
			 	  		if(((gTime.Seconds%2)==0)||(HAL_GPIO_ReadPin(GPIOB,SW_up_Pin)==0)||(HAL_GPIO_ReadPin(GPIOB,SW_down_Pin)==0))
			 	  		{
			 	  		 GPIOA->BRR  = (A[LED4]<<15)|(D[LED4]<<12);
			 	  		 GPIOF->BRR  = (B[LED4]<<7) |(C[LED4]<<6);
			 	  		 GPIOB->BRR  = (E[LED4]<<15)|(F[LED4]<<14)|(G[LED4]<<13);

			 	  		}
			 	  		else
			 	  		{


			 	  		}

			 	  	 }
			 	  	 else
			 	  	 {
			 	   	 GPIOA->BRR  = (A[LED4]<<15)|(D[LED4]<<12);
			 	     GPIOF->BRR  = (B[LED4]<<7) |(C[LED4]<<6);
			 	     GPIOB->BRR  = (E[LED4]<<15)|(F[LED4]<<14)|(G[LED4]<<13);
			 	  	 }
			 	  	 GPIOA->BRR  =(uint32_t)LED4_Pin ;
			 	  	 break;
			 	   }



			 	   }
			  Segment++;
			 	 	    if(Segment>4)
			 	 	   {

			 	 	    Segment = 1;

			 	 	   }







}
