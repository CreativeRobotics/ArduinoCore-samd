/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "variant.h"

/*
PORTA, 8,  PIO_TIMER_ALT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH0, TCC1_CH2, EXTERNAL_INT_NMI
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  //PORT A 0-29 
	{ PORTA, 0,  PIO_DIGITAL,  PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, 		//0
	{ PORTA, 1,  PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, 		//1
	{ PORTA, 2,  PIO_ANALOG,   (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG), ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, //2
	{ PORTA, 3,  PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, 		//3
	{ PORTA, 4,  PIO_ANALOG,   (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG), ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, 		//4
	{ PORTA, 5,  PIO_ANALOG,   (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG), ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, 		//5
	{ PORTA, 6,  PIO_TIMER,    (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel6, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6 }, 		 //6
	{ PORTA, 7,  PIO_TIMER,    (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel7, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7 }, 		 //7
	{ PORTA, 8,  PIO_TIMER,    (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI  },//8
	{ PORTA, 9,  PIO_TIMER_ALT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH1, TCC1_CH3, EXTERNAL_INT_9 }, 	 //9
	{ PORTA, 10, PIO_TIMER_ALT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH2, TCC0_CH2, EXTERNAL_INT_10 },  //10
	{ PORTA, 11, PIO_TIMER_ALT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_11 },	 //11
	{ PORTA, 12, PIO_TIMER,    (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER),     No_ADC_Channel, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_12 },  //12
	{ PORTA, 13, PIO_TIMER,    (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER),     No_ADC_Channel, PWM2_CH1, TCC2_CH1, EXTERNAL_INT_13 },  //13
	{ PORTA, 14, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM,     NOT_ON_TIMER, EXTERNAL_INT_14 }, //14
	{ PORTA, 15, PIO_TIMER,    (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER),     No_ADC_Channel, PWM3_CH1, TC3_CH1, EXTERNAL_INT_15 },   //15
	{ PORTA, 16, PIO_SERCOM,   (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, //16 UART TX
	{ PORTA, 17, PIO_SERCOM,   (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, //17 UART RX
	{ PORTA, 18, PIO_TIMER,    (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER),     No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_2 },    //18
	{ PORTA, 19, PIO_TIMER_ALT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_3 }, //19
	{ PORTA, 20, PIO_TIMER_ALT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH6, TCC0_CH6, EXTERNAL_INT_4 }, //20
	{ PORTA, 21, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, //21
	{ PORTA, 22, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, 	//22
	{ PORTA, 23, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, 	//23
	{ PORTA, 24, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, 	//24
	{ PORTA, 25, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 }, 	//25
	{ PORTA, 27, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, //26
	{ PORTA, 28, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, //27
	{ PORTA, 30, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, //28
	{ PORTA, 31, PIO_DIGITAL,  (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, //29
	
	//PORT B 30-37
	
	{ PORTB, 2,  PIO_ANALOG,    (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG), ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, 		//30
	{ PORTB, 3,  PIO_DIGITAL,   (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, 	//31
	{ PORTB, 8,  PIO_ANALOG,    (PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel2, PWM4_CH0, TC4_CH0, EXTERNAL_INT_8 }, 	//32
	{ PORTB, 9,  PIO_ANALOG,    (PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel3, PWM4_CH1, TC4_CH1, EXTERNAL_INT_9 }, 	//33
	{ PORTB, 10, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH4, TCC0_CH4, EXTERNAL_INT_10 }, //34
	{ PORTB, 11, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH5, TCC0_CH5, EXTERNAL_INT_11 }, //35
	{ PORTB, 22, PIO_DIGITAL,   (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  //36
	{ PORTB, 23, PIO_DIGITAL,   (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },  //37
	
	
	//USB 38,39,40
	{ PORTB, 8,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, 		//38 	USB Host enable
	{ PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },   		//39 	USB/DM
	{ PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },   		//40 	USB/DP
	
	//ADC 41-54
	{ PORTA, 2,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },	//41
	{ PORTA, 3,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },	//42
	{ PORTB, 8,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel2,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },	//43
	{ PORTB, 9,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel3,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },	//44
	{ PORTA, 4,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },	//45
	{ PORTA, 5,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },	//46
	{ PORTA, 6,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel6,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },	//47
	{ PORTA, 7,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel7,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },	//48
	{ PORTB, 2,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },	//49
	{ PORTB, 3,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel11, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },	//50
	{ PORTA, 8,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel16, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI },//51
	{ PORTA, 9,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel17, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },	//52
	{ PORTA, 10, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel18, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },	//53
	{ PORTA, 11, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel19, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },	//54
	//DAC 55
	{ PORTA, 2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, //55 DAC/VOUT
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM1_Handler()
{
  Serial1.IrqHandler();
}
