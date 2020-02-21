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
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{ 
  //PORT A 0-24 
	{ PORTA, 2,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, 		//0 DAC0
	{ PORTA, 3,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, 		//1 Sensors ADC0 DAC EXT REF
	{ PORTA, 4,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, 		//2 Buttons ADC6
	{ PORTA, 5,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, 		//3 DAC1
	{ PORTA, 6,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, 		//4 SDHC CD
	{ PORTA, 7,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel7, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, 		//5 SDJC WP
	{ PORTA, 8,  PIO_SERCOM, PIN_ATTR_DIGITAL, ADC_Channel8, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, 		//6	SDHC SERCOM 0.0 MOSI	SDHC-CMD
	{ PORTA, 9,  PIO_SERCOM, PIN_ATTR_DIGITAL, ADC_Channel9, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, 		//7	SDHC SERCOM 0.1 SCK 	SDHC-DATA0
	{ PORTA, 10, PIO_SERCOM, PIN_ATTR_DIGITAL, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, 	//8	SDHC SERCOM 0.2 MISO 	SDHC-DATA1
	{ PORTA, 11, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel11, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, 	//9	SDHC 					SDHC-DATA2
	{ PORTA, 12, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, //10 IMU SDA
	{ PORTA, 13, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 }, //11 IMU SCL
	{ PORTA, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, //12 IMU INT
	{ PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, //13 VR EN
	{ PORTA, 16, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH0, TC2_CH0, EXTERNAL_INT_0 }, 			//14 Haptics SERCOM 3.1
	{ PORTA, 17, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH1, TC2_CH1, EXTERNAL_INT_1 }, //15 Haptics SERCOM 3.0
	{ PORTA, 18, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH2, TC3_CH0, EXTERNAL_INT_2 }, //16 Haptics SERCOM 3.2
	{ PORTA, 19, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH3, TC3_CH1, EXTERNAL_INT_3 }, //17 Haptics SERCOM 3.3
	{ PORTA, 20, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH0, NOT_ON_TIMER, EXTERNAL_INT_4 }, //18 I2S-FSO	Haptics Pin 15 PWM0_PIN
	{ PORTA, 21, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH1, NOT_ON_TIMER, EXTERNAL_INT_5 }, //19 I2S-SDO	Haptics Pin 16 PWM1_PIN
	{ PORTA, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, 	//20	I2S-SDI	Haptics Pin 17
	{ PORTA, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, 	//21	I2S-FS1	Haptics Pin 18
	{ PORTA, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, //22 	USER BUTTON
	{ PORTA, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, //23	SWCLK MUX0
	{ PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 }, //24	SWDIO MUX1
	
	//PORT B 25-46
	{ PORTB, 0, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, ADC_Channel12, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, 		//25  	Sensors SERCOM 5.2
	{ PORTB, 1, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, ADC_Channel13, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, 		//26  	Sensors SERCOM 5.3
	{ PORTB, 2, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, ADC_Channel14, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, 		//27  	Sensors SERCOM 5.0
	{ PORTB, 3, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, 	//28  	Sensors SERCOM 5.1
	{ PORTB, 4, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, 	//29	Sensors ADC1
	{ PORTB, 5, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, 	//30	Sensors ADC2
	{ PORTB, 6, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, 	//31	Sensors ADC3
	{ PORTB, 7, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, 	//32	Sensors ADC4
	{ PORTB, 8, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, 	//33	Battery Monitor
	{ PORTB, 9, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, 	//34	ADC5
	{ PORTB, 10, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, //35	SDHC CS 	SDHC DATA3
	{ PORTB, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, //36	SDHC-SDCK
	{ PORTB, 12, PIO_SERCOM, PIN_ATTR_PWM_F, No_ADC_Channel, TCC3_CH0, TC4_CH0, EXTERNAL_INT_12 },   //37	SERCOM 4 PAD0 ESP32
	{ PORTB, 13, PIO_SERCOM, PIN_ATTR_PWM_F, No_ADC_Channel, TCC3_CH1, TC4_CH1, EXTERNAL_INT_13 },   //38	SERCOM 4 PAD1 ESP32
	{ PORTB, 14, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC4_CH0, TC5_CH0, EXTERNAL_INT_14 }, //39	ESP BOOT
	{ PORTB, 15, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC4_CH1, TC5_CH1, EXTERNAL_INT_15 }, //40	ESP Enable
	{ PORTB, 16, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH4, NOT_ON_TIMER, EXTERNAL_INT_0 },  //41	I2S-SCK	Haptics Pin 13
	{ PORTB, 17, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH5, NOT_ON_TIMER,  EXTERNAL_INT_1 },  //42	I2S-MCK	Haptics Pin 14
	{ PORTB, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  //43 	NEOPIXEL ON SERCOM 1
	{ PORTB, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },  //44 	USB Host Enable
	{ PORTB, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, //45 	SWO 5V ENABLE
	{ PORTB, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, //46 	USER SWITCH
	
	
	//USB 47,48,49
	{ PORTB, 7, PIO_COM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, 		//47 	USB Host enable
	{ PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },   		//48 	USB/DM
	{ PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },   		//49 	USB/DP
	
	
	// 50 (AREF)
	{ PORTA, 3,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, 	//50 	DAC/VREFP
	
	
	//DAC 51,52
	{ PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, 		//51 	DAC/VOUT[0]
 	{ PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, 		//52 	DAC/VOUT[1]
	

	//ADC 53-60
	{ PORTB, 8,  PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel2, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },	//53 	Battery Monitor
	{ PORTB, 9,  PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel3, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },	//54 	Sensors ADC5
	{ PORTA, 8,  PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel8, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },	//55 	SDHC
	{ PORTA, 9,  PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel9, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },	//56 	SDHC
	{ PORTB, 4,  PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },	//57 	Sensors ADC1
	{ PORTB, 5,  PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel7, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },	//58 	Sensors ADC2
	{ PORTB, 6,  PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel8, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },	//59 	Sensors ADC3
	{ PORTB, 7,  PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel9, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },	//60 	Sensors ADC4
	
	//I2S 61-66
	{ PORTA, 20, PIO_I2S, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, //61 I2S-FSO	Haptics Pin 15
	{ PORTA, 21, PIO_I2S, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, //62 I2S-SDO	Haptics Pin 16
	{ PORTA, 22, PIO_I2S, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, //63 I2S-SDI	Haptics Pin 17
	{ PORTA, 23, PIO_I2S, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, //64 I2S-FS1	Haptics Pin 18
	{ PORTB, 16, PIO_I2S, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, //65 I2S-SCK	Haptics Pin 13
	{ PORTB, 17, PIO_I2S, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, //66 I2S-MCK	Haptics Pin 14

	//SDHC 67-74
	{ PORTA, 6,  PIO_SDHC, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  //67 SDHC-CD
	{ PORTA, 7,  PIO_SDHC, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },  //6S DJC-WP
	{ PORTA, 8,  PIO_SDHC, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },  //69 SDHC-CMD
	{ PORTA, 9,  PIO_SDHC, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },  //70 SDHC-DATA0
	{ PORTA, 10, PIO_SDHC, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, //71 SDHC-DATA1
	{ PORTA, 11, PIO_SDHC, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, //72 SDHC-DATA2
	{ PORTB, 10, PIO_SDHC, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, //73 SDHC-DATA3
	{ PORTB, 11, PIO_SDHC, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, //74 SDHC-SDCK

	//XTALS 75-80
	//XTAL 32
	{ PORTA, 0, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, //75 32kosc
	{ PORTA, 1, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, //76 32kosc
	//xtal
	{ PORTA, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, //77 OSC0 XIN
	{ PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, //78 OSC0 XOUT

	{ PORTB, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, //79 OSC1 XIN
	{ PORTB, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, //80 OSC1 XOUT
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5 } ;
const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID } ;
// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

//WiFi Module Serial port
Uart Serial1( &sercom4, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM4_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM4_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM4_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM4_3_Handler()
{
  Serial1.IrqHandler();
}
/*
//Sensor Port Module Serial port
Uart Serial2( &sercom5, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ;

void SERCOM5_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM5_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM5_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM5_3_Handler()
{
  Serial2.IrqHandler();
}*/