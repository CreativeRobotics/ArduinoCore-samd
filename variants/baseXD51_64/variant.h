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

#ifndef _VARIANT_BASEXD51_64_
#define _VARIANT_BASEXD51_64_
//#ifndef _VARIANT_METRO_M4_
//#define _VARIANT_METRO_M4_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK			  (120000000ul)

#define VARIANT_GCLK0_FREQ (120000000UL)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
//#define PINS_COUNT           (46u)
#define NUM_DIGITAL_PINS     (48u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (2u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
//#define PIN_LED_13           (13u)
//#define PIN_LED_RXL          (27u)
//#define PIN_LED_TXL          (28u)
//#define PIN_LED              PIN_LED_13
//#define PIN_LED2             PIN_LED_RXL
//#define PIN_LED3             PIN_LED_TXL
//#define LED_BUILTIN          PIN_LED_13
/*
 * Digital Pin Port ID's
 */

//Port A (PA00 and PA01 are connected to the 32k OSC)
#define PA00  (0u)
#define PA01  (1u)
#define PA02  (2u)
#define PA03  (3u)
#define PA04  (4u)
#define PA05  (5u)
#define PA06  (6u)
#define PA07  (7u)
#define PA08  (8u)
#define PA09  (9u)
#define PA10  (10u)
#define PA11  (11u)
#define PA12  (12u)
#define PA13  (13u)
#define PA14  (14u)
#define PA15  (15u)
#define PA16  (16u)
#define PA17  (17u)
#define PA18  (18u)
#define PA19  (19u)
#define PA20  (20u)
#define PA21  (21u)
#define PA22  (22u)
#define PA23  (23u)
#define PA27  (24u)
#define PA30  (25u)
#define PA31  (26u)

//Port B
#define PB00  (27u)
#define PB01  (28u)
#define PB02  (29u)
#define PB03  (30u)
#define PB04  (31u)
#define PB05  (32u)
#define PB06  (33u)
#define PB07  (34u)
#define PB08  (35u)
#define PB09  (36u)
#define PB10  (37u)
#define PB11  (38u)
#define PB12  (39u)
#define PB13  (40u)
#define PB14  (41u)
#define PB15  (42u)
#define PB16  (43u)
#define PB17  (44u)
#define PB22  (45u)
#define PB23  (46u)
#define PB30  (47u)
#define PB31  (48u)
/*
 * Analog pins
 */
#define PIN_A0               (3u)
#define PIN_A1               (59u)
#define PIN_A2               (60u)
#define PIN_A3               (61u)
#define PIN_A4               (62u)
#define PIN_A5               (56u)
#define PIN_A6               (4u)  
#define PIN_A7               (55u)  
#define PIN_DAC0             (53u)
#define PIN_DAC1             (54u)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
static const uint8_t A7  = PIN_A7;

static const uint8_t BATTERYMON = PIN_A7;
static const uint8_t USERANALOG = PIN_A6;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		12
#define ADC_TO_VOLTAGE    1240.909f //divide by this to convert raw ADC readings to a voltage
#define BATTERY_VOLTAGE_SCALE 2.0f //compensate for the potential divider

// Other pins
#define PIN_ATN              (35ul)
static const uint8_t ATN = PIN_ATN;

/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (40ul)
#define PIN_SERIAL1_TX       (39ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (26u)
#define PIN_SPI_MOSI         (28u)
#define PIN_SPI_SCK          (9u)
#define PERIPH_SPI           sercom2
#define PAD_SPI_TX           SPI_PAD_0_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_2
#define SPI_CS               (37ul)


static const uint8_t SS	  = SPI_CS;	
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

static const uint8_t SDCD  = (4u) ;
static const uint8_t SDWP  = (5u) ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (12u)
#define PIN_WIRE_SCL         (13u)
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (49ul)
#define PIN_USB_DM          (50ul)
#define PIN_USB_DP          (51ul)

/*
 * I2S Interfaces
 */

#define I2S_INTERFACES_COUNT 1

#define I2S_DEVICE          0
#define I2S_CLOCK_GENERATOR 3

#define PIN_I2S_SDO         (21u)
#define PIN_I2S_SDI         (22u)
#define PIN_I2S_SCK         (43u)
#define PIN_I2S_FS          (20u)
#define PIN_I2S_MCK			    (44u)
#define PIN_I2S_FS1         (23u)



//Define all the IOport pins here as well so any drivers can make use of them

#define PIN_SP1                       (xxu)


//QSPI Pins
#define PIN_QSPI_SCK	(37u)
#define PIN_QSPI_CS	(38u)
#define PIN_QSPI_IO0	(8u)
#define PIN_QSPI_IO1	(9u)
#define PIN_QSPI_IO2	(10u)
#define PIN_QSPI_IO3	(11u)

//PCC Pins
#define PIN_PCC_DEN1    (12u)
#define PIN_PCC_DEN2    (13u)
#define PIN_PCC_CLK     (14u)
#define PIN_PCC_D0      (16u)
#define PIN_PCC_D1      (17u)
#define PIN_PCC_D2      (18u)
#define PIN_PCC_D3      (19u)
#define PIN_PCC_D4      (20u)
#define PIN_PCC_D5      (21u)
#define PIN_PCC_D6      (22u)
#define PIN_PCC_D7      (23u)
#define PIN_PCC_D8      (41u)
#define PIN_PCC_D9      (42u)

//TODO: meaningful value for this
#define VARIANT_QSPI_BAUD_DEFAULT 5000000

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial1; 
extern Uart Serial2; 
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_BASEXD51_64_ */

