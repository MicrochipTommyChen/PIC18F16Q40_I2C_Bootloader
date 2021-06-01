/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F16Q40
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.31 and above
        MPLAB 	          :  MPLAB X 5.45	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set SWITCH aliases
#define SWITCH_TRIS                 TRISCbits.TRISC0
#define SWITCH_LAT                  LATCbits.LATC0
#define SWITCH_PORT                 PORTCbits.RC0
#define SWITCH_WPU                  WPUCbits.WPUC0
#define SWITCH_OD                   ODCONCbits.ODCC0
#define SWITCH_ANS                  ANSELCbits.ANSELC0
#define SWITCH_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define SWITCH_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define SWITCH_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define SWITCH_GetValue()           PORTCbits.RC0
#define SWITCH_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define SWITCH_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define SWITCH_SetPullup()          do { WPUCbits.WPUC0 = 1; } while(0)
#define SWITCH_ResetPullup()        do { WPUCbits.WPUC0 = 0; } while(0)
#define SWITCH_SetPushPull()        do { ODCONCbits.ODCC0 = 0; } while(0)
#define SWITCH_SetOpenDrain()       do { ODCONCbits.ODCC0 = 1; } while(0)
#define SWITCH_SetAnalogMode()      do { ANSELCbits.ANSELC0 = 1; } while(0)
#define SWITCH_SetDigitalMode()     do { ANSELCbits.ANSELC0 = 0; } while(0)

// get/set LED aliases
#define LED_TRIS                 TRISCbits.TRISC1
#define LED_LAT                  LATCbits.LATC1
#define LED_PORT                 PORTCbits.RC1
#define LED_WPU                  WPUCbits.WPUC1
#define LED_OD                   ODCONCbits.ODCC1
#define LED_ANS                  ANSELCbits.ANSELC1
#define LED_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define LED_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define LED_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define LED_GetValue()           PORTCbits.RC1
#define LED_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define LED_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define LED_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define LED_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define LED_SetPushPull()        do { ODCONCbits.ODCC1 = 0; } while(0)
#define LED_SetOpenDrain()       do { ODCONCbits.ODCC1 = 1; } while(0)
#define LED_SetAnalogMode()      do { ANSELCbits.ANSELC1 = 1; } while(0)
#define LED_SetDigitalMode()     do { ANSELCbits.ANSELC1 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);




#endif // PIN_MANAGER_H
/**
 End of File
*/