/*********************************************************************
* FileName:        Bootloader.c
* Dependencies:    See INCLUDES section below
* Processor:       
* Compiler:        
* Company:         Microchip Technology, Inc.
*
* Software License Agreement:
*
* The software supplied herewith by Microchip Technology Incorporated
* (the "Company") for its PICmicro® Microcontroller is intended and
* supplied to you, the Company's customer, for use solely and
* exclusively on Microchip PICmicro Microcontroller products. The
* software is owned by the Company and/or its supplier, and is
* protected under applicable copyright laws. All rights are reserved.
* Any use in violation of the foregoing restrictions may subject the
* user to criminal sanctions under applicable laws, as well as to
* civil liability for the breach of the terms and conditions of this
* license.
*
* THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
* IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
* CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*********************************************************************
* File Description:
*
* Change History:
* Author               Cristian Toma
********************************************************************/

#include <xc.h>
#include "main.h"
#include "typedef.h"
#include "pksa.h"
#include "flash_routines.h"
#include "delay.h"

// Configuration bits: selected in the GUI

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator Selection->Oscillator not enabled
#pragma config RSTOSC = HFINTOSC_1MHZ    // Reset Oscillator Selection->HFINTOSC with HFFRQ = 4 MHz and CDIV = 4:1

// CONFIG2
#pragma config CLKOUTEN = OFF    // Clock out Enable bit->CLKOUT function is disabled
#pragma config PR1WAY = ON    // PRLOCKED One-Way Set Enable bit->PRLOCKED bit can be cleared and set only once
#pragma config CSWEN = ON    // Clock Switch Enable bit->Writing to NOSC and NDIV is allowed
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable bit->Fail-Safe Clock Monitor enabled
#pragma config FCMENP = ON    // Fail-Safe Clock Monitor - Primary XTAL Enable bit->Fail-Safe Clock Monitor enabled; timer will flag FSCMP bit and OSFIF interrupt on EXTOSC failure.
#pragma config FCMENS = ON    // Fail-Safe Clock Monitor - Secondary XTAL Enable bit->Fail-Safe Clock Monitor enabled; timer will flag FSCMP bit and OSFIF interrupt on SOSC failure.

// CONFIG3
#pragma config MCLRE = EXTMCLR    // MCLR Enable bit->If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR 
#pragma config PWRTS = PWRT_OFF    // Power-up timer selection bits->PWRT is disabled
//#pragma config MVECEN = OFF    // Multi-vector enable bit->Interrupt contoller does not use vector table to prioritze interrupts
#pragma config MVECEN = ON    // Multi-vector enable bit->Multi-vector enabled, Vector table used for interrupts
#pragma config IVT1WAY = ON    // IVTLOCK bit One-way set enable bit->IVTLOCKED bit can be cleared and set only once
//#pragma config IVT1WAY = OFF    // IVTLOCK bit One-way set enable bit->IVTLOCKED bit can be cleared and set only once
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit->Low-Power BOR disabled
#pragma config BOREN = SBORDIS    // Brown-out Reset Enable bits->Brown-out Reset enabled , SBOREN bit is ignored

// CONFIG4
#pragma config BORV = VBOR_1P9    // Brown-out Reset Voltage Selection bits->Brown-out Reset Voltage (VBOR) set to 1.9V
#pragma config ZCD = OFF    // ZCD Disable bit->ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
#pragma config PPS1WAY = ON    // PPSLOCK bit One-Way Set Enable bit->PPSLOCKED bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON    // Stack Full/Underflow Reset Enable bit->Stack full/underflow will cause Reset
#pragma config LVP = ON    // Low Voltage Programming Enable bit->Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored
#pragma config XINST = OFF    // Extended Instruction Set Enable bit->Extended Instruction Set and Indexed Addressing Mode disabled

// CONFIG5
#pragma config WDTCPS = WDTCPS_31    // WDT Period selection bits->Divider ratio 1:65536; software control of WDTPS
#pragma config WDTE = OFF    // WDT operating mode->WDT Disabled; SWDTEN is ignored

// CONFIG6
#pragma config WDTCWS = WDTCWS_7    // WDT Window Select bits->window always open (100%); software control; keyed access not required
#pragma config WDTCCS = SC    // WDT input clock selector->Software Control

// CONFIG7
#pragma config BBSIZE = BBSIZE_512    // Boot Block Size selection bits->Boot Block size is 512 words
#pragma config BBEN = OFF    // Boot Block enable bit->Boot block disabled
#pragma config SAFEN = OFF    // Storage Area Flash enable bit->SAF disabled
#pragma config DEBUG = OFF    // Background Debugger->Background Debugger disabled

// CONFIG8
#pragma config WRTB = OFF    // Boot Block Write Protection bit->Boot Block not Write protected
#pragma config WRTC = OFF    // Configuration Register Write Protection bit->Configuration registers not Write protected
#pragma config WRTD = OFF    // Data EEPROM Write Protection bit->Data EEPROM not Write protected
#pragma config WRTSAF = OFF    // SAF Write protection bit->SAF not Write Protected
#pragma config WRTAPP = OFF    // Application Block write protection bit->Application Block not write protected

// CONFIG9
#pragma config CP = OFF    // PFM and Data EEPROM Code Protection bit->PFM and Data EEPROM code protection disabled

//#define _XTAL_FREQ 16000000
#define PAGESIZE 128 // PFM page size

unsigned char flash_buffer[128];   // 64

unsigned char pksa_wd_address;
unsigned char pksa_index;
unsigned char pksa_status;

ADDRESS	flash_addr_pointer;

//*****************************************************************************
// 	This project must be compiled with :
//	Optimization settings : SPEED must be set  
// 	ROM Ranges = 0 - 1FF 
//	Additional command line :  -L-pstrings=CODE
//  *All values here are in hex.
//*****************************************************************************



// The bootloader code does not use any interrupts.
// However, the downloaded code may use interrupts.
// The interrupt vector on a PIC16F1937 is located at 
// address 0x0004. The following function will be located 
// at the interrupt vector and will contain a jump to
// 0x0204

//asm ("psect  intentry,global,class=CODE,delta=2");
//asm ("pagesel 0x608");   // 0x204
//asm ("GOTO 0x608");   // 0x204
    asm ("psect  intcode,global,reloc=2,class=CODE,delta=1");
    asm ("GOTO 0x00608");

    asm ("psect  intcodelo,global,reloc=2,class=CODE,delta=1");
    asm ("GOTO 0x00618");

void main()
{    
	Initialize();
	I2C_Slave_Init();
    
    INTCON0bits.GIE = 0;
    
    DELAY_milliseconds(100);

	// if we have any application loaded, jump to it
    if(flash_memory_read (0xFFFE)  == 0xFF55)
	{
        asm ("goto 0x600");
	}
App:
	
		// main program loop
		while (1)
		{
			do_i2c_tasks();
		}
		
}					

void Initialize(void)
{
    // ======= MCC : PMD ==============//
    // CLKRMD CLKR enabled; SYSCMD SYSCLK enabled; SCANMD SCANNER enabled; LVDMD HLVD enabled; FVRMD FVR enabled; IOCMD IOC enabled; CRCMD CRC enabled;
    PMD0 = 0x00;
    // ZCDMD ZCD enabled; TMR0MD TMR0 enabled; TMR1MD TMR1 enabled; TMR4MD TMR4 enabled; SMT1MD SMT1 enabled; TMR2MD TMR2 enabled; TMR3MD TMR3 enabled; CM1MD CM1 enabled; 
    PMD1 = 0x00;
    // NCO1MD NCO1 enabled; ADCMD ADC enabled; DSM1MD DSM enabled; CWG1MD CWG1 enabled; ACTMD ACT enabled; CM2MD CM2 enabled; DAC1MD DAC1 enabled; 
    PMD2 = 0x00;
    // PWM2MD PWM2 enabled; PWM1MD PWM1 enabled; PWM3MD PWM3 enabled; SPI2MD SPI2 enabled; SPI1MD SPI1 enabled; U2MD UART2 enabled; U1MD UART1 enabled; I2C1MD I2C1 enabled; 
    PMD3 = 0x00;
    // CLC3MD CLC3 enabled; CLC4MD CLC4 enabled; DMA1MD DMA1 enabled; DMA2MD DMA2 enabled; DMA3MD DMA3 enabled; CLC1MD CLC1 enabled; CLC2MD CLC2 enabled; 
    PMD4 = 0x00;
    // DMA4MD DMA4 enabled; DAC2MD DAC2 enabled; 
    PMD5 = 0x00;    
    // ======= MCC : Pin Manager  ==============//
    /**
    LATx registers
    */
    LATA = 0x20;
    LATB = 0x00;
    LATC = 0x00;

    /**
    TRISx registers
    */
    TRISA = 0x1F;
    TRISB = 0x20;
    TRISC = 0xFD;

    /**
    ANSELx registers
    */
    ANSELC = 0xE0;
    ANSELB = 0x20;
    ANSELA = 0x13;

    /**
    WPUx registers
    */
    WPUB = 0x00;
    WPUA = 0x00;
    WPUC = 0x18;

    /**
    ODx registers
    */
    ODCONA = 0x00;
    ODCONB = 0x50;
    ODCONC = 0x00;

    /**
    SLRCONx registers
    */
    SLRCONA = 0x37;
    SLRCONB = 0xF0;
    SLRCONC = 0xFF;

    /**
    INLVLx registers
    */
    INLVLA = 0x3F;
    INLVLB = 0xF0;
    INLVLC = 0xFF;
    
    I2C1SDAPPS = 0x0C;   //RB4->I2C1:SDA1;    
    RB6PPS = 0x21;   //RB6->I2C1:SCL1;    
    RB4PPS = 0x22;   //RB4->I2C1:SDA1;    
    I2C1SCLPPS = 0x0E;   //RB6->I2C1:SCL1;    
    // ===== MCC Oscillator ============//
    // NOSC HFINTOSC; NDIV 1; 
    OSCCON1 = 0x60;
    // CSWHOLD may proceed; SOSCPWR Low power; 
    OSCCON3 = 0x00;
    // MFOEN disabled; LFOEN disabled; ADOEN disabled; PLLEN enabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    OSCEN = 0x01;
//    OSCEN = 0x00;
    // HFFRQ 16_MHz; 
    OSCFRQ = 0x05;
//    OSCFRQ = 0x00;
    // TUN 0; 
    OSCTUNE = 0x00;
    // ACTUD enabled; ACTEN disabled; 
    ACTCON = 0x00;
    
    LED_1 = 0;   
}

//void __interrupt() INTERRUPT_InterruptManager (void)
//{
//    // interrupt handler
//    if(INTCONbits.PEIE == 1)
//    {
//        if(PIE3bits.BCL1IE == 1 && PIR3bits.BCL1IF == 1)
//        {
//            do_i2c_tasks();
//        } 
//        else if(PIE3bits.SSP1IE == 1 && PIR3bits.SSP1IF == 1)
//        {
//            do_i2c_tasks();
//        } 
//        else
//        {
//            //Unhandled Interrupt
//        }
//    }      
//    else
//    {
//        //Unhandled Interrupt
//    }
//      do_i2c_tasks();
//}
