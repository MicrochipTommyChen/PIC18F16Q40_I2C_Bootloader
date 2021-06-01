/*********************************************************************
* FileName:        flash_routines.c
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

//Ken:2021/05/13
//Old:
//#include <htc.h>
//#include <pic16F1937.h>
//New:
//S======================
#include <xc.h>
#include "flash_routines.h"
//E======================

uint16_t bufferRAM __at(0x1500);

//****************************************************************
//  FLASH MEMORY READ
//  needs 16 bit address pointer in address
//  returns 14 bit value from selected address
//
//****************************************************************
unsigned int flash_memory_read (unsigned int address)
{
//    NVMADRL = (address & 0x00FF);
//    NVMADRH = ((address & 0xFF00) >> 8);
//
//    NVMCON1bits.NVMREGS = 0;    // Deselect Configuration space
//    NVMCON1bits.RD = 1;      // Initiate Read
//    NOP();
//    NOP();
//
//    return ((unsigned int)((NVMDATH << 8) | NVMDATL)); 
    uint8_t readWordL, readWordH;

    //Set TBLPTR with the target byte address
    TBLPTRU = (uint8_t) 0x00;
    TBLPTRH = (uint8_t) ((address & 0x0000FF00) >> 8);
    TBLPTRL = (uint8_t) (address & 0x000000FF);

    //Perform table read to move low byte from NVM to TABLAT
    asm("TBLRD*+");
#if 1
    readWordL = TABLAT;
#else
    readWordH = TABLAT;
#endif
    //Perform table read to move high byte from NVM to TABLAT
    asm("TBLRD");
#if 1
    readWordH = TABLAT;
#else
    readWordL = TABLAT;
#endif
#if 1
    return (((uint16_t) readWordH << 8) | (readWordL));
#else
    return (((uint16_t) readWordL << 8) | (readWordH));
#endif
}	
//****************************************************************
//  FLASH MEMORY WRITE
//  needs 16 bit address pointer in address, 16 bit data pointer
//
//****************************************************************

void flash_memory_write (unsigned int address, unsigned char *data )
{
#if 1
		unsigned char wdi;
		
		NVMCON1 = 0;
	
		//NVMADRL=(address & 0xFF);	// load address
        NVMADRU = 0x00;
        NVMADRL=(address & 0x00FF);             // load address
		NVMADRH=((address & 0xFF00)>>8);		// load address
	
		//for (wdi=0;wdi<62;wdi+=2)
        //for (wdi=0;wdi<126;wdi+=2)
        for (wdi=0;wdi<=126;wdi+=2)
        //for (wdi=0;wdi<254;wdi+=2)
        //for (wdi=0;wdi<=62;wdi+=2)
		{
#if 0
			NVMDATH = data[wdi];
			NVMDATL = data[wdi+1];
#else
			NVMDATL = data[wdi];
			NVMDATH = data[wdi+1];                
#endif		
            // Block write sequence
//            NVMCON1bits.NVMREGS = 0;    // Deselect Configuration space
//            NVMCON1bits.WREN = 1;    // Enable writes
//            NVMCON1bits.LWLO = 1;    // Only load write latches
//            NVMCON1bits.CMD = 0x03;
            NVMCON1bits.NVMCMD = 0b100;
            
//			NVMCON2 = 0x55;
//			NVMCON2 = 0xAA;
            NVMLOCK = 0x55;
            NVMLOCK = 0xAA;
			
//			NVMCON1bits.WR = 1;						// set WR to begin write
            NVMCON0bits.GO = 1;
//            while (NVMCON0bits.GO);
            
            //NVMCON1bits.CMD = 0x00;
            
            //Ken:2021/05/13      
            //New:
            //S=============
//               NOP();   
            NOP();
            NOP();          
            //E=============
			
			//NVMADR++;
//            NVMADR = NVMADR+2;
            NVMCON1bits.CMD = 0x00;
		}	

#if 0
//		NVMDATH = data[62];
//		NVMDATL = data[63];
		NVMDATH = data[126];
		NVMDATL = data[127];
//		NVMDATH = data[254];
//		NVMDATL = data[255];        
//		NVMCON1bits.NVMREGS = 0;					// access FLASH program, not config
//		NVMCON1bits.WREN = 1;					// allow program/erase
//		
//		NVMCON1bits.LWLO = 0;					// this time start write
        NVMCON1bits.CMD = 0x03;
//		NVMCON2 = 0x55;				
//		NVMCON2 = 0xAA;
        NVMLOCK = 0x55;
        NVMLOCK = 0xAA;
//		NVMCON1bits.WR = 1;						// set WR to begin write
        NVMCON0bits.GO = 1;
        while (NVMCON0bits.GO); 
        NOP();
        NOP();
#endif
//		NVMCON1bits.WREN = 0;					// disallow program/erase
        NVMCON1bits.CMD = 0x00;
#else    
    NVMADR = address;
    // NVMADR is already pointing to target page
    NVMCON1bits.CMD = 0x05; // Set the page write command
    //????????? Required Unlock Sequence ?????????
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    
    NVMCON0bits.GO = 1; // Start page write
    while (NVMCON0bits.GO); // Wait for the write operation to complete
    NVMCON1bits.CMD = 0x00; // Disable writes to memory 
#endif
}
//****************************************************************
//  FLASH MEMORY ERASE
//  Program memory can only be erased by rows. 
//  A row consists of 32 words where the EEADRL<4:0> = 0000.
//
//****************************************************************	
void flash_memory_erase (unsigned int address)
{
//		NVMADRL = (address & 0xFF);	// load address
//		NVMADRH = ((address & 0xFF00) >>8 );		// load address
//        
//        // Block erase sequence
//        NVMCON1bits.NVMREGS = 0;    // Deselect Configuration space
//        NVMCON1bits.FREE = 1;    // Specify an erase operation
//        NVMCON1bits.WREN = 1;    // Allows erase cycles
//        
//		NVMCON2 = 0x55;				// required sequence
//		NVMCON2 = 0xAA;				// required sequence
//		NVMCON1bits.WR = 1;						// set WR to begin erase cycle
//        
//        NOP();
//        NOP();
//        
//		NVMCON1bits.WREN = 0;					// disallow program/erase
    //uint32_t blockStartAddr = (uint32_t) (flashAddr & ((END_FLASH - 1) ^ ((ERASE_FLASH_BLOCKSIZE * 2) - 1)));
    uint16_t blockStartAddr = (uint16_t) (address & (0xFFFF ^ ((ERASE_FLASH_BLOCKSIZE * 2) - 1)));
    //uint8_t GIEBitValue = INTCON0bits.GIE;

    //The NVMADR[21:8] bits point to the page being erased.
    //The NVMADR[7:0] bits are ignored
    NVMADRU = 0x00;
    NVMADRH = (uint8_t) ((blockStartAddr & 0x0000FF00) >> 8);

    //Set the NVMCMD control bits for Erase Page operation
    NVMCON1bits.NVMCMD = 0b110;

    //Disable all interrupts
    //INTCON0bits.GIE = 0;

    //Perform the unlock sequence
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;

    //Start page erase and wait for the operation to complete
    NVMCON0bits.GO = 1;
    while (NVMCON0bits.GO);

    //Restore the interrupts
    //INTCON0bits.GIE = GIEBitValue;

    //Set the NVMCMD control bits for Word Read operation to avoid accidental writes
    NVMCON1bits.NVMCMD = 0b000;    
}

//void FLASH_ReadPage(uint32_t flashAddr)
//{
//    uint8_t GIEBitValue = INTCON0bits.GIE; // Save interrupt enable
//
//    //Set NVMADR with the target word address
//    NVMADRU = (uint8_t) ((flashAddr & 0x00FF0000) >> 16);
//    NVMADRH = (uint8_t) ((flashAddr & 0x0000FF00) >> 8);
//    NVMADRL = (uint8_t) (flashAddr & 0x000000FF);
//
//    //Set the NVMCMD control bits for Page Read operation
//    NVMCON1bits.NVMCMD = 0b010;
//
//    //Disable all interrupt
//    INTCON0bits.GIE = 0;
//
//    //Perform the unlock sequence
//    NVMLOCK = 0x55;
//    NVMLOCK = 0xAA;
//
//    //Start page read and wait for the operation to complete
//    NVMCON0bits.GO = 1;
//    while (NVMCON0bits.GO);
//
//    //Restore the interrupts
//    INTCON0bits.GIE = GIEBitValue;
//
//    //Set the NVMCMD control bits for Word Read operation to avoid accidental writes
//    NVMCON1bits.NVMCMD = 0b000;
//}


