/*********************************************************************
* FileName:        pksa.c
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
//E======================
#include "pksa.h"
#include "main.h"
#include "flash_routines.h"


#define I2C1_SLAVE_ADDRESS      0x18   //0x50 //0x18
#define I2C1_SLAVE_MASK         127

typedef enum
{
    I2C_IDLE,
    I2C_ADDR_TX,
    I2C_ADDR_RX,
    I2C_DATA_TX,
    I2C_DATA_RX
} i2c_slave_state_t;

i2c_slave_state_t i2cSlaveState = I2C_IDLE;

void I2C_Slave_Init()
{
    //EN disabled; RSEN disabled; S Cleared by hardware after Start; CSTR Enable clocking; MODE two 7-bit address with masking; 
    I2C1CON0 = 0x01;
    //ACKCNT Acknowledge; ACKDT Acknowledge; RXO 0; TXU 0; CSD Clock Stretching enabled; 
    I2C1CON1 = 0x00;
    //ACNT disabled; GCEN disabled; FME disabled; ABD enabled; SDAHT 300 ns hold time; BFRET 8 I2C Clock pulses; 
    I2C1CON2 = 0x00;
    //CNT 0; 
    I2C1CNTL = 0x00;
    I2C1CNTH = 0x00;
    
    I2C1ADR0 = (uint8_t) (I2C1_SLAVE_ADDRESS << 1);
    I2C1ADR2 = (uint8_t) (I2C1_SLAVE_ADDRESS << 1);
    
    I2C1ADR1 = (uint8_t) (I2C1_SLAVE_MASK << 1);
    I2C1ADR3 = (uint8_t) (I2C1_SLAVE_MASK << 1);

    PIE7bits.I2C1IE    = 1;
    I2C1PIEbits.PCIE = 1; 
    I2C1ERRbits.NACKIE = 1;
    I2C1PIEbits.ACKTIE = 1;
    
    PIR7bits.I2C1IF = 0;
    PIR7bits.I2C1EIF = 0;
    PIR7bits.I2C1RXIF = 0;
    PIR7bits.I2C1TXIF = 0;
    I2C1PIR = 0;
    
    I2C1CON0bits.EN = 1;

  
}
void _WriteData(unsigned char data)
{
    while(I2C1STAT1bits.TXBE != 1);
    I2C1TXB = data;
	I2C1CON0bits.CSTR = 0;
}
int do_i2c_tasks()
{
		unsigned int dat =0 ;
		unsigned char temp,idx;	
		
        if(PIR7bits.I2C1IF == 1)
		{
            if(I2C1PIRbits.PCIF)
            {
                //i2cSlaveState = I2C_IDLE;
                I2C1PIR = 0;
                I2C1STAT1bits.CLRBF = 1;
                I2C1CNTL = 0x00;
                I2C1CNTH = 0x00;
                I2C1CON0bits.CSTR = 0;
                
				pksa_status = I2C_NO_TRANSACTION;                 
            }
            else
            {
                
                if(I2C1PIRbits.ADRIF == 1)
                {  
                    if(I2C1STAT0bits.R)   // Master Read Address
                    {
                        i2cSlaveState = I2C_ADDR_TX;
                        LED_1 ^= 1;
                    }
                    else    // Master Write Address
                    {
                        i2cSlaveState = I2C_ADDR_RX;
                        //LED_1_Toggle();
                        LED_1 ^= 1;
                    }
                }
                else
                {  
                    if(I2C1STAT0bits.R)   // Master Read Data
                    {
                        i2cSlaveState = I2C_DATA_TX;
                        LED_1 ^= 1;
                    }
                    else    // Master Write Data
                    {
                        i2cSlaveState = I2C_DATA_RX;
                        LED_1 ^= 1;
                    }
                }
					
                switch (i2cSlaveState)
                {
                    case I2C_ADDR_RX:   //MASTER WRITES ADDRESS STATE
                        //temp=SSP1BUF;
                        //temp = I2C1RXB;
                        temp = I2C1ADB0;
                        I2C1STAT1bits.CLRBF = 1; 
                        pksa_status=I2C_SLAVE_ADDRESS_RECEIVED;
                    break;

                    case I2C_DATA_RX:   //MASTER WRITES DATA STATE
                        //temp=SSP1BUF;
                        if(I2C1STAT1bits.RXBF)
                        {
                            temp = I2C1RXB;
                        }
                        
                        if(	pksa_status == I2C_SLAVE_ADDRESS_RECEIVED )
                        {   // first time we get the slave address, after that set to word address
                            pksa_wd_address = temp;
                            pksa_index=0;
                            pksa_status = I2C_WORD_ADDRESS_RECEIVED;
                        }
                        else if ( pksa_status == I2C_WORD_ADDRESS_RECEIVED )
                        {	// second time we get the word address, so look into word address 
                            if ( pksa_wd_address == 0x01)	// 0x01 is buffer word address
                            {
                                if (pksa_index == 0)
                                {
                                    flash_addr_pointer.bytes.byte_H= temp;
                                    pksa_index++;
                                }
                                else if (pksa_index == 1)
                                {
                                     flash_addr_pointer.bytes.byte_L= temp;	

                                }
                            }
                            if ( pksa_wd_address == 0x02 )	// 0x02 write data word address
                            {
                                flash_buffer[pksa_index]=temp;
                                pksa_index++;
                                if (pksa_index == 128)   // 64
                                    pksa_index--;
                            }	
                        }					

                    break;

                    case I2C_ADDR_TX:   //MASTER READS ADDRESS STATE
                        temp = I2C1ADB0;
                        I2C1STAT1bits.CLRBF = 1;                        
                            if (pksa_wd_address == 0x01)			// buffer word address
                            {	
                                // Send first byte here, next byte will be send at MRD case, see below		
                                _WriteData (flash_addr_pointer.bytes.byte_H);
                            }
                            if (pksa_wd_address == 0x03)	// read data from flash memory
                            {
                                if (pksa_index == 0)
                                {
                                    LED_1 = 1;
                                    // read data into flash_buffer
                                    for (idx = 0; idx <128; idx+=2)   // 64
                                    {	
                                        dat = flash_memory_read (flash_addr_pointer.word.address);
                                        flash_buffer[idx  ] = dat>>8;
                                        flash_buffer[idx+1] = dat & 0xFF;
                                        flash_addr_pointer.word.address += 2;
                                    }		
                                    LED_1 = 0;
                                    // send first byte, the rest will be sent at MRD, see below							
                                    _WriteData(flash_buffer[pksa_index]);
                                    pksa_index++;
                                    if (pksa_index == 128)
                                        pksa_index--;	// should never get here....
                                }		
                            }
                            if (pksa_wd_address == 0x04)
                            {
                                // erase command, erases a row of 32 words
                                flash_memory_erase (flash_addr_pointer.word.address);
                                flash_addr_pointer.word.address +=256;
                                _WriteData(0x00);
                            }
                            if (pksa_wd_address == 0x05)
                            {
                                // write command. What's stored into flash_buffer is written 
                                // to FLASH memory at the address pointed to by the address pointer.
                                // The address pointer automatically increments by 8 units.	
                                flash_memory_write (flash_addr_pointer.word.address, flash_buffer );
                                flash_addr_pointer.word.address +=128;
                                _WriteData(0x00);	

                            }	
                            if (pksa_wd_address == 0x06)
                            {
                                // jump to application code
                                _WriteData(0x00);
                                
                                LED_1 ^= 1;
                                for (int i = 0; i < 0xFFFF; i++ )
                                {
                                    NOP();
                                }
                                //Ken:2021/05/13
                                //New:
                                //S===============
                                RESET();
                                //E===============
                            }		
                    break;
                    
                    case I2C_DATA_TX:   //MASTER READS DATA STATE
                            if (pksa_wd_address == 0x01)	// buffer word address
                            {		
                                _WriteData (flash_addr_pointer.bytes.byte_L);
                            }
                            if (pksa_wd_address == 0x03)
                            {
                                _WriteData(flash_buffer[pksa_index]);
                                pksa_index++;
                                if (pksa_index == 128)   // 64
                                    pksa_index--;
                            }								
                    break;		
                }
			}
            I2C1PIR = 0;
            PIR7bits.I2C1IF = 0;
            PIR7bits.I2C1EIF = 0;
            PIR7bits.I2C1RXIF = 0;
            PIR7bits.I2C1TXIF = 0;
            I2C1ERRbits.NACK1IF = 0;
            I2C1STAT1bits.CLRBF = 1;
            I2C1CON0bits.CSTR = 0;            
        }

        
        return 0;
}



