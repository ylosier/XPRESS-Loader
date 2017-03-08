/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

 Low Voltage Programming Interface 
 
  Bit-Banged implementation of the PIC16F1 (250K) LVP protocol
  Based on the PIC16F188XX specification
  
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*******************************************************************************/

#include "lvp16bit.h"
#include "leds.h"

#define  CMD_LOAD_ADDRESS     0x80
#define  CMD_LATCH_DATA       0x00
#define  CMD_LATCH_DATA_IA    0x02
#define  CMD_INC_ADDR         0xF8
#define  CMD_BEGIN_PROG       0xE0
#define  CMD_BULK_ERASE       0x18


void ICSP_Init(void )
{
    ICSP_TRIS_DAT  = INPUT_PIN;
    ICSP_CLK       = 0;
    ICSP_TRIS_CLK  = OUTPUT_PIN;
    ICSP_nMCLR = SLAVE_RUN;
    ICSP_TRIS_nMCLR = OUTPUT_PIN;
}

void ICSP_Release( void)
{
    ICSP_TRIS_DAT  = INPUT_PIN;
    ICSP_TRIS_CLK  = INPUT_PIN;
    ICSP_nMCLR = SLAVE_RUN;     
    ICSP_TRIS_nMCLR = OUTPUT_PIN;
    LED_Off(RED_LED);
    LED_On(GREEN_LED);
}


void sendEntryCode(void)
{
    uint8_t i, j;
    uint8_t b[4] = {'M','C','H','Q'};
    
    ICSP_TRIS_DAT = OUTPUT_PIN;
    for( j=0; j<4; j++)
    {
        for( i=0; i<8; i++)
        {
            if (((b[j] >> (7-i)) & 0x01) > 0)
                ICSP_DAT = 1;     // Msb first
            else
                ICSP_DAT = 0;     // Msb first
            ICSP_CLK = 1;
            __delay_us(1);
            ICSP_CLK = 0;
            __delay_us(1);
        }
    }
    __delay_us(1);    
}

void sendControlCode(uint8_t type)
{
    uint8_t i = 0;
    ICSP_TRIS_DAT = OUTPUT_PIN;
    for(i = 0; i < 4; i++)
    {
        if (type & 0x01)
            ICSP_DAT = 1;
        else
            ICSP_DAT = 0;
        
        ICSP_CLK = 1;
        type >>= 1;
        __delay_us(1);
        ICSP_CLK = 0;
        __delay_us(1);
    }    
    __delay_us(1);  
}

void sendData8(uint8_t b)
{   
    uint8_t i;
    ICSP_TRIS_DAT = OUTPUT_PIN; 
    for(i = 0; i < 8; i++){
        if (b & 0x01)
            ICSP_DAT = 1;     // Lsb first
        else
            ICSP_DAT = 0;     // Lsb first
        
        ICSP_CLK = 1;
        b >>= 1;
        __delay_us(1);
        ICSP_CLK = 0;
        __delay_us(1);
    }    
    __delay_us(1);
}

void sendData16( uint16_t data)
{
    uint8_t i;
    ICSP_TRIS_DAT = OUTPUT_PIN; 
    for(i = 0; i < 16; i++){
        if (data & 0x0001)
            ICSP_DAT = 1;     // Lsb first
        else
            ICSP_DAT = 0;     // Lsb first
        
        ICSP_CLK = 1;
        data >>= 1;
        __delay_us(1);
        ICSP_CLK = 0;
        __delay_us(1);
    }    
    __delay_us(1);
}

void sendData24( uint24_t data)
{
    uint8_t i;
    ICSP_TRIS_DAT = OUTPUT_PIN; 
    for(i = 0; i < 24; i++){
        if (data & 0x000001)
            ICSP_DAT = 1;     // Lsb first
        else
            ICSP_DAT = 0;     // Lsb first
        
        ICSP_CLK = 1;
        data >>= 1;
        __delay_us(1);
        ICSP_CLK = 0;
        __delay_us(1);
    }    
    __delay_us(1);
}

sendSIXCmd(uint24_t data)
{
    sendControlCode(SIX);
    sendData24(data);
}


//uint8_t getByte( void)
//{
//    uint8_t i, b;
//    ICSP_TRIS_DAT = INPUT_PIN;  
//    for( i=0; i < 8; i++){
//        ICSP_CLK = 1;
//        b <<= 1;
//        __delay_us(1);
//        b |= ICSP_DAT;
//        ICSP_CLK = 0;
//        __delay_us(1);
//    }    
//    return b;
//}
//
//uint16_t getData( void)
//{
//    uint8_t i;
//    uint16_t w = 0;
//    ICSP_TRIS_DAT = INPUT_PIN;  
//    for( i=0; i < 24; i++){
//        ICSP_CLK = 1;
//        w <<= 1;
//        __delay_us(1);
//        w |= ICSP_DAT;
//        ICSP_CLK = 0;
//        __delay_us(1);
//    }    
//    return (w >> 1);
//}

void LVP_enter( void)
{
    uint8_t i;
    
    LED_On(RED_LED);
    LED_Off(GREEN_LED);

    ICSP_Init();                 // configure I/Os   
    ICSP_nMCLR = SLAVE_RESET;    // MCLR = Vil (GND)
    __delay_ms(10);
    sendEntryCode();
    __delay_ms( 5);
    ICSP_nMCLR = SLAVE_RUN;
    
    // Input Data Hold Time from MCLR - P7 (25msec minimum)
    for (i = 0; i < 5; i++)
    {
        __delay_ms( 5);
    }
    
    // Send a NOP instruction in order to take care of the additional 5 PGC clocks
    // needed for the first SIX command following start up...
    sendControlCode(SIX);
    
    ICSP_DAT = 0;
    for (i = 0; i < 5; i++)
    {
        ICSP_CLK = 1;
        __delay_us(1);
        ICSP_CLK = 0;
        __delay_us(1);
    }   
    
    sendData24(0x000000);
}

void LVP_exit( void)
{
    ICSP_Release();             // release ICSP-DAT and ICSP-CLK
}

bool LVP_inProgress( void)
{
    return (ICSP_nMCLR==SLAVE_RESET);
}

void LVP_bulkErase( void)
{
    uint8_t i;
    
    sendSIXCmd(0x040200);
    sendSIXCmd(0x040200);
    sendSIXCmd(0x000000);
    sendSIXCmd(0x2404FA);
    sendSIXCmd(0x883B0A);
    sendSIXCmd(0xA8E761);
    sendSIXCmd(0x000000);
    sendSIXCmd(0x000000);
    sendSIXCmd(0x000000);
    sendSIXCmd(0x000000);
    
    // Bulk Erase delay - P11 (330msec minimum)
    for (i = 0; i < 35; i++)
    {
        __delay_ms( 10);
    }
}

void LVP_skip(uint16_t count)
{
    while(count-- > 0){
//to avoid warning        sendCmd( CMD_INC_ADDR);     // increment address     
    }
}

void LVP_addressLoad( uint24_t address)
{
    uint16_t destAddressHigh, destAddressLow; 
    
    destAddressHigh = ((uint16_t)(address >> 16)) & 0x00ff;
    destAddressLow  = ((uint16_t)(address      )) & 0xffff;
    
    // Step 1: Exit the Reset vector.
    sendSIXCmd(0x040200);  // GOTO 0x200
    sendSIXCmd(0x040200);  // GOTO 0x200
    sendSIXCmd(0x000000);  // NOP

    // Step 2: Set the NVMCON to program 64 instruction words.
    sendSIXCmd(0x24001A);  // MOV #0x4001, W10
    sendSIXCmd(0x883B0A);  // MOV W10, NVMCON

    // Step 3: Initialize the write pointer (W7) for TBLWT instruction.
    sendSIXCmd(0x200000+(((uint24_t)destAddressHigh)<<4));  // MOV #<DestinationAddress23:16>, W0
    sendSIXCmd(0x880190);  // MOV W0, TBLPAG
    sendSIXCmd(0x200007+(((uint24_t)destAddressLow)<<4));  // MOV #<DestinationAddress15:0>, W7 
}

void LVP_rowWrite( uint16_t *buffer, uint8_t w)
{   
    uint8_t i;
    uint16_t LSW0, MSB0;
    uint16_t LSW1, MSB1;
    uint16_t LSW2, MSB2;
    uint16_t LSW3, MSB3;
    
    i = w / 8; // loop 16 times to write 4 instructions at a time
    
    for(; i>0; i--)     // load n latches 
    {
        LSW0 = *buffer++;
        MSB0 = *buffer++;
        LSW1 = *buffer++;
        MSB1 = *buffer++;
        LSW2 = *buffer++;
        MSB2 = *buffer++;
        LSW3 = *buffer++;
        MSB3 = *buffer++;
                
        sendSIXCmd(0x200000+(((uint24_t)LSW0)<<4));            // MOV #<LSW0>, W0
        sendSIXCmd(0x200001+(((uint24_t)(MSB1<<8+MSB0))<<4));  // MOV #<MSB1:MSB0>, W1
        sendSIXCmd(0x200002+(((uint24_t)LSW1)<<4));            // MOV #<LSW1>, W2
        sendSIXCmd(0x200003+(((uint24_t)LSW2)<<4));            // MOV #<LSW2>, W3
        sendSIXCmd(0x200004+(((uint24_t)(MSB3<<8+MSB2))<<4));  // MOV #<MSB3:MSB2>, W4
        sendSIXCmd(0x200005+(((uint24_t)LSW3)<<4));            // MOV #<LSW3>, W5
                
        sendSIXCmd(0xEB0300);  // CLR W6
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0xBB0BB6);  // TBLWTL[W6++], [W7]
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0xBBDBB6);  // TBLWTH.B[W6++], [W7++]
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0xBBEBB6);  // TBLWTH.B[W6++], [++W7]
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0xBB1BB6);  // TBLWTL[W6++], [W7++]
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0xBB0BB6);  // TBLWTL[W6++], [W7]
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0xBBDBB6);  // TBLWTH.B[W6++], [W7++]
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0xBBEBB6);  // TBLWTH.B[W6++], [++W7]
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0xBB1BB6);  // TBLWTL[W6++], [W7++]
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
    }
    
    sendSIXCmd(0xA8E761);  // BSET NVMCON, #WR
    sendSIXCmd(0x000000);  // NOP
    sendSIXCmd(0x000000);  // NOP
    sendSIXCmd(0x000000);  // NOP
    sendSIXCmd(0x000000);  // NOP

    __delay_ms( 5); // Row Programming Time (P13 - minimum 1.28ms)

    
//    sendSIXCmd(0x803B00);  // MOV NVMCON, W0
//    sendSIXCmd(0x883C20);  // MOV W0, VISI
//    sendSIXCmd(0x000000);  // NOP
//    
//    //TODO: Implement the regout command for instruction below
//    //<VISI>  // Clock out contents of VISI register.
//    
//    sendSIXCmd(0x040200);  // GOTO 0x200
//    sendSIXCmd(0x000000);  // NOP
//
//    //Repeat until the WR bit is clear.
    
}

void LVP_cfgWrite( uint16_t *cfg, uint8_t count)
{
    uint8_t i;
    uint16_t data;
    
    // Step 1: Exit the Reset vector.
    sendSIXCmd(0x040200);  // GOTO 0x200
    sendSIXCmd(0x040200);  // GOTO 0x200
    sendSIXCmd(0x000000);  // NOP

    // Step 2: Initialize the write pointer (W7) for the TBLWT instruction.
    sendSIXCmd(0x200007);  // MOV #0x0000, W7
    
    // Step 3: Set the NVMCON register to program one Configuration register.
    sendSIXCmd(0x24000A);  // MOV #0x4000, W10
    sendSIXCmd(0x883B0A);  // MOV W10, NVMCON

    //  Step 4: Initialize the TBLPAG register.
    sendSIXCmd(0x200F80);  // MOV #0xF8, W0
    sendSIXCmd(0x880190);  // MOV W0, TBLPAG
    
    while( count-- > 0)
    {
        data = *cfg++;
        
        // Step 5: Load the Configuration register data to W6.
        sendSIXCmd(0x200000+(((uint24_t)data)<<4));  // MOV #<CONFIG_VALUE>, W0

        // Step 6: Write the Configuration register data to the write latch and increment the write pointer.
        sendSIXCmd(0xBB1B80);  // TBLWTL W0, [W7++]
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP

        // Step 7: Initiate the write cycle.
        sendSIXCmd(0xA8E761);  // BSET NVMCON, #WR
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP
        sendSIXCmd(0x000000);  // NOP

        // Externally time ‘P20’ (maximum 25msec)
        for (i = 0; i < 6; i++)
        {
            __delay_ms( 5);
        }

        // Step 8: Wait for the Configuration Register Write operation to complete and make sure WR bit is clear.

        //    sendSIXCmd(0x803B00);  // MOV NVMCON, W0
        //    sendSIXCmd(0x883C20);  // MOV W0, VISI
        //    sendSIXCmd(0x000000);  // NOP
        //    
        //    //TODO: Implement the regout command for instruction below
        //    //<VISI>  // Clock out contents of VISI register.
        //    
        //    sendSIXCmd(0x040200);  // GOTO 0x200
        //    sendSIXCmd(0x000000);  // NOP
        //
        //    //Repeat until the WR bit is clear.    
    }
}
