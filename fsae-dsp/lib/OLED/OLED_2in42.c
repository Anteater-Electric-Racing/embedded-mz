/*****************************************************************************
* | File      	:   OLED_2in42.c
* | Author      :   Waveshare team
* | Function    :   2.42inch OLED Module Drive function
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2022-07-14
* | Info        :
* -----------------------------------------------------------------------------
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "OLED_2in42.h"
#include "stdio.h"

/*******************************************************************************
function:
            Hardware reset
*******************************************************************************/
static void OLED_Reset(void)
{
    OLED_RST_1;
    DEV_Delay_ms(100);
    OLED_RST_0;
    DEV_Delay_ms(100);
    OLED_RST_1;
    DEV_Delay_ms(100);
}

/*******************************************************************************
function:
            Write register address and data
*******************************************************************************/
static void OLED_WriteReg(uint8_t Reg)
{
#if USE_SPI
    OLED_DC_0;
    DEV_SPI_WriteByte(Reg);
#elif USE_IIC
    I2C_Write_Byte(Reg,IIC_CMD);
#endif
}

static void OLED_WriteData(uint8_t Data)
{   
#if USE_SPI
    OLED_DC_1;
    DEV_SPI_WriteByte(Data);
#elif USE_IIC
    I2C_Write_Byte(Data,IIC_RAM);
#endif
}

/*******************************************************************************
function:
        Common register initialization
*******************************************************************************/
static void OLED_InitReg(void)
{
    OLED_WriteReg(0xAE);//--turn off oled panel

    OLED_WriteReg(0x00);//---set low column address
    OLED_WriteReg(0x10);//---set high column address
    
    OLED_WriteReg(0x20);
    OLED_WriteReg(0x00);
        
    OLED_WriteReg(0xFF);
    
    OLED_WriteReg(0xA6);
    
    OLED_WriteReg(0xA8); 
    OLED_WriteReg(0x3F);
    
    OLED_WriteReg(0xD3);
    OLED_WriteReg(0x00);
    
    OLED_WriteReg(0xD5);
    OLED_WriteReg(0x80);
    
    OLED_WriteReg(0xD9);
    OLED_WriteReg(0x22);

    OLED_WriteReg(0xDA);
    OLED_WriteReg(0x12);
    
    OLED_WriteReg(0xDB);
    OLED_WriteReg(0x40);

}

/********************************************************************************
function:
            initialization
********************************************************************************/
void OLED_2in42_Init(void)
{
    //Hardware reset
    OLED_Reset();

    //Set the initialization register
    OLED_InitReg();
    DEV_Delay_ms(200);

    //Turn on the OLED display
    OLED_WriteReg(0xAF);
}

/********************************************************************************
function:
            Clear screen
********************************************************************************/
void OLED_2in42_Clear(void)
{
    // UWORD Width, Height;
    UWORD i, j;
    // Width = (OLED_1IN3_WIDTH % 8 == 0)? (OLED_1IN3_WIDTH / 8 ): (OLED_1IN3_WIDTH / 8 + 1);
    // Height = OLED_1IN3_HEIGHT; 
    for (i=0; i<8; i++) {
        /* set page address */
        OLED_WriteReg(0xB0 + i);
        /* set low column address */
        OLED_WriteReg(0x00);
        /* set high column address */
        OLED_WriteReg(0x10);
        for(j=0; j<128; j++) {
            /* write data */
            OLED_WriteData(0x00);  
        }

    }
}

/********************************************************************************
function:   Update all memory to OLED
********************************************************************************/
void OLED_2in42_Display(UBYTE *Image)
{
    UWORD page, column, temp;

    for (page=0; page<8; page++) {
        /* set page address */
        OLED_WriteReg(0xB0 + page);
        /* set low column address */
        OLED_WriteReg(0x00);
        /* set high column address */
        OLED_WriteReg(0x10);

        /* write data */
        for(column=0; column<128; column++) {
            temp = Image[(7-page) + column*8];
            OLED_WriteData(temp);
        }       
    }
}
