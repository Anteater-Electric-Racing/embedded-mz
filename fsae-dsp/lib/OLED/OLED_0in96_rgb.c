/*****************************************************************************
* | File      	:   OLED_0in96_rgb.c
* | Author      :   Waveshare team
* | Function    :   0.96inch RGB OLED Module Drive function
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2023-06-16
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
#include "OLED_0in96_rgb.h"
#include "stdio.h"
#include "DEV_Config.h"
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
    OLED_CS_0;
    DEV_SPI_WriteByte(Reg);
    OLED_CS_1;
#elif USE_IIC
    I2C_Write_Byte(Reg,IIC_CMD);
#endif
}

static void OLED_WriteData(uint8_t Data)
{   
#if USE_SPI
    OLED_DC_1;
    OLED_CS_0;
    DEV_SPI_WriteByte(Data);
    OLED_CS_1;
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
    OLED_WriteReg(0xfd); // command lock
    OLED_WriteReg(0x12);

    OLED_WriteReg(0xae);  // display off
    // OLED_WriteReg(0xa4);  // display off

    OLED_WriteReg(0x15);  // set column address
    OLED_WriteReg(0x20);     // column address start 00
    OLED_WriteReg(0x5f);     // column address end 127
    OLED_WriteReg(0x75);  // set row address
    OLED_WriteReg(0x00);     // row address start 00
    OLED_WriteReg(0x7f);     // row address end 95   


    OLED_WriteReg(0xa0);  // set re-map & data format
    OLED_WriteReg(0x60);     // Horizontal address increment
    OLED_WriteReg(0x00);

    OLED_WriteReg(0xa1);  // set display start line
    OLED_WriteReg(0x00);     // start 0 line

    OLED_WriteReg(0xa2);  // set display offset
    OLED_WriteReg(0x00);

    OLED_WriteReg(0xB1);
    OLED_WriteReg(0x84);

    OLED_WriteReg(0xB3);
    OLED_WriteReg(0x20);

    OLED_WriteReg(0xB6);
    OLED_WriteReg(0x01);

    OLED_WriteReg(0xBB);
    OLED_WriteReg(0x00);

    OLED_WriteReg(0xBE);
    OLED_WriteReg(0x07);

    OLED_WriteReg(0xC7);
    OLED_WriteReg(0x0F);

    OLED_WriteReg(0xC1);
    OLED_WriteReg(0x32);
    OLED_WriteReg(0x29);
    OLED_WriteReg(0x53);

    OLED_WriteReg(0xCA);
    OLED_WriteReg(0x7F);

    // DEV_Delay_ms(100);
    // OLED_WriteReg(0xAF);  // turn on oled panel
}
/********************************************************************************
function:
            initialization
********************************************************************************/
void OLED_0in96_rgb_Init(void)
{
    
    //Hardware reset
    OLED_Reset();
    
    //Set the initialization register
    OLED_InitReg();
    DEV_Delay_ms(200);
    

    //Turn on the OLED display
    OLED_WriteReg(0xAF);

    OLED_WriteReg(0x15);  // set column address
    OLED_WriteData(0x20);     // column address start 00
    OLED_WriteData(0x5f);     // column address end 127
    OLED_WriteReg(0x75);  // set row address
    OLED_WriteData(0x00);     // row address start 00
    OLED_WriteData(0x7f);     // row address end 95   
    OLED_WriteReg(0x5C); 
}

/********************************************************************************
function:
            Clear screen
********************************************************************************/
void OLED_0in96_rgb_Clear(void)
{
    UWORD i;
    for(i=0; i<OLED_0in96_rgb_WIDTH*OLED_0in96_rgb_HEIGHT*2; i++){
        OLED_WriteData(0x00);
    }
}

void OLED_0in96_rgb_Clear_color(UWORD color)
{
    UWORD i;
    UBYTE temp1,temp2;
    temp1 = (color >> 8)  & 0xff;
    temp2 = color & 0xff;
    for(i=0; i<OLED_0in96_rgb_WIDTH*OLED_0in96_rgb_HEIGHT; i++){
        OLED_WriteData(temp1);
        OLED_WriteData(temp2);
    }
}
  

/********************************************************************************
function:   Draw a point
********************************************************************************/
void OLED_0in96_rgb_Set_Point(UBYTE Xpoint, UBYTE Ypoint, UWORD Color)
{
    OLED_WriteReg(0x15);
    OLED_WriteReg(Xpoint);
    OLED_WriteReg(Xpoint);
    OLED_WriteReg(0x75);
    OLED_WriteReg(Ypoint);
    OLED_WriteReg(Ypoint);  

    OLED_WriteData(Color>>8);
    OLED_WriteData(Color);    

}

/********************************************************************************
function:   Update all memory to OLED
********************************************************************************/
void OLED_0in96_rgb_Display(UBYTE *Image)
{
    UWORD i, j;     
    for(i=0; i<OLED_0in96_rgb_HEIGHT; i++)
        for(j=0; j<OLED_0in96_rgb_WIDTH*2; j++)
        {
            OLED_WriteData(Image[j + i*OLED_0in96_rgb_WIDTH*2]);
        }
}
