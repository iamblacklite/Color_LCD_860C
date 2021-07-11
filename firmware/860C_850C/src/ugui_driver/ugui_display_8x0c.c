/*******************************************************************************
 * File Name: ugui_SSD1963.c
 *
 * Description:
 *  This is a driver for the uGui graphical library developed by
 *  Achim Döbler.
 *  It is for ILI9481 (formerly SSD1963, but changed by kevinh@geeksville.com)
 *  graphic controller found in a lot of low cost graphics
 *  chinese displays an to be used with PSoC microcontrollers from Cypress.
 *  Will test it with other PSoC micros as soon as I can.
 *
 * Note:
 *  For more information about uGui...
 *  Website: http://www.embeddedlightning.com/ugui/
 *  Git: https://github.com/achimdoebler/UGUI
 *  Forum: http://www.embeddedlightning.com/forum/
 *  PDF Reference manual (excellent): http://www.embeddedlightning.com/download/reference-guide/?wpdmdl=205
 *
 *  Thanks to Achim Döbler for such a god job.
 *
 * Log version:
 *  1.0 - June, 2018.       First version.
 *  Sept 2019, changed to use a ILI9481 controller instead
 *
 ********************************************************************************
 * Copyright (c) 2018 Andres F. Navas
 * This driver follows the same license than the uGui library.
 *******************************************************************************/

#include "ugui_display_8x0c.h"

#include "ugui.h"
#include "../pins.h"
#include "../timers.h"

#define HDP (DISPLAY_WIDTH - 1)
#define VDP (DISPLAY_HEIGHT - 1)

UG_GUI gui;
uint8_t write_pulse_duration = 75;
uint16_t lcd_devcode[6]; // per 8.2.39 of datasheet, six words, first will be filled with garbage

void lcd_set_xy(uint16_t ui16_x1, uint16_t ui16_y1, uint16_t ui16_x2,
                uint16_t ui16_y2);

void lcd_read_data_16bits(uint16_t command, uint16_t *out, int numtoread);

inline void Display_Reset() {
    
}

typedef void (*PushPixelFn)(UG_COLOR);

static void push_pixel_850(UG_COLOR c) {
    if (c == C_TRANSPARENT)
        c = C_BLACK; // FIXME, not quite correct - we really should skip that pixel
    
    lcd_write_data_8bits(c);
}

/**
 * A ugui acceleration function.  Given a rectangle, return a callback to set pixels in that rect.
 * The draw order will be by rows, starting from x1,y1 down to x2,y2.
 */
PushPixelFn HW_FillArea(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2) {
    
    /**************************************************/
    // Set XY
    //
    lcd_write_command(0x2a); // write coladdr
    lcd_write_data_8bits(x1 >> 8);   // write data to BUS - start addr
    lcd_write_data_8bits(x1);   // write data to BUS
    lcd_write_data_8bits(x2 >> 8);   // write data to BUS - end addr
    lcd_write_data_8bits(x2);   // write data to BUS
    
    lcd_write_command(0x2b); // write pageaddr
    lcd_write_data_8bits(y1 >> 8);   // write data to BUS
    lcd_write_data_8bits(y1);   // write data to BUS
    lcd_write_data_8bits(y2 >> 8);   // write data to BUS
    lcd_write_data_8bits(y2);   // write data to BUS
    
    lcd_write_command(0x2c); // start writing pixels
    
    return push_pixel_850;
}

lcd_IC_t detect_lcd_type()
{
    lcd_read_data_16bits(0xbf, lcd_devcode, 6); // ILI9481 doesn't support Read ID4 command (0xD3)
    if(0x94 == lcd_devcode[3] && 0x81 == lcd_devcode[4]) {
        return LCD_ILI9481;
    }
    lcd_read_data_16bits(0xd3, lcd_devcode, 4);
    if (0x77 == lcd_devcode[2] && 0x96 == lcd_devcode[3]) {
        return LCD_ST7796;
    }
    return LCD_Unknown;
}

lcd_IC_t display_8x0C_lcd_init(void) {
    // next step is needed to have PB3 and PB4 working as GPIO
    /* Disable the Serial Wire Jtag Debug Port SWJ-DP */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = LCD_READ__PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LCD_READ__PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = LCD_RESET__PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LCD_RESET__PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = LCD_COMMAND_DATA__PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LCD_COMMAND_DATA__PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = LCD_CHIP_SELECT__PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LCD_CHIP_SELECT__PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = LCD_WRITE__PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LCD_WRITE__PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = 0xffff;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LCD_BUS__PORT, &GPIO_InitStructure);
    
    // disable reset
    GPIO_SetBits(LCD_RESET__PORT, LCD_RESET__PIN);
    
    // default to write mode
    GPIO_SetBits(LCD_READ__PORT, LCD_READ__PIN);
    
    // keep chip select active
    GPIO_ResetBits(LCD_CHIP_SELECT__PORT, LCD_CHIP_SELECT__PIN);
    
    //lcd_IC_t type = LCD_Unknown;
	lcd_IC_t type = LCD_ST7796;
	
#ifdef DISPLAY_850C
    delay_ms(120);
    lcd_write_command(0x11);
    delay_ms(120);

    lcd_write_command(0x36);
    lcd_write_data_8bits(0x48);

    lcd_write_command(0x3A);
    lcd_write_data_8bits(0x55);

    lcd_write_command(0xF0);
    lcd_write_data_8bits(0xC3);

    lcd_write_command(0xF0);
    lcd_write_data_8bits(0x96);

    lcd_write_command(0xB4);
    lcd_write_data_8bits(0x01);

    lcd_write_command(0xB7);
    lcd_write_data_8bits(0xC6);

    lcd_write_command(0xC0);
    lcd_write_data_8bits(0xF0);
    lcd_write_data_8bits(0x35);

    lcd_write_command(0xC1);
    lcd_write_data_8bits(0x15);

    lcd_write_command(0xC2);
    lcd_write_data_8bits(0xAF);

    lcd_write_command(0xC3);
    lcd_write_data_8bits(0x09);

    lcd_write_command(0xC5);     //VCOM
    lcd_write_data_8bits(0x06);

    lcd_write_command(0xC6);
    lcd_write_data_8bits(0x00);

    lcd_write_command(0xE8);
    lcd_write_data_8bits(0x40);
    lcd_write_data_8bits(0x8A);
    lcd_write_data_8bits(0x00);
    lcd_write_data_8bits(0x00);
    lcd_write_data_8bits(0x29);
    lcd_write_data_8bits(0x19);
    lcd_write_data_8bits(0xA5);
    lcd_write_data_8bits(0x33);

    lcd_write_command(0xE0);
    lcd_write_data_8bits(0x70);
    lcd_write_data_8bits(0x00);
    lcd_write_data_8bits(0x05);
    lcd_write_data_8bits(0x03);
    lcd_write_data_8bits(0x02);
    lcd_write_data_8bits(0x20);
    lcd_write_data_8bits(0x29);
    lcd_write_data_8bits(0x01);
    lcd_write_data_8bits(0x45);
    lcd_write_data_8bits(0x30);
    lcd_write_data_8bits(0x09);
    lcd_write_data_8bits(0x07);
    lcd_write_data_8bits(0x22);
    lcd_write_data_8bits(0x29);

    lcd_write_command(0xE1);
    lcd_write_data_8bits(0x70);
    lcd_write_data_8bits(0x0C);
    lcd_write_data_8bits(0x10);
    lcd_write_data_8bits(0x0F);
    lcd_write_data_8bits(0x0E);
    lcd_write_data_8bits(0x09);
    lcd_write_data_8bits(0x35);
    lcd_write_data_8bits(0x64);
    lcd_write_data_8bits(0x48);
    lcd_write_data_8bits(0x3A);
    lcd_write_data_8bits(0x14);
    lcd_write_data_8bits(0x13);
    lcd_write_data_8bits(0x2E);
    lcd_write_data_8bits(0x30);

    //lcd_write_command(0x21);

    lcd_write_command(0xF0);
    lcd_write_data_8bits(0xC3);

    lcd_write_command(0xF0);
    lcd_write_data_8bits(0x96);

    delay_ms(120);

    lcd_write_command(0x29);
#elif DISPLAY_860C
    type = LCD_ST7796;

    delay_ms(120);
    lcd_write_command(0x11);
    delay_ms(120);

    lcd_write_command(0x36);
    lcd_write_data_8bits(0x48);

    lcd_write_command(0x3A);
    lcd_write_data_8bits(0x55);

    lcd_write_command(0xF0);
    lcd_write_data_8bits(0xC3);

    lcd_write_command(0xF0);
    lcd_write_data_8bits(0x96);

    lcd_write_command(0xB4);
    lcd_write_data_8bits(0x01);

    lcd_write_command(0xB7);
    lcd_write_data_8bits(0xC6);

    lcd_write_command(0xC0);
    lcd_write_data_8bits(0xF0);
    lcd_write_data_8bits(0x35);

    lcd_write_command(0xC1);
    lcd_write_data_8bits(0x15);

    lcd_write_command(0xC2);
    lcd_write_data_8bits(0xAF);

    lcd_write_command(0xC3);
    lcd_write_data_8bits(0x09);

    lcd_write_command(0xC5);     //VCOM
    lcd_write_data_8bits(0x06);

    lcd_write_command(0xC6);
    lcd_write_data_8bits(0x00);

    lcd_write_command(0xE8);
    lcd_write_data_8bits(0x40);
    lcd_write_data_8bits(0x8A);
    lcd_write_data_8bits(0x00);
    lcd_write_data_8bits(0x00);
    lcd_write_data_8bits(0x29);
    lcd_write_data_8bits(0x19);
    lcd_write_data_8bits(0xA5);
    lcd_write_data_8bits(0x33);

    lcd_write_command(0xE0);
    lcd_write_data_8bits(0x70);
    lcd_write_data_8bits(0x00);
    lcd_write_data_8bits(0x05);
    lcd_write_data_8bits(0x03);
    lcd_write_data_8bits(0x02);
    lcd_write_data_8bits(0x20);
    lcd_write_data_8bits(0x29);
    lcd_write_data_8bits(0x01);
    lcd_write_data_8bits(0x45);
    lcd_write_data_8bits(0x30);
    lcd_write_data_8bits(0x09);
    lcd_write_data_8bits(0x07);
    lcd_write_data_8bits(0x22);
    lcd_write_data_8bits(0x29);

    lcd_write_command(0xE1);
    lcd_write_data_8bits(0x70);
    lcd_write_data_8bits(0x0C);
    lcd_write_data_8bits(0x10);
    lcd_write_data_8bits(0x0F);
    lcd_write_data_8bits(0x0E);
    lcd_write_data_8bits(0x09);
    lcd_write_data_8bits(0x35);
    lcd_write_data_8bits(0x64);
    lcd_write_data_8bits(0x48);
    lcd_write_data_8bits(0x3A);
    lcd_write_data_8bits(0x14);
    lcd_write_data_8bits(0x13);
    lcd_write_data_8bits(0x2E);
    lcd_write_data_8bits(0x30);

    lcd_write_command(0x21);

    lcd_write_command(0xF0);
    lcd_write_data_8bits(0xC3);

    lcd_write_command(0xF0);
    lcd_write_data_8bits(0x96);

    delay_ms(120);

    lcd_write_command(0x29);
#endif

    delay_ms(25);
    // End of display configuration
    // @geeksville board reads back as 0x2, 0x4, 0x94, 0x81, 0xff - a legit ili9481
    write_pulse_duration = 0; // enable fast writes
    
    // Note: if we have some devices still not working, we might need to add a READ command to 0xbf (8.2.39) to read
    // the chip id of the failing units - this would allow us to see the vendor code of whoever made the display and
    // confirm it is a 9481 (or if different - what it is)
    
    // It is worth noting that the display controller has a small amount of non volatile memory.  I bet the mfg of the
    // 8x0C is checking that code in their firmware, and based on that value chosing to flip the display horizontally
    // if needed (via command 0x36)
    
    // Initialize global structure and set PSET to this.PSET.
    UG_Init(&gui, lcd_pixel_set, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    // Register acceleratos.
    UG_DriverRegister(DRIVER_FILL_FRAME, (void*) HW_FillFrame);
    UG_DriverRegister(DRIVER_DRAW_LINE, (void*) HW_DrawLine);
    UG_DriverRegister(DRIVER_FILL_AREA, (void*) HW_FillArea);

    return type;
}

void lcd_window_set(unsigned int s_x, unsigned int e_x, unsigned int s_y,
                    unsigned int e_y) {
    uint16_t x1, x2, y1, y2;
    
    x1 = s_x;
    x2 = e_x;
    y1 = s_y;
    y2 = e_y;
    
    // set XY
    lcd_write_command(0x2a);
    lcd_write_data_8bits(x1 >> 8);
    lcd_write_data_8bits(x1);
    lcd_write_data_8bits(x2 >> 8);
    lcd_write_data_8bits(x2);
    lcd_write_command(0x2b);
    lcd_write_data_8bits(y1 >> 8);
    lcd_write_data_8bits(y1);
    lcd_write_data_8bits(y2 >> 8);
    lcd_write_data_8bits(y2);
    lcd_write_command(0x2c);
}

void lcd_pixel_set(UG_S16 i16_x, UG_S16 i16_y, UG_COLOR ui32_color) {
    if (ui32_color == C_TRANSPARENT)
        return;
    
    // first 8 bits are the only ones that count for the LCD driver
    uint32_t ui32_x_high = i16_x >> 8;
    uint32_t ui32_x_low = i16_x;
    uint32_t ui32_y_high = i16_y >> 8;
    uint32_t ui32_y_low = i16_y;
    
    /**************************************************/
    // Set XY
    //
    lcd_write_command(0x2a); // write coladdr
    lcd_write_data_8bits(ui32_x_high);   // write data to BUS - start addr
    lcd_write_data_8bits(ui32_x_low);   // write data to BUS
    lcd_write_data_8bits(ui32_x_high);   // write data to BUS - end addr
    lcd_write_data_8bits(ui32_x_low);   // write data to BUS
    
    lcd_write_command(0x2b); // write pageaddr
    lcd_write_data_8bits(ui32_y_high);   // write data to BUS
    lcd_write_data_8bits(ui32_y_low);   // write data to BUS
    lcd_write_data_8bits(ui32_y_high);   // write data to BUS
    lcd_write_data_8bits(ui32_y_low);   // write data to BUS
    
    lcd_write_command(0x2c); // write data to BUS
    
    /**************************************************/
    // Set data on bus
    //
    lcd_write_data_8bits(ui32_color);
    
#if 0
    // I @geeksville don't think this is required and it cost cycles
    
    // NOP
    lcd_write_command(0); // write data to BUS
    
    // NOP
    lcd_write_command(0); // write data to BUS
    
    // NOP
    lcd_write_command(0); // write data to BUS
#endif
}


// pulse low WR pin tPWLW min time 30ns (shortest possible CPU cycle on our CPU is 9ns)
void wait_pulse() {
    // WOW @r0mko says his screen needs this delay to be 80 which is really slow.  Hopefully we only have to
    // be this slow on a particular chip vendor
    for(volatile int numnops = 0; numnops < write_pulse_duration; numnops++)
        __asm volatile(
                "nop\n\t"
                );
}

void lcd_write_cycle() {
    GPIOC->BRR = LCD_WRITE__PIN;
    wait_pulse();
    GPIOC->BSRR = LCD_WRITE__PIN;
    
    // FIXME, total write cycle min time is 100ns, we are probably fine, but nothing is currently guaranteeing it
}


UG_RESULT HW_FillFrame(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2,
                       UG_COLOR ui32_color) {
    if (ui32_color == C_TRANSPARENT)
        return UG_RESULT_OK;
    
    uint32_t ui32_pixels;
    int32_t i32_dx, i32_dy;
    UG_S16 temp;
    
    // calc total of pixels
    if (x2 >= x1) {
        i32_dx = (uint32_t) (x2 - x1 + 1);
    } else {
        i32_dx = (uint32_t) (x1 - x2 + 1);
        temp = x2;
        x2 = x1;
        x1 = temp;
    }
    
    if (y2 >= y1) {
        i32_dy = (uint32_t) (y2 - y1 + 1);
    } else {
        i32_dy = (uint32_t) (y1 - y2 + 1);
        temp = y2;
        y2 = y1;
        y1 = temp;
    }
    
    ui32_pixels = i32_dx * i32_dy;
    
    /**************************************************/
    // Set XY
    //
    lcd_write_command(0x2a); // write data to BUS - set col addr
    lcd_write_data_8bits(x1 >> 8);   // write data to BUS
    lcd_write_data_8bits(x1);   // write data to BUS
    lcd_write_data_8bits(x2 >> 8);   // write data to BUS
    lcd_write_data_8bits(x2);   // write data to BUS
    
    lcd_write_command(0x2b); // write data to BUS
    lcd_write_data_8bits(y1 >> 8);   // write data to BUS
    lcd_write_data_8bits(y1);   // write data to BUS
    lcd_write_data_8bits(y2 >> 8);   // write data to BUS
    lcd_write_data_8bits(y2);   // write data to BUS
    
    lcd_write_command(0x2c); // write data to BUS
    
    LCD_COMMAND_DATA__PORT->BSRR = LCD_COMMAND_DATA__PIN; // data
    
    // set the color only once since is equal to all pixels
    LCD_BUS__PORT->ODR = ui32_color;
    
    while (ui32_pixels-- > 0) {
        lcd_write_cycle();
    }
    
    return UG_RESULT_OK;
}

UG_RESULT HW_DrawLine(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c) {
    if (c == C_TRANSPARENT)
        return UG_RESULT_OK;
    
    if ((x1 < 0) || (x1 >= DISPLAY_WIDTH) || (y1 < 0) || (y1 >= DISPLAY_HEIGHT))
        return UG_RESULT_FAIL;
    if ((x2 < 0) || (x2 >= DISPLAY_WIDTH) || (y2 < 0) || (y2 >= DISPLAY_HEIGHT))
        return UG_RESULT_FAIL;
    
    // If it is a vertical or a horizontal line, draw it.
    // If not, then use original drawline routine.
    if ((x1 == x2) || (y1 == y2)) {
        return HW_FillFrame(x1, y1, x2, y2, c);
    }
    
    return UG_RESULT_FAIL;
}




/**
 * For timing information see 13.2.2 in the datasheet
 */
void lcd_write_command(uint16_t ui32_command) {
#if 0
    // We briefly deassert chip select to ensure that each new command is considered totally atomic - i.e.
    // if we wedge the display controller it will keep talking to us
    LCD_CHIP_SELECT__PORT->BSRR = LCD_CHIP_SELECT__PIN; // let chip select go high
    wait_pulse();
    LCD_CHIP_SELECT__PORT->BRR = LCD_CHIP_SELECT__PIN; // reassert chip select
#endif
    
    // command
    LCD_COMMAND_DATA__PORT->BRR = LCD_COMMAND_DATA__PIN;
    
    // write data to BUS
    LCD_BUS__PORT->ODR = ui32_command;
    
    lcd_write_cycle();
    
    // data
    LCD_COMMAND_DATA__PORT->BSRR = LCD_COMMAND_DATA__PIN;
}

void lcd_write_data_8bits(uint16_t ui32_data) {
    // data
    // LCD_COMMAND_DATA__PORT->BSRR = LCD_COMMAND_DATA__PIN;
    
    // write data to BUS
    LCD_BUS__PORT->ODR = ui32_data;
    
    // pulse low WR pin
    lcd_write_cycle();
}

void lcd_read_data_16bits(uint16_t command, uint16_t *out, int numtoread) {
    // FIXME - doesn't yet worK!
    
    lcd_write_command(command);
    
    // Make data pins inputs
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = 0xffff;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(LCD_BUS__PORT, &GPIO_InitStructure);
    
    while (numtoread--) {
        LCD_READ__PORT->BRR = LCD_READ__PIN; // drive from high to low (as an open drain output)
        
        delay_ms(2); // min 340ns delay needed before reading
        
        // Now the data should be valid for reading
        *out++ = (uint16_t) LCD_BUS__PORT->IDR;
        
        LCD_READ__PORT->BSRR = LCD_READ__PIN; // raise read high (LCD will stop driving data lines)
        
        delay_ms(2); // min 250ns of read high needed per datasheet
    }
    
    // Make data pins outputs again
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint16_t *getLcdDevcode()
{
    return lcd_devcode;
}
