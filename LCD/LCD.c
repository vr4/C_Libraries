/*
	Author: Eric Conner
	Project: AVR LCD Library
	Date: 05-01-2013
	File: LCD.c
	Change Log:
		v1.1 - 05-31-2013 - Added the ability to turn the backlight on & off using lcd_backlight();.
*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "lcd.h"

#define DDR(x) (*(&x - 1))
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__)
    #define PIN(x) ( &PORTF==&(x) ? _SFR_IO8(0x00) : (*(&x - 2)) )
#else
	#define PIN(x) (*(&x - 2))
#endif

#if LCD_IO_MODE
	#define lcd_e_delay()			__asm__ __volatile__( "rjmp 1f\n 1:" );
	#define lcd_e_high()			LCD_E_PORT  |=  _BV(LCD_E_PIN);
	#define lcd_e_low()				LCD_E_PORT  &= ~_BV(LCD_E_PIN);
	#define lcd_e_toggle()			toggle_e()
	#define lcd_rw_high()			LCD_RW_PORT |=  _BV(LCD_RW_PIN)
	#define lcd_rw_low()			LCD_RW_PORT &= ~_BV(LCD_RW_PIN)
	#define lcd_rs_high()			LCD_RS_PORT |=  _BV(LCD_RS_PIN)
	#define lcd_rs_low()			LCD_RS_PORT &= ~_BV(LCD_RS_PIN)
	#define lcd_backlight_on()		BACKLIGHT_PORT |=  _BV(BACKLIGHT_PIN)
	#define lcd_backlight_off()		BACKLIGHT_PORT &= ~_BV(BACKLIGHT_PIN)
#endif

#if LCD_IO_MODE
	#if LCD_LINES==1
		#define LCD_FUNCTION_DEFAULT	LCD_FUNCTION_4BIT_1LINE 
	#else
		#define LCD_FUNCTION_DEFAULT	LCD_FUNCTION_4BIT_2LINES 
	#endif
#else
	#if LCD_LINES==1
		#define LCD_FUNCTION_DEFAULT	LCD_FUNCTION_8BIT_1LINE
	#else
		#define LCD_FUNCTION_DEFAULT	LCD_FUNCTION_8BIT_2LINES
	#endif
#endif

#if LCD_CONTROLLER_KS0073
	#if LCD_LINES==4
		#define KS0073_EXTENDED_FUNCTION_REGISTER_ON	0x2C
		#define KS0073_EXTENDED_FUNCTION_REGISTER_OFF	0x28
		#define KS0073_4LINES_MODE						0x09
	#endif
#endif

#if LCD_IO_MODE
	static void toggle_e(void);
#endif

static inline void _delayFourCycles(unsigned int __count) {
    if ( __count == 0 )    
        __asm__ __volatile__( "rjmp 1f\n 1:" );
    else
        __asm__ __volatile__ (
    	    "1: sbiw %0,1" "\n\t"                  
    	    "brne 1b"
    	    : "=w" (__count)
    	    : "0" (__count)
    	   );
}

#define delay(us)	_delayFourCycles( ( ( 1*(XTAL/4000) )*us)/1000 )

#if LCD_IO_MODE
	static void toggle_e(void) {
		lcd_e_high();
		lcd_e_delay();
		lcd_e_low();
	}
#endif

#if LCD_IO_MODE
	static void lcd_write(uint8_t data,uint8_t rs) {
		unsigned char dataBits ;
		if (rs) {
		   lcd_rs_high();
		} else {
		   lcd_rs_low();
		}
		lcd_rw_low();
		if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT ) && (LCD_DATA0_PIN == 0) && (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) ) {
			DDR(LCD_DATA0_PORT) |= 0x0F;
			dataBits = LCD_DATA0_PORT & 0xF0;
			LCD_DATA0_PORT = dataBits |((data>>4)&0x0F);
			lcd_e_toggle();
			LCD_DATA0_PORT = dataBits | (data&0x0F);
			lcd_e_toggle();
			LCD_DATA0_PORT = dataBits | 0x0F;
		} else {
			DDR(LCD_DATA0_PORT) |= _BV(LCD_DATA0_PIN);
			DDR(LCD_DATA1_PORT) |= _BV(LCD_DATA1_PIN);
			DDR(LCD_DATA2_PORT) |= _BV(LCD_DATA2_PIN);
			DDR(LCD_DATA3_PORT) |= _BV(LCD_DATA3_PIN);
			LCD_DATA3_PORT &= ~_BV(LCD_DATA3_PIN);
			LCD_DATA2_PORT &= ~_BV(LCD_DATA2_PIN);
			LCD_DATA1_PORT &= ~_BV(LCD_DATA1_PIN);
			LCD_DATA0_PORT &= ~_BV(LCD_DATA0_PIN);
			if(data & 0x80) LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
			if(data & 0x40) LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
			if(data & 0x20) LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
			if(data & 0x10) LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);   
			lcd_e_toggle();
			LCD_DATA3_PORT &= ~_BV(LCD_DATA3_PIN);
			LCD_DATA2_PORT &= ~_BV(LCD_DATA2_PIN);
			LCD_DATA1_PORT &= ~_BV(LCD_DATA1_PIN);
			LCD_DATA0_PORT &= ~_BV(LCD_DATA0_PIN);
			if(data & 0x08) LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
			if(data & 0x04) LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
			if(data & 0x02) LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
			if(data & 0x01) LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);
			lcd_e_toggle();        
			LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);
			LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
			LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
			LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
		}
	}
#else
	#define lcd_write(d,rs) if (rs) *(volatile uint8_t*)(LCD_IO_DATA) = d; else *(volatile uint8_t*)(LCD_IO_FUNCTION) = d;
#endif

#if LCD_IO_MODE
	static uint8_t lcd_read(uint8_t rs) {
		uint8_t data;
		if (rs) {
			lcd_rs_high();
		} else {
			lcd_rs_low();
		}
		lcd_rw_high();
		if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT ) && ( LCD_DATA0_PIN == 0 )&& (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) ) {
			DDR(LCD_DATA0_PORT) &= 0xF0;
			lcd_e_high();
			lcd_e_delay();        
			data = PIN(LCD_DATA0_PORT) << 4;
			lcd_e_low();
			lcd_e_delay();
			lcd_e_high();
			lcd_e_delay();
			data |= PIN(LCD_DATA0_PORT)&0x0F;
			lcd_e_low();
		} else {
			DDR(LCD_DATA0_PORT) &= ~_BV(LCD_DATA0_PIN);
			DDR(LCD_DATA1_PORT) &= ~_BV(LCD_DATA1_PIN);
			DDR(LCD_DATA2_PORT) &= ~_BV(LCD_DATA2_PIN);
			DDR(LCD_DATA3_PORT) &= ~_BV(LCD_DATA3_PIN);
			lcd_e_high();
			lcd_e_delay();        
			data = 0;
			if ( PIN(LCD_DATA0_PORT) & _BV(LCD_DATA0_PIN) ) data |= 0x10;
			if ( PIN(LCD_DATA1_PORT) & _BV(LCD_DATA1_PIN) ) data |= 0x20;
			if ( PIN(LCD_DATA2_PORT) & _BV(LCD_DATA2_PIN) ) data |= 0x40;
			if ( PIN(LCD_DATA3_PORT) & _BV(LCD_DATA3_PIN) ) data |= 0x80;
			lcd_e_low();
			lcd_e_delay();
			lcd_e_high();
			lcd_e_delay();
			if ( PIN(LCD_DATA0_PORT) & _BV(LCD_DATA0_PIN) ) data |= 0x01;
			if ( PIN(LCD_DATA1_PORT) & _BV(LCD_DATA1_PIN) ) data |= 0x02;
			if ( PIN(LCD_DATA2_PORT) & _BV(LCD_DATA2_PIN) ) data |= 0x04;
			if ( PIN(LCD_DATA3_PORT) & _BV(LCD_DATA3_PIN) ) data |= 0x08;        
			lcd_e_low();
		}
		return data;
	}
#else
	#define lcd_read(rs) (rs) ? *(volatile uint8_t*)(LCD_IO_DATA+LCD_IO_READ) : *(volatile uint8_t*)(LCD_IO_FUNCTION+LCD_IO_READ)
#endif

static uint8_t lcd_waitbusy(void) {
    register uint8_t c;
    while ( (c=lcd_read(0)) & (1<<LCD_BUSY)) {}
    delay(2);
    return (lcd_read(0));
}

static inline void lcd_newline(uint8_t pos) {
    register uint8_t addressCounter;
	#if LCD_LINES==1
		addressCounter = 0;
	#endif
	#if LCD_LINES==2
		if ( pos < (LCD_START_LINE2) )
			addressCounter = LCD_START_LINE2;
		else
			addressCounter = LCD_START_LINE1;
	#endif
	#if LCD_LINES==4
		#if KS0073_4LINES_MODE
			if ( pos < LCD_START_LINE2 )
				addressCounter = LCD_START_LINE2;
			else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE3) )
				addressCounter = LCD_START_LINE3;
			else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE4) )
				addressCounter = LCD_START_LINE4;
			else 
				addressCounter = LCD_START_LINE1;
		#else
			if ( pos < LCD_START_LINE3 )
				addressCounter = LCD_START_LINE2;
			else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE4) )
				addressCounter = LCD_START_LINE3;
			else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE2) )
				addressCounter = LCD_START_LINE4;
			else 
				addressCounter = LCD_START_LINE1;
		#endif
	#endif
    lcd_command((1<<LCD_DDRAM)+addressCounter);
}

void lcd_command(uint8_t cmd) {
    lcd_waitbusy();
    lcd_write(cmd,0);
}

void lcd_data(uint8_t data) {
    lcd_waitbusy();
    lcd_write(data,1);
}

void lcd_gotoxy(uint8_t x, uint8_t y) {
	#if LCD_LINES==1
		lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
	#endif
	#if LCD_LINES==2
		if ( y==0 ) {
			lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
		} else {
			lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
		}
	#endif
	#if LCD_LINES==4
		if ( y==0 ) {
			lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
		} else if ( y==1) {
			lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
		} else if ( y==2) {
			lcd_command((1<<LCD_DDRAM)+LCD_START_LINE3+x);
		} else {
			lcd_command((1<<LCD_DDRAM)+LCD_START_LINE4+x);
		}
	#endif
}

int lcd_getxy(void) {
    return lcd_waitbusy();
}

void lcd_clrscr(void) {
    lcd_command(1<<LCD_CLR);
}

void lcd_home(void) {
    lcd_command(1<<LCD_HOME);
}

// Display a character on LCD
void lcd_putc(char c) {
    uint8_t pos;
    pos = lcd_waitbusy();
    if (c=='\n') {
        lcd_newline(pos);
    } else {
		#if LCD_WRAP_LINES==1
			#if LCD_LINES==1
				if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
					lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
				}
			#elif LCD_LINES==2
				if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
					lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
				} else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ) {
					lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
				}
			#elif LCD_LINES==4
				if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
					lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
				} else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ) {
					lcd_write((1<<LCD_DDRAM)+LCD_START_LINE3,0);
				} else if ( pos == LCD_START_LINE3+LCD_DISP_LENGTH ) {
					lcd_write((1<<LCD_DDRAM)+LCD_START_LINE4,0);
				} else if ( pos == LCD_START_LINE4+LCD_DISP_LENGTH ) {
					lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
				}
			#endif
			lcd_waitbusy();
		#endif
		lcd_write(c, 1);
    }
}

// Display string on LCD
void lcd_puts(const char *s) {
    register char c;
    while ( (c = *s++) ) {
        lcd_putc(c);
    }
}

// Display string from memory on LCD
void lcd_puts_p(const char *progmem_s) {
    register char c;
    while ( (c = pgm_read_byte(progmem_s++)) ) {
        lcd_putc(c);
    }
}

// Turn the LCD backlight on or off
void lcd_backlight(uint8_t val) {
	if (val == ON) {
		lcd_backlight_on();
	} else {
		lcd_backlight_off();
	}
}

// Initilize the LCD
void lcd_init(uint8_t dispAttr) {
	#if LCD_IO_MODE
		if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT ) && ( &LCD_RS_PORT == &LCD_DATA0_PORT) && ( &LCD_RW_PORT == &LCD_DATA0_PORT) && (&LCD_E_PORT == &LCD_DATA0_PORT) && (LCD_DATA0_PIN == 0 ) && (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3)  && (LCD_RS_PIN == 4 ) && (LCD_RW_PIN == 5) && (LCD_E_PIN == 6 ) ) {
			DDR(LCD_DATA0_PORT) |= 0x7F;
			DDR(BACKLIGHT_PORT) |= _BV(BACKLIGHT_PIN);
		} else if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT ) && (LCD_DATA0_PIN == 0 ) && (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) ) {
			DDR(LCD_DATA0_PORT) |= 0x0F;
			DDR(LCD_RS_PORT)    |= _BV(LCD_RS_PIN);
			DDR(LCD_RW_PORT)    |= _BV(LCD_RW_PIN);
			DDR(LCD_E_PORT)     |= _BV(LCD_E_PIN);
			DDR(BACKLIGHT_PORT) |= _BV(BACKLIGHT_PIN);
		} else {
			DDR(LCD_RS_PORT)    |= _BV(LCD_RS_PIN);
			DDR(LCD_RW_PORT)    |= _BV(LCD_RW_PIN);
			DDR(LCD_E_PORT)     |= _BV(LCD_E_PIN);
			DDR(LCD_DATA0_PORT) |= _BV(LCD_DATA0_PIN);
			DDR(LCD_DATA1_PORT) |= _BV(LCD_DATA1_PIN);
			DDR(LCD_DATA2_PORT) |= _BV(LCD_DATA2_PIN);
			DDR(LCD_DATA3_PORT) |= _BV(LCD_DATA3_PIN);
			DDR(BACKLIGHT_PORT) |= _BV(BACKLIGHT_PIN);
		}
		delay(16000);
		LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
		LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);
		lcd_e_toggle();
		delay(4992);
		lcd_e_toggle();      
		delay(64);
		lcd_e_toggle();      
		delay(64);
		LCD_DATA0_PORT &= ~_BV(LCD_DATA0_PIN);
		lcd_e_toggle();
		delay(64);
	#else
		MCUCR = _BV(SRE) | _BV(SRW);
		delay(16000);
		lcd_write(LCD_FUNCTION_8BIT_1LINE,0);
		delay(4992);
		lcd_write(LCD_FUNCTION_8BIT_1LINE,0);
		delay(64);
		lcd_write(LCD_FUNCTION_8BIT_1LINE,0);
		delay(64);
	#endif
	#if KS0073_4LINES_MODE
		lcd_command(KS0073_EXTENDED_FUNCTION_REGISTER_ON);
		lcd_command(KS0073_4LINES_MODE);
		lcd_command(KS0073_EXTENDED_FUNCTION_REGISTER_OFF);
	#else
		lcd_command(LCD_FUNCTION_DEFAULT);
	#endif
    lcd_command(LCD_DISP_OFF);
    lcd_clrscr();
    lcd_command(LCD_MODE_DEFAULT);
    lcd_command(dispAttr);
}
