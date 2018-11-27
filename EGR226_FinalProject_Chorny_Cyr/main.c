#include <msp.h>
#include <stdio.h>

#define DELAY 10

void initialize_Sys();
void delay_ms(uint16_t ms);
void CommandWrite(uint8_t cmd);
void PushByte(uint8_t x);
void PushNibble(uint8_t y);
void LCD_init();
void PulseEnable();
void dataWrite(uint8_t str);

void main()
{
initialize_Sys();
LCD_init();

__enable_interrupt();


}

void initialize_Sys()
{
    SysTick->CTRL = 0; //OFF
    SysTick->LOAD = 0xBB8; //Counts to 1ms
    SysTick->VAL = 0; //Reset
    SysTick->CTRL = 5;
}
void delay_ms(uint16_t ms)
{
    SysTick->LOAD = (3000*ms)-1;
    SysTick->VAL = 0; //delays specified ms
    while((SysTick->CTRL & (BIT(16)))==0){}
}

void CommandWrite(uint8_t cmd)
{
    P5->OUT &= ~BIT0;
    PushByte(cmd);
}

void PushByte(uint8_t x)
{
    uint8_t z;
    z = (x & 0xF0)>>4;
    PushNibble(z);
    z = x & 0x0F;
    PushNibble(z);
    delay_ms(DELAY);
}

void PushNibble(uint8_t y)
{
    P5->OUT &= ~0x3C;
    P5->OUT |= (y & 0x0F) << 2;
    PulseEnable();
}

void LCD_init()
{
     P5->SEL0 &= ~0x3F;
     P5->SEL1 &= ~0x3F;
     P5->DIR |= 0x3F;

    CommandWrite(3);
    delay_ms(DELAY);
    CommandWrite(3);
    delay_ms(DELAY*2);
    CommandWrite(3);
    delay_ms(DELAY);
    CommandWrite(2);
    delay_ms(DELAY);
    CommandWrite(2);
    delay_ms(DELAY);
    CommandWrite(8);
    delay_ms(DELAY);
    CommandWrite(0x0F);
    delay_ms(DELAY);
    CommandWrite(1);
    delay_ms(DELAY);
    CommandWrite(6);
    delay_ms(DELAY/10);
}
void PulseEnable()
{
    P5->OUT &= ~BIT1;
    delay_ms(DELAY);

    P5->OUT |= BIT1;
    delay_ms(DELAY);
    P5->OUT &= ~BIT1;
}

void dataWrite(uint8_t str)
{
    P5->OUT |= BIT0;
    PushByte(str);
}
