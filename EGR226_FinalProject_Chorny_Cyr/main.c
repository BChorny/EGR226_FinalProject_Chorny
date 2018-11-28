#include <msp.h>
#include <stdio.h>
#include <string.h>

#define DELAY 10
#define ADC_CONVERSION_RATE 30000

void ADC14init();
void initialize_Sys();
void delay_ms(uint16_t ms);
void CommandWrite(uint8_t cmd);
void PushByte(uint8_t x);
void PushNibble(uint8_t y);
void LCD_init();
void PulseEnable();
void dataWrite(uint8_t str);
void initialize_LEDs();
void set_LEDs(int red, int blue, int green);
void button_setup();

void RTC_Init(uint16_t minC,uint16_t hrC,uint16_t Ahr,uint16_t Amin);

float voltage,tempF;
uint16_t hours,mins,secs,alarm_update=0,time_update=0;

void main()
{
    __disable_irq();
    initialize_Sys();
    LCD_init();
    initialize_LEDs();
    button_setup();
    //RTC_Init();
   __enable_interrupt();

    set_LEDs(0,0,0);

    char tempF[12];
    uint8_t Ahrs,Amins,Asecs,Chrs,Cmins,Csecs;

    while(1){

    }

 
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

void initialize_LEDs()
{
    P10->SEL0 |= BIT5;      //RED 10.5
    P9->SEL0 |= (BIT3|BIT2);      //BLUE 9.3 Green 9.2

    P10->SEL1 &= ~BIT5;
    P9->SEL1 &= ~(BIT3|BIT2);

    P10->DIR |= BIT5;
    P9->DIR |= (BIT3|BIT2);

    P10->OUT &= ~BIT5;
    P9->OUT &= ~(BIT3|BIT2);

    TIMER_A3->CCR[0] = 999;

    TIMER_A3->CCTL[1] = 0b0000000011100000;//red
    TIMER_A3->CCTL[2] = 0b0000000011100000;//green
    TIMER_A3->CCTL[4] = 0b0000000011100000;//blue

    TIMER_A3->CTL = 0b0000001000010100;
}

void set_LEDs(int red, int blue, int green)  //0-100 for each
{

    if(red == 0){
        TIMER_A3->CCR[1] = 0;
    }
    else{
        TIMER_A3->CCR[1] = red * 10 - 1;
    }
    if(green == 0){
        TIMER_A3->CCR[2] = 0;
    }
    else{
        TIMER_A3->CCR[2] = green * 10 - 1;
    }
    if(blue == 0){
        TIMER_A3->CCR[4] = 0;
    }
    else{
        TIMER_A3->CCR[4] = blue * 10 - 1;
    }
}

void PORT1_IRQHandler()
{

}

void button_setup()
{
    P4->SEL0 &= ~0xF;
    P1->SEL0 &= ~BIT4;

    P4->SEL1 &= ~0xF;
    P1->SEL1 &= ~BIT4;

    P4->DIR &= ~0xF;
    P1->DIR &= ~BIT4;

    P4->REN |= 0xF;
    P1->REN |= BIT4;

    P4->OUT |= 0xF;
    P1->OUT |= BIT4;

    P1->IES |= BIT4;
    //P4->IES |= BIT;

    P1->IE |= BIT4;
    //P4->IE |= BIT;

    NVIC_EnableIRQ(PORT1_IRQn);
    //NVIC_EnableIRQ(PORT4_IRQn);
}

void ADC14init()
{
   P4->SEL0            |=   BIT7;
   P4->SEL1            |=   BIT7;
   ADC14->CTL0         =   0;
   ADC14->CTL0         =   0b10000100001000000000001100010000;
   ADC14->CTL1         =   0b110000;
   ADC14->MCTL[0]      =   6;
   ADC14->IER0         |=   BIT0;
   ADC14->CTL0         |=   0b10;
   NVIC->ISER[0]       |=   1<<ADC14_IRQn;
}

void ADC14_IRQHandler()
{
    int sample;

    if(ADC14->IFGR0 & BIT0)
    {
        sample = ADC14->MEM[0];
        voltage = ((3.3/16535)*sample);

        ADC14->CLRIFGR0 &= ~0b0000000;
    }
    ADC14->CLRIFGR1 &= ~0b1111110;
}

void RTC_Init(uint16_t minC,uint16_t hrC,uint16_t Ahr,uint16_t Amin){
    //Initialize time to 2:45:55 pm

    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = minC<<8 | 0;//45 min, 55 secs
    RTC_C->TIM1 = 0<<8 | hrC;  //Monday, 2 pm
    //RTC_C->YEAR = 2018;

    RTC_C->AMINHR = Ahr<<8 | Amin | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits //Alarm at 2:46 pm
    RTC_C->ADOWDAY = 0;
    RTC_C->PS1CTL = 0b00010;  //1/64 second interrupt

    RTC_C->CTL0 = (0xA500) | BIT5; //turn on interrupt
    RTC_C->CTL13 = 0;

    NVIC_EnableIRQ(RTC_C_IRQn);
}

void RTC_C_IRQHandler()
{
    if(RTC_C->PS1CTL & BIT0){
        hours = RTC_C->TIM1 & 0x00FF;
        mins = (RTC_C->TIM0 & 0xFF00) >> 8;
        secs = RTC_C->TIM0 & 0x00FF;
        if(secs != 59){
            RTC_C->TIM0 = RTC_C->TIM0 + 1;
        }
        else {
            RTC_C->TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8)+1)<<8;
            time_update = 1;
        }
        RTC_C->PS1CTL &= ~BIT0;
    }
    if(RTC_C->CTL0 & BIT1)
    {
        alarm_update = 1;
        RTC_C->CTL0 = (0xA500) | BIT5;
    }
}
