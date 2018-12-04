#include <msp.h>
#include <stdio.h>
#include <string.h>

#define DELAY 10
#define ADC_CONVERSION_RATE 30000
#define ALARM 2
#define MAIN_CLOCK 1

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
void convert(uint8_t string[50]);
//void RTC_Init();
void RTC_Init(uint16_t minC,uint16_t hrC,uint16_t Ahr,uint16_t Amin); 
void set_clocks();
int button_press();
int set_hours(int clock_type);
int set_minutes(int clock_type);

float voltage,temperatureF, temperatureC;
uint8_t hours,mins,secs;
int alarm_update=0,time_update=0;
uint16_t minC, hrC, Ahr, Amin;
char tempc[12],tempf[12];

void main(void)
        {
    __disable_irq();
    initialize_Sys();
    LCD_init();
    initialize_LEDs();
    button_setup();
    //RTC_Init();
   __enable_interrupt();
// RTC_Init(uint16_t minC,uint16_t hrC,uint16_t Ahr,uint16_t Amin);

//    set_LEDs(0,0,0);


                       int i=0;
                    temperatureC = (((voltage*1000)-500)/10);

                           sprintf(tempc,"temp %.1f C",temperatureC);
                          dataWrite(tempc[i]);
                           temperatureF = (temperatureC * 1.8) +32;
                           sprintf(tempf,"temp %.1f F",temperatureF);

                          dataWrite(tempf[i]);

                           for(i=0;i<12;i++){
                               CommandWrite(0x90+(i));
                               dataWrite(tempc[i]);
                               CommandWrite(0xD0+(i));
                               dataWrite(tempf[i]);

                               }
   hrC = set_hours(MAIN_CLOCK);
   minC = set_minutes(MAIN_CLOCK);
   Ahr = set_hours(ALARM);
   Amin = set_minutes(ALARM);

   RTC_Init(uint16_t minC,uint16_t hrC,uint16_t Ahr,uint16_t Amin);

   uint8_t hrs[2],minutes[2],seconds[2]; //array

    while(1){                                       // Main loop of program
            if(time_update){                            // Time Update Occurred (from interrupt handler)
                time_update = 0;                        // Reset Time Update Notification Flag

             CommandWrite(0x84); //LCD Writes to line 1 for hours
             hrs[0] = (hours/10)+48; //+48 for ascii conversion
             hrs[1]= (hours%10)+48; //+48 for ascii conversion
             convert(hrs); //Receives time from RTC gets sent to convert

                CommandWrite(0x86);
                dataWrite(':'); //'' to pass character "" to pass string to LCD

             CommandWrite(0x87); //LCD Writes to line 1 for minutes
                minutes[0] = (mins/10)+48; //+48 for ascii conversion
                minutes[1]= (mins%10)+48; //+48 for ascii conversion
                convert(minutes); //Receives time from RTC gets sent to convert

                CommandWrite(0x89);
                dataWrite(':'); //'' to pass character "" to pass string to LCD

                CommandWrite(0x8A); //LCD Writes to line 1 for seconds
                seconds[0] = (secs/10)+48; //+48 for ascii conversion
                seconds[1]= (secs%10)+48; //+48 for ascii conversion
                convert(seconds); //Receives time from RTC gets sent to convert

                printf("%02d:%02d:%02d\n",hours,mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds
            }
            if(alarm_update){                           // Alarm Update Occurred (from interrupt handler)
                printf("ALARM\n");                      // Display Alarm status to user
                alarm_update = 0;                       // Reset Alarm Update Notification Flag
            }
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

void PORT4_IRQHandler()
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

   // P1->IES |= BIT4;
    //P4->IES |= BIT;

   // P1->IE |= BIT4;
    //P4->IE |= BIT;

  //  NVIC_EnableIRQ(PORT1_IRQn);
    NVIC_EnableIRQ(PORT4_IRQn);
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
void RTC_Init(uint16_t minC,uint16_t hrC,uint16_t Ahr,uint16_t Amin)
//void RTC_Init()
{
    //Initialize time to 2:45:55 pm
//    RTC_C->TIM0 = 0x2D00;  //45 min, 0 secs
    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = minC<<8 | 55;//45 min, 55 secs
    RTC_C->TIM1 = 1<<8 | hrC;  //Monday, 2 pm
    RTC_C->YEAR = 2018;
    //Alarm at 2:46 pm
    RTC_C->AMINHR = Ahr<<8 | Amin | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
    RTC_C->ADOWDAY = 0;
    RTC_C->PS1CTL = 0b11010;  //1/128 second interrupt

    RTC_C->CTL0 = (0xA500) | BIT5; //turn on interrupt
    RTC_C->CTL13 = 0;
    //TODO
    NVIC_EnableIRQ(RTC_C_IRQn);
}


void RTC_C_IRQHandler()
{
    if(RTC_C->PS1CTL & BIT0){                           // PS1 Interrupt Happened
        hours = RTC_C->TIM1 & 0x00FF;                   // Record hours (from bottom 8 bits of TIM1)
        mins = (RTC_C->TIM0 & 0xFF00) >> 8;             // Record minutes (from top 8 bits of TIM0)
        secs = RTC_C->TIM0 & 0x00FF;                    // Record seconds (from bottom 8 bits of TIM0)
        // For increasing the number of seconds  every PS1 interrupt (to allow time travel)
//        if(secs != 59){                                 // If not  59 seconds, add 1 (otherwise 59+1 = 60 which doesn't work)
//            RTC_C->TIM0 = RTC_C->TIM0 + 1;
//        }
//        else {
//            RTC_C->TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8)+1)<<8;  // Add a minute if at 59 seconds.  This also resets seconds.
//                                                                 // TODO: What happens if minutes are at 59 minutes as well?
            time_update = 1;                                     // Send flag to main program to notify a time update occurred.
       // }
        RTC_C->PS1CTL &= ~BIT0;                         // Reset interrupt flag
    }
    if(RTC_C->CTL0 & BIT1)                              // Alarm happened!
    {
        alarm_update = 1;                               // Send flag to main program to notify a time update occurred.
        RTC_C->CTL0 = (0xA500) | BIT5;                  // Resetting the alarm flag.  Need to also write the secret code
                                                        // and rewrite the entire register.
                                                        // TODO: It seems like there is a better way to preserve what was already
                                                        // there in case the setup of this register needs to change and this line
                                                        // is forgotten to be updated.
    }
}
void convert(uint8_t string[50]) //Grabs array and puts it integers into characters
{

    int length,i=0;

    length = strlen(string);

    for(i;i<length;i++)
    {
        dataWrite(string[i]);
    }
}

void set_clocks()
{
    int button_read;
    int hrC=0,minC=0,Amin=0,Ahr=0;
    uint8_t setHrs[2],setMins[2];

    CommandWrite(0x84);
    convert("00:00:00");
    CommandWrite(0xC4);
    convert("00:00:00");

     do{
    button_read=button_press();

        if(button_read ==1) //set clock time
            {
            do{
                button_read =button_press();
              if(button_read ==2) //up
                     {
                      ++hrC;   //up
                     if(hrC==24)
                         hrC=0;
                     }
              else if(button_read ==3) //down
                     {
                      --hrC;    //down
                        if(hrC==-1)
                            hrC=23;
                     }
              else hrC =hrC;

              CommandWrite(0x84);

              if(hrC < 10)
              {
                  //CommandWrite(0x85);
                  setHrs[0]= ' ';
                  setHrs[1]=(hrC%10)+48;
                  convert(setHrs);
              }
              else if((hrC>9)&&(hrC<20))
              {
                  setHrs[0]='1';
                  setHrs[1]=(hrC%10)+48;
                  convert(setHrs);
              }
              else if(hrC>=20)
              {
                  setHrs[0]='2';
                  setHrs[1]=(hrC%10)+48;
                  convert(setHrs);
              }
              else convert("00");

         }while(button_read != 1);

                do{
                    button_read =button_press();
              if(button_read ==2) //up
                     {
                      ++minC;   //up
                     if(minC==60)
                         minC=0;
                     }
              else if(button_read ==3) //down
                     {
                      --minC;    //down
                        if(minC==-1)
                            minC=59;
                     }
              else minC=minC;

              CommandWrite(0x87);
              if(minC < 10)
                {
                    //CommandWrite(0x85);
                    setMins[0]= '0';
                    setMins[1]=(minC%10)+48;
                    convert(setMins);
                }
              else{
                  setMins[0]=(uint8_t)((minC/10)+48);
                  setMins[1]=(minC%10)+48;
              }

         }while(button_read != 1);
        }

        //button_read=button_press();

         if(button_read ==4) //set alarm
        {
            do{
                button_read =button_press();
              if(button_read ==2) //up
                     {
                      ++Ahr;   //up
                     if(Ahr==24)
                         Ahr=0;
                     }
              else if(button_read ==3) //down
                     {
                      --Ahr;    //down
                        if(Ahr==-1)
                            Ahr=23;
                     }
              else Ahr =Ahr;

              CommandWrite(0xC5);
                    if(Ahr < 10)
              {

                  setHrs[0]= ' ';
                  setHrs[1]=(Ahr%10)+48;
                  convert(setHrs);
              }
              else if((Ahr>9)&&(Ahr<20))
              {
                  setHrs[0]='1';
                  setHrs[1]=(Ahr%10)+48;
                  convert(setHrs);
              }
              else if(Ahr>=20)
              {
                  setHrs[0]='2';
                  setHrs[1]=(Ahr%10)+48;
                  convert(setHrs);
              }
              else convert("00");

         }while(button_read != 4);

                do{
                button_read =button_press();
              if(button_read ==2) //up
                     {
                      ++Amin;   //up
                     if(Amin==24)
                         Amin=0;
                     }
              else if(button_read ==3) //down
                     {
                      --Amin;    //down
                        if(Amin==-1)
                            Amin=23;
                     }
              else Amin =Amin;

                setMins[0] = (Amin/10)+48;
                setMins[1] = (Amin%10)+48;
                CommandWrite(0xC4);
                convert(setHrs);

         }while(button_read != 4);
        }
    }while(button_read!=2);
     }

int button_press()
{
    int buttOut = 0;

    if(!(P4->IN & BIT0)){
    {
        buttOut = 1;
        __delay_cycles(15000);
    }
    while(!(P4->IN & BIT0)){}

    return buttOut;
    }
     if(!(P4->IN & BIT1)){
    {
        buttOut = 2;
        __delay_cycles(15000);
    }
    while(!(P4->IN & BIT1)){}

    return buttOut;
    }
     if(!(P4->IN & BIT2)){
    {
        buttOut = 3;
        __delay_cycles(15000);
    }
    while(!(P4->IN & BIT2)){}

    return buttOut;
    }
     if(!(P4->IN & BIT3)){
    {
        buttOut = 4;
        __delay_cycles(15000);
    }
    while(!(P4->IN & BIT3)){}

    return buttOut;
    }
    return buttOut;
}

int set_hours(int clock_type)
{
    int button_read;
    int hrs=0;//,minC=0,Amin=0,Ahr=0;
    uint8_t setHrs[2];//,setMins[2];

    if(clock_type == MAIN_CLOCK){
    CommandWrite(0x84);
    convert("00:00:00");}
    if(clock_type == ALARM){
    CommandWrite(0xC4);
    convert("00:00:00");}

            do{
                button_read =button_press();
              if(button_read ==2) //up
                     {
                      ++hrs;   //up
                     if(hrs==24)
                         hrs=0;
                     }
              else if(button_read ==3) //down
                     {
                      --hrs;    //down
                        if(hrs==-1)
                            hrs=23;
                     }
              else hrs =hrs;

              if(clock_type == MAIN_CLOCK)
                CommandWrite(0x84);
              else if(clock_type == ALARM)
                CommandWrite(0xC4);

              if(hrs < 10)
              {
                  //CommandWrite(0x85);
                  setHrs[0]= ' ';
                  setHrs[1]=(hrs%10)+48;
                  convert(setHrs);
              }
              else if((hrs>9)&&(hrs<20))
              {
                  setHrs[0]='1';
                  setHrs[1]=(hrs%10)+48;
                  convert(setHrs);
              }
              else if(hrs>=20)
              {
                  setHrs[0]='2';
                  setHrs[1]=(hrs%10)+48;
                  convert(setHrs);
              }
              else convert("00");

         }while(button_read != 1);

    return hrs;
}

int set_minutes(int clock_type)
{
    int button_read;
    int hrs=0;//,minC=0,Amin=0,Ahr=0;
    uint8_t setMins[2];

    if(clock_type == MAIN_CLOCK){
    CommandWrite(0x87);
    convert("00:00:00");}
    if(clock_type == ALARM){
    CommandWrite(0xC7);
    convert("00:00:00");}

            do{
                button_read =button_press();
              if(button_read ==2) //up
                     {
                      ++mins;   //up
                     if(mins==60)
                         mins=0;
                     }
              else if(button_read ==3) //down
                     {
                      --mins;    //down
                        if(mins==-1)
                            mins=59;
                     }
              else hrs =hrs;

              if(clock_type == MAIN_CLOCK)
                CommandWrite(0x87);
              else if(clock_type == ALARM)
                CommandWrite(0xC7);

              if(mins < 10)
              {
                  //CommandWrite(0x85);
                  setMins[0]= ' ';
                  setMins[1]=(mins%10)+48;
                  convert(setMins);
              }
              else if((mins>9)&&(mins<20))
              {
                  setMins[0]='1';
                  setMins[1]=(mins%10)+48;
                  convert(setMins);
              }
              else if(mins>=20 && mins <30)
              {
                  setMins[0]='2';
                  setMins[1]=(mins%10)+48;
                  convert(setMins);
              }
              else if(mins>=30 && mins <40)
              {
                  setMins[0]='3';
                  setMins[1]=(mins%10)+48;
                  convert(setMins);
              }
              else if(mins>=40 && mins <50)
              {
                  setMins[0]='4';
                  setMins[1]=(mins%10)+48;
                  convert(setMins);
              }
               else if(mins>=50 && mins <60)
              {
                  setMins[0]='5';
                  setMins[1]=(mins%10)+48;
                  convert(setMins);
              }
              else convert("00");

         }while(button_read != 1);

    return mins;
}
