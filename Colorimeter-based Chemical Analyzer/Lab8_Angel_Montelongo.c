//Angel Montelongo
//1001665238

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "clock.h"
#include "uart0.h"
#include "wait.h"
#include "rgb_led.h"
#include "adc0.h"
#include "tm4c123gh6pm.h"

#define A1    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
#define A2    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define B2    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define B1    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))

//PORTE MASKS

#define A1_MASK 16
#define A2_MASK 32
#define B2_MASK 8
#define B1_MASK 4
#define PR 6
#define P5 39
#define P4 73
#define P3 106
#define P2 139
#define P1 172

//PORTA MASKS

#define PB5_MASK 32

//PortD MASKS
#define FREQ_IN_MASK 1

//REMOTE DATA
int address[16] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1};
uint8_t buttons[20][16] = {
                       {0,0,1,1,1,0,1,0,1,1,0,0,0,1,0,1},
                       {1,0,1,1,1,0,1,0,0,1,0,0,0,1,0,1},
                       {0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1},
                       {1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,1},
                       {0,1,1,1,0,0,0,0,1,0,0,0,1,1,1,1},
                       {0,0,0,1,0,0,0,0,1,1,1,0,1,1,1,1},
                       {1,0,0,1,0,0,0,0,0,1,1,0,1,1,1,1},
                       {0,1,0,1,0,0,0,0,1,0,1,0,1,1,1,1},
                       {0,0,1,0,1,0,0,0,1,1,0,1,0,1,1,1},
                       {1,0,1,0,1,0,0,0,0,1,0,1,0,1,1,1},
                       {0,1,1,0,1,0,0,0,1,0,0,1,0,1,1,1},
                       {0,0,0,0,1,0,0,0,1,1,1,1,0,1,1,1},
                       {1,0,0,0,1,0,0,0,0,1,1,1,0,1,1,1},
                       {0,1,0,0,1,0,0,0,1,0,1,1,0,1,1,1},
                       {0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,1},
                       {1,0,0,1,1,0,1,0,0,1,1,0,0,1,0,1},
                       {1,0,1,0,0,0,1,0,0,1,0,1,1,1,0,1},
                       {0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1},
                       {1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1},
                       {1,0,0,1,0,0,1,0,0,1,1,0,1,1,0,1}
};

#define T (1.25/2.0)
#define CLOCKSPERMS 40000.0

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

uint32_t time[50];
uint8_t count = 0;
uint8_t code[32];
bool validcode = false;
int codeindex;
uint8_t temp;
int phase;
int position;

#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

typedef struct _ph
{
    float pH;
    float red;
    float green;
    float blue;
    float distance;
} pH;

uint16_t pwm_r;
uint16_t pwm_g;
uint16_t pwm_b;
uint16_t raw_r = 0;
uint16_t raw_g = 0;
uint16_t raw_b = 0;

pH ref[] = {{6.8, 3049.0/3075.0, 3102.0/3094.0, 773.0/3074.0, 0.0},
            {7.2, 3085.0/3075.0, 1706.0/3094.0, 731.0/3074.0, 0.0},
            {7.5, 3081.0/3075.0, 1396.0/3094.0, 794.0/3074.0, 0.0},
            {7.8, 3081.0/3075.0, 1121.0/3094.0, 686.0/3074.0, 0.0},
            {8.2, 2952.0/3075.0, 522.0/3094.0, 653.0/3074.0, 0.0}};

float pH_measure;

uint16_t red_measure;
uint16_t green_measure;
uint16_t blue_measure;

char str2[100];

void goTo(int tube);

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void applyPhase (int phasein)
{
    switch (phasein)
    {
        case 0:
            A1 = 1;
            A2 = 0;
            B2 = 0;
            B1 = 0;
            break;
        case 1:
            A1 = 0;
            A2 = 0;
            B2 = 1;
            B1 = 0;
            break;
        case 2:
            A1 = 0;
            A2 = 1;
            B2 = 0;
            B1 = 0;
            break;
        case 3:
            A1 = 0;
            A2 = 0;
            B2 = 0;
            B1 = 1;
            break;
    }

    phase = phasein;
}

void stepCw()
{
    phase = (phase+1)%4;
    applyPhase(phase);
}

void stepCcw()
{
    phase--;
    if(phase<0)
    {
        phase = 3;
    }
    applyPhase(phase);
}

void setPosition( int positionin )
{
    position = positionin;
}

void home()
{

    int i;
    for(i = 0; i < 200; i++)
    {
        stepCw();
        waitMicrosecond(10000);
    }

    setPosition(0);

    goTo(0);

}

void goTo(int tube)
{
    int positionin;
    switch (tube)
    {
    case 0:
        positionin = PR;
        break;
    case 1:
        positionin = P1;
        break;
    case 2:
        positionin = P2;
        break;
    case 3:
        positionin = P3;
        break;
    case 4:
        positionin = P4;
        break;
    case 5:
        positionin = P5;
        break;
    }
    int i;
    if(positionin > position)
    {
         for(i = position; i < positionin; i++)
         {
             stepCcw();
             waitMicrosecond(10000);
         }
    }
    else
    {
         for(i = position; i > positionin; i--)
         {
             stepCw();
             waitMicrosecond(10000);
         }
    }

    setPosition(positionin);
}

void getsUart0(USER_DATA * data)
{
    int count = 0;

    while(true)
    {
        char c = getcUart0();
        if(c == 8 && count > 0)
        {
            count--;
        }
        else if(c == 127 && count > 0)
        {
            count--;
        }
        else if(c == 13)
        {
            data->buffer[count] = '\0';
            return;
        }
        else if(c >= 32)
        {
            data->buffer[count] = c;
            count++;
            if(count == MAX_CHARS)
            {
                data->buffer[count] = '\0';
                return;
            }
        }
    }
}

void parseFields(USER_DATA * data)
{
    int count = 0;
    data->fieldCount = 0;
    int previoustype = 0;
    int currenttype = 0;

    while(data->buffer[count] != NULL)
    {
        if(data->buffer[count] >= 65 && data->buffer[count] <= 90)
        {
            currenttype = 1;
            if(currenttype != previoustype)
            {
                data->fieldType[data->fieldCount] = 'a';
                data->fieldPosition[data->fieldCount] = count;
                data->fieldCount += 1;
            }
            previoustype = currenttype;
        }
        else if (data->buffer[count] >= 97 && data->buffer[count] <= 122)
        {
            currenttype = 1;
            if(currenttype != previoustype)
            {
                data->fieldType[data->fieldCount] = 'a';
                data->fieldPosition[data->fieldCount] = count;
                data->fieldCount += 1;
            }
            previoustype = currenttype;
        }
        else if (data->buffer[count] >= 48 && data->buffer[count] <= 57)
        {
            currenttype = 2;
            if(currenttype != previoustype)
            {
                data->fieldType[data->fieldCount] = 'n';
                data->fieldPosition[data->fieldCount] = count;
                data->fieldCount += 1;
            }
            previoustype = currenttype;
        }
        else if (data->buffer[count] == 46)
        {
            currenttype = 2;
            if(currenttype != previoustype)
            {
                data->fieldType[data->fieldCount] = 'n';
                data->fieldPosition[data->fieldCount] = count;
                data->fieldCount += 1;
            }
            previoustype = currenttype;
        }
        else
        {
            data->buffer[count] = '\0';
            previoustype = 0;
        }
        count++;
    }
    return;
}

char * getFieldString(USER_DATA * data, uint8_t fieldNumber)
{
    if(fieldNumber <= (data->fieldCount))
    {
    return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
    return NULL;
    }
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    int count = 0;
    int32_t result = 0;

    if(data->fieldType[fieldNumber] == 'n')
    {
            count = data->fieldPosition[fieldNumber];
            while(data->buffer[count] != '\0')
            {

                result = result * 10 + (data->buffer[count] - 48);
                count++;
            }

            return result;
        }
        else
        {
            return 0;
        }
}

bool isCommand(USER_DATA * data, const char strCommand[], uint8_t minArguments)
{

    if(strcmp(strCommand, &data->buffer[data->fieldPosition[0]]) == 0)
    {
        if(minArguments <= data->fieldCount)
        {
            return true;
        }
        return false;
    }
    else
    {
        return false;
    }
}

void calibrate()
{
    int i = 0;

    goTo(0);
    raw_r = 0;
    raw_g = 0;
    raw_b = 0;

    waitMicrosecond(100000);

    while( raw_r < 3072 && i < 1023)
    {
        setRgbColor(i, 0, 0);
        waitMicrosecond(10000);
        raw_r = readAdc0Ss3();
        i++;
    }

    pwm_r = i;
    i = 0;

    waitMicrosecond(100000);

    while( raw_g < 3072 && i < 1023)
    {
        setRgbColor(0, i, 0);
        waitMicrosecond(10000);
        raw_g = readAdc0Ss3();
        i++;
    }

    pwm_g = i;
    i = 0;

    waitMicrosecond(100000);

    while( raw_b < 3072 && i < 1023)
    {
        setRgbColor(0, 0, i);
        waitMicrosecond(10000);
        raw_b = readAdc0Ss3();
        i++;
    }

    pwm_b = i;

    waitMicrosecond(10000);
    setRgbColor(0, 0, 0);

    char str2[100];
    char str3[100];
    char str4[100];
    char str5[100];
    char str6[100];
    char str7[100];

    sprintf(str2, "Calibrated values:    (%4u, ", raw_r);
                        putsUart0(str2);
                        sprintf(str3, "%4u, ", raw_g);
                        putsUart0(str3);
                        sprintf(str4, "%4u)\n", raw_b);
                        putsUart0(str4);
    sprintf(str5, "PWM values:    (%4u, ", pwm_r);
         putsUart0(str5);
         sprintf(str6, "%4u, ", pwm_g);
         putsUart0(str6);
         sprintf(str7, "%4u)\n", pwm_b);
         putsUart0(str7);


}

void measure(int tube, uint16_t *r, uint16_t *g, uint16_t *b)
{
    goTo(tube);

    waitMicrosecond(100000);

    int i = 0;

    *r = 0;

    while( *r < 3072 && i < pwm_r)
    {
        setRgbColor(i, 0, 0);
        waitMicrosecond(10000);
        *r = readAdc0Ss3();
        i++;
    }

    i = 0;
    *g = 0;

    waitMicrosecond(100000);

    while( *g < 3072 && i < pwm_g)
    {
        setRgbColor(0, i, 0);
        waitMicrosecond(10000);
        *g = readAdc0Ss3();
        i++;
    }

    i = 0;
    *b = 0;

    waitMicrosecond(100000);

    while( *b < 3072 && i < pwm_b)
    {
        setRgbColor(0, 0, i);
        waitMicrosecond(10000);
        *b = readAdc0Ss3();
        i++;
    }

    waitMicrosecond(100000);
    setRgbColor(0, 0, 0);

}

void measurePH(int tube, uint16_t *r, uint16_t *g, uint16_t *b, float *pH)
{
    goTo(tube);

        waitMicrosecond(100000);

        int i = 0;

        *r = 0;

        while( *r < 3072 && i < pwm_r)
        {
            setRgbColor(i, 0, 0);
            waitMicrosecond(10000);
            *r = readAdc0Ss3();
            i++;
        }

        float red = *r / 3075.0;

        i = 0;
        *g = 0;

        waitMicrosecond(100000);

        while( *g < 3072 && i < pwm_g)
        {
            setRgbColor(0, i, 0);
            waitMicrosecond(10000);
            *g = readAdc0Ss3();
            i++;
        }

        float green = *g/3094.0;

        i = 0;
        *b = 0;

        waitMicrosecond(100000);

        while( *b < 3072 && i < pwm_b)
        {
            setRgbColor(0, 0, i);
            waitMicrosecond(10000);
            *b = readAdc0Ss3();
            i++;
        }

        float blue = *b / 3074.0;

        waitMicrosecond(100000);
        setRgbColor(0, 0, 0);

        ref[0].distance = ((red - ref[0].red)*(red - ref[0].red) + (green - ref[0].green)*(green - ref[0].green) + (blue - ref[0].blue)*(blue - ref[0].blue));
        ref[1].distance = ((red - ref[1].red)*(red - ref[1].red) + (green - ref[1].green)*(green - ref[1].green) + (blue - ref[1].blue)*(blue - ref[1].blue));
        ref[2].distance = ((red - ref[2].red)*(red - ref[2].red) + (green - ref[2].green)*(green - ref[2].green) + (blue - ref[2].blue)*(blue - ref[2].blue));
        ref[3].distance = ((red - ref[3].red)*(red - ref[3].red) + (green - ref[3].green)*(green - ref[3].green) + (blue - ref[3].blue)*(blue - ref[3].blue));
        ref[4].distance = ((red - ref[4].red)*(red - ref[4].red) + (green - ref[4].green)*(green - ref[4].green) + (blue - ref[4].blue)*(blue - ref[4].blue));
//1
        if(ref[0].distance < ref[1].distance && ref[1].distance < ref[2].distance && ref[1].distance < ref[3].distance && ref[1].distance < ref[4].distance)
        {
            *pH = ((ref[1].pH - ref[0].pH)*(ref[0].distance/(ref[1].distance+ref[0].distance))) + ref[0].pH;
        }
        else if(ref[0].distance < ref[2].distance && ref[2].distance < ref[1].distance && ref[2].distance < ref[3].distance && ref[2].distance < ref[4].distance)
        {
            *pH = ((ref[2].pH - ref[0].pH)*(ref[0].distance/(ref[2].distance+ref[0].distance))) + ref[0].pH;
        }
        else if(ref[0].distance < ref[3].distance && ref[3].distance < ref[1].distance && ref[3].distance < ref[2].distance && ref[3].distance < ref[4].distance)
                {
                    *pH = ((ref[3].pH - ref[0].pH)*(ref[0].distance/(ref[3].distance+ref[0].distance))) + ref[0].pH;
                }
        else if(ref[0].distance < ref[4].distance && ref[4].distance < ref[1].distance && ref[4].distance < ref[3].distance && ref[4].distance < ref[2].distance)
                {
                    *pH = ((ref[4].pH - ref[0].pH)*(ref[0].distance/(ref[4].distance+ref[0].distance))) + ref[0].pH;
                }
        //2
        else if(ref[1].distance < ref[0].distance && ref[0].distance < ref[2].distance && ref[0].distance < ref[3].distance && ref[0].distance < ref[4].distance)
        {
            *pH = ((ref[0].pH - ref[1].pH)*(ref[1].distance/(ref[1].distance+ref[0].distance))) + ref[1].pH;
        }
        else if(ref[1].distance < ref[2].distance && ref[2].distance < ref[0].distance && ref[2].distance < ref[3].distance && ref[2].distance < ref[4].distance)
        {
            *pH = ((ref[2].pH - ref[1].pH)*(ref[1].distance/(ref[1].distance+ref[2].distance))) + ref[1].pH;
        }
        else if(ref[1].distance < ref[3].distance && ref[3].distance < ref[0].distance && ref[3].distance < ref[2].distance && ref[3].distance < ref[4].distance)
                {
                    *pH = ((ref[3].pH - ref[1].pH)*(ref[1].distance/(ref[1].distance+ref[3].distance))) + ref[1].pH;
                }
        else if(ref[1].distance < ref[4].distance && ref[4].distance < ref[0].distance && ref[4].distance < ref[3].distance && ref[4].distance < ref[2].distance)
                {
                    *pH = ((ref[4].pH - ref[1].pH)*(ref[1].distance/(ref[1].distance+ref[4].distance))) + ref[1].pH;
                }
        //3
        else if(ref[2].distance < ref[0].distance && ref[0].distance < ref[1].distance && ref[0].distance < ref[3].distance && ref[0].distance < ref[4].distance)
        {
            *pH = ((ref[0].pH - ref[2].pH)*(ref[2].distance/(ref[0].distance+ref[2].distance))) + ref[2].pH;
        }
        else if(ref[2].distance < ref[1].distance && ref[1].distance < ref[0].distance && ref[1].distance < ref[3].distance && ref[1].distance < ref[4].distance)
        {
           *pH = ((ref[1].pH - ref[2].pH)*(ref[2].distance/(ref[1].distance+ref[2].distance))) + ref[2].pH;
        }
        else if(ref[2].distance < ref[3].distance && ref[3].distance < ref[0].distance && ref[3].distance < ref[1].distance && ref[3].distance < ref[4].distance)
                {
                    *pH = ((ref[3].pH - ref[2].pH)*(ref[2].distance/(ref[3].distance+ref[2].distance))) + ref[2].pH;
                }
        else if(ref[2].distance < ref[4].distance && ref[4].distance < ref[0].distance && ref[4].distance < ref[3].distance && ref[4].distance < ref[1].distance)
                {
                    *pH = ((ref[4].pH - ref[2].pH)*(ref[2].distance/(ref[4].distance+ref[2].distance))) + ref[2].pH;
                }
        //4
        else if(ref[3].distance < ref[0].distance && ref[0].distance < ref[1].distance && ref[0].distance < ref[2].distance && ref[0].distance < ref[4].distance)
                {
                    *pH = ((ref[0].pH - ref[3].pH)*(ref[3].distance/(ref[0].distance+ref[3].distance))) + ref[3].pH;
                }
                else if(ref[3].distance < ref[1].distance && ref[1].distance < ref[0].distance && ref[1].distance < ref[2].distance && ref[1].distance < ref[4].distance)
                {
                    *pH = ((ref[1].pH - ref[3].pH)*(ref[3].distance/(ref[1].distance+ref[3].distance))) + ref[3].pH;
                }
                else if(ref[3].distance < ref[4].distance && ref[4].distance < ref[0].distance && ref[4].distance < ref[1].distance && ref[4].distance < ref[2].distance)
                        {
                            *pH = ((ref[4].pH - ref[3].pH)*(ref[3].distance/(ref[4].distance+ref[3].distance))) + ref[3].pH;
                        }
                else if(ref[3].distance < ref[2].distance && ref[2].distance < ref[0].distance && ref[2].distance < ref[4].distance && ref[2].distance < ref[1].distance)
                        {
                            *pH = ((ref[2].pH - ref[3].pH)*(ref[3].distance/(ref[3].distance+ref[2].distance))) + ref[3].pH;
                        }
        //5
         else if(ref[4].distance < ref[0].distance && ref[0].distance < ref[1].distance && ref[0].distance < ref[3].distance && ref[0].distance < ref[2].distance)
         {
                            *pH = ((ref[0].pH - ref[4].pH)*(ref[4].distance/(ref[0].distance+ref[4].distance))) + ref[4].pH;
         }
                        else if(ref[4].distance < ref[1].distance && ref[1].distance < ref[0].distance && ref[1].distance < ref[3].distance && ref[1].distance < ref[2].distance)
                        {
                            *pH = ((ref[1].pH - ref[4].pH)*(ref[4].distance/(ref[1].distance+ref[4].distance))) + ref[4].pH;
                        }
                        else if(ref[4].distance < ref[3].distance && ref[3].distance < ref[0].distance && ref[3].distance < ref[1].distance && ref[3].distance < ref[2].distance)
                                {
                                    *pH = ((ref[3].pH - ref[4].pH)*(ref[4].distance/(ref[3].distance+ref[4].distance))) + ref[4].pH;
                                }
                        else if(ref[4].distance < ref[2].distance && ref[2].distance < ref[0].distance && ref[2].distance < ref[1].distance && ref[2].distance < ref[3].distance)
                                {
                                    *pH = ((ref[2].pH - ref[4].pH)*(ref[4].distance/(ref[2].distance+ref[4].distance))) + ref[4].pH;
                                }
}

void enableTimerMode()
{
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_NEG;           // measure time from positive edge to positive edge
    WTIMER2_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER2_TAV_R = 0;                               // zero counter for first period
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER2A-16-96);         // turn-on interrupt 114 (WTIMER2A)
}

bool validateCode()
{
    int i;
    int j;
    bool check = true;
    //Validate address
    for(i = 0; i < 16; i++)
    {
        if(code[i] != address[i])
        {
            return false;
        }
    }
    for(i = 0; i < 20; i++)
    {
        for(j = 0; j < 16; j++)
        {
            if(code[16+j] != buttons[i][j])
            {
                check = false;
            }
        }
        if(check == true)
        {
            codeindex = i;
            return validcode = true;
        }
        else
        {
            check = true;
        }
    }
    return validcode = false;
}

void wideTimer2Isr()
{
    if (WTIMER2_TAV_R >= 80.0 * CLOCKSPERMS) count = 0;

    if(count == 0)
    {
         WTIMER2_TAV_R = 0;
         time[count] = 0;
         count++;
    }
    else if(count == 1)
    {
         time[count] = WTIMER2_TAR_R;
         if((time[1]-time[0]) >= (13.0*CLOCKSPERMS) && (time[1]-time[0]) <= (14.0*CLOCKSPERMS))
         {
             count++;
         }
         else
         {
             count = 0;
         }
     }
    else if(count > 1)
     {
         time[count] = WTIMER2_TAR_R;
         if((time[count]-time[count-1]) > 1.5*T*CLOCKSPERMS && (time[count]-time[count-1]) < 2.5*T*CLOCKSPERMS)
         {
             count++;
         }
         else if((time[count]-time[count-1]) > 3.5*T*CLOCKSPERMS && (time[count]-time[count-1]) < 4.5*T*CLOCKSPERMS)
         {
             count++;
         }
         else
         {
             count = 0;
         }
      }

      WTIMER2_ICR_R = TIMER_ICR_CAECINT;

      uint8_t i;

      uint32_t t;


        if(count == 34)
        {
            count = 0;
            for(i = 1; i < 33; i++)
            {
                t = time[i+1] - time[i];
                if( t > 1.5*T*CLOCKSPERMS && t < 2.5*T*CLOCKSPERMS)
                {
                    temp = 0;
                    code[i-1] = temp;
                    //code |= (0 << (i-1));
                }
                else if(t > 3.5*T*CLOCKSPERMS && t < 4.5*T*CLOCKSPERMS)
                {
                    temp = 1;
                    code[i-1] = temp;
                    //code |= (1 << (i-1));
                } else {
                    validcode = false;
                    break;
                }


            }
            validcode = validateCode();

        }
}



// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    GPIO_PORTE_DIR_R |= A1_MASK | A2_MASK | B2_MASK | B1_MASK;
    GPIO_PORTE_DR2R_R |= A1_MASK | A2_MASK | B2_MASK | B1_MASK;
    GPIO_PORTE_DEN_R |=  A1_MASK | A2_MASK | B2_MASK | B1_MASK;

    // Configure AIN3 as an analog input
    GPIO_PORTB_AFSEL_R |= PB5_MASK;                 // select alternative functions for AN3 (PE0)
    GPIO_PORTB_DEN_R &= ~PB5_MASK;                  // turn off digital operation on pin PE0
    GPIO_PORTB_AMSEL_R |= PB5_MASK;                 // turn on analog operation on pin PE0

    GPIO_PORTD_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for SIGNAL_IN pin
        GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD0_M;           // map alt fns to SIGNAL_IN
        GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_WT2CCP0;
        GPIO_PORTD_DEN_R |= FREQ_IN_MASK;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    USER_DATA data;
    bool valid;
    int tube;

    initHw();
    initUart0();
    initRgb();
    initAdc0Ss3();

    enableTimerMode();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    setAdc0Ss3Mux(11);
    setAdc0Ss3Log2AverageCount(2);

    phase = 0 ;

    home();

    while(true)
    {
        if(validcode)
                           {
                               char str2[100];
                               char str3[100];
                               char str4[100];

                               switch(codeindex)
                               {
                               case 0:
                                   home();
                                   break;
                               case 1:
                                   calibrate();
                                   break;
                               case 2:
                                   goTo(1);
                                   break;
                               case 3:
                                   goTo(2);
                                   break;
                               case 4:
                                   goTo(3);
                                   break;
                               case 5:
                                   goTo(4);
                                   break;
                               case 6:
                                   goTo(5);
                                   break;
                               case 7:
                                   goTo(0);
                                   break;
                               case 8:
                                               measure(1, &red_measure, &green_measure, &blue_measure);
                                               sprintf(str2, "Measured RAW value:    (%4u, ", red_measure);
                                               putsUart0(str2);
                                               sprintf(str3, "%4u, ", green_measure);
                                               putsUart0(str3);
                                               sprintf(str4, "%4u)\n", blue_measure);
                                               putsUart0(str4);
                                               break;
                               case 9:
                                               measure(2, &red_measure, &green_measure, &blue_measure);
                                               sprintf(str2, "Measured RAW value:    (%4u, ", red_measure);
                                               putsUart0(str2);
                                               sprintf(str3, "%4u, ", green_measure);
                                               putsUart0(str3);
                                               sprintf(str4, "%4u)\n", blue_measure);
                                               putsUart0(str4);
                                               break;
                               case 10:
                                               measure(3, &red_measure, &green_measure, &blue_measure);
                                               sprintf(str2, "Measured RAW value:    (%4u, ", red_measure);
                                               putsUart0(str2);
                                               sprintf(str3, "%4u, ", green_measure);
                                               putsUart0(str3);
                                               sprintf(str4, "%4u)\n", blue_measure);
                                               putsUart0(str4);
                                               break;
                               case 11:
                                               measure(4, &red_measure, &green_measure, &blue_measure);
                                               sprintf(str2, "Measured RAW value:    (%4u, ", red_measure);
                                               putsUart0(str2);
                                               sprintf(str3, "%4u, ", green_measure);
                                               putsUart0(str3);
                                               sprintf(str4, "%4u)\n", blue_measure);
                                               putsUart0(str4);
                                               break;
                               case 12:
                                               measure(5, &red_measure, &green_measure, &blue_measure);
                                               sprintf(str2, "Measured RAW value:    (%4u, ", red_measure);
                                               putsUart0(str2);
                                               sprintf(str3, "%4u, ", green_measure);
                                               putsUart0(str3);
                                               sprintf(str4, "%4u)\n", blue_measure);
                                               putsUart0(str4);
                                               break;
                               case 13:
                                               measure(0, &red_measure, &green_measure, &blue_measure);
                                               sprintf(str2, "Measured RAW value:    (%4u, ", red_measure);
                                               putsUart0(str2);
                                               sprintf(str3, "%4u, ", green_measure);
                                               putsUart0(str3);
                                               sprintf(str4, "%4u)\n", blue_measure);
                                               putsUart0(str4);
                                               break;
                               case 14:
                                               measurePH(1, &red_measure, &green_measure, &blue_measure, &pH_measure);
                                               sprintf(str2, "Measured pH level:    %.8f\n", pH_measure);
                                               putsUart0(str2);
                                               break;
                               case 15:
                                               measurePH(2, &red_measure, &green_measure, &blue_measure, &pH_measure);
                                               sprintf(str2, "Measured pH level:    %.8f\n", pH_measure);
                                               putsUart0(str2);
                                               break;
                               case 16:
                                               measurePH(3, &red_measure, &green_measure, &blue_measure, &pH_measure);
                                               sprintf(str2, "Measured pH level:    %.8f\n", pH_measure);
                                               putsUart0(str2);
                                               break;
                               case 17:
                                               measurePH(4, &red_measure, &green_measure, &blue_measure, &pH_measure);
                                               sprintf(str2, "Measured pH level:    %.8f\n", pH_measure);
                                               putsUart0(str2);
                                               break;
                               case 18:
                                               measurePH(5, &red_measure, &green_measure, &blue_measure, &pH_measure);
                                               sprintf(str2, "Measured pH level:    %.8f\n", pH_measure);
                                               putsUart0(str2);
                                               break;
                               case 19:
                                               measurePH(0, &red_measure, &green_measure, &blue_measure, &pH_measure);
                                               sprintf(str2, "Measured pH level:    %.8f\n", pH_measure);
                                               putsUart0(str2);
                                               break;

                               }
                               validcode = 0;
                           }
        if(kbhitUart0())
        {
            getsUart0(&data);
                    parseFields(&data);

                    valid = false;

                    if(isCommand(&data, "home", 0))
                    {
                        valid = true;
                        home();
                    }
                    if(isCommand(&data, "tube", 1))
                    {
                        tube = getFieldInteger(&data, 1);
                        if(tube < 0 || tube > 5)
                        {
                            valid = false;
                        }
                        else
                        {
                            goTo(tube);
                            valid = true;
                        }
                    }
                    if(isCommand(&data, "calibrate", 0))
                    {
                        valid = true;
                        calibrate();
                    }
                    if(isCommand(&data, "measure", 2))
                    {
                        valid = true;
                        tube = getFieldInteger(&data, 1);
                        char * str = getFieldString(&data, 2);
                        if(tube < 0 || tube > 5)
                        {
                            valid = false;
                        }
                        else
                        {
                            char str2[100];
                            char str3[100];
                            char str4[100];
                            valid = true;
                            if(strcmp("raw", str) == 0)
                            {
                                measure(tube, &red_measure, &green_measure, &blue_measure);
                                sprintf(str2, "Measured RAW value:    (%4u, ", red_measure);
                                putsUart0(str2);
                                sprintf(str3, "%4u, ", green_measure);
                                putsUart0(str3);
                                sprintf(str4, "%4u)\n", blue_measure);
                                putsUart0(str4);

                            }
                            else if(strcmp("pH", str) == 0)
                            {
                                measurePH(tube, &red_measure, &green_measure, &blue_measure, &pH_measure);
                                sprintf(str2, "Measured pH level:    %.8f\n", pH_measure);
                                putsUart0(str2);
                            }
                            else
                            {
                                valid = false;
                            }
                        }
                      }

                    if (!valid)
                    putsUart0("Invalid command\n");
        }
    }
    }
