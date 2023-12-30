/*
 * main.c
 *
 * ECE 3849 Lab 3
 * Adam Grabowski, Michael Rideout
 * Created on April 28, 2022
 *
 * EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack
 */

// XDCtools Header files
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

// BIOS Header files
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "Crystalfontz128x128_ST7735.h"
#include "sysctl_pll.h"
#include "peripherals.h"

// clock globals
uint32_t gSystemClock = 120000000;  // [Hz] system clock frequency
float clockCycle = 8.3;             // [ns] one CPU clock cycle time
float pwmPeriod = 50000;            // [ns] period of pwm waveform
float pwmFrequency;                 // [Hz] frequency of pwm waveform

// display globals
tContext sContext;
const char * const gVoltageScaleStr[] = {"100mV", "200mV", "500mV", "1V", "2V"};
const char * const gTriggerSlopeStr[] = {"Rising", "Falling"};

// CPU load globals
uint32_t countUnloaded = 0;    // CPU count unloaded
uint32_t countLoaded = 0;      // CPU count loaded
float cpuLoad = 0.0;           // CPU load value

// initialize signal source
void signalInit(void);

// main function
int main(void)
{
    IntMasterDisable(); // disable interrupts

    pwmFrequency = 1 / pwmPeriod * 1000000000;  // compute pwmFrequency
    signalInit();                               // initialize signal source

    // initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init();                             // initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    GrContextInit(&sContext, &g_sCrystalfontz128x128);  // initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8);      // select font

    ButtonInit();   // initialize all button and joystick handling hardware
    ADC_Init();     // initialize ADC hardware

    countUnloaded = cpuLoadCount(); // get initial cpu load count

    BIOS_start(); // start BIOS

    return (0);
}

// TI-RTOS display task function
void displayTask_func(UArg arg1, UArg arg2)
{
    IntMasterEnable(); // enable interrupts

    char tscale_str[50];        // time string buffer for time scale
    char vscale_str[50];        // time string buffer for voltage scale
    char tslope_str[50];        // time string buffer for trigger edge
    char frequency_str[50];     // time string buffer for frequency edge
    char period_str[50];        // time string buffer for period

    while(true){
        pwmFrequency = 1 / pwmPeriod * 1000000000;  // compute pwmFrequency
        signalInit();                               // initialize signal source

        Semaphore_pend(semDisplay, BIOS_WAIT_FOREVER);  // from user input
        Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER);      // protect critical section

        countLoaded = cpuLoadCount();
        cpuLoad = 1.0f - (float)countLoaded/countUnloaded; // compute CPU load

        Semaphore_post(sem_cs); // protect critical section

        // full-screen rectangle
        tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);         // fill screen with black
        GrContextForegroundSet(&sContext, ClrWhite);    // yellow text

        // draw grid in blue
        GrContextForegroundSet(&sContext, ClrBlue); // yellow text
        int i;
        for (i = 1; i < 128; i+=21){
            GrLineDraw(&sContext, i, 0, i, 128);
            GrLineDraw(&sContext, 0, i, 128, i);
        }

        // draw center grid lines in dark blue
        GrContextForegroundSet(&sContext, ClrDarkBlue); // blue
        if (spectrumMode){
            GrLineDraw(&sContext, 0, 22, 128, 22);
        } else {
            GrLineDraw(&sContext, 64, 0, 64, 128);
            GrLineDraw(&sContext, 0, 64, 128, 64);
        }

        // draw waveform
        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
        int x;
        int y_old;
        Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
        for (x = 0; x < LCD_HORIZONTAL_MAX - 1; x++) {
            if (x!=0)
                GrLineDraw(&sContext, x-1, y_old, x, processedWaveform[x]);
            y_old = processedWaveform[x];
        }

        Semaphore_post(sem_cs);

        // time scale, voltage scale, trigger slope and CPU load
        GrContextForegroundSet(&sContext, ClrWhite); // yellow text
        if (spectrumMode){
            snprintf(tscale_str, sizeof(tscale_str), "20kHz");  // convert time scale to string
            snprintf(vscale_str, sizeof(vscale_str), "20dB");   // convert vscale to string
        } else {
            snprintf(tscale_str, sizeof(tscale_str), "20us");                               // convert time scale to string
            snprintf(vscale_str, sizeof(vscale_str), gVoltageScaleStr[stateVperDiv]);       // convert vscale to string
            snprintf(tslope_str, sizeof(tslope_str), gTriggerSlopeStr[risingSlope]);        // convert slope to string
            snprintf(frequency_str, sizeof(frequency_str), "f=%.0fHz", avg_frequency);      // convert frequency to string
            snprintf(period_str, sizeof(period_str), "T=%.0fns", pwmPeriod);                // convert period to string

            GrStringDraw(&sContext, tslope_str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 + 20, /*y*/ 5, /*opaque*/ false);
            GrStringDraw(&sContext, frequency_str, /*length*/ -1, /*x*/ 6, /*y*/ 118, /*opaque*/ false);
            GrStringDraw(&sContext, period_str, /*length*/ -1, /*x*/ 70, /*y*/ 118, /*opaque*/ false);
        }

        GrStringDraw(&sContext, tscale_str, /*length*/ -1, /*x*/ 7, /*y*/ 5, /*opaque*/ false);
        GrStringDraw(&sContext, vscale_str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 - 18, /*y*/ 5, /*opaque*/ false);

        GrFlush(&sContext); // flush the frame buffer to the LCD

        Semaphore_pend(semDisplay, BIOS_WAIT_FOREVER); // block again
    }
}

// initialize signal source
void signalInit(void)
{
    // configure M0PWM2, at GPIO PF2, which is BoosterPack 1 header C1 (2nd from right) pin 2
    // configure M0PWM3, at GPIO PF3, which is BoosterPack 1 header C1 (2nd from right) pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3); // PF2 = M0PWM2, PF3 = M0PWM3
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral, generator 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, roundf((float)gSystemClock/pwmFrequency));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf((float)gSystemClock/pwmFrequency*0.4f)); // 40% duty cycle
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf((float)gSystemClock/pwmFrequency*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}
