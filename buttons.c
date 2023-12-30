/*
 * buttons.c
 *
 * ECE 3849 Lab 3
 * Adam Grabowski, Michael Rideout
 * Created on April 28, 2022
 *
 * ECE 3849 Lab ADC handling
 */

// XDCtools Header files
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

// BIOS Header files
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"
#include "Crystalfontz128x128_ST7735.h"
#include "math.h"
#include "peripherals.h"

// ANALOG COMPARATOR
#include "driverlib/comp.h"
#include "driverlib/pin_map.h"

// PWM INIT
#include "driverlib/pwm.h"

// clock globals
extern uint32_t gSystemClock;   // [Hz] system clock frequency
extern float clockCycle;        // [ns] one CPU clock cycle time
extern float pwmPeriod;         // [ns] period of pwm waveform

// button globals
volatile uint32_t gButtons = 0; // debounced button state, one per bit in the lowest bits
uint32_t gJoystick[2] = {0};    // joystick coordinates

// state globals
volatile bool risingSlope = true; // determines whether the slope is rising or falling
volatile uint32_t stateVperDiv = 4; // 5 states

// ADC globals
uint32_t gADCSamplingRate;      // [Hz] actual ADC sampling rate
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];           // circular buffer
extern volatile int32_t gADCBufferIndex;    // latest sample index

// CPU load globals
extern uint32_t countUnloaded;
extern uint32_t countLoaded;
extern float cpuLoad;

// initialize all button and joystick handling hardware
void ButtonInit(void)
{
    // initialize timer 3 in one-shot mode for polled timing
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, (gSystemClock)/100); // 1 sec interval

    // GPIO PJ0 and PJ1 = EK-TM4C1294XL buttons 1 and 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // analog input AIN13, at GPIO PD2 = BoosterPack Joystick HOR(X)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    // analog input AIN17, at GPIO PK1 = BoosterPack Joystick VER(Y)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    // GPIO PD4 = BoosterPack Joystick SEL
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4);

    // GPIO PH1 = BoosterPack Button 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);

    // GPIO PK6 = BoosterPack Button 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_6);

    // initialize ADC peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round divisor up
    gADCSamplingRate = pll_frequency / (16 * pll_divisor); // actual sampling rate may differ from ADC_SAMPLING_RATE
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor); // only ADC0 has PLL clock divisor control

    // initialize ADC0 sampling sequence
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH13);                            // Joystick HOR(X)
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END); // Joystick VER(Y)
    ADCSequenceEnable(ADC0_BASE, 0);
}

// update the debounced button state gButtons
void ButtonDebounce(uint32_t buttons)
{
    int32_t i, mask;
    static int32_t state[BUTTON_COUNT]; // button state

    for (i = 0; i < BUTTON_COUNT; i++) {
        mask = 1 << i;
        if (buttons & mask) {
            state[i] += BUTTON_STATE_INCREMENT;
            if (state[i] >= BUTTON_PRESSED_STATE) {
                state[i] = BUTTON_PRESSED_STATE;
                gButtons |= mask; // update debounced button state
            }
        }
        else {
            state[i] -= BUTTON_STATE_DECREMENT;
            if (state[i] <= 0) {
                state[i] = 0;
                gButtons &= ~mask;
            }
        }
    }
}

// sample joystick and convert to button presses
void ButtonReadJoystick(void)
{
    ADCProcessorTrigger(ADC0_BASE, 0);              // trigger the ADC sample sequence for Joystick X and Y
    while(!ADCIntStatus(ADC0_BASE, 0, false));      // wait until the sample sequence has completed
    ADCSequenceDataGet(ADC0_BASE, 0, gJoystick);    // retrieve joystick data
    ADCIntClear(ADC0_BASE, 0);                      // clear ADC sequence interrupt flag

    // process joystick movements as button presses using hysteresis
    if (gJoystick[0] > JOYSTICK_UPPER_PRESS_THRESHOLD) gButtons |= 1 << 5; // joystick right in position 5
    if (gJoystick[0] < JOYSTICK_UPPER_RELEASE_THRESHOLD) gButtons &= ~(1 << 5);

    if (gJoystick[0] < JOYSTICK_LOWER_PRESS_THRESHOLD) gButtons |= 1 << 6; // joystick left in position 6
    if (gJoystick[0] > JOYSTICK_LOWER_RELEASE_THRESHOLD) gButtons &= ~(1 << 6);

    if (gJoystick[1] > JOYSTICK_UPPER_PRESS_THRESHOLD) gButtons |= 1 << 7; // joystick up in position 7
    if (gJoystick[1] < JOYSTICK_UPPER_RELEASE_THRESHOLD) gButtons &= ~(1 << 7);

    if (gJoystick[1] < JOYSTICK_LOWER_PRESS_THRESHOLD) gButtons |= 1 << 8; // joystick down in position 8
    if (gJoystick[1] > JOYSTICK_LOWER_RELEASE_THRESHOLD) gButtons &= ~(1 << 8);
}

// autorepeat button presses if a button is held long enough
uint32_t ButtonAutoRepeat(void)
{
    static int count[BUTTON_AND_JOYSTICK_COUNT] = {0}; // autorepeat counts
    int i;
    uint32_t mask;
    uint32_t presses = 0;

    for (i = 0; i < BUTTON_AND_JOYSTICK_COUNT; i++) {
        mask = 1 << i;
        if (gButtons & mask)
            count[i]++;     // increment count if button is held
        else
            count[i] = 0;   // reset count if button is let go
        if (count[i] >= BUTTON_AUTOREPEAT_INITIAL &&
                (count[i] - BUTTON_AUTOREPEAT_INITIAL) % BUTTON_AUTOREPEAT_NEXT == 0)
            presses |= mask; // register a button press due to auto-repeat
    }
    return presses;
}

// returns the cpu load count
uint32_t cpuLoadCount(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

// signal button task periodically using semaphore
void clock_func(UArg arg1){
    Semaphore_post(semButtons); // to buttons
}

// TI-RTOS button task function
void buttonTask_func(UArg arg1, UArg arg2)
{
    IntMasterEnable(); // enable interrupts

    while(true){
        Semaphore_pend(semButtons, BIOS_WAIT_FOREVER); // from clock

        // read hardware button state
        uint32_t gpio_buttons =
                ~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0) |   // EK-TM4C1294XL buttons in positions 0 and 1
                (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1)) << 1 |         // BoosterPack button 1
                (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6)) >> 3 |         // BoosterPack button 2
                ~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4);                 // BoosterPack buttons joystick select button

        uint32_t old_buttons = gButtons;            // save previous button state
        ButtonDebounce(gpio_buttons);               // Run the button debouncer. The result is in gButtons.
        ButtonReadJoystick();                       // Convert joystick state to button presses. The result is in gButtons.
        uint32_t presses = ~old_buttons & gButtons; // detect button presses (transitions from not pressed to pressed)
        presses |= ButtonAutoRepeat();              // autorepeat presses if a button is held long enough
        char button_char;

        if (presses & 2) { // button 1 pressed
            // trigger slope change
            button_char = 't';
            Mailbox_post(mailbox0, &button_char, TIMEOUT);
        }

        if (presses & 4) { // boosterpack button 1 pressed
            // increment
            button_char = 'u';
            Mailbox_post(mailbox0, &button_char, TIMEOUT);
        }

        if (presses & 8) { // boosterpack button 2 pressed
            // spectrum mode
            button_char = 's';
            Mailbox_post(mailbox0, &button_char, TIMEOUT);
        }
    }
}

// TI-RTOS user input task function
void userInputTask_func(UArg arg1, UArg arg2)
{
    IntMasterEnable(); // enable interrupts

    char bpresses[10]; // holds fifo button presses

    while(true){
        // read bpresses and change stats
        if (Mailbox_pend(mailbox0, &bpresses, TIMEOUT)) {
            int i;

            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section

            for (i = 0; i < 10; i++){
                if (bpresses[i]==('t') && gButtons == 2) {          // increment state
                    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_2);
                } else if (bpresses[i]==('u') && gButtons == 4) {   // trigger
                    pwmPeriod += clockCycle;
                } else if (bpresses[i]==('s') && gButtons == 8) {   // spectrum mode
                    pwmPeriod -= clockCycle;
                }
            }

            Semaphore_post(sem_cs);
        }

        Semaphore_post(semDisplay); // to display
    }
}
