/*
 * sampling.c
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
#include "audio_waveform.h"

// ANALOG COMPARATOR
#include "driverlib/comp.h"
#include "driverlib/pin_map.h"

// PWM INIT
#include "driverlib/pwm.h"

// KISS FFT header files
#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

// KISS FFT constants
#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

// clock globals
extern uint32_t gSystemClock; // [Hz] system clock frequency

// ADC globals
uint32_t gADCSamplingRate;                              // [Hz] actual ADC sampling rate
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];          // circular buffer
volatile uint32_t gADCErrors;                           // number of missed ADC deadlines
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1; // latest sample index

// waveform globals
volatile uint32_t trigger_value;
volatile uint16_t trigger_samples[ADC_TRIGGER_SIZE];
volatile int16_t processedWaveform[ADC_TRIGGER_SIZE];
volatile uint16_t fft_samples[NFFT];

// state globals
volatile bool spectrumMode = false;             // determines the mode of the oscilloscope
float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2};   // array of voltage scale per division
float fScale;

// DMA Setup
#pragma DATA_ALIGN(gDMAControlTable, 1024)  // address alignment required
tDMAControlTable gDMAControlTable[64];      // uDMA control table (global)
volatile bool gDMAPrimary = true;           // is DMA occurring in the primary channel?

// time capture ISR
uint32_t prevCount = 0;
uint32_t timerPeriod = 0;
uint32_t multiPeriodInterval = 0;
uint32_t accumulatedPeriods = 0;
float avg_frequency;

// PWM INIT constants
#define PWM_PERIOD 258 // PWM period = 2^8 + 2 system clock cycles
#define PWM_WAVEFORM_INDEX_BITS 10
#define PWM_WAVEFORM_TABLE_SIZE (1 << PWM_WAVEFORM_INDEX_BITS)

// PWM INIT globals
uint32_t gPWMSample = 0;            // PWM sample counter
uint32_t gSamplingRateDivider;      // sampling rate divider

// initialize ADC hardware
void ADC_Init(void)
{
    // GPIO setup for analog input AIN3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

    // initialize ADC1 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1;  // round divisor up
    gADCSamplingRate = pll_frequency / (16 * pll_divisor);                      // actual sampling rate may differ from ADC_SAMPLING_RATE
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);

    // initialize ADC1 sampling sequence
    ADCSequenceDisable(ADC1_BASE, 0);                                                   // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);                          // specify the "timer" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH3);  // in the 0th step, sample channel 3 (AIN3)

    // ANALOG COMPARATOR CONFIGURE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);
    ComparatorRefSet(COMP_BASE, COMP_REF_1_65V);
    ComparatorConfigure(COMP_BASE, 1,
                        COMP_TRIG_NONE | COMP_INT_BOTH | // can also use COMP_INT_RISE
                        COMP_ASRCP_REF | COMP_OUTPUT_NORMAL);

    // configure GPIO for comparator input C1- at BoosterPack Connector #1 pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // GPIO PC_4
    GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_4);
    // configure GPIO for comparator output C1o at BoosterPack Connector #1 pin 15
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // GPIO PD_1
    GPIOPinTypeComparatorOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD1_C1O);

    // configure GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);

    // count positive edges
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff);     // use maximum load value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);   // use maximum prescale value

    // capture A event interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // initialize DMA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);
    uDMAChannelAssign(UDMA_CH24_ADC1_0); // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);

    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                           UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                           (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);

    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                           UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                           (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);

    ADCSequenceDMAEnable(ADC1_BASE, 0);         // enable DMA for ADC1 sequence 0
    ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0); // enable ADC1 sequence 0 DMA interrupt
    ADCSequenceEnable(ADC1_BASE, 0);            // enable the sequence.  it is now sampling

    // configure M0PWM5, at GPIO PG1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1); // PG1 = M0PWM5
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral, generator 2, output 5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWM_PERIOD);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, PWM_PERIOD / 2); // 50% duty cycle
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    // set gSamplingRateDivider, enable PWM interrupt in the PWM peripheral
    gSamplingRateDivider = gSystemClock / AUDIO_SAMPLING_RATE;
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO);
}

// ADC interrupt service routine
void ADC_ISR(void){
    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0); // clear the ADC1 sequence 0 DMA interrupt flag

    // check the primary DMA channel for end of transfer, restart if needed
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) == UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2); // restart primary channel (same as setup)
        gDMAPrimary = false; // DMA currently occurring in alternate buffer
    }

    // check the alternate DMA channel for end of transfer, restart if needed, and set gDMAPrimary global
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) == UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
        gDMAPrimary = true; // DMA currently occurring in alternate buffer
    }

    // DMA channel may be disabled if the CPU is paused by debugger
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
        uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10); // re-enable DMA channel
    }
}

// get index of ADC buffer
int32_t getADCBufferIndex(void)
{
    int32_t index;
    IArg key;
    key = GateHwi_enter(gateHwi0);
    if (gDMAPrimary) { // DMA currently in primary channel
        index = ADC_BUFFER_SIZE/2 - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
    }
    else { // DMA currently in alternate channel
        index = ADC_BUFFER_SIZE - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
    }
    GateHwi_leave(gateHwi0, key);
    return index;
}

// search for sample trigger
void triggerSearch(void)
{
    int32_t trigger_index;
    int32_t i;

    trigger_index = getADCBufferIndex();

    // local buffer retrieves 128 samples of gADCBuffer from trigger_index previously found
    for (i = 0; i < ADC_TRIGGER_SIZE; i++){
        trigger_samples[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger_index - (ADC_TRIGGER_SIZE - 1) + i)];
    }
}

// returns zero-crossing point of the ADC waveform by finding the max and min points, averaging them
uint32_t zeroCrossPoint(void)
{
    int max = 0;
    int min = 10000;

    int i;
    for (i = 0; i < ADC_BUFFER_SIZE; i++){
        if (gADCBuffer[i] > max){
            max = gADCBuffer[i];
        }

        if (gADCBuffer[i] < min){
            min = gADCBuffer[i];
        }
    }

    return (max+min)/2;
}

// TI-RTOS processing task function
void processingTask_func(UArg arg1, UArg arg2)
{
    IntMasterEnable(); // enable interrupts

    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE];                 // KISS FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;                             // KISS FFT buffer size
    kiss_fft_cfg cfg;                                                   // KISS FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT];                            // complex waveform and spectrum buffers
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size);   // init Kiss FFT
    int i;

    static float w[NFFT]; // window function
    for (i = 0; i < NFFT; i++) {
        // blackman window
        w[i] = (0.42f - 0.5f * cosf(2*PI*i/(NFFT-1)) + 0.08f * cosf(4*PI*i/(NFFT-1)));
    }

    while(true){
        Semaphore_pend(semProcessing, BIOS_WAIT_FOREVER); // from waveform

        if (spectrumMode){
            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section

            for (i = 0; i < NFFT; i++) { // generate an input waveform
                in[i].r = ((float)fft_samples[i] - trigger_value) * w[i];   // real part of waveform
                in[i].i = 0;                                                // imaginary part of waveform
            }

            Semaphore_post(sem_cs);

            kiss_fft(cfg, in, out); // compute FFT

            // convert first 128 bins of out[] to dB for display
            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section

            for (i = 0; i < ADC_TRIGGER_SIZE - 1; i++) {
                processedWaveform[i] = (int)roundf(128 - 10*log10f(out[i].r*out[i].r +out[i].i*out[i].i));
            }

            Semaphore_post(sem_cs);
        } else {
            // determines fScale
            fScale = (VIN_RANGE/(1 << ADC_BITS))*(PIXELS_PER_DIV/fVoltsPerDiv[stateVperDiv]);
            int i;

            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section

            for (i = 0; i < ADC_TRIGGER_SIZE - 1; i++) {
                processedWaveform[i] = ((int)(ADC_TRIGGER_SIZE/2) - (int)roundf(fScale*(int)(trigger_samples[i] - trigger_value)));
            }

            Semaphore_post(sem_cs);
        }

        Semaphore_post(semWaveform);    // to waveform
        Semaphore_post(semDisplay);     // to display
    }
}

// TI-RTOS waveform task function
void waveformTask_func(UArg arg1, UArg arg2)
{
    IntMasterEnable(); // enable interrupts

    while(true){
        Semaphore_pend(semWaveform, BIOS_WAIT_FOREVER); // from processing

        trigger_value = zeroCrossPoint(); // Dynamically finds the ADC_OFFSET
        if (spectrumMode){
            int i;
            int buffer_ind = gADCBufferIndex;

            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
            for (i = 0; i < NFFT; i++){
                fft_samples[i] = gADCBuffer[ADC_BUFFER_WRAP(buffer_ind - NFFT + i)];
            }
            Semaphore_post(sem_cs);

        } else {
            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section

            triggerSearch(); // searches for trigger

            Semaphore_post(sem_cs);
        }

        Semaphore_post(semProcessing); // to processing
    }
}

// interrupt that calculates period of ISR
void timercapture_ISR(UArg arg0){
    // clear timer0A capture interrupt flag
    TIMER0_ICR_R = TIMER_ICR_CAECINT;

    // use timervalueget() to read full 24 bit captured time count
    uint32_t currCount = TimerValueGet(TIMER0_BASE, TIMER_A);
    timerPeriod = (currCount - prevCount) & 0xFFFFFF;
    prevCount = currCount;

    multiPeriodInterval += timerPeriod;
    accumulatedPeriods++;
}

// clock to post to frequency task
void clockfreq_func(UArg arg1){
    Semaphore_post(semFrequency); // to buttons
}

// determines the average frequency
void frequencyTask_func(UArg arg1, UArg arg2) {
    IArg key;
    uint32_t accu_int, accu_count;

    while (true) {
        Semaphore_pend(semFrequency, BIOS_WAIT_FOREVER); // from clock

        key = GateHwi_enter(gateHwi0); // protect global shared data

        // retrieve global data and reset globals to 0
        accu_int = multiPeriodInterval;
        accu_count = accumulatedPeriods;
        multiPeriodInterval = 0;
        accumulatedPeriods = 0;

        GateHwi_leave(gateHwi0, key);

        // determine average frequency
        float avg_period = (float)accu_int/accu_count;
        avg_frequency = 1/(avg_period) * (float)gSystemClock;
    }
}

void PWM_ISR(void)
{
    PWM0_0_ISC_R = PWM_0_ISC_INTCNTZERO;            // clear PWM interrupt flag
    int i = (gPWMSample++) / gSamplingRateDivider;  // waveform sample index
    PWM0_2_CMPB_R = 1 + gWaveform[i];               // write directly to the PWM compare B register

    if (i = gWaveformSize - 1) {                    // if at the end of the waveform array
        PWMIntDisable(PWM0_BASE, PWM_INT_GEN_2);    // disable these interrupts
        gPWMSample = 0;                             // reset sample index so the waveform starts from the beginning
    }
}
