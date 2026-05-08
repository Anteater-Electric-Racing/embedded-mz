// Anteater Electric Racing, 2025

#include "adc.h"
#include "./vehicle/telemetry.h"
#include "utils/utils.h"
#include "vehicle/apps.h"
#include "vehicle/bse.h"
#include "vehicle/faults.h"
#include "vehicle/motor.h"
#include "vehicle/shockTravel.h"
#include "vehicle/telemetry.h"
#include "vehicle/thermal.h"
#include <ADC.h>
#include <chrono>
#include <stdint.h>

#include "imxrt.h"
#include "DMAChannel.h"
#include "wiring.h"
#include "QuadEncoder.h"

static void DMA_ISR();

constexpr uint8_t NUM_CHANNELS              = 8;       // Number of active channels
constexpr uint8_t NUM_SCANS_PER_CHANNEL     = 1;      // Scan each sensor channel per buffer half
constexpr uint8_t NUM_CHANNELS_IN_CH0       = 4;       // Chain/DMA 0 channel count
constexpr uint8_t NUM_CHANNELS_IN_CH1       = 4;       // Chain/DMA 1 channel count
constexpr uint32_t SAMPLE_RATE_HZ           = 1000;    // 1kHz (adjust as needed)
constexpr uint8_t WORDS_PER_SCAN            = 12;      // Number of words we read per scan

#define ADC_HW_TRIGGER_0    ( 1U << 0 )
#define ADC_HW_TRIGGER_1    ( 1U << 1 )

constexpr uint32_t PIT_CLOCK_FREQ_HZ        = 50'000'000;
constexpr uint32_t PIT0_SAMPLE_PERIOD_HZ    = 1'000;
constexpr uint32_t PIT1_SAMPLE_PERIOD_HZ    = 1'000;
constexpr uint32_t PHASE_OFFSET_US          = 50;

constexpr uint8_t BYTE_OFFSET_PER_TRANSFER  = 4;       // each ADC_ETC result register is 4 bytes
constexpr uint8_t BITS_READ_PER_TRANSACTION = 2;       // Not actually 2 bits, 0b0010 -> 32 bits

// ============================================================
//  Sensor → Analog Pin → ADC1 Channel mapping (10.1.1 Rev3)
//  Chain 0: first 4 sensors  (Trigger 0, slots 0-3)
//  Chain 1: last  4 sensors  (Trigger 1, slots 0-3)
//
//  Sensor                | A-Pin | Pad              | ADC1 ch
//  ----------------------|-------|------------------|--------
//  Linear pot 1          |  A9   | GPIO_AD_B1_09    |  14
//  Linear pot 2          |  A8   | GPIO_AD_B1_08    |  13
//  Linear pot 3          |  A7   | GPIO_AD_B1_11    |   0
//  Linear pot 4          |  A6   | GPIO_AD_B1_10    |  15
//  APPS1                 |  A5   | GPIO_AD_B1_00    |   5
//  APPS2                 |  A4   | GPIO_AD_B1_01    |   6
//  BSE1                  |  A3   | GPIO_AD_B1_06    |  11
//  BSE2                  |  A2   | GPIO_AD_B1_07    |  12
// ============================================================
static const uint8_t adc1_ch_c0[ NUM_CHANNELS_IN_CH0 ] = { 14, 13, 0, 15 };
static const uint8_t adc1_ch_c1[ NUM_CHANNELS_IN_CH1 ] = { 5, 6, 11, 12 };

constexpr uint8_t LINEAR_POT_1      = 0;
constexpr uint8_t LINEAR_POT_2      = 1;
constexpr uint8_t LINEAR_POT_3      = 2;
constexpr uint8_t LINEAR_POT_4      = 3;
constexpr uint8_t APPS_1            = 4;
constexpr uint8_t APPS_2            = 5;
constexpr uint8_t BSE_1             = 6;
constexpr uint8_t BSE_2             = 7;

DMAChannel dma;

// ============================================================
// One DMA minor-loop copies this whole struct. (67.5.1.12 Rev2)
// Starts at: ADC_ETC_TRIG0_RESULT_1_0
// Ends at:   ADC_ETC_TRIG1_RESULT_3_2
// ============================================================
struct AdcEtcRawFrame
{
    volatile uint32_t result[ WORDS_PER_SCAN ];
};

__attribute__( ( section(".noinit.$RAM2"), aligned( 32 ) ) )
volatile uint16_t adc_dma_buffer[ 2 ][ NUM_CHANNELS * NUM_SCANS_PER_CHANNEL ];

volatile uint8_t active_buffer = 0;
volatile uint8_t ready_buffer = 0xFF;
volatile bool adc_dma_buffer_ready = false;


uint16_t adc0Pins[SENSOR_PIN_AMT_ADC0] = {
    A0, A1, A2, A3, A4, A5, A6, A7, A8, A9 //, A16
}; // A4, A4, 18, 17, 17, 17, 17}; // real values: {21,
   // 24, 25, 19, 18, 14, 15, 17};
uint16_t adc0Reads[SENSOR_PIN_AMT_ADC0];

uint16_t adc1Pins[SENSOR_PIN_AMT_ADC1] = {
    // A17, A16, A15,
    A7, A6, A5, A4,
    A3, A2, A1, A0}; // A4, A4, 18, 17, 17, 17, 17}; // real values: {21,
                     // 24, 25, 19, 18, 14, 15, 17};
uint16_t adc1Reads[SENSOR_PIN_AMT_ADC1];


// Initialize clock gating
static void clocks_init()
{
    // PIT - periodic interrupt timer
    CCM_CCGR1 |= CCM_CCGR1_PIT( CCM_CCGR_ON );

    // XBAR - cross-bar switch (PIT pulse -> XBAR -> DMA trigger)
    CCM_CCGR2 |= CCM_CCGR2_XBAR1( CCM_CCGR_ON );

    // ADC0 (ADC0 in code is adc1 in datasheet)
    CCM_CCGR1 |= CCM_CCGR1_ADC1( CCM_CCGR_ON );

    // DMA / DMAMUX
    CCM_CCGR5 |= CCM_CCGR5_DMA( CCM_CCGR_ON );
}

static void ADC_Init()
{
    ADC * adc = new ADC();

    // ADC 0
    adc->adc0->setAveraging( 0 ); // TODO - turned off averaging for now, change back later?
    adc->adc0->setResolution( ADC_RESOLUTION );
    adc->adc0->setConversionSpeed( ADC_CONVERSION_SPEED::HIGH_SPEED );
    adc->adc0->setReference( ADC_REFERENCE::REF_3V3 );

    // Recalibrate to allow for enabling hard-trigger mode
    adc->adc0->recalibrate();

    // Enable hardware trigger mode (ADC_ETC sends trigger pulses to rotate channels)
    ADC1_CFG |= ADC_CFG_ADTRG;

    // Enable DMA request after conversion completed
    ADC1_GC |= ADC_GC_DMAEN;

    // Deleted ADC 1 bc we're not using it

#if DEBUG_FLAG
    Serial.println("Done initializing ADCs");
#endif
}

// Initialize XBAR
QuadEncoder * quadEncoder = new QuadEncoder();

static void XBAR_init()
{
    // PIT_TRIGGER0 -> XBAR trigger 0 -> ADC_ETC trigger input 0 (fires Chain 0)
    quadEncoder->xbar_connect( XBARA1_IN_PIT_TRIGGER0, XBARA1_OUT_ADC_ETC_TRIG00 );

    // PIT_TIGGER1 -> XBAR trigger 1 -> ADC_ETC trigger input 1 (fires chain 1)
    // Use 0 and 10 to keep them on separate ADC1 triggr registers.
    quadEncoder->xbar_connect( XBARA1_IN_PIT_TRIGGER1, XBARA1_OUT_ADC_ETC_TRIG01 );
}

// Initialize ADC ETC to rotate channels
static void ADC_ETC_init()
{
    // 67.6.1.2 Rev3
    // Software reset pulse
    ADC_ETC_CTRL = ADC_ETC_CTRL_SOFTRST;
    ADC_ETC_CTRL = 0;

    // Enable trigger input, allows for XBAR to signal ADC_ETC
    // bit 0 = TRIG00, fed by PIT0 via XBAR
    // bit 1 = TRIG01, fed by PIT1 via XBAR
    ADC_ETC_CTRL = ADC_ETC_CTRL_TRIG_ENABLE( ( 1U << 0 ) | ( 1U << 1) ) |
                   ADC_ETC_CTRL_PRE_DIVIDER( 0 ); // Controls how much the ADC_ETC clock is divided (default 0)

    // Enable DMA request generation ONLY WHEN TRIG1 FINISHES since it's the last chain
    ADC_ETC_DMA_CTRL = ( 1U << 1 );

    ADC_ETC_TRIG0_CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN( NUM_CHANNELS_IN_CH0 - 1 ) | // 4 conversions
                         ADC_ETC_TRIG_CTRL_TRIG_PRIORITY( 7 );
    ADC_ETC_TRIG1_CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN( NUM_CHANNELS_IN_CH1 - 1 ) | // 4 conversions
                         ADC_ETC_TRIG_CTRL_TRIG_PRIORITY( 7 );

    // -----------------------------
    // TRIG0 handles sensors 0-3 (update as needed) 67.5.1.8 rev2
    // CSEL - which ADC input channel to select
    // B2B - whether to immediatly start the next channel converison
    // IE - whether to generate an interrupt when this segment finishes
    // HWT - which ADC hardware trigger input to use
    // -----------------------------
    ADC_ETC_TRIG0_CHAIN_1_0 = ADC_ETC_TRIG_CHAIN_CSEL0( adc1_ch_c0[ 0 ] ) |
                              ADC_ETC_TRIG_CHAIN_B2B0 |
                              ADC_ETC_TRIG_CHAIN_IE0( 0 ) |
                              ADC_ETC_TRIG_CHAIN_HWTS0( ADC_HW_TRIGGER_0 ) | 

                              ADC_ETC_TRIG_CHAIN_CSEL1( adc1_ch_c0[ 1 ] ) |
                              ADC_ETC_TRIG_CHAIN_B2B1 |
                              ADC_ETC_TRIG_CHAIN_IE1( 0 ) |
                              ADC_ETC_TRIG_CHAIN_HWTS1( ADC_HW_TRIGGER_0 );

    ADC_ETC_TRIG0_CHAIN_3_2 = ADC_ETC_TRIG_CHAIN_CSEL0( adc1_ch_c0[ 2 ] ) |
                              ADC_ETC_TRIG_CHAIN_B2B0 |
                              ADC_ETC_TRIG_CHAIN_IE0( 0 ) |
                              ADC_ETC_TRIG_CHAIN_HWTS0( ADC_HW_TRIGGER_0 ) |

                              ADC_ETC_TRIG_CHAIN_CSEL1( adc1_ch_c0[ 3 ] ) |
                              ADC_ETC_TRIG_CHAIN_B2B1 |
                              ADC_ETC_TRIG_CHAIN_IE1( 0 ) |
                              ADC_ETC_TRIG_CHAIN_HWTS1( ADC_HW_TRIGGER_0 );

    ADC_ETC_TRIG0_CHAIN_5_4 = 0;
    ADC_ETC_TRIG0_CHAIN_7_6 = 0;

    // -----------------------------
    // TRIG1 handles sensors 4-7 (update as needed)
    // -----------------------------
    ADC_ETC_TRIG1_CHAIN_1_0 = ADC_ETC_TRIG_CHAIN_CSEL0( adc1_ch_c1[ 0 ] ) |
                              ADC_ETC_TRIG_CHAIN_B2B0 |
                              ADC_ETC_TRIG_CHAIN_IE0( 0 ) |
                              ADC_ETC_TRIG_CHAIN_HWTS0( ADC_HW_TRIGGER_1 ) | 

                              ADC_ETC_TRIG_CHAIN_CSEL1( adc1_ch_c1[ 1 ] ) |
                              ADC_ETC_TRIG_CHAIN_B2B1 |
                              ADC_ETC_TRIG_CHAIN_IE1( 0 ) |
                              ADC_ETC_TRIG_CHAIN_HWTS1( ADC_HW_TRIGGER_1 );

    ADC_ETC_TRIG1_CHAIN_3_2 = ADC_ETC_TRIG_CHAIN_CSEL0( adc1_ch_c1[ 2 ] ) |
                              ADC_ETC_TRIG_CHAIN_B2B0 |
                              ADC_ETC_TRIG_CHAIN_IE0( 0 ) |
                              ADC_ETC_TRIG_CHAIN_HWTS0( ADC_HW_TRIGGER_1 ) |

                              ADC_ETC_TRIG_CHAIN_CSEL1( adc1_ch_c1[ 3 ] ) |
                              ADC_ETC_TRIG_CHAIN_B2B1 |
                              ADC_ETC_TRIG_CHAIN_IE1( 0 ) |
                              ADC_ETC_TRIG_CHAIN_HWTS1( ADC_HW_TRIGGER_1 );

    ADC_ETC_TRIG1_CHAIN_5_4 = 0;
    ADC_ETC_TRIG1_CHAIN_7_6 = 0;
}

// Initialize PIT timers
static void PIT_init()
{
    // Disable both PIT channels while configuring
    IMXRT_PIT_CHANNELS[ 0 ].TCTRL = 0;
    IMXRT_PIT_CHANNELS[ 1 ].TCTRL = 0;

    // Enable PIT
    PIT_MCR = PIT_MCR_FRZ;

    // Clear any previous flags
    IMXRT_PIT_CHANNELS[ 0 ].TFLG = 1;
    IMXRT_PIT_CHANNELS[ 1 ].TFLG = 1;

    // Set up sample rates
    IMXRT_PIT_CHANNELS[ 0 ].LDVAL = ( PIT_CLOCK_FREQ_HZ / PIT0_SAMPLE_PERIOD_HZ) - 1;
    IMXRT_PIT_CHANNELS[ 1 ].LDVAL = ( PIT_CLOCK_FREQ_HZ / PIT1_SAMPLE_PERIOD_HZ) - 1;

    // Start PIT0
    IMXRT_PIT_CHANNELS[ 0 ].TCTRL = PIT_TCTRL_TEN;

    // Delay PIT1 start by a bit
    delayMicroseconds( PHASE_OFFSET_US );
    IMXRT_PIT_CHANNELS[ 1 ].TCTRL = PIT_TCTRL_TEN;
}

static void DMA_init()
{
    active_buffer = 0;
    ready_buffer = 0xFF;
    adc_dma_buffer_ready = false;

    // Delete any old CPU cache
    arm_dcache_delete( ( void * ) adc_dma_buffer[ 0 ], sizeof( adc_dma_buffer[ 0 ]) );
    arm_dcache_delete( ( void * ) adc_dma_buffer[ 1 ], sizeof( adc_dma_buffer[ 1 ]) );

    dma.disable();
    dma.clearComplete();
    dma.clearInterrupt();

    // ------------------------------------------------------------
    // Source: ADC_ETC result register window
    // ------------------------------------------------------------

    dma.TCD->SADDR = ( volatile const void * ) &ADC_ETC_TRIG0_RESULT_1_0; // Where to start (source)
    dma.TCD->ATTR_SRC = BITS_READ_PER_TRANSACTION; // How many bits the DMA reads in one transaction
    dma.TCD->SOFF = BYTE_OFFSET_PER_TRANSFER; // How many bytes to advance the SADDR after read
    
    dma.TCD->NBYTES = sizeof( AdcEtcRawFrame );

    dma.TCD->SLAST = -( int32_t )sizeof( AdcEtcRawFrame );

    // ------------------------------------------------------------
    // Destination: active ping-pong buffer
    // ------------------------------------------------------------

    dma.TCD->DADDR = ( volatile void * ) &adc_dma_buffer[ active_buffer ][ 0 ];
    dma.TCD->ATTR_DST = BITS_READ_PER_TRANSACTION;
    dma.TCD->DOFF = BYTE_OFFSET_PER_TRANSFER;

    dma.TCD->DLASTSGA = -( int32_t )sizeof( adc_dma_buffer[ active_buffer ] );

    // Current iteration counter, decrements every minor loop completion
    dma.TCD->CITER = NUM_SCANS_PER_CHANNEL;
    // THe value CITER gets reset to
    dma.TCD->BITER = NUM_SCANS_PER_CHANNEL;
    
    // Clear all status flags
    dma.TCD->CSR = 0;

    dma.interruptAtCompletion();
    dma.disableOnCompletion();
    dma.attachInterrupt( DMA_ISR );

    dma.triggerAtHardwareEvent( DMAMUX_SOURCE_ADC_ETC );

    dma.enable();
}

static void DMA_ISR()
{
    dma.clearInterrupt();

    ready_buffer = active_buffer;
    adc_dma_buffer_ready = true;

    // DMA wrote to RAM -> Delete CPU cache for this buffer so the CPU will read new data
    arm_dcache_delete( ( void * ) adc_dma_buffer[ ready_buffer ],
                         sizeof( adc_dma_buffer [ ready_buffer ] ) );

    // Swap to next destination buffer
    active_buffer ^= 1;
    arm_dcache_delete( ( void * ) adc_dma_buffer[ active_buffer ], 
                         sizeof( adc_dma_buffer[ active_buffer ] ) );
    dma.TCD->DADDR = ( volatile void * ) &adc_dma_buffer[ active_buffer ][ 0 ];

    // Reset everything
    dma.TCD->CITER = dma.TCD->BITER;
    dma.clearComplete();
    dma.enable();

    // Wake the ADC decoder task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR( adcDecoderTaskHandle, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    asm volatile( "dsb" );
}

void Full_ADC_DMA_Init()
{
    clocks_init();
    ADC_Init();
    XBAR_init();
    ADC_ETC_init();
    DMA_init();
    PIT_init();
}

void threadADC( void *pvParameters ) 
{
#if DEBUG_FLAG
    Serial.print("Beginning adc thread");
#endif

    while ( true )
    {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

        // Chain 0: linear pots
        uint16_t lp1 = adc_dma_buffer[ ready_buffer ][ LINEAR_POT_1 ];
        uint16_t lp2 = adc_dma_buffer[ ready_buffer ][ LINEAR_POT_2 ];
        uint16_t lp3 = adc_dma_buffer[ ready_buffer ][ LINEAR_POT_3 ];
        uint16_t lp4 = adc_dma_buffer[ ready_buffer ][ LINEAR_POT_4 ];

        // Chain 1: APPS + BSE
        uint16_t apps1 = adc_dma_buffer[ ready_buffer ][ APPS_1 ];
        uint16_t apps2 = adc_dma_buffer[ ready_buffer ][ APPS_2 ];
        uint16_t bse1  = adc_dma_buffer[ ready_buffer ][ BSE_1 ];
        uint16_t bse2  = adc_dma_buffer[ ready_buffer ][ BSE_2 ];

        ShockTravelUpdateData( lp1, lp2, lp3, lp4 );
        APPS_UpdateData( apps1, apps2 );
        BSE_UpdateData( bse1, bse2 );
    }
}
