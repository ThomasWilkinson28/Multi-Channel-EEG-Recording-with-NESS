#include "mbed.h"
#include "BLEDevice.h"
#include "UARTService.h"
#include "nrf_temp.h"
// ★★★ New addition: Included the us_ticker_api.h hardware abstraction layer ★★★ 
//This gives us access to us_ticker_read(), a function that directly reads the nRF51822's 
//internal quartz-driven microsecond counter.
#include "us_ticker_api.h" // Required for hardware microsecond timer
// ★★★ New addition End ★★★
#include <cstdint>
#include <string.h>

#define MAX_DATA_LEN           (UARTService::BLE_UART_SERVICE_MAX_DATA_LEN)
#define APP_ADV_DURATION       0
#define MIN_CONN_INTERVAL      MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL      MSEC_TO_UNITS(75, UNIT_1_25_MS)
#define SLAVE_LATENCY          0
#define CONN_SUP_TIMEOUT       MSEC_TO_UNITS(4000, UNIT_10_MS)
#define UART_BAUD_RATE         (19200UL)
#define DEVICE_NAME            ("DEMO NODE 3")  // Node Name
#define SHORT_NAME             ("DEMONOD3")     // Node Name Short
#define ANALOG_IN_PIN          P0_1

// ★★★ New addition: MODIFIED PACKET FORMATTING ★★★
// ====================== NEW PACKET FORMATTING ======================
// Max BLE payload is 20 bytes.
// Header ('d') = 1 byte
// Timestamp (uint32) = 4 bytes
// Total Overhead = 5 bytes. 
// 15 bytes remaining -> Can fit 7 samples (14 bytes).
// Dropped the union and switched to a raw uint8_t byte array. 
// Dedicated first 5 bytes to overhead: 1 byte for a packet identifier (the letter 'd' for data) 
// and 4 bytes for a 32-bit microsecond timestamp.
#define OVERHEAD_LEN           5
#define SAMPLES_PER_BLOCK      7
#define PAYLOAD_LEN            (OVERHEAD_LEN + (SAMPLES_PER_BLOCK * 2)) // 19 bytes

#define INTERVAL_MS            5
#define OS_FACTOR              4 // 4x oversampling: 800 Hz ADC, decimate to 200 Hz

BLEDevice   m_ble;
UARTService *m_uart_service_ptr;

static uint16_t flag = 0;

// Byte arrays instead of structs to prevent memory padding issues
static uint8_t bufferA[PAYLOAD_LEN];
static uint8_t bufferB[PAYLOAD_LEN];
static uint8_t* writeBuffer = bufferA;
static uint8_t* sendBuffer  = bufferB;
static uint16_t writeIndex = 0;
static bool bufferReady = false;
// ★★★ New addition End ★★★

// ====================== Sixth order iir low pass  ======================

typedef struct {
    double b0, b1, b2;
    double a1, a2;  // a0=1 
    double x1, x2;
    double y1, y2;
} biquad_t;

// store parameters
static biquad_t biquads[3] = {
    // SOS #1 - Gain 0.170422728203046
    { 1.0 * 0.170422728203046,  2.0 * 0.170422728203046,  1.0 * 0.170422728203046, 
     -0.972036705142560,        0.653727617954745,        0.0, 0.0, 0.0, 0.0 },

    // SOS #2 - Gain 0.131106439916626
    { 1.0 * 0.131106439916626,  2.0 * 0.131106439916626,  1.0 * 0.131106439916626,
     -0.747789178258503,        0.272214937925007,        0.0, 0.0, 0.0, 0.0 },

    // SOS #3 - Gain 0.115696385843074
    { 1.0 * 0.115696385843074,  2.0 * 0.115696385843074,  1.0 * 0.115696385843074,
     -0.659895161153711,        0.122680704526008,        0.0, 0.0, 0.0, 0.0 }
};

static inline double biquad_process(biquad_t* __restrict s, double x)
{
    double x1 = s->x1;
    double x2 = s->x2;
    double y1 = s->y1;
    double y2 = s->y2;
    double y = s->b0 * x + s->b1 * x1 + s->b2 * x2 - s->a1 * y1 - s->a2 * y2;
    s->x2 = x1;
    s->x1 = x;
    s->y2 = y1;
    s->y1 = y;
    return y;
}

static uint8_t os_cnt = 0;
static const double SCALE_FACTOR = 65535.0 / 1023.0;

// ====================== BLE Send ======================
void send_ble_block(uint8_t* block)
{
    m_ble.updateCharacteristicValue(
        m_uart_service_ptr->getRXCharacteristicHandle(),
        // ★★★ New addition ★★★
        //(uint8_t*)(block->buff),
        //SAMPLE_COUNT * 2
        block,
        PAYLOAD_LEN
        // ★★★ New addition end ★★★
    );
}

// ====================== GAP Callback ======================
void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
    m_ble.startAdvertising();
    flag = 0;
}

// ====================== Hardware Timers & ADC Setup ======================
void timer2_init(void)
{
    NRF_TIMER2->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER2->PRESCALER = 4;          // 1 MHz
    NRF_TIMER2->CC[0]     = 1250;       // 800 Hz
    NRF_TIMER2->SHORTS    = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_TIMER2->TASKS_START = 1;
}

void ppi_init(void)
{
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_ADC->TASKS_START;
    NRF_PPI->CHEN      = (1 << 0);
}

void adc_init(void)
{
    NRF_ADC->CONFIG =
        (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
        (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
        (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
        (ADC_CONFIG_PSEL_AnalogInput2 << ADC_CONFIG_PSEL_Pos) |
        (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);

    NRF_ADC->ENABLE   = ADC_ENABLE_ENABLE_Enabled;
    NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;
    NVIC_SetPriority(ADC_IRQn, 3);
    NVIC_EnableIRQ(ADC_IRQn);
}

// ====================== ADC IRQ ======================
extern "C" void ADC_IRQHandler(void)
{
    if (!NRF_ADC->EVENTS_END) return;
    NRF_ADC->EVENTS_END = 0;

    uint16_t value = NRF_ADC->RESULT;

    if (flag == 1)
    {
        double x = (double)value;

        // IIR filter
        double y = biquad_process(&biquads[0], x);
        y = biquad_process(&biquads[1], y);
        y = biquad_process(&biquads[2], y);

        os_cnt++;
        if (os_cnt >= OS_FACTOR)
        {
            os_cnt = 0;
            
            // ★★★ New addition Start: RECORD TIMESTAMP FOR THE FIRST SAMPLE OF THE BLOCK ★★★
            // When the ADC interrupt fires for the very first sample of a new block (writeIndex == 0), 
            // it instantly freezes the hardware clock (us_ticker_read()).
            // Use memcpy to carefully place the 16-bit EEG samples into the buffer after the 5 bytes of overhead.
            if (writeIndex == 0) {
                uint32_t current_us = us_ticker_read();
                memcpy(&writeBuffer[1], &current_us, sizeof(uint32_t));
            }
            // ★★★ New addition End ★★★

            // Apply scale factor and saturation protect
            double y_norm = y * SCALE_FACTOR;
            if (y_norm < 0.0) y_norm = 0.0;
            if (y_norm > 65535.0) y_norm = 65535.0;

            // Save sample to buffer
            uint16_t out = (uint16_t)(y_norm + 0.5);
            
            // ★★★ New addition: RECORD TIMESTAMP FOR THE FIRST SAMPLE OF THE BLOCK ★★★
            memcpy(&writeBuffer[OVERHEAD_LEN + (writeIndex * 2)], &out, sizeof(uint16_t));
            writeIndex++;
            // ★★★ New addition End ★★★
            
            // If block is full, swap and flag for sending
            if (writeIndex >= SAMPLES_PER_BLOCK)
            {
                writeIndex = 0;
                //sample_block_t* tmp = sendBuffer;
                uint8_t* tmp = sendBuffer;
                sendBuffer = writeBuffer;
                writeBuffer = tmp;
                bufferReady = true;
            }
        }
    }
    else
    {
        os_cnt = 0;
        writeIndex = 0;
        NRF_ADC->TASKS_STOP = 1;
    }
}

// ★★★ New Addition: Battery Reading When Device is not Recording ★★★
// ====================== Battery Reading (VDD) ======================
uint8_t read_battery_level() {
    // Safety Lockout: Never touch the ADC if we are currently recording EEG
    if (flag == 1) return 255; // 255 is an error code meaning "Busy Recording"

    // 1. Temporarily disable the ADC interrupt so it doesn't trigger our EEG code
    NVIC_DisableIRQ(ADC_IRQn);

    // 2. Save the old EEG ADC configuration so we can restore it later
    uint32_t old_config = NRF_ADC->CONFIG;

    // 3. Reconfigure ADC to read internal VDD (Battery)
    // We use 1/3 prescaling, against the 1.2V internal bandgap reference.
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);

    // 4. Start a manual ADC conversion
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->TASKS_START = 1;

    // 5. Block the CPU and wait for the conversion to finish
    while (!NRF_ADC->EVENTS_END) {}
    uint16_t result = NRF_ADC->RESULT;
    
    // 6. Restore the ADC back to EEG mode
    NRF_ADC->CONFIG = old_config;
    NRF_ADC->EVENTS_END = 0;
    NVIC_ClearPendingIRQ(ADC_IRQn);
    NVIC_EnableIRQ(ADC_IRQn);

    // 7. Calculate real voltage
    // VDD = (ADC_Result / 1023.0) * 1.2V (Ref) * 3 (Prescaler inverse)
    float voltage = (result / 1023.0) * 3.6;

    // 8. Map to roughly 0-100% for a standard LiPo (Max 4.2V, Empty ~3.3V)
    float percentage = ((voltage - 3.3) / (4.2 - 3.3)) * 100.0;
    
    // Clamp the values
    if (percentage > 100.0) percentage = 100;
    if (percentage < 0.0) percentage = 0;

    return (uint8_t)percentage;
}
// ★★★ New Addition End ★★★


// ====================== GATT RX Callback (Commands from Master) ======================
void dataWrittenCallback(const GattCharacteristicWriteCBParams *params)
{
    if ((m_uart_service_ptr != NULL) &&
        (params->charHandle == m_uart_service_ptr->getTXCharacteristicHandle()))
    {
        if (params->len > 0) {
            switch (params->data[0]) {
                // ★★★ New Addition: READING BATTERY VOLTAGE ★★★
                case 'b': { 
                    // Call our new hardware function
                    uint8_t batt_level = read_battery_level();
                    
                    uint8_t batt_packet[2];
                    batt_packet[0] = 'b';
                    batt_packet[1] = batt_level;
                    
                    // Fire the battery percentage back to the ESP32
                    m_ble.updateCharacteristicValue(
                        m_uart_service_ptr->getRXCharacteristicHandle(),
                        batt_packet,
                        2
                    );
                    break;
                }
                // ★★★ New Addition End ★★★
                
                // ★★★ New Addition: RECEIVING PING FROM HOST ★★★
                // This is the ping-pong protocol in action. The absolute microsecond the ESP32's 'p' (Ping) command arrives,
                // the nRF51822 processor stops what it is doing, grabs the exact hardware time, constructs a tiny 5-byte reply packet starting with 'p',
                // and sends it back to the ESP32. This allows the ESP32 to calculate the round-trip delay and time difference the node is at relative to it.
                case 'p': { 
                    // Instantly capture the hardware timer
                    uint32_t pong_time = us_ticker_read();
                    
                    // Construct the Pong packet: ['p', byte1, byte2, byte3, byte4]
                    uint8_t pong_packet[5];
                    pong_packet[0] = 'p';
                    memcpy(&pong_packet[1], &pong_time, sizeof(uint32_t));
                    
                    // Fire it back immediately
                    m_ble.updateCharacteristicValue(
                        m_uart_service_ptr->getRXCharacteristicHandle(),
                        pong_packet,
                        5
                    );
                    break;
                }

                case 'a':
                    flag = 1;
                    break;
                case 's':
                    flag = 0;
                    break;
                default:
                    flag = 0;
                    break;
            }
        }
    }
}

void dataSentCallback(unsigned count) {}
void error(ble_error_t err, uint32_t line) {}

// ====================== Main ======================
int main(void)
{
    // ★★★ New Addition: INITIALIZE PACKET HEADERS ★★★ 
    // Letter 'd' hardcoded into the very first byte of both data buffers when the chip boots up. 
    // Tells the ESP32, "This packet contains EEG data, not a pong response."
    bufferA[0] = 'd';
    bufferB[0] = 'd';
    // ★★★ New Addition End ★★★

    m_ble.init();
    m_ble.onDisconnection(disconnectionCallback);
    m_ble.onDataWritten(dataWrittenCallback);
    m_ble.onDataSent(dataSentCallback);

    m_ble.setTxPower(4);

    m_ble.setDeviceName(DEVICE_NAME);
    m_ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    m_ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    m_ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                        (const uint8_t *)SHORT_NAME,
                                        (sizeof(SHORT_NAME) - 1));
    m_ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                        (const uint8_t *)UARTServiceUUID_reversed,
                                        sizeof(UARTServiceUUID_reversed));

    UARTService uartService(m_ble);
    m_uart_service_ptr = &uartService;

    adc_init();
    timer2_init();
    ppi_init();

    m_ble.setAdvertisingInterval(200);
    m_ble.startAdvertising();
    
    while (true) {
        if (flag == 1 && bufferReady) {
            bufferReady = false;
            send_ble_block(sendBuffer);
        }
        m_ble.waitForEvent();
    }
}