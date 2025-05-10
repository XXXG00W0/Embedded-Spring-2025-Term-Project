// Project members
// Clarence Luo cl5672
// Hongyu Jin hj2806
// Kailin Zhang kz2739
// Yumeng Qian yq2480
// Ziyi Liang zl5604

#include "mbed.h"
#include "arm_math.h"
#include <chrono>

// BLE related headers
#include "ble/BLE.h"
#include "events/EventQueue.h"
#include "ble/gatt/GattService.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "rtos/Thread.h"

using namespace ble;
using namespace events;

//------------------------------------------------------------------------------
// Constants and Configuration
//------------------------------------------------------------------------------
// FFT Configuration
#define FFT_SIZE 256
#define SAMPLE_RATE 100                                // 100Hz
#define SAMPLER_TICK_PERIOD_US (1000000 / SAMPLE_RATE) // 10ms
#define RESOLUTION (SAMPLE_RATE / (float)FFT_SIZE)
#define MAGNITUDE_THRESHOLD 1.0f

// Accelerometer Configuration
#define LSM6DSL_ADDR (0x6A << 1)  // LSM6DSL I2C addr (7-bit addr, left shift)
#define OUTX_L_XL 0x28            // Accelerometer register address (LSB)
#define WHO_AM_I 0x0F             // Device ID register

// Buffer Configuration
#define FIFO_SIZE 512                      // Size of the FIFO buffer
#define HISTORY_STATE_LENGTH 5             // Length of state history buffer
#define HISTORY_MAG_LENGTH 5               // Length of magnitude history buffer

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
// I/O and Communication
BufferedSerial serial_port(USBTX, USBRX, 115200);
DigitalOut led_tremor(LED1);              // LED for tremor indication
DigitalOut led_dyskinesia(LED2);          // LED for dyskinesia indication
I2C i2c(PB_11, PB_10);                    // I2C pins: SDA=PB_11, SCL=PB_10 (B-L475E-IOT01A)

// Movement State Tracking
enum State {
    NONE,
    TREMOR,
    DYSKINESIA
};

State last_state = NONE;
State this_state = NONE;
uint32_t blink_period_ms;                 // Blink period in milliseconds

// State in string format
const char *state_strings[] = {
    "NONE",
    "TREMOR",
    "DYSKINESIA"
};

// Data Buffers
float32_t accel_fifo_buffer[3][FIFO_SIZE]; // Circular buffer for accelerometer data
int write_index = 0;                       // Write index for the circular buffer
float32_t accel_data[3][FFT_SIZE];         // 3 axes data for FFT processing
float32_t fft_input[FFT_SIZE];             // Input buffer for FFT
float32_t fft_output[FFT_SIZE];            // Output buffer for FFT
float32_t magnitude[FFT_SIZE / 2];         // Magnitude buffer

// State History
State state_history[HISTORY_STATE_LENGTH] = {NONE}; // FIFO buffer for state history
int history_state_index = 0;

// Magnitude History
float32_t max_magnitude_history[HISTORY_MAG_LENGTH] = {0.0f}; // FIFO buffer for magnitude history
int history_mag_index = 0;

// Sample Control
bool sample_ready = false;

// Timers and Tickers
Ticker led_ticker;                         // Ticker for LED blinking
Ticker notification_ticker;                // Ticker for BLE notifications

// FFT Instance
arm_rfft_fast_instance_f32 FFT_Instance;

//------------------------------------------------------------------------------
// BLE Configuration
//------------------------------------------------------------------------------
#define MAX_MVMT_BUFFER_SIZE 19 // Maximum size of the movement buffer
BLE &ble_interface = BLE::Instance();
EventQueue event_queue;
bool ble_device_connected = false;

const UUID MOVEMENT_SERVICE_UUID("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
const UUID MOVEMENT_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");

uint8_t movement_buffer[MAX_MVMT_BUFFER_SIZE] = {0};
uint8_t movement_length = 0;

ReadOnlyArrayGattCharacteristic<uint8_t, MAX_MVMT_BUFFER_SIZE> movement_char(
    MOVEMENT_CHAR_UUID,
    movement_buffer,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

GattCharacteristic *charTable[] = { &movement_char };
GattService movement_service(MOVEMENT_SERVICE_UUID, charTable, 1);

//------------------------------------------------------------------------------
// Console Functions
//------------------------------------------------------------------------------
FileHandle *mbed::mbed_override_console(int) {
    return &serial_port;
}

//------------------------------------------------------------------------------
// State Management Functions
//------------------------------------------------------------------------------
/**
 * Update the state history with a new state
 */
void update_state_history(State new_state) {
    state_history[history_state_index] = new_state; // Update the history with the new state
    history_state_index = (history_state_index + 1) % HISTORY_STATE_LENGTH; // Circular buffer index
}

/**
 * Get the most frequent state from the history
 */
State get_most_frequent_state() {
    int count[3] = {0}; // Count occurrences of each state
    for (int i = 0; i < HISTORY_STATE_LENGTH; i++) {
        count[state_history[i]]++;
    }
    // Find the most frequent state
    int max_count = 0;
    State most_frequent_state = NONE;
    for (int i = 0; i < 3; i++) {
        if (count[i] > max_count) {
            max_count = count[i];
            most_frequent_state = (State)i;
        }
    }
    return most_frequent_state;
}

/**
 * Update the magnitude history with a new magnitude
 */
void update_magnitude_history(float32_t new_magnitude) {
    max_magnitude_history[history_mag_index] = new_magnitude;
    history_mag_index = (history_mag_index + 1) % HISTORY_MAG_LENGTH;
}

/**
 * Get the average magnitude from the history
 */
float32_t get_avg_magnitude() {
    float32_t sum = 0.0f;
    for (int i = 0; i < HISTORY_MAG_LENGTH; i++) {
        sum += max_magnitude_history[i];
    }
    return sum / HISTORY_MAG_LENGTH;
}

//------------------------------------------------------------------------------
// LED Control Functions
//------------------------------------------------------------------------------
/**
 * Toggle the appropriate LED based on the current state
 */
void toggle_led() {
    if (this_state == TREMOR) {
        led_tremor = !led_tremor; // Toggle tremor LED
    } 
    else if (this_state == DYSKINESIA) {
        led_dyskinesia = !led_dyskinesia; // Toggle dyskinesia LED
    }
}

/**
 * Update LED indicators based on the current state and magnitude
 */
void update_indicator(float32_t max_magnitude, float32_t peak_freq) {
    // Normalize magnitude for intensity display with LED blink
    float intensity = fminf(max_magnitude / 100.0f, 1.0f);   // Normalize to [0, 1]
    blink_period_ms = (uint32_t)(1000 * (1.0f - intensity)); // Blink period in milliseconds
    if (blink_period_ms < 100) {
        blink_period_ms = 100; // Minimum blink period
    }
    printf("Blink period: %lu ms\n", blink_period_ms);

    // LED blink based on magnitude
    if (this_state != last_state) {
        // Turn off old ticker
        led_ticker.detach();
        led_tremor = 0;
        led_dyskinesia = 0;

        if (this_state != NONE) {
            // Convert milliseconds to microseconds
            auto blink_period_us = std::chrono::duration<uint32_t, std::milli>(blink_period_ms);
            led_ticker.attach(&toggle_led, blink_period_us);
        } 
        else {
            led_tremor = 0;     // Turn off tremor LED
            led_dyskinesia = 0; // Turn off dyskinesia LED
        }
        last_state = this_state;
    }
}

//------------------------------------------------------------------------------
// BLE Functions
//------------------------------------------------------------------------------
/**
 * Send current state via BLE
 */
void send_ble_state() {
    
    if (!ble_device_connected) {
        printf("No device connected, skipping notification\n");
        return;
    }
    const char *state_str = state_strings[this_state];
    float32_t ble_max_magnitude = get_avg_magnitude(); // Get the average magnitude from history

    snprintf((char *)movement_buffer, sizeof(movement_buffer), "State: %s", state_str);
    movement_length = strlen((char *)movement_buffer) + 1; // +1 for null terminator

    ble_interface.gattServer().write(
        movement_char.getValueHandle(),
        movement_buffer,
        movement_length
    );

    printf("BLE Notification: %s\n", movement_buffer);

    
    snprintf((char *)movement_buffer, sizeof(movement_buffer), "Magnitude: %f", ble_max_magnitude);
    movement_length = strlen((char *)movement_buffer) + 1; // +1 for null terminator

    ble_interface.gattServer().write(
        movement_char.getValueHandle(),
        movement_buffer,
        movement_length
    );

    printf("BLE Notification: %s\n", movement_buffer);

}

/**
 * Handler for BLE events
 */
class MovementEventHandler : public ble::Gap::EventHandler {
    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) override {
        if (event.getStatus() == BLE_ERROR_NONE) {
            printf("Device connected!\n");
            ble_device_connected = true;
            
            // Start sending notifications periodically
            notification_ticker.attach([]() {
                event_queue.call(send_ble_state);
            }, 1000ms);
        }
    }

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) override {
        printf("Device disconnected!\n");
        ble_device_connected = false;
        
        // Stop the notification ticker
        notification_ticker.detach();
        
        ble_interface.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        printf("Restarted advertising\n");
    }
};

MovementEventHandler event_handler;

/**
 * BLE initialization callback
 */
void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        printf("BLE initialization failed: %d\n", params->error);
        return;
    }
    
    printf("BLE initialization successful\n");
    ble_interface.gattServer().addService(movement_service);

    uint8_t adv_buffer[LEGACY_ADVERTISING_MAX_SIZE];
    AdvertisingDataBuilder adv_data(adv_buffer);
    adv_data.setFlags();
    adv_data.setName("MovementBLE");

    ble_interface.gap().setAdvertisingParameters(
        LEGACY_ADVERTISING_HANDLE,
        AdvertisingParameters(ble::advertising_type_t::CONNECTABLE_UNDIRECTED, adv_interval_t(160))
    );

    ble_interface.gap().setAdvertisingPayload(
        LEGACY_ADVERTISING_HANDLE,
        adv_data.getAdvertisingData()
    );

    ble_interface.gap().setEventHandler(&event_handler);
    ble_interface.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);
    printf("BLE advertising started!\n");
}

/**
 * Schedule BLE events to be processed
 */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(callback(&ble_interface, &BLE::processEvents));
}

//------------------------------------------------------------------------------
// Accelerometer Functions
//------------------------------------------------------------------------------
/**
 * Initialize the LSM6DSL accelerometer
 */
void init_lsm6dsl() {
    // CTRL1_XL register: Set to 104Hz ODR, +/-2g range, 50Hz bandwidth
    char ctrl1_xl[2] = {0x10, 0x60};
    i2c.write(LSM6DSL_ADDR, ctrl1_xl, 2);
}

/**
 * Read raw accelerometer data
 */
void read_accel(int16_t &ax, int16_t &ay, int16_t &az) {
    char reg = OUTX_L_XL;
    char data[6] = {0};

    // Write register address
    i2c.write(LSM6DSL_ADDR, &reg, 1, true); // Repeated start
    i2c.read(LSM6DSL_ADDR, data, 6);

    // Combine high and low bytes (note: LSB first, little endian)
    ax = (int16_t)(data[1] << 8 | data[0]);
    ay = (int16_t)(data[3] << 8 | data[2]);
    az = (int16_t)(data[5] << 8 | data[4]);
}

/**
 * ISR for setting sample flag
 */
void sample_ready_isr() {
    sample_ready = true;
}

/**
 * Collect data from accelerometer
 */
void collect_accel_data() {
    int16_t ax, ay, az;
    read_accel(ax, ay, az);
    write_index = (write_index + 1) % FIFO_SIZE; // Circular buffer write index
    accel_fifo_buffer[0][write_index] = (float32_t)ax;
    accel_fifo_buffer[1][write_index] = (float32_t)ay;
    accel_fifo_buffer[2][write_index] = (float32_t)az;
}

/**
 * Extract the last window of data for FFT processing
 */
void extract_last_window() {
    int start = (write_index - FFT_SIZE + FIFO_SIZE) % FIFO_SIZE; // Start index for the last window
    for (int i = 0; i < FFT_SIZE; i++) {
        int index = (start + i) % FIFO_SIZE;
        accel_data[0][i] = accel_fifo_buffer[0][index];
        accel_data[1][i] = accel_fifo_buffer[1][index];
        accel_data[2][i] = accel_fifo_buffer[2][index];
    }
}

//------------------------------------------------------------------------------
// FFT and Analysis Functions
//------------------------------------------------------------------------------
/**
 * Process FFT on the accelerometer data
 */
void process_FFT() {
    // Clear the FFT input buffer first
    memset(fft_input, 0, sizeof(fft_input));

    // Define a window function (Hanning window) for the actual sample size
    float32_t window[FFT_SIZE];
    arm_hanning_f32(window, FFT_SIZE);

    float32_t x_mean = 0.0f, y_mean = 0.0f, z_mean = 0.0f;
    // Calculate mean values for each axis
    arm_mean_f32(accel_data[0], FFT_SIZE, &x_mean);
    arm_mean_f32(accel_data[1], FFT_SIZE, &y_mean);
    arm_mean_f32(accel_data[2], FFT_SIZE, &z_mean);

    // Scale factor for normalization
    // For +/-2g range, full scale is typically 32768 (for 16-bit values)
    const float32_t scale_factor = 16384.0f; // = 2^14 LSB/g for +/-2g range

    // Apply the window function, remove DC bias, and normalize
    for (int i = 0; i < FFT_SIZE; i++) {
        // Normalize and remove DC component
        float32_t x_norm = (accel_data[0][i] - x_mean) / scale_factor;
        float32_t y_norm = (accel_data[1][i] - y_mean) / scale_factor;
        float32_t z_norm = (accel_data[2][i] - z_mean) / scale_factor;

        // Calculate magnitude
        float32_t mag = sqrtf(x_norm * x_norm + y_norm * y_norm + z_norm * z_norm);

        // Apply window function and store in FFT input buffer
        fft_input[i] = mag * window[i];
    }

    // Perform FFT on real input data
    arm_rfft_fast_f32(&FFT_Instance, fft_input, fft_output, 0);

    // Calculate magnitude spectrum (note: output is complex)
    arm_cmplx_mag_f32(fft_output, magnitude, FFT_SIZE / 2);
}

/**
 * Analyze frequency components in the FFT result
 */
void analyze_frequency(float32_t *max_magnitude, float32_t *peak_freq) {
    uint32_t max_index = 0;

    // Only analyze frequencies in the range of interest
    const float32_t min_freq = 1.0f;  // 1Hz
    const float32_t max_freq = 10.0f; // 10Hz
    uint32_t start_index = ceilf(min_freq / RESOLUTION);
    uint32_t end_index = ceilf(max_freq / RESOLUTION);
    if (end_index > FFT_SIZE / 2) {
        end_index = FFT_SIZE / 2;
    }

    // Find maximal magnitude in the specified range
    arm_max_f32(&magnitude[start_index], end_index - start_index, max_magnitude, &max_index);
    max_index += start_index; // Adjust index to the full range

    *peak_freq = max_index * RESOLUTION; // Convert index to frequency

    // Print magnitude analysis results
    printf("Analysis range: %.1f-%.1f Hz\n", min_freq, max_freq);
    printf("Max Magnitude: %.2f at Bin %lu (%.2f Hz)\n", *max_magnitude, max_index, *peak_freq);
}

/**
 * Classify the movement state based on frequency and magnitude
 */
State classify_state(float32_t max_magnitude, float32_t peak_freq) {
    // Determine state based on frequency
    State instant_state = NONE;
    if (peak_freq >= 3.0f && peak_freq <= 5.0f && max_magnitude > MAGNITUDE_THRESHOLD) {
        instant_state = TREMOR;
    }
    else if (peak_freq > 5.0f && peak_freq <= 7.0f && max_magnitude > MAGNITUDE_THRESHOLD) {
        instant_state = DYSKINESIA;
    }
    printf("Instant state: %d\n", instant_state);
    return instant_state;
}

//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------
int main() {
    printf("LSM6DSL Movement Disorder Analysis System\r\n");
    printf("Sampling at %dHz with FFT size %d\r\n", SAMPLE_RATE, FFT_SIZE);

    // Initialize FFT instance
    if (arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE) != ARM_MATH_SUCCESS) {
        printf("ERROR: FFT initialization failed\r\n");
    } else {
        printf("FFT initialized successfully\r\n");
    }

    // Check WHO_AM_I register to confirm sensor is working
    char whoami_reg = WHO_AM_I;
    char whoami;
    i2c.write(LSM6DSL_ADDR, &whoami_reg, 1, true);
    i2c.read(LSM6DSL_ADDR, &whoami, 1);

    if (whoami != 0x6A) {
        printf("LSM6DSL not detected! WHO_AM_I = 0x%X\r\n", whoami);
        return 1;
    }

    printf("LSM6DSL detected! WHO_AM_I = 0x%X\r\n", whoami);
    init_lsm6dsl();

    // BLE initialization
    ble_interface.onEventsToProcess(schedule_ble_events);
    ble_interface.init(on_ble_init_complete);
    Thread ble_thread;
    ble_thread.start(callback(&event_queue, &EventQueue::dispatch_forever));
    printf("BLE started\r\n");

    // Initialize LEDs
    led_tremor = 0;
    led_dyskinesia = 0;

    // Initialize the circular buffer
    for (int i = 0; i < FIFO_SIZE; i++) {
        accel_fifo_buffer[0][i] = 0.0f;
        accel_fifo_buffer[1][i] = 0.0f;
        accel_fifo_buffer[2][i] = 0.0f;
    }
    
    // Initialize sampler ticker
    Ticker sampler_ticker;
    auto sample_interval_us = std::chrono::microseconds(SAMPLER_TICK_PERIOD_US);
    sampler_ticker.attach(&sample_ready_isr, sample_interval_us);

    // Initialize analyzer timer
    Timer analyzer_timer;
    analyzer_timer.start();

    // Main loop
    while (true) {
        // Sample accelerometer data when ready
        if (sample_ready) {
            sample_ready = false;
            collect_accel_data();
        }

        // Analyze data every second
        if (analyzer_timer.elapsed_time() >= 1000ms) {
            analyzer_timer.reset();

            // FFT Analysis Results
            float32_t max_magnitude = 0.0f, peak_freq = 0.0f;
            
            // Process data
            extract_last_window();
            process_FFT();
            analyze_frequency(&max_magnitude, &peak_freq);
            
            // Classify and update state
            State instant_state = classify_state(max_magnitude, peak_freq);
            update_state_history(instant_state);
            this_state = get_most_frequent_state();

            update_magnitude_history(max_magnitude);
            printf("Avg Magnitude: %.2f\r\n", get_avg_magnitude());

            printf("Current state: %d\r\n", this_state);

            // Update indicators
            update_indicator(max_magnitude, peak_freq);

            // Debug output
            // printf("State History: ");
            // for (int i = 0; i < HISTORY_STATE_LENGTH; i++) {
            //     printf("%d ", state_history[i]);
            // }
            // printf("\r\n");
        }

        // Handle BLE events
        // event_queue.dispatch_for(1000ms);
    }
}