#include "mbed.h"
#include "arm_math.h"
#include <chrono>

BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int)
{
    return &serial_port;
}

// LED pins
DigitalOut led_tremor(LED1);
DigitalOut led_dyskinesia(LED2);
// Initialize LED state to off in the main function

// I2C pins：SDA=PB_11, SCL=PB_10（B-L475E-IOT01A）
I2C i2c(PB_11, PB_10);

// LSM6DSL I2C addr（7-bit addr，left shift 0xD4）
const int LSM6DSL_ADDR = 0x6A << 1;

// accelerometer register addr（LSB/MSB）
#define OUTX_L_XL 0x28
#define WHO_AM_I 0x0F

// FFT instance
arm_rfft_fast_instance_f32 FFT_Instance;
// Define discorder states
enum State
{
    NONE,
    TREMOR,
    DYSKINESIA
};
State last_state = NONE;
State this_state = NONE;
uint32_t blink_period_ms; // Blink period in milliseconds
// History of the last 20 states
#define HISTORY_LENGTH 20
State state_history[HISTORY_LENGTH] = {NONE}; // fifo buffer
int history_index = 0;

void update_state_history(State new_state)
{
    state_history[history_index] = new_state; // Update the history with the new state
    history_index = (history_index + 1) % HISTORY_LENGTH; // Circular buffer index
}

State get_most_frequent_state()
{
    int count[3] = {0}; // Count occurrences of each state
    for (int i = 0; i < HISTORY_LENGTH; i++)
    {
        count[state_history[i]]++;
    }
    // Find the most frequent state
    int max_count = 0;
    State most_frequent_state = NONE;
    for (int i = 0; i < 3; i++)
    {
        if (count[i] > max_count)
        {
            max_count = count[i];
            most_frequent_state = (State)i;
        }
    }
    return most_frequent_state;
}

Ticker led_ticker; // Ticker for LED blinking
void toggle_led()
{
    // printf("Toggling LED\n");
    if (this_state == TREMOR)
    {
        led_tremor = !led_tremor; // Toggle tremor LED
    }
    else if (this_state == DYSKINESIA)
    {
        led_dyskinesia = !led_dyskinesia; // Toggle dyskinesia LED
    }
}

// Threshold for detection (calibration required)
// #define THRESHOLD 100.0f

#define FFT_SIZE 256
#define SAMPLE_RATE 100                                // 100Hz
#define SAMPLER_TICK_PERIOD_US (1000000 / SAMPLE_RATE) // 10ms
#define RESOLUTION (SAMPLE_RATE / (float)FFT_SIZE)
#define MAGNITUDE_THRESHOLD 1.0f

// Buffers
#define FIFO_SIZE 512                      // Size of the FIFO buffer
float32_t accel_fifo_buffer[3][FIFO_SIZE]; // circular buffer
int write_index = 0;                       // Write index for the circular buffer
float32_t accel_data[3][FFT_SIZE];         // 3 axes
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];

// Read accelerometer data
void read_accel(int16_t &ax, int16_t &ay, int16_t &az)
{
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

// ISR for setting sample flag
bool sample_ready = false;
void sample_ready_isr()
{
    sample_ready = true;
}

// Collect data from accelerometer
void collect_accel_data()
{
    int16_t ax, ay, az;
    read_accel(ax, ay, az);
    write_index = (write_index + 1) % FIFO_SIZE; // Circular buffer write index
    accel_fifo_buffer[0][write_index] = (float32_t)ax;
    accel_fifo_buffer[1][write_index] = (float32_t)ay;
    accel_fifo_buffer[2][write_index] = (float32_t)az;
}

void extract_last_window()
{
    int start = (write_index - FFT_SIZE + FIFO_SIZE) % FIFO_SIZE; // Start index for the last window
    for (int i = 0; i < FFT_SIZE; i++)
    {
        int index = (start + i) % FIFO_SIZE;
        accel_data[0][i] = accel_fifo_buffer[0][index];
        accel_data[1][i] = accel_fifo_buffer[1][index];
        accel_data[2][i] = accel_fifo_buffer[2][index];
    }
}

void process_FFT()
{
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
    for (int i = 0; i < FFT_SIZE; i++)
    {
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

void analyze_frequency(float32_t *max_magnitude, float32_t *peak_freq)
{
    uint32_t max_index = 0;

    // Only analyze frequencies in the range of interest
    const float32_t min_freq = 1.0f;  // 1Hz
    const float32_t max_freq = 10.0f; // 10Hz
    uint32_t start_index = ceilf(min_freq / RESOLUTION);
    uint32_t end_index = ceilf(max_freq / RESOLUTION);
    if (end_index > FFT_SIZE / 2)
    {
        end_index = FFT_SIZE / 2;
    }

    // Find maximal magnitude in the specified range
    arm_max_f32(&magnitude[start_index], end_index - start_index, max_magnitude, &max_index);
    max_index += start_index; // Adjust index to the full range

    *peak_freq = max_index * RESOLUTION; // Convert index to frequency

    // Print magnitude analysis results
    printf("Analysis range: %.1f-%.1f Hz\n", min_freq, max_freq);
    printf("Max Magnitude: %.2f at Bin %lu (%.2f Hz)\n", *max_magnitude, max_index, *peak_freq);

    // // Calculate energy in specific frequency bands
    // // bins in 3–5Hz
    // uint32_t tremor_start = (uint32_t)ceilf(3.0f / RESOLUTION);
    // uint32_t tremor_end = (uint32_t)floorf(5.0f / RESOLUTION);

    // // bins in 5–7Hz
    // uint32_t dysk_start = (uint32_t)ceilf(5.0f / RESOLUTION);
    // uint32_t dysk_end = (uint32_t)floorf(7.0f / RESOLUTION);

    // for (uint32_t i = tremor_start; i <= tremor_end && i < FFT_SIZE / 2; ++i)
    // {
    //     *tremor_energy += magnitude[i] * magnitude[i];
    // }
    // for (uint32_t i = dysk_start; i <= dysk_end && i < FFT_SIZE / 2; ++i)
    // {
    //     *dyskinesia_energy += magnitude[i] * magnitude[i];
    // }

    // printf("Tremor Energy: %.2f | Dyskinesia Energy: %.2f\n", *tremor_energy, *dyskinesia_energy);
    
}

State classify_state(float32_t max_magnitude, float32_t peak_freq){
    // Determine state based on frequency
    State instant_state = NONE;
    if (peak_freq >= 3.0f && peak_freq <= 5.0f && max_magnitude > MAGNITUDE_THRESHOLD)
    {
        instant_state = TREMOR;
    }
    else if (peak_freq > 5.0f && peak_freq <= 7.0f && max_magnitude > MAGNITUDE_THRESHOLD)
    {
        instant_state = DYSKINESIA;
    }
    printf("Instant state: %d\n", instant_state);
    return instant_state;
}


void update_indicator(float32_t max_magnitude, float32_t peak_freq)
{

    // normalize magnitude for intensity display with LED blink
    float intensity = fminf(max_magnitude / 100.0f, 1.0f);   // Normalize to [0, 1]
    blink_period_ms = (uint32_t)(1000 * (1.0f - intensity)); // Blink period in milliseconds
    if (blink_period_ms < 100)
    {
        blink_period_ms = 100; // Minimum blink period
    }
    printf("Blink period: %lu ms\n", blink_period_ms);

    // LED blink based on magnitude
    if (this_state != last_state)
    {
        // turn off old ticker
        led_ticker.detach();
        led_tremor = 0;
        led_dyskinesia = 0;

        if (this_state != NONE)
        {
            // convert miliseconds to microseconds
            auto blink_period_us = std::chrono::duration<uint32_t, std::milli>(blink_period_ms);
            led_ticker.attach(&toggle_led, blink_period_us);
        }
        else
        {
            led_tremor = 0;     // Turn off tremor LED
            led_dyskinesia = 0; // Turn off dyskinesia LED
        }
        last_state = this_state;
    }
}

// Initialize the LSM6DSL accelerometer
void init_lsm6dsl()
{
    // CTRL1_XL register: Set to 104Hz ODR, +/-2g range, 50Hz bandwidth
    char ctrl1_xl[2] = {0x10, 0x60};
    i2c.write(LSM6DSL_ADDR, ctrl1_xl, 2);
}

int main()
{
    printf("LSM6DSL Movement Disorder Analysis System\r\n");
    printf("Sampling at %dHz with FFT size %d\r\n", SAMPLE_RATE, FFT_SIZE);

    // Initialize FFT instance
    if (arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE) != ARM_MATH_SUCCESS)
    {
        printf("ERROR: FFT initialization failed\r\n");
    }
    else
    {
        printf("FFT initialized successfully\r\n");
    }

    // Check WHO_AM_I register to confirm sensor is working
    char whoami_reg = WHO_AM_I;
    char whoami;
    i2c.write(LSM6DSL_ADDR, &whoami_reg, 1, true);
    i2c.read(LSM6DSL_ADDR, &whoami, 1);

    if (whoami != 0x6A)
    {
        printf("LSM6DSL not detected! WHO_AM_I = 0x%X\r\n", whoami);
        return 1;
    }

    printf("LSM6DSL detected! WHO_AM_I = 0x%X\r\n", whoami);
    init_lsm6dsl();

    led_tremor = 0;
    led_dyskinesia = 0;

    // Initialize the circular buffer
    for (int i = 0; i < FIFO_SIZE; i++)
    {
        accel_fifo_buffer[0][i] = 0.0f;
        accel_fifo_buffer[1][i] = 0.0f;
        accel_fifo_buffer[2][i] = 0.0f;
    }
    // Initialize sampler ticker
    Ticker sampler_ticker;
    auto sample_interval_us = chrono::microseconds(SAMPLER_TICK_PERIOD_US);
    sampler_ticker.attach(&sample_ready_isr, sample_interval_us);

    // Initialize analyzer timer
    Timer analyzer_timer;
    analyzer_timer.start();

    while (true)
    {
        if (sample_ready)
        {
            sample_ready = false;
            collect_accel_data();
        }

        if (analyzer_timer.elapsed_time() >= 1000ms)
        {
            analyzer_timer.reset();
            extract_last_window();
            process_FFT();
            float32_t max_magnitude, peak_freq;
            analyze_frequency(&max_magnitude, &peak_freq);
            
            State instant_state = classify_state(max_magnitude, peak_freq);
            update_state_history(instant_state);
            this_state = get_most_frequent_state();

            printf("Current state: %d\r\n", this_state);

            update_indicator(max_magnitude, peak_freq);

            printf("State History: ");
            for (int i = 0; i < HISTORY_LENGTH; i++)
            {
                printf("%d ", state_history[i]);
            }
            printf("\r\n");
        }
    }
}