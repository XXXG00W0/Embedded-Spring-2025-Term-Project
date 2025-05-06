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

Ticker led_ticker; // Ticker for LED blinking
void toggle_led(){
    // printf("Toggling LED\n");
    if(this_state == TREMOR){
        led_tremor = !led_tremor; // Toggle tremor LED
    } else if(this_state == DYSKINESIA){
        led_dyskinesia = !led_dyskinesia; // Toggle dyskinesia LED
    }
}

// Threshold for detection (calibration required)
// #define THRESHOLD 100.0f

// Settings for 3-second intervals at 20Hz sampling rate
#define INTERVAL_SECONDS 3
#define FFT_SIZE 256
#define SAMPLE_RATE 100 // 100Hz
#define SAMPLER_TICK_PERIOD_US (1000000 / SAMPLE_RATE) // 10ms
// #define SAMPLES_PER_INTERVAL (SAMPLE_RATE * INTERVAL_SECONDS) // 50 * 3 = 150 samples
#define SAMPLES_PER_INTERVAL 256
#define RESOLUTION (SAMPLE_RATE / (float)FFT_SIZE)
#define MAGNITUDE_THRESHOLD 1.0f 

// Timestamp array for recording actual sample times
uint64_t sample_timestamps[SAMPLES_PER_INTERVAL];
float32_t actual_sample_rate = SAMPLE_RATE; // Will be updated based on measurements

// Buffers
float32_t accel_data[3][SAMPLES_PER_INTERVAL]; // 3 axes
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];
int buffer_index = 0;

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

// // Calculate the magnitude of the acceleration vector
// void vector_magnitude(int16_t x, int16_t y, int16_t z, float32_t *result)
// {
//     float32_t sum_of_squares = (float32_t)(x * x + y * y + z * z);
//     arm_sqrt_f32(sum_of_squares, result);
// }

// Collect data from accelerometer
void collect_accel_data(){
    int16_t ax, ay, az;

    Ticker sampler_ticker;
    Timer sampler_timer;
    bool sample_ready = false;

    // lambda function for setting flag
    auto sample_ready_trigger = [&sample_ready]()
    {
        sample_ready = true;
    };

    sampler_timer.start();
    auto sample_interval_us = chrono::microseconds(SAMPLER_TICK_PERIOD_US);
    sampler_ticker.attach(sample_ready_trigger, sample_interval_us);
    for (unsigned int i = 0; i < SAMPLES_PER_INTERVAL; i++){
        // Wait for ticker to do a sample
        while (!sample_ready){
            ThisThread::yield(); // Could cause busy waiting
        }

        sample_ready = false;

        read_accel(ax, ay, az);
        accel_data[0][i] = (float32_t)ax;
        accel_data[1][i] = (float32_t)ay;
        accel_data[2][i] = (float32_t)az;
    }
    sampler_timer.stop();
    printf("Data collected\n");
}

void process_FFT() {
    // Clear the FFT input buffer first
    memset(fft_input, 0, sizeof(fft_input));
    
    // Define a window function (Hanning window) for the actual sample size
    float32_t window[FFT_SIZE];
    arm_hanning_f32(window, FFT_SIZE);
    
    float32_t x_mean = 0.0f, y_mean = 0.0f, z_mean = 0.0f;
    // Calculate mean values for each axis
    arm_mean_f32(accel_data[0], SAMPLES_PER_INTERVAL, &x_mean);
    arm_mean_f32(accel_data[1], SAMPLES_PER_INTERVAL, &y_mean);
    arm_mean_f32(accel_data[2], SAMPLES_PER_INTERVAL, &z_mean);
    
    // Scale factor for normalization
    // For +/-2g range, full scale is typically 32768 (for 16-bit values)
    const float32_t scale_factor = 16384.0f; // = 2^14 LSB/g for +/-2g range
    
    // Apply the window function, remove DC bias, and normalize
    for (int i = 0; i < SAMPLES_PER_INTERVAL; i++) {
        // Normalize and remove DC component
        float32_t x_norm = (accel_data[0][i] - x_mean) / scale_factor;
        float32_t y_norm = (accel_data[1][i] - y_mean) / scale_factor;
        float32_t z_norm = (accel_data[2][i] - z_mean) / scale_factor;
        
        // Calculate magnitude
        float32_t mag = sqrtf(x_norm*x_norm + y_norm*y_norm + z_norm*z_norm);
        
        // Apply window function and store in FFT input buffer
        fft_input[i] = mag * window[i];
    }
    
    // Perform FFT on real input data
    arm_rfft_fast_f32(&FFT_Instance, fft_input, fft_output, 0);
    
    // Calculate magnitude spectrum (note: output is complex)
    arm_cmplx_mag_f32(fft_output, magnitude, FFT_SIZE / 2);
}

void analyze_frequency(float32_t* max_magnitude, float32_t* peak_freq) {
    uint32_t max_index = 0;

    // Only analyze frequencies in the range of interest
    const float32_t min_freq = 1.0f; // 1Hz
    const float32_t max_freq = 10.0f; // 10Hz
    uint32_t start_index = ceilf(min_freq / RESOLUTION);
    uint32_t end_index = ceilf(max_freq / RESOLUTION);
    if (end_index > FFT_SIZE / 2) {
        end_index = FFT_SIZE / 2;
    }

    // Find maximal magnitude in the specified range
    arm_max_f32(&magnitude[start_index], end_index - start_index, max_magnitude, &max_index);
    max_index += start_index; // Adjust index to the full range

    if (*max_magnitude < MAGNITUDE_THRESHOLD) {
        *peak_freq = 0.0f; // No significant peak detected
    }else{
        *peak_freq = max_index * RESOLUTION; // Convert index to frequency
    }

    // Print analysis results
    printf("Analysis range: %.1f-%.1f Hz\n", min_freq, max_freq);
    printf("Max Magnitude: %.2f at Bin %lu (%.2f Hz)\n", *max_magnitude, max_index, *peak_freq);
}

void update_indicator(float32_t max_magnitude, float32_t peak_freq){

    // normalize magnitude for intensity display with LED blink
    float intensity = fminf(max_magnitude / 100.0f, 1.0f); // Normalize to [0, 1]
    blink_period_ms = (uint32_t)(1000 * (1.0f - intensity)); // Blink period in milliseconds
    if (blink_period_ms < 100) {
        blink_period_ms = 100; // Minimum blink period
    }
    printf("Blink period: %lu ms\n", blink_period_ms);

    // Determine state based on frequency
    this_state = NONE;
    if (peak_freq >= 3.0f && peak_freq <= 5.0f && max_magnitude > MAGNITUDE_THRESHOLD) {
        this_state = TREMOR;
    } else if (peak_freq > 5.0f && peak_freq <= 7.0f && max_magnitude > MAGNITUDE_THRESHOLD) {
        this_state = DYSKINESIA;
    }
    printf("State: %d\n", this_state);

    // LED blink based on magnitude
    if (this_state != last_state) {
        // turn off old ticker
        led_ticker.detach();    
        led_tremor = 0;
        led_dyskinesia = 0;
        
        if (this_state != NONE) {
            // convert miliseconds to microseconds
            auto blink_period_us = std::chrono::duration<uint32_t, std::milli>(blink_period_ms);
            led_ticker.attach(&toggle_led, blink_period_us);
        }else{
            led_tremor = 0; // Turn off tremor LED
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
    printf("Sampling at %dHz for %d second intervals\r\n", SAMPLE_RATE, INTERVAL_SECONDS);

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

    if (whoami != 0x6A)
    {
        printf("LSM6DSL not detected! WHO_AM_I = 0x%X\r\n", whoami);
        return 1;
    }

    printf("LSM6DSL detected! WHO_AM_I = 0x%X\r\n", whoami);
    init_lsm6dsl();

    led_tremor = 0; // Initialize LED state to off
    led_dyskinesia = 0; // Initialize LED state to off

    while (true)
    {
        // led_dyskinesia = 1;
        // led_tremor = 1;
        collect_accel_data();
        process_FFT();
        float32_t max_magnitude, peak_freq;
        analyze_frequency(&max_magnitude, &peak_freq);
        update_indicator(max_magnitude, peak_freq);
    }
}