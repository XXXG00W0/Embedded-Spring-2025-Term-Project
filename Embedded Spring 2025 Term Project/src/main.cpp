#include "mbed.h"
#include "arm_math.h"

BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int)
{
    return &serial_port;
}

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

// Threshold for detection (calibration required)
// #define THRESHOLD 100.0f

// Settings for 3-second intervals at 20Hz sampling rate
#define INTERVAL_SECONDS 3
#define FFT_SIZE 256
#define SAMPLE_RATE 20 // 20Hz
#define SAMPLES_PER_INTERVAL (SAMPLE_RATE * INTERVAL_SECONDS) // 20 * 3 = 60
#define RESOLUTION (SAMPLE_RATE / (float)FFT_SIZE)

// Timestamp array for recording actual sample times
uint64_t sample_timestamps[SAMPLES_PER_INTERVAL];
float actual_sample_rate = SAMPLE_RATE; // Will be updated based on measurements

// Buffers
float32_t sample_buffer[SAMPLES_PER_INTERVAL];
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];

// Buffer lock
Mutex buffer_mutex;
int buffer_index = 0;
bool buffer_full = false;

// Flag for running a new FFT
bool fft_needs_init = true;

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

// Calculate the magnitude of the acceleration vector
void vector_magnitude(int16_t x, int16_t y, int16_t z, float32_t *result)
{
    float32_t sum_of_squares = (float32_t)(x * x + y * y + z * z);
    arm_sqrt_f32(sum_of_squares, result);
}

// Initialize the LSM6DSL accelerometer
void init_lsm6dsl()
{
    // CTRL1_XL register: Set to 104Hz ODR, +/-2g range, 50Hz bandwidth
    char ctrl1_xl[2] = {0x10, 0x60};
    i2c.write(LSM6DSL_ADDR, ctrl1_xl, 2);
}

void sampler()
{
    int16_t ax, ay, az;
    float32_t magnitude_val;

    // Hardware timer for regular sampling intervals
    Ticker sampler_ticker;
    Timer sampler_timer;
    bool sample_ready = false;

    // lambda function for setting flag
    auto sample_ready_trigger = [&sample_ready]()
    {
        sample_ready = true;
    };

    sampler_timer.start();
    sampler_ticker.attach(sample_ready_trigger, 50ms); // 20Hz

    uint64_t interval_start_time = sampler_timer.elapsed_time().count();

    while (true)
    {
        // Wait for ticker to do a sample
        while (!sample_ready){
            ThisThread::yield();
        }

        sample_ready = false;

        read_accel(ax, ay, az);
        vector_magnitude(ax, ay, az, &magnitude_val);

        // Record timestamp along with sample
        uint64_t current_time = sampler_timer.elapsed_time().count();

        buffer_mutex.lock();
        sample_buffer[buffer_index] = magnitude_val;
        sample_timestamps[buffer_index] = current_time;
        buffer_index++;

        // If the program collects a full 3 sec interval
        if (buffer_index >= SAMPLES_PER_INTERVAL)
        {
            buffer_index = 0;
            buffer_full = true;
            printf("Setting buffer_full to true\n");

            // Calculate actual sampling rate for debugging
            uint64_t interval_duration = current_time - interval_start_time;
            float32_t actual_rate = (SAMPLES_PER_INTERVAL * 1000000.0f) / interval_duration;
            printf("Interval complete: %d samples\n", SAMPLES_PER_INTERVAL);
            printf("Duration: %lu microseconds\n", (unsigned long)interval_duration);
            int rate_int = (int)actual_rate;
            int rate_frac = (int)((actual_rate - rate_int) * 1000);
            printf("Rate: %d.%03d Hz\n", rate_int, rate_frac);

            // Reset timer for next interval
            interval_start_time = current_time;
        }
        buffer_mutex.unlock();
    }
}

void analyzer()
{
    printf("Analyzer thread started\n");
    while (true)
    {
        printf("Analyzer loop started\n");
        if (buffer_full)
        {   
            // Copy samples from buffer to a local buffer
            buffer_mutex.lock();
            float32_t local_buffer[SAMPLES_PER_INTERVAL];
            float32_t local_timestamps[SAMPLES_PER_INTERVAL];
            memcpy(local_buffer, sample_buffer, sizeof(local_buffer));
            memcpy(local_timestamps, sample_timestamps, sizeof(local_timestamps));
            buffer_full = false;
            buffer_mutex.unlock();

            // Calculate actual sample rate
            uint64_t total_time_us = local_timestamps[SAMPLES_PER_INTERVAL - 1] - local_timestamps[0];
            float32_t measured_rate = ((SAMPLES_PER_INTERVAL - 1) * 1000000.0f) / total_time_us;

            // Only update if there's a significant change to avoid frequent reinits
            if (fabsf(measured_rate - actual_sample_rate) > 0.5f)
            {
                actual_sample_rate = measured_rate;
                // fft_needs_init = true;
                printf("Sample rate adjusted to %.2f Hz\n", actual_sample_rate);
            }

            // Reinitialize FFT if needed based on the new sample rate
            if (fft_needs_init)
            {
                arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);
                // fft_needs_init = false;
            }

            // Prepare input for FFT (copy samples to FFT input, zero-pad if necessary)
            memset(fft_input, 0, sizeof(fft_input));
            memcpy(fft_input, local_buffer, sizeof(local_buffer) > sizeof(fft_input) ? sizeof(fft_input) : sizeof(local_buffer));

            // to-do: Apply a window function to reduce spectral leakage (Hann window)

            // Perform FFT
            arm_rfft_fast_f32(&FFT_Instance, fft_input, fft_output, 0);

            // Calculate magnitude
            arm_cmplx_mag_f32(fft_output, magnitude, FFT_SIZE / 2);

            // Find peak frequency
            float max_freq;
            uint32_t max_idx;

            // Skip DC component (index 0)
            arm_max_f32(&magnitude[1], (FFT_SIZE / 2) - 1, &max_freq, &max_idx);
            max_idx += 1; // Adjust index to account for skipping DC

            // Calculate frequency based on ACTUAL sample rate
            float actual_freq_resolution = actual_sample_rate / (float)FFT_SIZE;
            float peak_freq = max_idx * actual_freq_resolution;

            // Classify the movement
            this_state = NONE;
            if (peak_freq >= 3.0f && peak_freq <= 5.0f)
                this_state = TREMOR;
            else if (peak_freq > 5.0f && peak_freq <= 7.0f)
                this_state = DYSKINESIA;

            // Print and signal changes in state
            if (this_state != last_state)
            {
                printf("Peak Frequency: %.2f Hz | Magnitude: %.2f | State: ", peak_freq, max_freq);

                switch (this_state)
                {
                case TREMOR:
                    printf("TREMOR\n");
                    break;
                case DYSKINESIA:
                    printf("DYSKINESIA\n");
                    break;
                default:
                    printf("NONE\n");
                    break;
                }

                last_state = this_state;
            }

            // Print FFT spectrum occasionally for debugging
            static int debug_counter = 0;
            if (++debug_counter % 10 == 0)
            {
                printf("FFT Spectrum (using sample rate %.2f Hz):\n", actual_sample_rate);
                for (int i = 1; i < 20; i++)
                { // Show first 20 frequency bins
                    float freq = i * actual_freq_resolution;
                    printf("%.1f Hz: %.1f, ", freq, magnitude[i]);
                    if (i % 5 == 0)
                        printf("\n");
                }
                printf("\n");
            }

            ThisThread::sleep_for(1000ms);
        }
    }
}

int main()
{
    printf("LSM6DSL Movement Disorder Analysis System\r\n");
    printf("Sampling at %dHz for %d second intervals\r\n", SAMPLE_RATE, INTERVAL_SECONDS);

    // Initialize FFT instance
    arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);

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

    // Create and start threads
    Thread sampler_thread(osPriorityNormal, 2048);
    Thread analyzer_thread(osPriorityNormal, 4096);

    sampler_thread.start(sampler);
    analyzer_thread.start(analyzer);

    while (true)
    {
        ThisThread::sleep_for(5000ms);
        printf("System running: %d samples collected\r\n", buffer_index);
    }
}