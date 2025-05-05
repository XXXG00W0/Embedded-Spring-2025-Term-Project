#include "mbed.h"
#include "arm_math.h"

BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) { return &serial_port; }

#define FFT_SIZE 256
#define SAMPLE_RATE 10000

float32_t input_fft[FFT_SIZE];
float32_t fft_out[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];

float32_t freq = 1000.0f;  // 1kHz test signal

arm_rfft_fast_instance_f32 FFT_Instance;

void make_sine_wave(float freq) {
    for (int i = 0; i < FFT_SIZE; i++) {
        float t = (float)i / SAMPLE_RATE;  // FIXED
        input_fft[i] = arm_sin_f32(2 * PI * freq * t);
    }
}

void run_fft() {
    arm_rfft_fast_f32(&FFT_Instance, input_fft, fft_out, 0);
    arm_cmplx_mag_f32(fft_out, magnitude, FFT_SIZE / 2);
}

void show_results() {
    float resolution = SAMPLE_RATE / FFT_SIZE;

    printf("Bin\tFreq (Hz)\tMagnitude\n");
    for (int i = 0; i < 28; i++) {
        printf(" %d\t%.2f\t\t%.4f\n",i, i * resolution, magnitude[i]);
    }

    uint32_t max_index = 0;
    float32_t max_val = 0.0f;
    arm_max_f32(magnitude, FFT_SIZE/2, &max_val, &max_index);

    printf("Peak Frequency: %.2f Hz\n\n", max_index * resolution);

    printf("Max magnitude: %.1f at bin %lu (%.2f Hz)\r\n", 
        max_val, max_index, max_index * resolution);
}

int main() {
    
    arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);

    make_sine_wave(freq);  
    run_fft();
    // show_results();
    
    while (true) {
        ThisThread::sleep_for(1000ms);
    }
}

// #include "mbed.h"
// #include "arm_math.h"

// BufferedSerial serial_port(USBTX, USBRX, 115200);
// FileHandle *mbed::mbed_override_console(int)
// {
//     return &serial_port;
// }    

// // I2C pins：SDA=PB_11, SCL=PB_10（B-L475E-IOT01A）
// I2C i2c(PB_11, PB_10);

// // LSM6DSL I2C addr（7-bit addr，left shift 0xD4）
// const int LSM6DSL_ADDR = 0x6A << 1;

// // accelerometer register addr（LSB/MSB）
// #define OUTX_L_XL 0x28
// #define WHO_AM_I 0x0F

// // FFT instance
// arm_rfft_fast_instance_f32 FFT_Instance;
// // Define discorder states
// enum State
// {
//     NONE,
//     TREMOR,
//     DYSKINESIA
// };
// State last_state = NONE;
// State this_state = NONE;

// // Threshold for detection (calibration required)
// // #define THRESHOLD 100.0f

// // Settings for 3-second intervals at 20Hz sampling rate
// #define INTERVAL_SECONDS 3
// #define FFT_SIZE 256
// #define SAMPLE_RATE 20 // 20Hz
// #define SAMPLES_PER_INTERVAL (SAMPLE_RATE * INTERVAL_SECONDS) // 20 * 3 = 60
// #define RESOLUTION (SAMPLE_RATE / (float)FFT_SIZE)

// // Timestamp array for recording actual sample times
// uint64_t sample_timestamps[SAMPLES_PER_INTERVAL];
// float32_t actual_sample_rate = SAMPLE_RATE; // Will be updated based on measurements

// // Buffers
// float32_t fft_input[FFT_SIZE];
// float32_t fft_output[FFT_SIZE];
// float32_t magnitude[FFT_SIZE / 2];
// int buffer_index = 0;

// float max_freq, peak_freq;

// // Read accelerometer data
// void read_accel(int16_t &ax, int16_t &ay, int16_t &az)
// {
//     char reg = OUTX_L_XL;
//     char data[6] = {0};

//     // Write register address
//     i2c.write(LSM6DSL_ADDR, &reg, 1, true); // Repeated start
//     i2c.read(LSM6DSL_ADDR, data, 6);

//     // Combine high and low bytes (note: LSB first, little endian)
//     ax = (int16_t)(data[1] << 8 | data[0]);
//     ay = (int16_t)(data[3] << 8 | data[2]);
//     az = (int16_t)(data[5] << 8 | data[4]);
// }

// // Calculate the magnitude of the acceleration vector
// void vector_magnitude(int16_t x, int16_t y, int16_t z, float32_t *result)
// {
//     float32_t sum_of_squares = (float32_t)(x * x + y * y + z * z);
//     arm_sqrt_f32(sum_of_squares, result);
// }

// void collect_accel_data(){
//     int16_t ax, ay, az;
//     float32_t magnitude_val;

//     Ticker sampler_ticker;
//     Timer sampler_timer;
//     bool sample_ready = false;

//     // lambda function for setting flag
//     auto sample_ready_trigger = [&sample_ready]()
//     {
//         sample_ready = true;
//     };

//     sampler_timer.start();
//     sampler_ticker.attach(sample_ready_trigger, 50ms); // 20Hz
//     for (unsigned int i = 0; i < SAMPLES_PER_INTERVAL; i++){
//         // Wait for ticker to do a sample
//         while (!sample_ready){
//             ThisThread::yield();
//         }

//         sample_ready = false;

//         read_accel(ax, ay, az);
//         vector_magnitude(ax, ay, az, &magnitude_val);

//         fft_input[i] = magnitude_val;
//     }
//     sampler_timer.stop();
//     printf("Data collected\n");
// }

// void process_FFT(){
//     arm_rfft_fast_f32(&FFT_Instance, fft_input, fft_output, 0);
    
//     arm_cmplx_mag_f32(fft_output, magnitude, FFT_SIZE / 2);
// }

// float analyze_frequency(){
//     uint32_t max_idx;

//     // Skip DC component (index 0)
//     arm_max_f32(&magnitude[1], (FFT_SIZE / 2) - 1, &max_freq, &max_idx);
//     max_idx += 1; // Adjust index to account for skipping DC

//     // Calculate frequency based on ACTUAL sample rate
//     float actual_freq_resolution = actual_sample_rate / (float)FFT_SIZE;
//     float peak_freq = max_idx * actual_freq_resolution;
//     return peak_freq;
// }

// void update_indicator(float peak_freq){
//     // Classify the movement
//     this_state = NONE;
//     if (peak_freq >= 3.0f && peak_freq <= 5.0f)
//         this_state = TREMOR;
//     else if (peak_freq > 5.0f && peak_freq <= 7.0f)
//         this_state = DYSKINESIA;

//     // Print and signal changes in state
//     if (this_state != last_state)
//     {
//         printf("Peak Frequency: %.2f Hz | Magnitude: %.2f | State: ", peak_freq, max_freq);

//         switch (this_state)
//         {
//         case TREMOR:
//             printf("TREMOR\n");
//             break;
//         case DYSKINESIA:
//             printf("DYSKINESIA\n");
//             break;
//         default:
//             printf("NONE\n");
//             break;
//         }

//         last_state = this_state;
//     }

// }

// // Initialize the LSM6DSL accelerometer
// void init_lsm6dsl()
// {
//     // CTRL1_XL register: Set to 104Hz ODR, +/-2g range, 50Hz bandwidth
//     char ctrl1_xl[2] = {0x10, 0x60};
//     i2c.write(LSM6DSL_ADDR, ctrl1_xl, 2);
// }

// int main()
// {
//     printf("LSM6DSL Movement Disorder Analysis System\r\n");
//     printf("Sampling at %dHz for %d second intervals\r\n", SAMPLE_RATE, INTERVAL_SECONDS);

//     // Initialize FFT instance
//     if (arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE) != ARM_MATH_SUCCESS) {
//         printf("ERROR: FFT initialization failed\r\n");
//     } else {
//         printf("FFT initialized successfully\r\n");
//     }

//     // Check WHO_AM_I register to confirm sensor is working
//     char whoami_reg = WHO_AM_I;
//     char whoami;
//     i2c.write(LSM6DSL_ADDR, &whoami_reg, 1, true);
//     i2c.read(LSM6DSL_ADDR, &whoami, 1);

//     if (whoami != 0x6A)
//     {
//         printf("LSM6DSL not detected! WHO_AM_I = 0x%X\r\n", whoami);
//         return 1;
//     }

//     printf("LSM6DSL detected! WHO_AM_I = 0x%X\r\n", whoami);
//     init_lsm6dsl();


//     while (true)
//     {
//         collect_accel_data();
//         process_FFT();
//         // float peak_freq = analyze_frequency();
//         // update_indicator(peak_freq);
//     }
// }