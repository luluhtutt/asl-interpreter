/**
 * V. Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 (pin 21) ---> VGA Hsync
 *  - GPIO 17 (pin 22) ---> VGA Vsync
 *  - GPIO 18 (pin 24) ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 (pin 25) ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 (pin 26) ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 (pin 27) ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 (pin 11) ---> MPU6050 SDA (yellow)
 *  - GPIO 9 (pin 12) ---> MPU6050 SCL (orange)
 *  - 3.3v (pin 36) ---> MPU6050 VCC (red)
 *  - RP2040 GND ---> MPU6050 GND (black)
 *  - GPIO 0 (pin 1)  -->     UART RX (yellow)
 *  - GPIO 1 (pin 2) -->     UART TX (orange)
 *  - RP2040 GND    -->     UART GND
 *  - GPIO 12 (pin 16) ---> button ---> GND
 *  - FLEX SENSORS
 */

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
// Include custom libraries
#include "vga16_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_3.h"

bool do_detect = false;

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[40];
int line_num = 10;

// Some macros for max/min/abs
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a < b) ? b : a)
#define abs(a) ((a > 0) ? a : -a)

// semaphore
static struct pt_sem vga_semaphore;
static struct pt_sem button_semaphore;

// IMU filter constants
#define PI 3.14
#define ALPHA 0.35

volatile fix15 curr_angle = float2fix15(0.0f);
fix15 threshold;

fix15 prev_filtered_ax = 0;
fix15 prev_filtered_ay = 0;
fix15 prev_filtered_az = 0;

fix15 filtered_ax;
fix15 filtered_ay;
fix15 filtered_az;
// Button GPIO
#define PIN_BUTTON 7

#define CONTACT1_PIN 26 // your first contact sensor
#define CONTACT2_PIN 27 // your second contact sensor
#define CONTACT3_PIN 28 // your third contact sensor

static const uint contact_pins[] = {
    CONTACT1_PIN,
    CONTACT2_PIN,
    CONTACT3_PIN};

static const size_t NUM_CONTACTS = sizeof(contact_pins) / sizeof(contact_pins[0]);

// debouncing
int STATE = 0;
// 0: not pressed, 1: maybe pressed, 2: pressed, 3: maybe not pressed

// flex sensor variables
char measured_flex[5]; // thumb, pointer, middle, ring, pinky
fix15 measured_imu[3]; // pitch, roll, yaw

char cur_letter;

volatile int mid_side = 0, mid_front = 0, ring_side = 0;

// data structure for letter detection
#define NUM_FLEX 5
#define NUM_IMU 3
#define NUM_READINGS 20
#define DELAY 20

// IRQ
// beep beep define statements
// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// 8 channel adc stuff
// Define the SPI pins
#define SPI_PORT spi1
#define PIN_SPI_CS 13   // Chip Select
#define PIN_SPI_CLK 14  // Clock
#define PIN_SPI_MISO 12 // MISO
#define PIN_SPI_MOSI 15 // MOSI

// vga vars
int curs_x = 10;
int curs_y = 10;

typedef struct
{
    int finger_num;
    uint16_t thresh_pos_hb;
    uint16_t thresh_pos_sh;

    char bend;
} Finger;

Finger finger_map[] = {
    {0, 0, 275, 's'},   // thumb
    {1, 210, 400, 's'}, // index
    {2, 285, 450, 's'}, // middle
    {3, 275, 400, 's'}, // ring
    {4, 260, 410, 's'}  // pinky
};

typedef struct
{
    char letter;
    char flex[NUM_FLEX]; // thumb, pointer, middle, ring, pinky
    bool check_imu;
    int accel[NUM_IMU]; // rotational (pitch, roll, yaw)
    int threshold_imu[NUM_IMU];
    bool check_contact;
    char contact[3]; // (pin/gpio) 31/26, 32/27, 34/28
} Gesture;

// placeholder, need to actually experiment
Gesture sign_map[] = {
    {'A', {'s', 'b', 'b', 'b', 'b'}, false, {90, 0, -90}, {45, 45, 45}, false, {'o', 'o', 'o'}},
    {'B', {'b', 's', 's', 's', 's'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
    {'C', {'s', 'h', 'h', 'h', 'h'}, false, {90, -90, -90}, {45, 45, 45}, true, {'o', 'o', 'c'}},
    {'D', {'b', 's', 'h', 'h', 'h'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
    {'E', {'b', 'b', 'b', 'b', 'b'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
    {'F', {'b', 'h', 's', 's', 's'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},         // iffy
    {'G', {'s', 's', 'b', 'b', 'b'}, true, {90, 0, -133}, {20, 20, 20}, false, {'o', 'o', 'o'}},       // IMU
    {'H', {'b', 's', 's', 'b', 'b'}, true, {90, -10, -170}, {1000, 20, 1000}, false, {'c', 'o', 'c'}}, // could contact
    {'I', {'b', 'b', 'b', 'b', 's'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
    {'J', {'b', 'b', 'b', 'b', 's'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},      // all gyro must be neg
    {'K', {'b', 's', 's', 'b', 'b'}, true, {142, 54, -130}, {20, 20, 20}, true, {'o', 'o', 'c'}},   // IMU, contact
    {'L', {'s', 's', 'b', 'b', 'b'}, true, {90, -60, 0}, {1000, 20, 1000}, false, {'o', 'o', 'o'}}, // IMU
    {'M', {'b', 'h', 'h', 'h', 'b'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
    {'N', {'b', 'h', 'h', 'b', 'b'}, false, {90, 0, 0}, {10, 10, 10}, true, {'c', 'o', 'o'}},        // contact
    {'O', {'b', 'h', 'h', 'h', 'h'}, false, {90, 0, 0}, {10, 10, 10}, true, {'o', 'c', 'c'}},        // CONTACT DETECTS C FIRST !!!!!!!!!!!!!!!!
    {'P', {'s', 's', 's', 'b', 'b'}, true, {-13, 40, 170}, {45, 45, 100000}, true, {'o', 'o', 'c'}}, // IMU, contact
    {'Q', {'s', 's', 'b', 'b', 'b'}, true, {-22, 47, 152}, {20, 20, 20}, false, {'o', 'o', 'o'}},    // IMU
    {'R', {'b', 's', 's', 'h', 'h'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
    {'S', {'b', 'b', 'b', 'b', 'b'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},     // ???
    {'T', {'b', 'h', 'b', 'b', 'b'}, false, {90, 0, 0}, {10, 10, 10}, true, {'o', 'o', 'c'}},      // CONTACT
    {'U', {'b', 's', 's', 'b', 'b'}, true, {90, -60, 0}, {1000, 20, 1000}, true, {'c', 'o', 'c'}}, // CONTACT, imu !!!!
    {'V', {'b', 's', 's', 'b', 'b'}, true, {90, -60, 0}, {1000, 20, 1000}, true, {'c', 'o', 'o'}}, // CONTACT
    {'W', {'b', 's', 's', 's', 'b'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
    {'X', {'b', 'h', 'b', 'b', 'b'}, false, {90, 0, 0}, {10, 10, 10}, true, {'o', 'o', 'o'}},
    {'Y', {'s', 'b', 'b', 'b', 's'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
    {'Z', {'b', 's', 'b', 'b', 'b'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
    {' ', {'s', 's', 's', 's', 's'}, false, {90, 0, 0}, {10, 10, 10}, false, {'o', 'o', 'o'}},
};

char current_letter;

const int num_signs = sizeof(sign_map) / sizeof(Gesture);

// Function to read from MCP3008
uint16_t read_adc_ext(uint8_t channel)
{
    uint8_t tx_buffer[3] = {0x01, (0x80 | (channel << 4)), 0x00};
    uint8_t rx_buffer[3];

    gpio_put(PIN_SPI_CS, 0); // CS low
    spi_write_read_blocking(SPI_PORT, tx_buffer, rx_buffer, 3);
    gpio_put(PIN_SPI_CS, 1); // CS high

    // Extract 10-bit result
    return ((rx_buffer[1] & 0x03) << 8) | rx_buffer[2];
}

// Interrupt service routine 1 khZ
void alarm_irq()
{
    // // Clear the alarm irq
    // hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // // Reset the alarm register
    // timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;
    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    mpu6050_read_raw(acceleration, gyro);

    // // Apply low-pass filter to accelerometer readings
    fix15 alpha_fix = float2fix15(ALPHA);
    fix15 one_minus_alpha = float2fix15(1.0f) - alpha_fix;

    filtered_ax = prev_filtered_ax + ((acceleration[0] - prev_filtered_ax) >> 4);
    filtered_ay = prev_filtered_ay + ((acceleration[1] - prev_filtered_ay) >> 4);
    filtered_az = prev_filtered_az + ((acceleration[2] - prev_filtered_az) >> 4);

    prev_filtered_ax = filtered_ax;
    prev_filtered_ay = filtered_ay;
    prev_filtered_az = filtered_az;

    // NO SMALL ANGLE APPROXIMATION
    fix15 accel_angle[3];
    accel_angle[0] = multfix15(float2fix15(atan2(-acceleration[0], acceleration[2])), oneeightyoverpi); // atan2(x/z) rotation around y
    accel_angle[1] = multfix15(float2fix15(atan2(acceleration[2], -acceleration[1])), oneeightyoverpi); // atan2(z/-y) rotation around x
    accel_angle[2] = multfix15(float2fix15(atan2(acceleration[0], acceleration[1])), oneeightyoverpi);  // atan2(-x/y) rotation around z

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    fix15 gyro_angle_delta[3];
    gyro_angle_delta[0] = multfix15(gyro[0], zeropt001); // y rotation
    gyro_angle_delta[1] = multfix15(gyro[1], zeropt001);
    gyro_angle_delta[2] = multfix15(gyro[2], zeropt001);
    // gyro_angle_delta[2] = multfix15(gyro[2], zeropt001);

    measured_imu[0] = accel_angle[0];
    measured_imu[1] = accel_angle[1];
    measured_imu[2] = accel_angle[2];

    // Complementary angle (degrees - 15.16 fixed point)
    // measured_imu[0] = multfix15(measured_imu[0] + gyro_angle_delta[0], zeropt999) + multfix15(accel_angle[0], zeropt001);
    // measured_imu[1] = multfix15(measured_imu[1] + gyro_angle_delta[1], zeropt999) + multfix15(accel_angle[1], zeropt001);
    // measured_imu[2] = multfix15(measured_imu[2] + gyro_angle_delta[2], zeropt999) + multfix15(accel_angle[2], zeropt001);

    // printf("measured imu 0: %f\n", fix2float15(measured_imu[0])); // YAS QUEEN
    // printf("measured imu 1: %f\n", fix2float15(measured_imu[1])); // WATCH ME WHIP
    // printf("measured imu 2: %f\n", fix2float15(measured_imu[2])); // YOU GOTTA STOP

    // printf("measured gyro 0: %f\n", fix2float15(gyro_angle_delta[0]));
    // printf("measured gyro 1: %f\n", fix2float15(gyro_angle_delta[1]));
    // printf("measured gyro 2: %f\n", fix2float15(gyro_angle_delta[2]));

    for (int i = 0; i < NUM_FLEX; i++)
    {

        uint16_t sum_flex_meas = 0;
        // printf("reading adc \n");
        for (int j = 0; j < NUM_READINGS; j++)
        {
            sum_flex_meas += read_adc_ext(i);
        }

        uint16_t temp_flex_meas = sum_flex_meas / NUM_READINGS;

        if (temp_flex_meas > finger_map[i].thresh_pos_sh)
        {
            finger_map[i].bend = 's';
        }
        else if (temp_flex_meas < finger_map[i].thresh_pos_hb || i == 0)
        {
            finger_map[i].bend = 'b';
        }
        else
        {

            finger_map[i].bend = 'h';
        }

        // map to range
        // printf("Flex %d: %u\n", i + 1, (int)read_adc_ext(i)); // Print values
    }

    int match = 1;

    // use sensor readings to determine which letter
    for (int i = 0; i < num_signs; i++)
    {
        match = 1;
        // Compare flex sensor values
        for (int j = 0; j < NUM_FLEX; j++)
        {
            if (finger_map[j].bend != sign_map[i].flex[j])
            { // use threshold as needed
                match = 0;
                break;
            }
        }

        if (match && sign_map[i].check_contact)
        {
            for (size_t j = 0; j < 3; j++)
            {
                uint pin = contact_pins[j];
                int raw = gpio_get(pin);
                // wiring-to-3.3 V + pull-down: HIGH means CLOSED → 'c', LOW means OPEN → 'o'
                char contact_char = raw ? 'c' : 'o';
                // printf("Contact %zu (GPIO %u): %c\n", i + 1, pin, contact_char);
                if (sign_map[i].contact[j] == 'c')
                {
                    if (contact_char != 'c')
                    {
                        // printf("Contact mismatch for letter %c\n", sign_map[i].letter);
                        match = 0;
                        break;
                    }
                }
            }
        }

        // Compare IMU values if flex matched
        if (match && sign_map[i].check_imu)
        {
            // printf("WE MADE IT HERE\n");
            for (int j = 0; j < NUM_IMU; j++)
            {

                if (abs(fix2int15(measured_imu[j] - int2fix15(sign_map[i].accel[j]))) > (sign_map[i].threshold_imu[j]))
                { // adjust tolerance
                    int diff = fix2int15(measured_imu[j] - int2fix15(sign_map[i].accel[j]));
                    // printf("IMU difference %i\n", diff);
                    // printf("IMU mismatch for letter %c\n", sign_map[i].letter);
                    match = 0;
                    break;
                }
            }
        }

        if (match)
        {
            cur_letter = sign_map[i].letter;
            if (cur_letter == 'I')
            {
                if (gyro_angle_delta[0] < 0 && gyro_angle_delta[1] < 0 && gyro_angle_delta[2] < 0)
                {
                    cur_letter = 'J';
                }
            }
            // printf("letter detected: %c\n", cur_letter);
            break;
        }
    }
    if (!match)
    {
        cur_letter = '?';
        // printf("letter detected: %c\n", cur_letter);
    }

    // PT_SEM_SIGNAL(pt, &vga_semaphore);
}

void update_vga()
{

    // fillRect(10, 30, 40, 50, WHITE);
    char str[40];
    snprintf(str, sizeof(str), "%c", cur_letter);
    // write text
    setCursor(curs_x, curs_y);
    writeString(str);
    curs_x += 30;
    if (curs_x > 520)
    {
        curs_y += 30;
        curs_x = 10;
    }

    // printf("in vga thread\n");
    // printf("flex reading:");
    // printf("%u\n", measured_flex[0]);
}

// Thread that draws to VGA display
static PT_THREAD(protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt);

    printf("in vga thread\n");
    // button press parameters
    int possible = 0; // 0: pressed, 1: not pressed
    uint16_t button_pressed;
    button_pressed = gpio_get(PIN_BUTTON);

    setTextColor('2');
    setTextSize(3.0);

    while (true)
    {
        // printf("in while\n");
        alarm_irq();
        printf("current letter: %c\n", cur_letter);
        button_pressed = gpio_get(PIN_BUTTON);
        // printf("button pressed: %u", button_pressed);
        if (STATE == 0) // not pressed
        {
            if (!button_pressed)
            {
                STATE = 1;
                possible = 0;
            }
        }
        else if (STATE == 1) // maybe pressed
        {
            if (possible == button_pressed)
            {
                STATE = 2;
                // Signal VGA to draw
                do_detect = true;
                update_vga();
                // PT_SEM_SIGNAL(pt, &button_semaphore);

                // do sequence
            }
        }
        else if (STATE == 2) // pressed
        {
            if (button_pressed != possible)
            {
                STATE = 3;
            }
        }
        else if (STATE == 3) // maybe not pressed
        {
            if (possible == button_pressed)
            {
                STATE = 2;
            }
            else
            {
                STATE = 0;
            }
        }

        // Wait on semaphore
        // Increment drawspeed controller
        // PT_SEM_WAIT(pt, &vga_semaphore);
        PT_YIELD_usec(30000);
    }
    // Indicate end of thread
    PT_END(pt);
}

int main()
{

    // Initialize stdio
    stdio_init_all();
    printf("initialized stdio\n");

    for (int i = 0; i < NUM_CONTACTS; i++)
    {
        uint pin = contact_pins[i];
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_down(pin); // assume each closes to GND
    }

    // Initialize VGA
    initVGA();
    printf("initialized vga\n");

    setTextColor('7');
    setTextSize(3.0);

    // // write text
    // setCursor(curs_x, curs_y);
    // writeString("HELLO");

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);
    printf("initialized imu\n");

    // initialize ADC for reading flex
    // adc_init();
    // adc_gpio_init(FLEX1_PIN);
    // adc_select_input(0);
    // printf("initialized adc\n");

    // Set up button to be PULL-UP (HIGH when not pressed, LOW when pressed)
    gpio_init(PIN_BUTTON);
    gpio_set_dir(PIN_BUTTON, GPIO_IN);
    gpio_pull_up(PIN_BUTTON);
    // printf("initialized button\n");

    // Setup SPI interface for MCP3008
    spi_init(SPI_PORT, 500000);                       // Lower to 1MHz for stability
    spi_set_format(SPI_PORT, 8, 0, 0, SPI_MSB_FIRST); // 8-bit mode

    gpio_set_function(PIN_SPI_CLK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_SPI_CS, GPIO_FUNC_SPI);
    gpio_init(PIN_SPI_CS);
    gpio_set_dir(PIN_SPI_CS, GPIO_OUT);
    gpio_put(PIN_SPI_CS, 1); // Start with CS high (inactive)

    ///////////////////////////// IRQ //////////////////////////////////////
    ////////////////// INTERRUPT INIT ///////////////////////
    // // Enable the interrupt for the alarm (we're using Alarm 0)
    // hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // // Associate an interrupt handler with the ALARM_IRQ
    // irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // // Enable the alarm interrupt
    // irq_set_enabled(ALARM_IRQ, true);
    // // Write the lower 32 bits of the target time to the alarm register, arming it.
    // timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;
    // // printf("set irq\n");

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1
    // multicore_reset_core1();
    // multicore_launch_core1(core1_entry);

    // pt_add_thread(protothread_core_0);

    // start core 0
    pt_add_thread(protothread_vga);
    pt_schedule_start;
}
