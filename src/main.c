#include <stdio.h>
#include <string.h> // Include for memset
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h" // Added for SPI
#include "hardware/gpio.h" // Added for GPIO
#include "hardware/adc.h"  // Added for ADC
#include "can2040.h"
#include "RP2040.h"

struct sched_msg {
    struct can2040 *dest;
    uint16_t id;
    uint8_t len;
    uint8_t *buff;
    uint32_t freq_ms; // Changed freq to freq_ms to reflect milliseconds
    bool checksum;
    bool active; // Flag to indicate if this slot is used
};

// --- Steering Angle Sensor Defines ---
#define PIN_SPI_MISO 16
#define PIN_SPI_CS   17
#define PIN_SPI_SCK  18
#define PIN_SPI_MOSI 19
#define SPI_PORT spi0

#define PIN_IGNITION 6      // Ignition Detection Pin
#define PIN_SIG_LIGHT_L 4   // Signal Light Left Input Pin
#define PIN_SIG_LIGHT_R 5   // Signal Light Right Input Pin
#define PIN_CRUISE_MODE 8   // Cruise Control Mode Toggle Button Pin (Verify this assignment)
#define PIN_STEERING_BTNS 26 // ADC Pin for Steering Wheel Buttons (ADC0)

// AS5048A Commands (Parity bit is automatically calculated for SPI)
#define CMD_NOP              0x0000 // No operation
#define CMD_READ_ERROR_REG   0x0001 // Read Error Register
#define CMD_PROG_CONTROL_REG 0x0003 // Program Control Register
#define CMD_READ_ANGLE_REG   0x3FFF // Read Angle Register (14 bits)
#define CMD_READ_MAG_REG     0x3FFE // Read Magnitude Register
#define CMD_READ_AGC_REG     0x3FFD // Read AGC Register

// --- Function Definitions ---

bool is_allowed_message(uint16_t message_id);
void attach_checksum(uint16_t id, uint8_t len, uint8_t *msg); // Forward declaration
int get_checksum(uint8_t *msg, uint8_t len, uint16_t addr); // Forward declaration
uint16_t get_raw_rotation(); // Forward declaration for steering sensor
uint16_t read_steering_buttons_adc(); // Forward declaration for ADC read

// --- Variables ---

// Existing Buffers
uint8_t angle_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // For 0x25
uint8_t wheel_speeds_buffer[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}; // For 0xAA
uint8_t gear_msg_buffer[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0}; // For 0x3BC
uint8_t msg15_buffer[8] = {0x05, 0xea, 0x1b, 0x08, 0x00, 0x00, 0xc0, 0x9f}; // For 0x1C4
uint8_t msg12_buffer[8] = {0x66, 0x06, 0x08, 0x0a, 0x02, 0x00, 0x00, 0x00}; // For 0x48B

// New Buffers from STM32 Code
uint8_t pcm_cruise_msg[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}; // For 0x1D2
uint8_t pcm_cruise_2_msg[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}; // For 0x1D3
uint8_t signal_lights_msg[2] = {0x00, 0x00}; // For 0x614 (Note: STM32 code sent 2 bytes, check if correct)

// --- Steering Angle Sensor Variables ---
// uint8_t ignition = 0; // 0 = off, 1 = on - Replaced by global ignition state

uint16_t current_position;
uint16_t last_position;
int16_t position_movement;
int32_t rotational_position = 0; // Initialize to 0
uint16_t bitshift_cur_pos;
uint16_t bitshift_last_pos;
int16_t bitshift_pos_delta;

uint16_t raw_wheel_speeds[4] = {0x00, 0x00, 0x00, 0x00};
// Increased size to accommodate new messages
uint16_t ALLOWED_MESSAGES[5] = {0xb4}; // TODO: Add other allowed messages if needed, make size dynamic or define

#define MAX_SCHED_MSGS 20 // Increased max scheduled messages
#define STEERING_ANGLE_MAX_ABS_TICKS 49152 // Max absolute rotation ticks (16384 * 3 rotations)
#define ALLOWED_MESSAGES_COUNT 5 // Number of entries in ALLOWED_MESSAGES

// --- Cruise Control / Button Variables (from STM32 code) ---
uint8_t ignition = 0; // 0 = off, 1 = on
bool cruise_on = false;
uint16_t set_speed_kph = 50; // Target speed in KPH
uint16_t current_speed_kph = 0; // Current vehicle speed from CAN
uint16_t last_set_speed_kph = 50; // Speed before cruise was turned off

// Button state tracking
uint32_t btn_press_start_time_ms[4] = {0, 0, 0, 0}; // Timestamps for button presses [spd-, spd+, dist-, dist+]
bool btn_pressed_state[4] = {false, false, false, false}; // Current pressed state
bool btn_long_press_activated[4] = {false, false, false, false}; // If long press action (5kph jump) was done

bool cruise_mode_button_last_state = false; // Track previous state of the mode button
bool cruise_mode_button_changed = false; // Flag to debounce mode button

// ADC thresholds for steering wheel buttons (Needs Calibration for Pico 3.3V/12-bit ADC!)
// These are placeholders based on STM32 12-bit ADC values, likely need significant adjustment.
#define ADC_THR_SPD_UP_LOW   3000 // Placeholder value ~2.4V
#define ADC_THR_SPD_UP_HIGH  3500 // Placeholder value ~2.8V
#define ADC_THR_SPD_DN_LOW   2000 // Placeholder value ~1.6V
#define ADC_THR_SPD_DN_HIGH  2500 // Placeholder value ~2.0V
#define ADC_THR_DST_UP_LOW   1000 // Placeholder value ~0.8V
#define ADC_THR_DST_UP_HIGH  1500 // Placeholder value ~1.2V
#define ADC_THR_DST_DN_LOW    300 // Placeholder value ~0.2V
#define ADC_THR_DST_DN_HIGH   800 // Placeholder value ~0.6V
#define ADC_BUTTON_NONE_THR  3800 // Value when no button is pressed (adjust based on pull-up/down)

#define BTN_IDX_SPD_DN 0
#define BTN_IDX_SPD_UP 1
#define BTN_IDX_DST_DN 2 // Placeholder index
#define BTN_IDX_DST_UP 3 // Placeholder index

#define LONG_PRESS_DELAY_MS 250 // Time in ms to trigger long press (5kph jump)

struct sched_msg messages[MAX_SCHED_MSGS];
uint32_t last_sent[MAX_SCHED_MSGS];
uint8_t scheduled_msg_count = 0; // Counter for scheduled messages

// CAN Receive

static struct can2040 car_bus, acc_bus;

static void on_car_bus(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());

    // --- Handle Messages Relevant to Cruise Control (from STM32 code) ---
    if (msg->id == 0x001) { // Assuming 0x001 contains vehicle speed (verify byte/format)
        // ignition = 1; // Ignition status now comes from GPIO PIN_IGNITION
        current_speed_kph = msg->data[0]; // Example: Assuming speed is in data[0] in KPH. VERIFY THIS!
        if (!cruise_on) {
            // If cruise is off, keep set_speed aligned with current speed
            // This differs slightly from STM32 code which set it on button press.
            // This feels safer, preventing sudden jumps if cruise is activated later.
            set_speed_kph = current_speed_kph;
        }
    } else if (msg->id == 0x002) { // Assuming 0x002 is a cruise cancel message (e.g., brake pressed)
        if (cruise_on) {
            last_set_speed_kph = set_speed_kph; // Store the speed before cancelling
            cruise_on = false;
            printf("Cruise cancelled by CAN message 0x002\n");
        }
    }
    // --- End Cruise Control Message Handling ---

    else if (msg->id == 0xB0) { // Existing Wheel Speed Handling
        raw_wheel_speeds[0] = ((msg->data[0] << 8) | (msg->data[1] & 0xFF)) * 0.01; // Example conversion, verify accuracy
        raw_wheel_speeds[1] = ((msg->data[2] << 8) | (msg->data[3] & 0xFF)) * 0.01;
        wheel_speeds_buffer[0] = msg->data[0] + 0x1a; // Verify these offsets
        wheel_speeds_buffer[1] = msg->data[1] + 0x6f;
        wheel_speeds_buffer[2] = msg->data[2] + 0x1a;
        wheel_speeds_buffer[3] = msg->data[3] + 0x6f;
    } else if (msg->id == 0xb2) { // Existing Wheel Speed Handling
        raw_wheel_speeds[2] = ((msg->data[0] << 8) | (msg->data[1] & 0xFF)) * 0.01;
        raw_wheel_speeds[3] = ((msg->data[2] << 8) | (msg->data[3] & 0xFF)) * 0.01;
        wheel_speeds_buffer[4] = msg->data[0] + 0x1a;
        wheel_speeds_buffer[5] = msg->data[1] + 0x6f;
        wheel_speeds_buffer[6] = msg->data[2] + 0x1a;
        wheel_speeds_buffer[7] = msg->data[3] + 0x6f;
    }

    // Transmit allowed messages from car to ACC bus (power steering, etc.)
    if (is_allowed_message(msg->id) && can2040_check_transmit(&acc_bus)) {
        can2040_transmit(&acc_bus, msg);
    }

}

static void on_acc_bus(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    // Forward specific ACC messages to the main car bus
    if (msg->id == 0x260 || msg->id == 0x262 || msg->id == 0x394) {
        can2040_transmit(&car_bus, msg);
    }
}

static void PIOx_IRQHandler0(void) {
    can2040_pio_irq_handler(&car_bus);
}

static void PIOx_IRQHandler1(void) {
    can2040_pio_irq_handler(&acc_bus);
}

int get_checksum(uint8_t *msg, uint8_t len, uint16_t addr) {
    uint8_t checksum = 0;
    checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1; // Length byte itself is included (+1)
    for (int ii = 0; ii < len; ii++) {
        checksum += (msg[ii]);
    }
    return checksum;
}

void attach_checksum(uint16_t id, uint8_t len, uint8_t *msg) {
    // Checksum calculation uses 'len' bytes (0 to len-1), result stored at index 'len'
    msg[len] = get_checksum(msg, len, id);
}

// Modified schedule_msg to add to the array
bool schedule_msg(struct can2040 *dest, uint16_t id, uint8_t len, uint32_t freq_ms, uint8_t *buff, bool checksum) {
    if (scheduled_msg_count >= MAX_SCHED_MSGS) {
        // Array is full
        printf("Error: Cannot schedule message ID 0x%X, scheduler full.\n", id);
        return false;
    }

    messages[scheduled_msg_count].dest = dest;
    messages[scheduled_msg_count].id = id;
    messages[scheduled_msg_count].len = len; // Store the actual data length
    messages[scheduled_msg_count].freq_ms = freq_ms;
    messages[scheduled_msg_count].buff = buff;
    messages[scheduled_msg_count].checksum = checksum;
    messages[scheduled_msg_count].active = true; // Mark as active
    last_sent[scheduled_msg_count] = 0; // Initialize last sent time

    scheduled_msg_count++;
    return true;
}

// Modified check_send_msg to use freq_ms
void check_send_msg(uint8_t idx) {

    struct sched_msg *smsg = &messages[idx];
    // Only process active messages
    if (!smsg->active) {
        return;
    }

    uint32_t last = last_sent[idx];
    uint32_t cur_time = to_ms_since_boot(get_absolute_time());

    // Check if freq_ms milliseconds have passed
    if (cur_time - last >= smsg->freq_ms) {
        struct can2040_msg tmp;
        tmp.id = smsg->id;
        tmp.dlc = smsg->len; // Start with data length

        // Copy buffer content to the message data
        memcpy(tmp.data, smsg->buff, smsg->len);

        if (smsg->checksum) {
            // Pass the actual data length (smsg->len) for checksum calculation
            // The result is placed at index smsg->len
            if (tmp.dlc < 8) { // Ensure there's space for checksum byte
               attach_checksum(tmp.id, tmp.dlc, tmp.data);
               tmp.dlc++; // Increment DLC to include checksum byte
            } else {
                // Handle error: No space for checksum
                printf("Error: No space for checksum in msg ID 0x%X\n", tmp.id);
                // Optionally skip sending or handle differently
                return; // Don't attempt to send
            }
        }

        if (can2040_check_transmit(smsg->dest)) {
            can2040_transmit(smsg->dest, &tmp);
            last_sent[idx] = cur_time; // Update last sent time only on successful transmit check
        } else {
            // Optional: Handle buffer full case, maybe retry later or log
            // printf("CAN TX buffer full for msg ID 0x%X\n", tmp.id);
            // Consider if last_sent should be updated even if TX fails,
            // to prevent immediate retries overwhelming the buffer.
            // For now, it only updates on success.
        }
    }
}

bool is_allowed_message(uint16_t message_id) {
    // Allow diagnostic messages and explicitly listed messages
    if (message_id >= 0x700) { // Standard diagnostic range start
        return true;
    }
    for (uint8_t i = 0; i < ALLOWED_MESSAGES_COUNT; i++) {
        if (message_id == ALLOWED_MESSAGES[i]) {
            return true;
        }
    }
    return false;
}

// --- Steering Angle Sensor Function ---
// Helper function to calculate even parity for AS5048A
uint16_t add_parity_bit(uint16_t command) {
    int count = 0;
    for (int i = 0; i < 15; i++) { // Check bits 0-14
        if ((command >> i) & 1) {
            count++;
        }
    }
    if (count % 2 == 0) { // If even number of 1s
        return command; // Parity bit is 0 (already is)
    } else {
        return command | (1 << 15); // Set parity bit (bit 15)
    }
}

// Read raw angle from AS5048A
uint16_t get_raw_rotation() {
    uint16_t tx_buf[1];
    uint16_t rx_buf[1];

    // Command to read angle (0x3FFF), add READ flag (bit 14 = 1 -> 0x4000)
    tx_buf[0] = add_parity_bit(CMD_READ_ANGLE_REG | 0x4000);

    gpio_put(PIN_SPI_CS, 0); // Assert CS
    sleep_us(1); // Short delay after CS assert

    spi_write16_read16_blocking(SPI_PORT, tx_buf, rx_buf, 1);

    sleep_us(1); // Short delay before CS deassert
    gpio_put(PIN_SPI_CS, 1); // Deassert CS
    sleep_us(1); // Short delay after transaction

    // Send NOP command to allow sensor time to prepare angle data for next read
    tx_buf[0] = add_parity_bit(CMD_NOP | 0x4000); // NOP is 0x0000, add READ flag
    gpio_put(PIN_SPI_CS, 0);
    sleep_us(1);
    spi_write16_read16_blocking(SPI_PORT, tx_buf, rx_buf, 1); // Read previous result
    sleep_us(1);
    gpio_put(PIN_SPI_CS, 1);
    sleep_us(1);

    // Check for error flag (bit 14)
    if (rx_buf[0] & 0x4000) {
        // Error occurred, handle it (e.g., log, return error code, try reading error register)
        printf("AS5048A Error Flag Set!\n");
        // Optionally read error register:
        // tx_buf[0] = add_parity_bit(CMD_READ_ERROR_REG | 0x4000);
        // gpio_put(PIN_SPI_CS, 0); ... spi_write16_read16 ... gpio_put(PIN_SPI_CS, 1);
        // printf("Error Register: 0x%04X\n", rx_buf[0] & 0x3FFF);
        return last_position; // Return last known good position on error, maybe signal error differently
    }

    return rx_buf[0] & 0x3FFF; // Mask out parity and error bits (keep lower 14 bits)
}

// --- CAN Bus Setup ---
void canbus_setup(void) {
    uint32_t sys_clock = 125000000, bitrate = 500000;

    // Setup canbus
    can2040_setup(&car_bus, 0);
    can2040_setup(&acc_bus, 1);
    can2040_callback_config(&car_bus, on_car_bus);
    can2040_callback_config(&acc_bus, on_acc_bus);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler0);
    NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
    NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

    irq_set_exclusive_handler(PIO1_IRQ_0_IRQn, PIOx_IRQHandler1);
    NVIC_SetPriority(PIO1_IRQ_0_IRQn, 1);
    NVIC_EnableIRQ(PIO1_IRQ_0_IRQn);

    // Start canbus
    can2040_start(&car_bus, sys_clock, bitrate, 4, 5);
    can2040_start(&acc_bus, sys_clock, bitrate, 6, 7); // Using GP6/7 for ACC CAN
}

// --- GPIO and SPI Initialization ---
void peripherals_init() {
    // --- SPI Init ---
    spi_init(SPI_PORT, 1000 * 1000); // Initialize SPI at 1 MHz
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so initialize high
    gpio_init(PIN_SPI_CS);
    gpio_set_dir(PIN_SPI_CS, GPIO_OUT);
    gpio_put(PIN_SPI_CS, 1);

    // --- Ignition Pin Init ---
    gpio_init(PIN_IGNITION);
    gpio_set_dir(PIN_IGNITION, GPIO_IN);
    gpio_pull_down(PIN_IGNITION); // Pull down, assume high means ignition ON

    // --- Signal Light Pins Init ---
    gpio_init(PIN_SIG_LIGHT_L);
    gpio_set_dir(PIN_SIG_LIGHT_L, GPIO_IN);
    gpio_pull_up(PIN_SIG_LIGHT_L); // Pull up, signal is active LOW

    gpio_init(PIN_SIG_LIGHT_R);
    gpio_set_dir(PIN_SIG_LIGHT_R, GPIO_IN);
    gpio_pull_up(PIN_SIG_LIGHT_R); // Pull up, signal is active LOW

    // --- Cruise Mode Button Pin Init ---
    gpio_init(PIN_CRUISE_MODE);
    gpio_set_dir(PIN_CRUISE_MODE, GPIO_IN);
    gpio_pull_up(PIN_CRUISE_MODE); // Pull up, assume button press pulls LOW (Verify this)

    // --- ADC Init for Steering Wheel Buttons ---
    adc_init();
    adc_gpio_init(PIN_STEERING_BTNS); // Initialize the ADC pin
    // adc_select_input(0); // Select ADC input 0 (GP26) - Done before each read

    // --- Onboard LED (Optional, for status - Using GP25 as before) ---
    gpio_init(25); // Use GP25 for the LED
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0); // LED off initially
}

// Helper function to read ADC (assumes ADC0 = GP26)
uint16_t read_steering_buttons_adc() {
    adc_select_input(0); // Select ADC0 (GP26)
    return adc_read();   // Perform conversion and return 12-bit result
}


// --- Main ---
int main() {
    stdio_init_all();
    peripherals_init(); // Initialize GPIO, SPI, ADC

    // Initialize scheduler arrays
    memset(messages, 0, sizeof(messages));
    memset(last_sent, 0, sizeof(last_sent));
    scheduled_msg_count = 0;

    canbus_setup(); // Initialize CAN buses

    // Get initial steering position
    sleep_ms(10); // Small delay for sensor startup
    current_position = get_raw_rotation();
    last_position = current_position;
    rotational_position = 0; // Reset absolute position

    // Schedule messages - 12 Hz corresponds to ~83ms interval.
    uint32_t freq_83ms = 83; // 1000ms / 12Hz

    // Schedule messages with frequency in milliseconds
    // For checksummed messages, 'len' is the data length *before* checksum.
    // Schedule existing messages
    schedule_msg(&acc_bus, 0x25, 7, freq_83ms, angle_buffer, true);
    schedule_msg(&acc_bus, 0xAA, 8, freq_83ms, wheel_speeds_buffer, false);
    schedule_msg(&acc_bus, 0x1C4, 8, freq_83ms, msg15_buffer, false);
    schedule_msg(&acc_bus, 0x48B, 8, freq_83ms, msg12_buffer, false);
    schedule_msg(&car_bus, 0x3BC, 8, freq_83ms, gear_msg_buffer, false);

    // Schedule new messages from STM32 code (adjust frequencies as needed)
    // Assuming 50ms interval (20Hz) for cruise messages, 100ms (10Hz) for signals
    schedule_msg(&car_bus, 0x1D2, 7, 50, pcm_cruise_msg, true); // PCM_CRUISE (7 data + checksum)
    schedule_msg(&car_bus, 0x1D3, 7, 50, pcm_cruise_2_msg, true); // PCM_CRUISE_2 (7 data + checksum)
    schedule_msg(&acc_bus, 0x614, 2, 100, signal_lights_msg, false); // Signal Lights (2 data bytes)

    printf("Scheduler initialized with %d messages.\n", scheduled_msg_count);
    printf("Steering sensor initialized.\n");
    printf("Cruise control logic initialized.\n");

    while (true) {
        uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());

        // --- Read Inputs ---
        // Read ignition state directly from the GPIO pin
        // Assuming PIN_IGNITION is HIGH when ignition is ON
        ignition = gpio_get(PIN_IGNITION);

        // --- Steering Angle Calculation (Existing) ---
        current_position = get_raw_rotation();
        bitshift_cur_pos = current_position << 2;
        bitshift_last_pos = last_position << 2;
        bitshift_pos_delta = (bitshift_cur_pos - bitshift_last_pos); // Calculate the encoder "ticks" moved since the last reading - in bitshift math
        position_movement = bitshift_pos_delta >> 2; // Calculate the encoder "ticks" moved since the last reading - converting bitshift math
        rotational_position += position_movement; // Update the absolute position values.
        last_position = current_position;

        // Sanity check
        if (rotational_position > STEERING_ANGLE_MAX_ABS_TICKS || rotational_position < -STEERING_ANGLE_MAX_ABS_TICKS) {
             // SET_ERROR(2); // Error handling needs definition/implementation
             printf("Warning: Steering angle out of sane range (%ld)\n", rotational_position);
        } else {
             // CLEAR_ERROR(2); // Error handling needs definition/implementation
        }

        // --- Update CAN Message Buffers & Handle Logic ---
        if (ignition == 1) {
            gpio_put(25, 1); // Turn on LED (GP25) when ignition is on

            // --- Steering Angle Update (Existing) ---
            // Update the angle_buffer for the scheduled message (0x25)
            // Assuming the format for 0x25 requires the angle in specific bytes.
            // Checksum will be added by check_send_msg.
            // Need to confirm the exact format required for message 0x25.
            // Sending the raw rotational_position split into bytes.
            int32_t angle_to_send = rotational_position; // Use the calculated absolute position
            angle_buffer[0] = (angle_to_send >> 24) & 0xFF; // MSB
            angle_buffer[1] = (angle_to_send >> 16) & 0xFF;
            angle_buffer[2] = (angle_to_send >> 8) & 0xFF;
            angle_buffer[3] = angle_to_send & 0xFF;        // LSB
            // Bytes 4-6 are often status/unused in Toyota angle messages - keeping them 0
            angle_buffer[4] = 0x00;
            angle_buffer[5] = 0x00;
            angle_buffer[6] = 0x00;
            angle_buffer[6] = 0x00;
            // Byte 7 is checksum, handled by scheduler

            // --- Steering Wheel Button Logic (New) ---
            uint16_t adc_value = read_steering_buttons_adc();

            // Map ADC value to button index (0: spd-, 1: spd+, 2: dist-, 3: dist+)
            int current_btn_idx = -1; // -1 means no button pressed
            if (adc_value >= ADC_THR_SPD_UP_LOW && adc_value <= ADC_THR_SPD_UP_HIGH) {
                current_btn_idx = BTN_IDX_SPD_UP;
            } else if (adc_value >= ADC_THR_SPD_DN_LOW && adc_value <= ADC_THR_SPD_DN_HIGH) {
                current_btn_idx = BTN_IDX_SPD_DN;
            } else if (adc_value >= ADC_THR_DST_UP_LOW && adc_value <= ADC_THR_DST_UP_HIGH) {
                current_btn_idx = BTN_IDX_DST_UP; // Add logic if needed
            } else if (adc_value >= ADC_THR_DST_DN_LOW && adc_value <= ADC_THR_DST_DN_HIGH) {
                current_btn_idx = BTN_IDX_DST_DN; // Add logic if needed
            }

            // Process button presses and releases
            for (int i = 0; i < 4; i++) {
                if (current_btn_idx == i) { // This button is currently pressed
                    if (!btn_pressed_state[i]) { // Rising edge (button just pressed)
                        btn_pressed_state[i] = true;
                        btn_press_start_time_ms[i] = current_time_ms;
                        btn_long_press_activated[i] = false; // Reset long press flag

                        // Initial press action for Speed Up/Down
                        if (i == BTN_IDX_SPD_UP) {
                            if (!cruise_on) { // Activate cruise if off
                                cruise_on = true;
                                set_speed_kph = current_speed_kph > 30 ? current_speed_kph : 30; // Set to current or min 30kph
                                printf("Cruise Activated (Spd+). Set: %d\n", set_speed_kph);
                            }
                            // Initial press for speed up does nothing else until release or long press
                        } else if (i == BTN_IDX_SPD_DN) {
                            // Initial press for speed down does nothing else until release or long press
                             if (!cruise_on) { // Activate cruise if off (SET function)
                                cruise_on = true;
                                set_speed_kph = current_speed_kph > 30 ? current_speed_kph : 30; // Set to current or min 30kph
                                printf("Cruise Activated (Spd-). Set: %d\n", set_speed_kph);
                            }
                        }
                        // Add initial press logic for distance buttons if needed

                    } else { // Button is held down
                        if (!btn_long_press_activated[i] && (current_time_ms - btn_press_start_time_ms[i] >= LONG_PRESS_DELAY_MS)) {
                            // Long press detected
                            btn_long_press_activated[i] = true;
                            if (cruise_on) {
                                if (i == BTN_IDX_SPD_UP) {
                                    set_speed_kph += 5; // Jump 5 kph
                                    printf("Cruise Long Press (Spd+). Set: %d\n", set_speed_kph);
                                } else if (i == BTN_IDX_SPD_DN) {
                                    set_speed_kph -= 5; // Jump 5 kph
                                    printf("Cruise Long Press (Spd-). Set: %d\n", set_speed_kph);
                                }
                                // Add long press logic for distance buttons if needed
                                // Reset timer to allow repeated long presses if held
                                btn_press_start_time_ms[i] = current_time_ms;
                                btn_long_press_activated[i] = false; // Allow next long press check
                            }
                        }
                    }
                } else { // This button is NOT currently pressed
                    if (btn_pressed_state[i]) { // Falling edge (button just released)
                        btn_pressed_state[i] = false;
                        if (!btn_long_press_activated[i]) { // Only trigger short press if long press didn't happen
                            // Short press action
                            if (cruise_on) {
                                if (i == BTN_IDX_SPD_UP) {
                                    set_speed_kph += 1; // Increment 1 kph
                                    printf("Cruise Short Press (Spd+). Set: %d\n", set_speed_kph);
                                } else if (i == BTN_IDX_SPD_DN) {
                                    set_speed_kph -= 1; // Decrement 1 kph
                                    printf("Cruise Short Press (Spd-). Set: %d\n", set_speed_kph);
                                }
                                // Add short press logic for distance buttons if needed
                            }
                        }
                        // Reset timer and flags
                        btn_press_start_time_ms[i] = 0;
                        btn_long_press_activated[i] = false;
                    }
                }
            }

            // Clamp set_speed (e.g., between 30 and 140 kph)
            if (set_speed_kph < 30) set_speed_kph = 30;
            if (set_speed_kph > 140) set_speed_kph = 140; // Adjust max speed as needed

            // --- Signal Light Logic (New) ---
            // Signal light is ON when GPIO is LOW (due to pull-up)
            signal_lights_msg[0] = gpio_get(PIN_SIG_LIGHT_L) ? 0x00 : 0xFF; // Left signal
            signal_lights_msg[1] = gpio_get(PIN_SIG_LIGHT_R) ? 0x00 : 0xFF; // Right signal

            // --- Cruise Mode Button Logic (New) ---
            bool mode_button_pressed = !gpio_get(PIN_CRUISE_MODE); // Pressed = LOW
            if (mode_button_pressed != cruise_mode_button_last_state) { // State changed
                 sleep_ms(10); // Simple debounce delay
                 mode_button_pressed = !gpio_get(PIN_CRUISE_MODE); // Read again after delay
                 if (mode_button_pressed != cruise_mode_button_last_state) { // Still different? Debounced change.
                    if (mode_button_pressed) { // Button was pressed (rising edge of press = falling edge of signal)
                        if (cruise_on) {
                            last_set_speed_kph = set_speed_kph;
                            cruise_on = false;
                            printf("Cruise Deactivated (Mode Button)\n");
                        } else {
                            cruise_on = true;
                            // Resume or Set? STM32 code toggled. Let's toggle.
                            // If we want RESUME, use last_set_speed_kph. For SET, use current_speed_kph.
                            // set_speed_kph = last_set_speed_kph; // RESUME
                            set_speed_kph = current_speed_kph > 30 ? current_speed_kph : 30; // SET
                            printf("Cruise Activated (Mode Button). Set: %d\n", set_speed_kph);
                        }
                    }
                    cruise_mode_button_last_state = mode_button_pressed;
                 }
            }


            // --- Update Cruise CAN Message Buffers (New) ---
            // 0x1D2 msg PCM_CRUISE
            pcm_cruise_msg[0] = (cruise_on << 5) & 0x20; // Cruise active bit
            pcm_cruise_msg[1] = 0x00; // Placeholder - Verify byte meanings
            pcm_cruise_msg[2] = 0x00; // Placeholder
            pcm_cruise_msg[3] = 0x00; // Placeholder
            pcm_cruise_msg[4] = 0x00; // Placeholder
            pcm_cruise_msg[5] = (cruise_on << 7) & 0x80; // Another cruise active bit? Verify.
            pcm_cruise_msg[6] = 0x00; // Placeholder
            // Byte 7 is checksum, added by scheduler

            // 0x1D3 msg PCM_CRUISE_2
            pcm_cruise_2_msg[0] = 0x00; // Placeholder - Verify byte meanings
            // Original STM code: ((0x001 << 7) & 0x80) | 0x28; -> This seems fixed? Let's use it.
            pcm_cruise_2_msg[1] = ((0x001 << 7) & 0x80) | 0x28;
            pcm_cruise_2_msg[2] = (uint8_t)set_speed_kph; // Set speed
            pcm_cruise_2_msg[3] = 0x00; // Placeholder
            pcm_cruise_2_msg[4] = 0x00; // Placeholder
            pcm_cruise_2_msg[5] = 0x00; // Placeholder
            pcm_cruise_2_msg[6] = 0x00; // Placeholder
            // Byte 7 is checksum, added by scheduler

        } else { // Ignition is OFF
            gpio_put(25, 0); // Turn off LED (GP25)

            // Reset cruise state
            if (cruise_on) {
                 last_set_speed_kph = set_speed_kph; // Save last speed
            }
            cruise_on = false;
            set_speed_kph = 30; // Reset to default/min speed
            current_speed_kph = 0; // Assume speed is 0

            // Reset button states
            for(int i=0; i<4; i++) {
                btn_pressed_state[i] = false;
                btn_press_start_time_ms[i] = 0;
                btn_long_press_activated[i] = false;
            }

            // Clear CAN buffers that should be off when ignition is off
            memset(pcm_cruise_msg, 0, sizeof(pcm_cruise_msg));
            memset(pcm_cruise_2_msg, 0, sizeof(pcm_cruise_2_msg));
            memset(signal_lights_msg, 0, sizeof(signal_lights_msg));
        }

        // --- Send Scheduled CAN Messages ---
        for (uint8_t i = 0; i < scheduled_msg_count; i++) {
            // Optionally skip sending cruise messages if ignition is off
            if (ignition == 0 && (messages[i].id == 0x1D2 || messages[i].id == 0x1D3)) {
                continue;
            }
            check_send_msg(i);
        }

        // --- Loop Delay ---
        sleep_ms(10); // Adjust delay as needed (was 5ms, increased slightly for ADC/GPIO reads)
    }
}
