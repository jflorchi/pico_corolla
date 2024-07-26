#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "can2040.h"
#include "RP2040.h"

struct sched_msg {
    struct can2040 *dest;
    uint16_t id;
    uint8_t len;
    uint8_t *buff;
    uint8_t freq;
    bool checksum;
};

// Function Definitions

bool is_allowed_message(uint16_t message_id);

// Variables

uint8_t ANGLE[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t WHEEL_SPEEDS[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t GEAR_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0};
uint8_t STEERING_LEVER_MSG[2] = {0x00, 0x00};
uint8_t IGNITION_MSG[3] = {0x00, 0x00, 0x00};

uint8_t MSG15[8] = {0x05, 0xea, 0x1b, 0x08, 0x00, 0x00, 0xc0, 0x9f};
uint8_t MSG12[8] = {0x66, 0x06, 0x08, 0x0a, 0x02, 0x00, 0x00, 0x00};

uint16_t raw_wheel_speeds[4] = {0x00, 0x00, 0x00, 0x00};
uint16_t ALLOWED_MESSAGES[5] = {0xb4};

struct sched_msg messages[15];
uint32_t last_sent[15];

// CAN Receive

static struct can2040 car_bus, acc_bus;

static void on_car_bus(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    
    if (msg->id == 0xB0) {
        raw_wheel_speeds[0] = ((msg->data[0] << 8) | (msg->data[1] & 0xFF)) * 0.01;
        raw_wheel_speeds[1] = ((msg->data[2] << 8) | (msg->data[3] & 0xFF)) * 0.01;                    
        WHEEL_SPEEDS[0] = msg->data[0] + 0x1a;
        WHEEL_SPEEDS[1] = msg->data[1] + 0x6f;
        WHEEL_SPEEDS[2] = msg->data[2] + 0x1a;
        WHEEL_SPEEDS[3] = msg->data[3] + 0x6f;        
    } else if (msg->id == 0xb2) {
        raw_wheel_speeds[2] = ((msg->data[0] << 8) | (msg->data[1] & 0xFF)) * 0.01;
        raw_wheel_speeds[3] = ((msg->data[2] << 8) | (msg->data[3] & 0xFF)) * 0.01;
        WHEEL_SPEEDS[4] = msg->data[0] + 0x1a;
        WHEEL_SPEEDS[5] = msg->data[1] + 0x6f;
        WHEEL_SPEEDS[6] = msg->data[2] + 0x1a;
        WHEEL_SPEEDS[7] = msg->data[3] + 0x6f;
    } else if (msg->id == 0x224) {
        IGNITION_MSG[2] = (msg->data[0] & 0x20) >> 5;
    }

    // Transmit messages from car to power steering to keep it happy
    if (is_allowed_message(msg->id) && can2040_check_transmit(&acc_bus)) {
        can2040_transmit(&acc_bus, msg);
    }

}

static void on_acc_bus(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    
    if (msg->id == 0x260 || msg->id == 0x262 || msg->id == 0x394) {
        can2040_transmit(&car_bus, msg);
        if (msg->id == 0x262) {
            IGNITION_MSG[1] = (msg->data[3] & 0xFE) >> 1;
        }
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
    checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
    for (int ii = 0; ii < len; ii++) {
        checksum += (msg[ii]);
    }
    return checksum;
}

void attach_checksum(uint16_t id, uint8_t len, uint8_t *msg) {
    msg[len -1] = get_checksum(msg, len - 1, id);
}

void schedule_msg(struct can2040 *dest, uint16_t id, uint8_t len, uint8_t freq, uint8_t *buff, bool checksum) {
    struct sched_msg msg;
    msg.dest = dest;
    msg.id = id;
    msg.len = len;
    msg.freq = freq;
    msg.buff = buff;
    msg.checksum = checksum;
}

void check_send_msg(uint8_t idx) {
    
    struct sched_msg *smsg = &messages[idx];
    if (smsg->id == 0) {
        return;
    }

    uint32_t last = last_sent[idx];
    uint32_t cur_time = to_ms_since_boot(get_absolute_time());

    if (cur_time - last >= 1000.0 / smsg->freq) {
        struct can2040_msg tmp;
        tmp.dlc = smsg->len;
        tmp.id = smsg->id;
        for (size_t i = 0; i < smsg->len; i++) {
           tmp.data[i] = smsg->buff[i]; 
        }
        if (smsg->checksum) {
            attach_checksum(tmp.id, tmp.dlc - 1, tmp.data);
        }

        can2040_transmit(smsg->dest, &tmp);
        last_sent[idx] = to_ms_since_boot(get_absolute_time());
    }  

    
}

bool is_allowed_message(uint16_t message_id) {
    if (message_id >= 0x700) {
        return true;
    }
    for (uint8_t i = 0; i < 5; i++) {
        if (message_id == ALLOWED_MESSAGES[i]) {
            return true;
        }
    }
    return false;
}

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
    can2040_start(&acc_bus, sys_clock, bitrate, 6, 7);
}

int main() {
    stdio_init_all();
    canbus_setup();

    // schedule messages
    schedule_msg(&acc_bus, 0x25, 8, 84, ANGLE, true);
    schedule_msg(&acc_bus, 0xAA, 8, 84, WHEEL_SPEEDS, false);
    schedule_msg(&acc_bus, 0x1C4, 8, 84, MSG15, false);
    schedule_msg(&acc_bus, 0x48B, 8, 84, MSG12, false);
    
    schedule_msg(&car_bus, 0x3BC, 8, 84, GEAR_MSG, false);
    // won't need ignition_msg if everything runs on single board?
     
    while (true) {   
        
    }

}
