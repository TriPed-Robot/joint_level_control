#include "joint_level_control/motor/hacker_const.h"


typedef enum State {
    OFF = 0x00,
    DETECTING = 0x01,
    RUNNING = 0x02,
    FULL_BRAKE = 0x03,
    RUMP = 0x04
} state_t;


typedef enum Fault {
    None = 0x00,
    Overvoltage = 0x01,
    Undervoltage = 0x02,
    Driver_Error = 0x03,
    Overcurrent = 0x04,
    Overtemperature_Board = 0x05,
    Overtemperature_Motor = 0x06
} fault_t;


typedef struct ControlResponse {
    int16_t battery_current;
    int16_t motor_current;
    uint16_t battery_voltage;
    uint16_t reserved_data;
    uint16_t ppm_adc;
    int16_t motor_temperature;
    int16_t erpm;
    int16_t rpm;
    uint16_t pwm_frequency;
    int16_t pwm_duty;
    int16_t pcb_temperature;
    state_t state;
    fault_t fault;
} controlresponse_t;


typedef struct ScanResponse {
    uint16_t can_address;
    uint8_t mode;
    uint32_t uid;
} scanresponse_t;
