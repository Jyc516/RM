//
// Created by JiangYC on 25-10-3.
//

#include "Motor.h"

float M3508Motor::normalize_angle(float angle) {
    while (angle >= 360.f) {
        angle -= 360.f;
    }
    while (angle < 0.f) {
        angle += 360.f;
    }
    return angle;
}

void M3508Motor::can_RxMsgCallback(const uint8_t rx_data[]) {
    last_ecd_angle = ecd_angle;
    uint16_t rx_ecd = rx_data[0] <<8 | rx_data[1];
    ecd_angle = linear_mapping(rx_ecd, rx_ecd_angle, res_ecd_angle);
    delta_ecd_angle = ecd_angle - last_ecd_angle;
    delta_angle = delta_ecd_angle * ratio;
    angle = normalize_angle(angle + delta_angle);

    int16_t rx_speed = rx_data[2] << 8 | rx_data[3];
    rotate_speed =linear_mapping(rx_speed, rx_rotate_speed, res_rotate_speed);

    int16_t rx_current = rx_data[4] << 8 | rx_data[5];
    current = linear_mapping(rx_current, rx_max_current, res_max_current);

    temperature = rx_data[6];
}