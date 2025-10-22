//
// Created by JiangYC on 25-10-3.
//

#include "Motor.h"

#include <cmath>
#include <algorithm>

float M3508Motor::normalize_angle(float angle, bool has_direction) {
    if (!has_direction) {
        while (angle >= 360.f) {
            angle -= 360.f;
        }
        while (angle < 0.f) {
            angle += 360.f;
        }
    }
    else {
        while (angle >= 180.f) {
            angle -= 360.f;
        }
        while (angle < -180.f) {
            angle += 360.f;
        }
    }
    return angle;
}

void M3508Motor::read_RxMsg(const uint8_t rx_data[]) {
    last_ecd_angle = ecd_angle;
    uint16_t rx_ecd = rx_data[0] <<8 | rx_data[1];
    ecd_angle = linear_mapping(rx_ecd, rx_ecd_angle, res_ecd_angle);
    delta_ecd_angle = normalize_angle(ecd_angle - last_ecd_angle, 180);
    delta_angle = delta_ecd_angle / ratio;
    angle = normalize_angle(angle + delta_angle);

    int16_t rx_speed = rx_data[2] << 8 | rx_data[3];
    rotate_speed = linear_mapping(rx_speed, rx_rotate_speed, res_rotate_speed);
    rotate_speed /= ratio;

    int16_t rx_current = rx_data[4] << 8 | rx_data[5];
    current = linear_mapping(rx_current, rx_max_current, res_max_current);

    temperature = rx_data[6];
}

void M3508Motor::write_TxMsg(uint8_t tx_data[8]){
    int i = (id - 1) % 4 * 2;
    int16_t output = int16_t(output_intensity / 20 * 16384);
    tx_data[i] = output >> 8;
    tx_data[i+1] = output & 0xFF;
}

void M3508Motor::SetIntensity(float intensity){
    mode = TORQUE;

    output_intensity = intensity;
}

void M3508Motor::SetSpeed(float tgt_speed_, float ff_intensity_){
    mode = SPEED;

    tgt_speed = tgt_speed_;
    ff_intensity = ff_intensity_;
}

void M3508Motor::SetAngle(float tgt_angle_, float ff_speed_, float ff_intensity_){
    mode = POSITION_SPEED;

    tgt_angle = tgt_angle_;
    ff_intensity = ff_intensity_;
    ff_speed = ff_speed_;
}

void M3508Motor::calc_ff_intensity(){
    float angle_from_top = normalize_angle(angle - 174.f, true); // 0 deg为上方位置
    float torque = 0.5 * 9.8 * sin(angle_from_top);
    ff_intensity = linear_mapping(torque, 3.f, 8.f);
}

void M3508Motor::handle(){
    fdb_speed = rotate_speed;
    fdb_angle = angle;
    calc_ff_intensity();

    if (mode == TORQUE) {}
    else if (mode == SPEED) {
        output_intensity = ff_intensity + spid.calc(tgt_speed, fdb_speed);
    }
    else if (mode == POSITION_SPEED) {
        tgt_speed = ff_speed + ppid.calc(tgt_angle, fdb_angle);
        output_intensity = ff_intensity + spid.calc(tgt_speed, fdb_speed);
    }
}


M3508Motor motor(19.2, 6);