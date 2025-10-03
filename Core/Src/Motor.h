//
// Created by JiangYC on 25-10-3.
//

#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

class M3508_Motor {
private:
    const float ratio;                  //电机减速比

    float angle = 0.f;                  // deg 输出端累计转动角度
    float delta_angle = 0.f;            // deg 输出端新转动角度
    float ecd_angle = 0.f;              // deg 当前编码器转动角度
    float last_ecd_angle = 0.f;         // deg 上次编码器转动角度
    float delta_ecd_angle = 0.f;        // deg 编码器端新转动角度
    float rotate_speed = 0.f;           // dps 反馈转子速度
    float current = 0.f;                // A 反馈转矩电流
    float temperature = 0.f;            // °C 反馈电机温度

    static constexpr uint16_t rx_ecd_angle_ = 8191;
    static constexpr float res_ecd_angle_ = 360.f;      // deg
    static constexpr int16_t rx_rotate_speed_ = 1;      // rpm
    static constexpr float res_rotate_speed_ = 60.f;    // dps
    static constexpr int16_t rx_max_current_ = 16384;
    static constexpr float res_max_current_ = 20.f;     // A

    template<class T1, class T2>
    T2 linear_mapping(T1 org, T1 org_max, T2 res_max) {
        return T2(org) / org_max * res_max;
    }

    float normalize_angle(float angle);
public:
    explicit M3508_Motor(float _ratio): ratio(_ratio) {};
    void can_RxMsgCallback(const uint8_t rx_data[8]);
};

#endif //MOTOR_H
