//
// Created by JiangYC on 25-10-3.
//

#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

#include "PID.h"

class M3508Motor {
private:
    const float ratio;                  //电机减速比
    const int id;                       // CAN ID 1-8

    float angle = 0.f;                  // deg 输出端累计转动角度
    float delta_angle = 0.f;            // deg 输出端新转动角度
    float ecd_angle = 0.f;              // deg 当前编码器转动角度
    float last_ecd_angle = 0.f;         // deg 上次编码器转动角度
    float delta_ecd_angle = 0.f;        // deg 编码器端新转动角度
    float rotate_speed = 0.f;           // dps 反馈转子速度
    float current = 0.f;                // A 反馈转矩电流
    float temperature = 0.f;            // °C 反馈电机温度

    static constexpr uint16_t rx_ecd_angle = 8191;
    static constexpr float res_ecd_angle = 360.f;     // deg
    static constexpr int16_t rx_rotate_speed = 1;     // rpm
    static constexpr float res_rotate_speed = 6.f;    // dps
    static constexpr int16_t rx_max_current = 16384;
    static constexpr float res_max_current = 20.f;    // A

    PID ppid; // position PID
    PID spid; // speed PID
    float tgt_angle, fdb_angle;
    float tgt_speed, fdb_speed;
    float output_intensity = 0.f;       // 扭矩输出接口
    float ff_speed = 0.f, ff_intensity = 0.f;

    enum control_mode {
        TORQUE,                 // 开环力矩控制
        SPEED,                  // 速度闭环控制
        POSITION_SPEED          // 位置速度双环控制
    };
    control_mode mode = TORQUE;

    template<class T1, class T2>
    static T2 linear_mapping(T1 org, T1 org_max, T2 res_max) {
        return T2(org) / org_max * res_max;
    }

    static float normalize_angle(float angle, bool has_direction=false);

    void calc_ff_intensity();
public:
    explicit M3508Motor(float _ratio, int _id):
        ratio(_ratio),
        ppid(0, 0, 0),
        spid(0, 0, 0, 5, 1.1f, 1, 0.1f),
        id(_id){};
    void read_RxMsg(const uint8_t rx_data[8]);
    void write_TxMsg(uint8_t tx_data[8]);

    void SetIntensity(float intensity);
    void SetSpeed(float tgt_speed_, float ff_intensity_=0.f);
    void SetAngle(float tgt_angle_, float ff_speed_=0.f, float ff_intensity_=0.f);
    void handle();
};

#endif //MOTOR_H
