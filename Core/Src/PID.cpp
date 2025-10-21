//
// Created by JiangYC on 2025/10/18.
//

#include "PID.h"

float PID::clamp(const float val, const float min, const float max) {
    if (val > max) {
        return max;
    }
    if (val < min) {
        return min;
    }
    return val;
}

void PID::reset_err_queue() {
    for (int i=0; i < i_iter; ++i) {
        err_head->err = 0.f;
        err_head = err_head->next;
    }
    err_sum = 0.f;
}

void PID::update_err_queue() {
    err_sum = err_sum - err_head->err + cur_err;
    err_head->err = cur_err;
    err_head = err_head->next;
}

void PID::set_out_max(float _out_max) {
    out_max = _out_max > 0.f? _out_max: 0.f;
}

float PID::calc(float _ref, float _fdb) {
    ref = _ref;
    if (_fdb == fdb) {
        pre_err = cur_err;
        cur_err = ref - fdb;
        delta_err = d_filter * (cur_err - pre_err) + (1 - d_filter) * delta_err;
        update_err_queue();
    }
    else {
        fdb = _fdb;
        cur_err = ref - fdb;
        delta_err = 0;
        reset_err_queue();
        update_err_queue();
    }

    float out =  kp * cur_err + kd * delta_err + ki * clamp(err_sum / 5, -i_max, i_max);
    return out_max == 0.f? out: clamp(out, -out_max, out_max);
}