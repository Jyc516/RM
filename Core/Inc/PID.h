//
// Created by JiangYC on 2025/10/18.
//

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
    #endif

class PID {
private:
    constexpr float kp;
    constexpr float ki;
    constexpr float kd;

    constexpr float d_filter;  // 对d一阶线性滤波
    constexpr int i_iter;      // 循环链表大小
    constexpr float i_max;     // 限制误差积分大小
    float out_max;             // 限制输出大小，可被set_out_max()修改，0.f代表没有限制

    float ref;
    float fdb;
    float cur_err;
    float pre_err;
    float delta_err;
    float err_sum;

    struct ErrNode {
        float err;
        ErrNode *next;
    };
    ErrNode *err_head;          // 循环链表，用于更新最近i_iter个err

    static float clamp(const float val, const float min, const float max);
    void reset_err_queue();     // ref更新后重置err队列
    void update_err_queue();    // 计算新err并更新err队列
public:
    PID(float _kp, float _ki, float _kd, int _i_iter=5, float _out_max = 0.f, float _i_max=1.f, float _d_filter=0.1):
        kp(_kp),
        ki(_ki),
        kd(_kd),
        i_iter(_i_iter > 0? _i_iter: 5),
        i_max(_i_max > 0.f? _i_max: 1.f),
        d_filter(_d_filter > 0.f && _d_filter < 1.f? _d_filter: 0.1)
    {
        err_head = new ErrNode{0.f, nullptr};
        ErrNode *cur = err_head;
        for (int i=1; i < i_iter; ++i) {
            cur->next = new ErrNode{0.f, nullptr};
            cur = cur->next;
        }
        cur->next = err_head;
        err_sum = 0.f;

        set_out_max(_out_max);
    };
    ~PID() {
        ErrNode *cur = nullptr;
        for (int i=0; i < i_iter; ++i) {
            cur = err_head;
            err_head = err_head->next;
            delete cur;
        }
    }

    void set_out_max(float _out_max);           // 设置out_max（为虚弱状态预留更新接口）
    float calc(float _ref, float _fdb);
};

    #ifdef __cplusplus
    }
#endif

#endif //PID_H
