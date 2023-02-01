#pragma once
#include "cubic.ver1.8.h"
#include <limits.h>
#include "PID_controller.h"


class PID_position_controller
{
private:
    double Kp;
    double Ki;
    double Kd;
    double dt;
    double target;
    Cubic_encoder &encoder;
    Cubic_motor &motor;

    double preDiff;
    long double integral;
    unsigned long preMicros;

    int duty = 0;
    int capableDuty;

    int dutyLimiter(); /* Limit the duty and reset integral if limited. */
    bool isCapped = false;

    int32_t encoderVal;
    int32_t preEncoderVal;

    int PPR;

    bool direction;

public:
    /**
     * @brief コントローラのコンストラクタ
     *
     * @param encoder エンコーダーの参照。コンストラクタが呼び出される前にbegin()されている必要がある。（コンストラクタ呼び出し時に値を読み込む仕様のため）
     * @param motor モーターの参照。
     * @param capableDuty 出力最大Duty比（絶対値）。0~255の範囲で指定する。
     * @param Kp 比例ゲイン
     * @param Ki 積分ゲイン
     * @param Kd 微分ゲイン
     * @param target 目標エンコーダ値。省略可能（デフォルトは0）
     * @param encoderDirection モーターに正のdutyを与えて回した際に、エンコーダーの値がプラスになるかどうか。省略可能（デフォルトはtrue）
     * @param PPR エンコーダのPPR。省略可能（デフォルトは-1) -1は未設定を示します。
     */
    PID_position_controller(Cubic_encoder &encoder, Cubic_motor &motor, int capableDuty, double Kp, double Ki, double Kd, double target = 0, bool encoderDirection = true, int PPR = -1);

    /**
     * @brief 制御量（モーターのduty比）の計算を行う。loop内で呼び出すことを想定している。
     *
     * @param ifPut 関数中でmotor->put()するかどうか。省略可能（デフォルトはtrue）
     * @param ifPrint 関数中で情報をSerial.print()するかどうか。省略可能（デフォルトはfalse）主にデバッグ時の使用を想定している。
     * @return int 計算されたduty比を返す。
     */
    int compute(bool ifPut = true, bool ifPrint = false);

    /**
     * @brief ゲインを変更する。
     *
     * @param Kp 比例ゲイン
     * @param Ki 積分ゲイン
     * @param Kd 微分ゲイン
     */
    void setGains(double Kp, double Ki, double Kd);

    /**
     * @brief 目標とする値を変更する。
     *
     * @param target 目標値
     */
    void setTarget(double target);

    /**
     * @brief エンコーダの分解能を設定する
     *
     * @param PPR 分解能
     */
    void setPPR(int PPR);

    /**
     * @brief Duty比の取得
     * @details この関数は、compute()によって計算されるduty比を取得するのに使用する。この関数内では、計算は行われない。基本的にこの関数を使用しなければならない場面は、マルチスレッドでもない限り想定されない。
     *
     * @return int duty比
     */
    int getDuty() const;
};

inline void PID_position_controller::setGains(const double Kp, const double Ki, const double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}
inline void PID_position_controller::setTarget(const double target)
{
    this->target = target;
}
inline void PID_position_controller::setPPR(const int PPR)
{
    this->PPR = PPR;
}
inline int PID_position_controller::getDuty() const
{
    return this->duty;
}
inline int PID_position_controller::dutyLimiter()
{
    if(abs(duty)>capableDuty)
        integral = 0; // Anti-windup
    duty = duty > capableDuty    ? capableDuty
           : duty < -capableDuty ? -capableDuty
                                 : duty;
    return duty;
}