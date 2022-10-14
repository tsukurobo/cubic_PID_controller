#pragma once
#include "Cubic1.7.h"
#include <limits.h>
#include "MovingAverage.h"

#define PID_CONTROLLER_LIBRARY_VERSION = 0.10;

constexpr int MAX_DUTY = 255;
constexpr int MIN_DUTY = -255;
constexpr int limitInPermitedDutyRange(const int duty)
{
    return duty > MAX_DUTY   ? MAX_DUTY
           : duty < MIN_DUTY ? MIN_DUTY
                             : duty;
}
constexpr unsigned long MAX_MICROSECONDS = ULONG_MAX;

class PID_controller
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

    int dutyLimiter();

    int encoderVal;
    int preEncoderVal;

public:
    /**
     * @brief コントローラのコンストラクタ
     *
     * @param encoder エンコーダーの参照。コンストラクタが呼び出される前にbegin()されている必要がある。（コンストラクタ呼び出し時に値を読み込む仕様のため）
     * @param motor モーターの参照。
     * @param capableDuty 出力最大Duty比。0~255の範囲で指定する。
     * @param Kp 比例ゲイン
     * @param Ki 積分ゲイン
     * @param Kd 微分ゲイン
     * @param target 目標回転速度。省略可能（デフォルトは0）
     */
    PID_controller(Cubic_encoder &encoder, Cubic_motor &motor, int capableDuty, double Kp, double Ki, double Kd, double target = 0);

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
     * @brief 目標とする回転速度を変更する。
     *
     * @param target 回転速度（エンコーダーの分解能準拠）
     */
    void setTarget(int target);

    /**
     * @brief Duty比の取得
     * @details この関数は、compute()によって計算されるduty比を取得するのに使用する。この関数内では、計算は行われない。基本的にこの関数を使用しなければならない場面は、マルチスレッドでもない限り想定されない。
     *
     * @return int duty比
     */
    int getDuty() const;
};

inline void PID_controller::setGains(const double Kp, const double Ki, const double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}
inline void PID_controller::setTarget(const int target)
{
    this->target = target;
}
inline int PID_controller::getDuty() const
{
    return this->duty;
}
inline int PID_controller::dutyLimiter()
{
    return duty = duty > capableDuty    ? capableDuty
                  : duty < -capableDuty ? -capableDuty
                                        : duty;
}