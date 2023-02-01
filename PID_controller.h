#pragma once
#include "cubic.ver1.8.h"
#include <limits.h>

#define PID_CONTROLLER_LIBRARY_VERSION = 0.20;

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
    double target;
    double diff;
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
    double dt;

    /**
     * @brief コントローラのコンストラクタ
     *
     * @param encoder エンコーダーの参照。コンストラクタが呼び出される前にbegin()されている必要がある。（コンストラクタ呼び出し時に値を読み込む仕様のため）
     * @param motor モーターの参照。
     * @param capableDuty 出力最大Duty比（絶対値）。0~255の範囲で指定する。
     * @param Kp 比例ゲイン
     * @param Ki 積分ゲイン
     * @param Kd 微分ゲイン
     * @param target 目標回転速度。省略可能（デフォルトは0）
     * @param encoderDirection モーターに正のdutyを与えて回した際に、エンコーダーの値がプラスになるかどうか。省略可能（デフォルトはtrue）
     * @param PPR エンコーダのPPR。省略可能（デフォルトは-1) -1は未設定を示します。
     */
    PID_controller(Cubic_encoder &encoder, Cubic_motor &motor, int capableDuty, double Kp, double Ki, double Kd, double target = 0, bool encoderDirection = true, int PPR = -1);

    /**
     * @brief 制御量（モーターのduty比）の計算を行う。loop内で呼び出すことを想定している。
     *
     * @param ifPut 関数中でmotor->put()するかどうか。省略可能（デフォルトはtrue）
     * @param ifPrint 関数中で情報をSerial.print()するかどうか。省略可能（デフォルトはfalse）主にデバッグ時の使用を想定している。
     * @return int 計算されたduty比を返す。
     */
    virtual int compute(bool ifPut = true, bool ifPrint = false){}

    /**
     * @brief ゲインを変更する。
     *
     * @param Kp 比例ゲイン
     * @param Ki 積分ゲイン
     * @param Kd 微分ゲイン
     */
    void setGains(double Kp, double Ki, double Kd);

    /**
     * @brief 目標を変更する。
     *
     * @param target 目標
     */
    void setTarget(double target);

    /**
     * @brief 目標を取得する。
     *
     * @return double 目標
     */
    double getTarget() const;


    /**
     * @brief エンコーダの分解能を設定する
     *
     * @param PPR 分解能
     */
    void setPPR(int PPR);

    /**
     * @brief エンコーダの分解能を取得する
     *
     * @return int
     */
    int getPPR() const;

    /**
     * @brief エンコーダの値を取得する
     *
     * @return int
     */
    int32_t getEncoderVal();
    /**
     * @brief 前ループにおけるエンコーダの値を取得する
     *
     * @return int
     */
    int32_t getPreEncoderVal() const;

    int32_t setPreEncoderVal(int32_t val);

    double setDiff(double diff);

    /**
     * @brief Duty比の取得
     * @details この関数は、compute()によって計算されるduty比を取得するのに使用する。この関数内では、計算は行われない。基本的にこの関数を使用しなければならない場面は、マルチスレッドでもない限り想定されない。
     *
     * @return int duty比
     */
    int getDuty() const;


    int compute_PID(bool ifPut, bool ifPrint);
};

class PID_velocity_controller : public PID_controller
{
    public:
    using PID_controller::PID_controller;

    int compute(bool ifPut = true, bool ifPrint = false) override;

    /**
     * @brief 目標とする回転速度を、秒間回転数で指定する。
     *
     * @param target 回転速度（秒間回転数）
     * @return int PPRが未設定のとき-1が返り、目標回転速度は変更されない。
     */
    int setTargetRotationPerSecond(double target);

};

class PID_position_controller : public PID_controller
{
    public:
    using PID_controller::PID_controller;
    int compute(bool ifPut = true, bool ifPrint = false) override;
};


inline void PID_controller::setGains(const double Kp, const double Ki, const double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}
inline void PID_controller::setTarget(const double target)
{
    this->target = target;
}
inline int PID_velocity_controller::setTargetRotationPerSecond(const double target)
{
    int PPR=this->getPPR();
    if (PPR == -1)
        return -1;
    this->setTarget(target * PPR);
    return 0;
}
inline void PID_controller::setPPR(const int PPR)
{
    this->PPR = PPR;
}
inline int PID_controller::getPPR() const
{
    return this->PPR;
}
inline int32_t PID_controller::getEncoderVal()
{
    encoder >> encoderVal;
    return encoderVal;
}
inline int32_t PID_controller::getPreEncoderVal() const
{
    return preEncoderVal;
}
inline int32_t PID_controller::setPreEncoderVal(const int32_t val)
{
    preEncoderVal = val;
    return preEncoderVal;
}
inline double PID_controller::setDiff(const double diff)
{
    this->diff = diff;
    return this->diff;
}
inline double PID_controller::getTarget() const
{
    return this->target;
}
inline int PID_controller::getDuty() const
{
    return this->duty;
}
inline int PID_controller::dutyLimiter()
{
    if(abs(duty)>capableDuty)
        integral = 0; // Anti-windup
    duty = duty > capableDuty    ? capableDuty
           : duty < -capableDuty ? -capableDuty
                                 : duty;
    return duty;
}