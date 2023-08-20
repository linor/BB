#include "pid_motor.h"
#include "motors/motor.h"
#include "config.h"

PidMotor::PidMotor(Motor *motor, double Kp, double Ki, double Kd, Direction direction) {
    mPID = new PID(&mInput, &mOutput, &mSetpoint, Kp, Ki, Kd, direction == PidMotor::Direction::Direct ? DIRECT : REVERSE);
    mPID->SetMode(AUTOMATIC);
    mPID->SetOutputLimits(-255, 255);

    mInput = 0.0;
    mSetpoint = 0.0;
    mMotor = motor;

}

double PidMotor::getSetpoint() {
    return mSetpoint;
}

void PidMotor::setSetpoint(double setpoint) {
    mSetpoint = setpoint;
}

void PidMotor::loop(double input) {
    mInput = input;

    if (mPID->Compute()) {
        if (mOutput < 0) {
            mMotor->setSpeed(Motor::Direction::Left, -mOutput);
        } else if (mOutput > 0) {
            mMotor->setSpeed(Motor::Direction::Right, mOutput);
        } else {
            mMotor->stop();
        }
    }
}