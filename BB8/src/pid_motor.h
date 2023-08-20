#pragma once

#include <PID_v1.h>

class Motor;

class PidMotor {
    public:
        enum Direction {
            Direct,
            Reverse    
        };

        PidMotor(Motor *motor, double Kp, double Ki, double Kd, Direction direction);

        void setSetpoint(double setpoint);
        double getSetpoint();
        void loop(double input);
    private:
        Motor *mMotor;
        PID *mPID;

        double mSetpoint;
        double mInput;
        double mOutput;
};
