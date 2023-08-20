#pragma once

#include <Arduino.h>
#include "motor.h"

class ReachedLimit {
    public:
        virtual bool atLimit(int potValue) = 0;
};

class ReachedLimitAtMinimum: public ReachedLimit {
    public:
        ReachedLimitAtMinimum(int minimum) {
            mMinimum = minimum;
        }

        virtual bool atLimit(int potValue) { return potValue <= mMinimum; }
    private:
        int mMinimum;
};

class ReachedLimitAtMaximum: public ReachedLimit {
    public:
        ReachedLimitAtMaximum(int maximum) {
            mMaximum = maximum;
        };

        virtual bool atLimit(int potValue) { return potValue >= mMaximum; }
    private:
        int mMaximum;
};

class PotLimitedMotor: public Motor {
    public:
        PotLimitedMotor(Motor *targetMotor, uint8_t potmeterPin, ReachedLimit *leftLimit, ReachedLimit *rightLimit);

        virtual void begin();
        virtual void setSpeed(Direction direction, uint8_t speed);
        virtual void stop();

        void updateLimit();

    private:
        bool isLimited(Motor::Direction direction, int value);

        Motor *mTargetMotor;
        uint8_t mPotmeterPin;
        int mLastPotmeterValue;
        Motor::Direction mLastDirection;
        ReachedLimit *mLeftLimit;
        ReachedLimit *mRightLimit;
};