#pragma once

#include "motor.h"

class Bts7960: public Motor {
    public:
        Bts7960(uint8_t enablePin, uint8_t leftDirectionPin, uint8_t rightDirectionPin);

        virtual void begin();
        virtual void setSpeed(Direction direction, uint8_t speed);
        virtual void stop();

        void setMaxSpeed(uint8_t maxSpeed);
        void setMinimumActivationSpeed(uint8_t minSpeed);
    private:
        uint8_t mEnablePin;
        uint8_t mLeftDirectionPin;
        uint8_t mRightDirectionPin;
        uint8_t mMaxSpeed;
        uint8_t mMinActivationSpeed;
};
