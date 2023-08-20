#include "bts7960.h"
#include "config.h"

Bts7960::Bts7960(uint8_t enablePin, uint8_t leftDirectionPin, uint8_t rightDirectionPin) {
    mEnablePin = enablePin;
    mLeftDirectionPin = leftDirectionPin;
    mRightDirectionPin = rightDirectionPin;

    mMaxSpeed = 255;
    mMinActivationSpeed = 0;
}

void Bts7960::setMaxSpeed(uint8_t maxSpeed) {
    mMaxSpeed = maxSpeed;
}

void Bts7960::setMinimumActivationSpeed(uint8_t minActivationSpeed) {
    mMinActivationSpeed = minActivationSpeed;
}

void Bts7960::begin() {
    pinMode(mEnablePin, OUTPUT);
    pinMode(mLeftDirectionPin, OUTPUT);
    pinMode(mRightDirectionPin, OUTPUT);

    stop();
}

void Bts7960::setSpeed(Bts7960::Direction direction, uint8_t speed) {
    uint8_t mappedSpeed = map(speed, 0, 255, 0, mMaxSpeed);
    if (mappedSpeed < mMinActivationSpeed) {
        stop();
        return;
    }

    digitalWrite(mEnablePin, HIGH);
    if (direction == Left) {
        analogWrite(mRightDirectionPin, 0);
        analogWrite(mLeftDirectionPin, mappedSpeed);
    } else {
        analogWrite(mLeftDirectionPin, 0);
        analogWrite(mRightDirectionPin, mappedSpeed);
    }
}

void Bts7960::stop() {
    digitalWrite(mEnablePin, LOW);
    analogWrite(mLeftDirectionPin, 0);
    analogWrite(mRightDirectionPin, 0);
}