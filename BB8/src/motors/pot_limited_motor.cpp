#include "pot_limited_motor.h"
#include "motor.h"

PotLimitedMotor::PotLimitedMotor(Motor *targetMotor, uint8_t potmeterPin, ReachedLimit *leftLimit, ReachedLimit *rightLimit) {
    mTargetMotor = targetMotor;
    mPotmeterPin = potmeterPin;
    mLeftLimit = leftLimit;
    mRightLimit = rightLimit;
    mLastDirection = Motor::Direction::Left;
}

void PotLimitedMotor::setSpeed(Direction direction, uint8_t speed) {
    mLastDirection = direction;
    if (isLimited(direction, mLastPotmeterValue)) {
        mTargetMotor->stop();
    } else {
        mTargetMotor->setSpeed(direction, speed);
    }
}

void PotLimitedMotor::stop() {
    mTargetMotor->stop();
}

void PotLimitedMotor::updateLimit() {
    mLastPotmeterValue = analogRead(mPotmeterPin);
    if (isLimited(mLastDirection, mLastPotmeterValue)) {
        mTargetMotor->stop();
    }
}

void PotLimitedMotor::begin() {
    pinMode(mPotmeterPin, INPUT);
    mTargetMotor->begin();
}

bool PotLimitedMotor::isLimited(Motor::Direction direction, int value) {
    if (direction == Motor::Direction::Left) {
        return mLeftLimit->atLimit(value);
    } else {
        return mRightLimit->atLimit(value);
    }
}