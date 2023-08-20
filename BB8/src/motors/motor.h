#pragma once

#include <Arduino.h>

class Motor {
    public:
        enum Direction {
            Left,
            Right
        };

        virtual void begin() = 0;
        virtual void setSpeed(Direction direction, uint8_t speed) = 0;
        virtual void stop() = 0;
};
