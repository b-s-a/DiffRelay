#ifndef SENSORS_H
#define SENSORS_H
#pragma once

#include <stdint.h>

class OneWire;
class DallasTemperature;
class AdaptiveRunningAverageFilter;

class Sensors
{
public:
    Sensors(uint8_t pin, uint8_t num);
    ~Sensors();
    void begin();
    void poll();
    bool isConnected() const;
    float operator[](uint8_t idx) const { return temp_[idx]; }
    uint64_t address(uint8_t idx) const;
    void setAddress(uint8_t idx, uint64_t addr);
private:
    Sensors(const Sensors&);
    union Addr;    
    OneWire *onewire_;
    DallasTemperature *sensors_;
    Addr *addr_;
    float *temp_;
    //AdaptiveRunningAverageFilter *filters_;
    unsigned long prevMs_;
    uint8_t num_;
    int8_t state_;

    bool requestCount();
    void loadTemperature(uint8_t idx);
};

#endif /* SENSORS_H */
