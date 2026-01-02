#include "sensors.h"

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//#include "araf.h"

union Sensors::Addr
{
    uint64_t i;
    DeviceAddress a;
};

Sensors::Sensors(uint8_t pin, uint8_t num) 
    : prevMs_(millis() - 10001)
    , num_(num)
    , state_(-128)
{
    onewire_ = new OneWire(pin);
    if (onewire_ == nullptr) {
        //digitalWrite(7, 1);
        exit(1);
    }
    sensors_ = new DallasTemperature(onewire_);
    if (sensors_ == nullptr) {
        //digitalWrite(8, 1);
        exit(1);
    }
    //sensors_->begin();
    sensors_->setResolution(12);
    addr_ = new Addr[num];
    if (addr_ == nullptr) {
        //digitalWrite(LED_BUILTIN, 1);
        exit(1);
    }
    temp_ = new float[num];
    if (temp_ == nullptr) {
        //digitalWrite(LED_BUILTIN, 1);
        exit(1);
    }
    for (uint8_t i = 0; i < num_; i++) {
        addr_[i].i = 0;
        temp_[i] = 0;
    }
    //addr_[0].i = 0xe45260f10a646128ull; //{0x28,0x61,0x64,0x0A,0xF1,0x60,0x52,0xE4}, //metal head (output)
    //addr_[1].i = 0x3e0000005d8d4128ull; //{0x28,0x41,0x8D,0x5D,0x00,0x00,0x00,0x3E}, //long cable (input)
    //addr_[2].i = 0xcf0000005da31628ull; //{0x28,0x16,0xA3,0x5D,0x00,0x00,0x00,0xCF}, //short cable (body)
    //filters_ = new AdaptiveRunningAverageFilter[num];
}

Sensors::~Sensors()
{
    delete sensors_;
    delete onewire_;
    delete []addr_;
    //delete []filters_;
}

void Sensors::begin()
{
    sensors_->begin();
    sensors_->requestTemperatures();
    state_ = -num_;
}

void Sensors::poll()
{
    unsigned long ms = millis();
    //just return if no sensors detected
    if (state_ == -128) {
        if (ms - prevMs_ >= 10000) {
            sensors_->begin();
            state_ = -127;
            prevMs_ = ms;
        }
        return;
    }

    //detect sensors count and do request for temperatures
    if (state_ == -127) {
        state_ = requestCount() ? 0 : -128;
        return;
    }

    if (state_ == 0) {
        sensors_->requestTemperatures();
        state_ = -num_; //negative function code means wait after the requestTemperatures
        prevMs_ = ms;
        return;
    }

    //negative function code means wait after the requestTemperatures() call
    if (state_ < 0) {
        if (ms - prevMs_ < 1000)
            return;
        state_ = -state_;
    }

    //positive function means read corresponding sensor temperature
    state_--;
    loadTemperature(state_);

    //after the last sensor func=0, which will cause requestTemperatures() call again
}

bool Sensors::isConnected() const
{
    bool ret = true;
    for (uint8_t i = 0; ret && i < num_; i++)
        ret = (addr_[i].i != 0);
    return ret;
}

bool Sensors::isConnected(uint8_t idx) const
{
    return (addr_[idx].i != 0);
}

uint64_t Sensors::address(uint8_t idx) const
{
    return (idx < num_) ? addr_[idx].i : 0;
}

void Sensors::setAddress(uint8_t idx, uint64_t addr)
{
    if (idx < num_)
        addr_[idx].i = addr;
}

bool Sensors::requestCount()
{
    if (sensors_->getDeviceCount() != 0)
        return true;
    //no sensors detected
    return false;
}

void Sensors::loadTemperature(uint8_t idx)
{
    //sensors autodetection
    if (addr_[idx].i == 0) {
        static uint8_t retry = 0;
        if (sensors_->getAddress(addr_[idx].a, idx)) {
            for (uint8_t i = 0; i < num_; i++) {
                if (i == idx)
                    continue;
                if (addr_[idx].i == addr_[i].i)
                    addr_[i].i = 0;
            }
            retry = 0;
        } else switch (retry) {
        case 0:
            retry = 10;
            break;
        case 1:
            state_ = -128; //reset onewire bus and repeat full sensors redetection
            retry = 0;
            break;
        default:
            retry--;
        }
        return;
    }
    float t = sensors_->getTempC(addr_[idx].a);
    if (t == DEVICE_DISCONNECTED_C)
        addr_[idx].i = 0;
    else
        temp_[idx] = t;
}

/*
void print(const DeviceAddress &addr) {
  Serial.print(addr[0], HEX);
  for (u8 i = 1; i < 8; i++) {
    Serial.print("-");
    if (addr[i] < 16)
      Serial.print('0');
    Serial.print(addr[i], HEX);
  }
}
*/
