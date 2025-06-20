#include <Arduino.h>
//#include <ModbusSlave.h>

#include "sensors.h"

#define PIN_LED_STARTED 7
#define PIN_LED_ERROR 8
#define PIN_TXEN 2
#define PIN_ONEWIRE PIN_A3
#define PIN_PUMP_RELAY 13

#define ERROR_NO_SENSOR (1 << 0)
#define ERROR_OVERHEAT  (1 << 1)

#define T_ON 40.0
#define T_OFF 20.0
#define T_OVH 100.0
#define DT_ON 15.0
#define DT_OFF 5.0

#define RELAY_ON LOW
#define RELAY_OFF HIGH

bool              pumpRelay;
uint8_t           error;
Sensors           sensors(PIN_ONEWIRE, 2);

void pollLeds();
void pollSensors();
void debugPrints();

void setup()
{
    error = 0;
    pinMode(PIN_LED_STARTED, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    pinMode(PIN_TXEN, OUTPUT);
    pinMode(PIN_PUMP_RELAY, OUTPUT);
    digitalWrite(PIN_LED_STARTED, HIGH);
    digitalWrite(PIN_LED_ERROR, LOW);
    digitalWrite(PIN_PUMP_RELAY, RELAY_OFF);
    Serial.begin(9600, SERIAL_8N1);
    sensors.begin();
}

void loop()
{
    delay(100);
    sensors.poll();
    pollSensors();
    pollLeds();
    debugPrints();
    if (error == 0)
        digitalWrite(PIN_LED_ERROR, LOW);
    else
        digitalWrite(PIN_LED_ERROR, HIGH);
}

void pollLeds()
{
    static bool ledStatus;
    if (pumpRelay) {
        auto ms = millis();
        static auto prevMs = ms;
        if (ms - prevMs < 500)
            return;
        prevMs = ms;
        ledStatus = !ledStatus;
    } else
        ledStatus = true;
    digitalWrite(PIN_LED_STARTED, ledStatus ? HIGH : LOW);
}

void pollSensors()
{
    unsigned long ms = millis();
    static unsigned long prevMs = ms;

    if (ms - prevMs < 1000)
        return;
    prevMs = ms;

    if (sensors.isConnected())
        error &= ~ERROR_NO_SENSOR;
    else
        error |= ERROR_NO_SENSOR;

    //Pump is STARTED when overheat (max temperature is above 100°) or
    //delta between sensors is above 20° and most heat sensor temperature is raised and it is greater than 60°
    //Pump is STOPPED when no overheat and most heat sensor temperature is lowered and
    //delta between sensors is below 5° or most heat sensor temperature is below 30°.

    static float t_max_avg;
    float t_max = fmaxf(sensors[0], sensors[1]);
    float dt_max = t_max - t_max_avg;
    t_max_avg = (t_max_avg + t_max) * 0.5;
    static float dt_max_avg;
    dt_max_avg = (dt_max_avg + dt_max)/2;

    bool overheat = (t_max >= T_OVH);
    if (overheat)
        error |= ERROR_OVERHEAT;
    else if (error & ERROR_OVERHEAT)
        error &= ~ERROR_OVERHEAT;

    float delta = (error & ERROR_NO_SENSOR) ? 0.0 : fabsf(sensors[0] - sensors[1]);
    bool start = overheat || (delta > DT_ON && dt_max_avg > 0.0 && t_max > T_ON);
    bool stop = !overheat && dt_max_avg <= 0.0 && (delta < DT_OFF || t_max < T_OFF);

    static int8_t counter; //delay between relay changes
    if (counter == 0) {
        if (start && !pumpRelay)
            counter = 5;
        else if (stop && pumpRelay)
            counter = 60;
        return;
    }
    counter--;
    if (counter == 0) {
        if (!pumpRelay && start)
            pumpRelay = true;
        else if (pumpRelay && stop)
            pumpRelay = false;
        digitalWrite(PIN_PUMP_RELAY, pumpRelay ? RELAY_ON : RELAY_OFF);
    }
}

void debugPrints()
{
    static uint8_t cnt;
    if (cnt > 0) {
        cnt--;
        return;
    }
    cnt = 10;
    Serial.print(F("t1="));
    Serial.print(sensors[0]);
    Serial.print(F(", t2="));
    Serial.print(sensors[1]);
    Serial.print(F(", run="));
    Serial.print(pumpRelay);
    Serial.print(F(", err="));
    Serial.print(error, 2);
    Serial.print('\n');
}
