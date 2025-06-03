#include <Arduino.h>
#include <DallasTemperature.h>
#include <GyverWDT.h>
//#include <ModbusSlave.h>
#include <OneWire.h>

#define PIN_LED_STARTED 7
#define PIN_LED_ERROR 8
#define PIN_TXEN 2
#define PIN_ONEWIRE PIN_A3
#define PIN_PUMP_RELAY 13

#define ERROR_NO_SENSOR (1 << 0)
#define ERROR_OVERHEAT  (1 << 1)

#define T_ON 60.0
#define T_OFF 30.0
#define DT_ON 20.0
#define DT_OFF 2.0

bool              pumpRelay;
uint8_t           error;
float             t[2];
OneWire           oneWire;
DallasTemperature sensors(&oneWire);

void pollSensors();
void pollSerial(); //Use started led to indicate error

void setup()
{
    pinMode(PIN_LED_STARTED, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    pinMode(PIN_TXEN, OUTPUT);
    pinMode(PIN_PUMP_RELAY, OUTPUT);
    digitalWrite(PIN_LED_STARTED, HIGH);
    digitalWrite(PIN_LED_ERROR, LOW);
    digitalWrite(PIN_PUMP_RELAY, LOW);
    Watchdog.enable(RESET_MODE, WDT_TIMEOUT_8S);
    Serial.begin(9600, SERIAL_8N1);
    oneWire.begin(PIN_ONEWIRE);
    sensors.begin();
    error = 0;
}

void loop()
{
    delay(100);
    digitalWrite(PIN_LED_ERROR, HIGH);
    pollSensors();
    pollSerial();
    Watchdog.reset();
    if (error == 0)
      digitalWrite(PIN_LED_ERROR, LOW);
}

void pollSensors()
{
    static uint8_t mode = 2;
    static unsigned long prevMs;
    unsigned long ms = millis();
    if (mode == 2) {
        sensors.requestTemperatures(); 
        mode = 0;
        prevMs = ms;
        return;
    }
    if (ms - prevMs < (mode ? 200 : 600))
        return;
    prevMs = ms;
    float temp = sensors.getTempCByIndex(mode);
    if (temp == DEVICE_DISCONNECTED_C)
        error |= ERROR_NO_SENSOR;
    else {
        t[mode] = temp;
        error &= ~ERROR_NO_SENSOR;
    }

    mode++;
    if (mode < 2)
        return;

    mode = 0;
    float t_max = fmaxf(t[0], t[1]);
    bool overheat = (t_max >= 100.0);
    if (overheat)
        error |= ERROR_OVERHEAT;
    else if (error & ERROR_OVERHEAT)
        error &= ~ERROR_OVERHEAT;

    float delta = (error & ERROR_NO_SENSOR) ? 0.0 : fabsf(t[0] - t[1]);
    bool start = overheat || (delta > DT_ON && t_max > T_ON);
    bool stop = !overheat && (delta < DT_OFF || t_max < T_OFF);

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
        digitalWrite(PIN_PUMP_RELAY, pumpRelay ? HIGH : LOW);
    }
}

void pollSerial()
{
    static uint8_t cnt;
    if (cnt > 0) {
        cnt--;
        return;
    }
    cnt = 10;
    Serial.print(F("t1="));
    Serial.print(t[0]);
    Serial.print(F(", t2="));
    Serial.print(t[1]);
    Serial.print(F(", err="));
    Serial.print(error, 2);
    Serial.print('\n');
}