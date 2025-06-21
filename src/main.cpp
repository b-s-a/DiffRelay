include <Arduino.h>
#include <LiquidCrystal_I2C.h>
//#include <ModbusSlave.h>

#include "sensors.h"

#define PIN_LED_STARTED 7
#define PIN_LED_ERROR 8
#define PIN_TXEN 2
#define PIN_ONEWIRE PIN_A3
#define PIN_PUMP_RELAY 11
#define PIN_PUMP_RELAY_I 12

#define ERROR_NO_SENSOR (1 << 0)
#define ERROR_OVERHEAT  (1 << 1)

#define T_ON 40.0
#define T_OFF 20.0
#define T_OVH 100.0
#define DT_ON 15.0
#define DT_OFF 5.0

bool              pumpRelay;
bool              overheat;
uint8_t           error;
Sensors           sensors(PIN_ONEWIRE, 2);
LiquidCrystal_I2C lcd(0x27, 16, 2);

static
const char customChars[8][8] PROGMEM = {
    { B00000, B00000, B10000, B10000, B11111, B00001, B00001, B00000 },
    { B00000, B00000, B00100, B01000, B00100, B00010, B00100, B00000 },
    { B00000, B00000, B00111, B00100, B00100, B00100, B11100, B00000 },
    { B00000, B00000, B00000, B00010, B10101, B01000, B00000, B00000 },
    { B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00100 }, //4 - ↑
    { B00100, B00100, B00100, B00100, B00100, B10101, B01110, B00100 }, //5 - ↓
};

void pollLeds();
void pollPump();
void pollLcd();
void setPump(bool enable, bool force = false);
void debugPrints();

void setup()
{
    error = 0;
    pinMode(PIN_LED_STARTED, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    pinMode(PIN_TXEN, OUTPUT);
    pinMode(PIN_PUMP_RELAY, OUTPUT);
    pinMode(PIN_PUMP_RELAY_I, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(PIN_LED_STARTED, HIGH);
    digitalWrite(PIN_LED_ERROR, LOW);
    setPump(false, true);
    Serial.begin(9600, SERIAL_8N1);
    sensors.begin();
    lcd.init();
    for (uint8_t i = 0; i < sizeof(customChars)/sizeof(customChars[0]); i++)
        lcd.createChar(i, customChars[i]);
}

void loop()
{
    delay(100);
    sensors.poll();
    pollPump();
    pollLeds();
    pollLcd();
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

void pollPump()
{
    unsigned long ms = millis();
    static unsigned long prevMs = ms + 100;

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

    overheat = (t_max >= T_OVH);
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
            setPump(true);
        else if (pumpRelay && stop)
            setPump(false);
    }
}

static void lcdPrintSensor(uint8_t idx)
{
    if (!sensors.isConnected(0)) {
        lcd.print(F("  n/c  "));
        return;
    }
    float t = sensors[0];
    if (t < 0) {
        t -=  0.005;
        if (t > -10.0)
            lcd.write(' ');
    } else {
        t += 0.005;
        if (t < 100) {
            lcd.write(' ');
            if (t < 10)
                lcd.write(' ');
        }
    }
    lcd.print(sensors[0], 2);
    lcd.write(223); // degree symbol
}

void pollLcd()
{
    unsigned long ms = millis();
    static unsigned long prevMs = ms + 200;

    if (ms - prevMs < 1000)
        return;
    prevMs = ms;

    lcd.noBlink();
    lcd.noCursor();
    lcd.noAutoscroll();
    lcd.backlight();
    lcd.leftToRight();

    // 0123456789abcdef
    // IDLE X
    // 000.00°  000.00°

    // WORK X
    // 000.00°  000.00°

    // WORK X  OVERHEAT
    // 000.00°  000.00°

    // --=== FAIL ===--
    //   n/c      n/c

    lcd.setCursor(0, 1);
    lcdPrintSensor(0);
    lcd.setCursor(9, 1);
    lcdPrintSensor(1);

    lcd.setCursor(0, 0);
    if (!sensors.isConnected()) {
        lcd.print(F("--=== FAIL ===--"));
        return;
    }

    lcd.print(pumpRelay ? F("WORK ") : F("IDLE  "));
    if (pumpRelay) {
        static uint8_t cnt;
        lcd.write(cnt++ % 4);
    }
    if (overheat)
        lcd.print(F("  OVERHEAT"));
    else {
        for (uint8_t i = 6; i <= 0x0f; i++)
            lcd.write(' ');
    }
}

void setPump(bool enable, bool force)
{
    if (!force && enable == pumpRelay)
        return;
    pumpRelay = enable;
    digitalWrite(LED_BUILTIN, enable ? HIGH : LOW);
    digitalWrite(PIN_PUMP_RELAY, enable ? HIGH : LOW);
    digitalWrite(PIN_PUMP_RELAY_I, enable ? LOW : HIGH);
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
