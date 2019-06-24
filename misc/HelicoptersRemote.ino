/*
 * Управление вертолетами Syma S111G(S107G) и S026
 * оригинал: https://github.com/infusion/Fritzing/tree/master/Syma-S107G
 *
 ************************************************************************
 Формат пакета от пульта управления для S107G/S111G:
    HHH TTT YYY PPP MMM SSSS\n
    HHH = 107
    TTT - Throttle          0..127
    YYY - Yaw               0..127
    PPP - Pitch             0..127
    MMM - Trim              0..127
    SSSS - контрольная сумма HHH + TTT + YYY + PPP + MMM
    
    для S026:
    HHH TTT YYY PP B MMM SSSS\n
    HHH = 026
    TTT - Throttle          0..127
    YYY - Yaw               0..63
    PP  - Pitch             0..16
    B   - Buttons           0..2
    MMM - Trim              0..31
    SSSS - контрольная сумма HHH + TTT + YYY + PP + B + MMM

 Пакет от Ардуино:
          ----------
    текущие параметры:
    107 TTT YYY PPP VVV SSSS\n - 20 байт
    или
    026 TTT YYY PP B VVV SSSS\n

    VVV - напряжение батареи 512 -> 5 вольт, 999 - максимум

    После установления соединения c пультом через BLE
    Ардуино с интервалом в 500мС посылает пакеты с состоянием.
    Пульт должен в ответ посылать пакет с подтверждением текущих параметров,
    отсутствие 4-х пакетов подряд воспринимается как потеря управления и 
    приводит к сбросу параметров движения, передаваемых вертолету.
 ***************************************************************************
 
  Формат пакета к S107:
  HH 0YYYYYYY 0PPPPPPP CTTTTTTT 0AAAAAAA F
  Y - Yaw 0-127, (0=left, 63=center, 127=right)
  P - Pitch 0-127, (0=backwards, 63=hold, 127=forward)
  T - Throttle 0-127, (63=50%, 127=100%)

  Timings:
  HH - Header:
      2ms HIGH
      2ms LOW
  32 bit Command:
      "0": 312us HIGH, 288us LOW
      "1": 312us HIGH, 688us LOW
  F - Footer:
      312us HIGH

  Формат пакета к S026:
     0     6 7    12 13  16 17  20 21   26
  HH TTTTTTT YYYYYY   BBBB   PPPP   MMMMM  F
  Y - Yaw 0-63, (0=left, 31=center, 63=right)
  P - Pitch 0-16, (0=backwards, 7=hold, 15=forward)
  T - Throttle 0-127, (63=50%, 127=100%)
  B - buttons 22-LEFT, 26-RIGHT
  M - TRIM 0-31
    
********************************************************* */
 
#include <TimerOne.h>

// IR Timing
// set pwm active on pin 3
#define SET_HIGH(t)    TCCR2A |= _BV(COM2B1); delayMicroseconds(t)
 // set pwm inactive on pin3
#define SET_LOW(t)     TCCR2A &= ~_BV(COM2B1); delayMicroseconds(t)

#define SET_LOW_FINAL()   TCCR2A &= ~_BV(COM2B1)

#define THR_107   128
#define THR_026   128
#define YAW_107   128
#define YAW_026   64
#define PITCH_107 128
#define PITCH_026 16
#define TRIM_107  128
#define TRIM_026  32

// Начальные значения параметров
int copter = 1;       // Тип: 1 - S111/107, 0 - S026
uint8_t Throttle = 0;
uint8_t YawHalf = 63;
uint8_t Yaw = YawHalf;
uint8_t PitchHalf = 63;
uint8_t Pitch = PitchHalf;
uint8_t TrimHalf = 63;
uint8_t Trim = TrimHalf;
uint8_t butt026 = 0;

uint8_t ThrottleMax = THR_107 - 1;
uint8_t YawMax = YAW_107 - 1;
uint8_t PitchMax = PITCH_107 - 1;
uint8_t TrimMax = TRIM_107 - 1;

// D7 - кнопка аварийного сброса
int buttReset = 7;
// D12 - вход индикации установленного соединения
// от HM-10: HIGH - connected, LOW - disconnected
int pinConnected = 12;


// Отправка команды на Syma S107G/S111G
void sendCommand(uint8_t ya, uint8_t pit, uint8_t thr, uint8_t tri) {
    uint8_t data[4];
    data[3] = ya;
    data[2] = pit;
    data[1] = thr;    // | 0x80; // | (channel << 7);
    data[0] = tri;

    // SEND HEADER
    SET_HIGH(2000);
    SET_LOW(2000);
    // SEND DATA
    for (int8_t j = 3; j >= 0; j--) {
      uint8_t b = 0x80;
      for (uint8_t i = 0; i < 8; i++) {
        if ((data[j] & b) == b) {
          SET_HIGH(312);
          SET_LOW(688);
        }
        else {
          SET_HIGH(312);
          SET_LOW(288);
        }
        b = b >> 1;
      }
    }
    // SEND FOOTER
    SET_HIGH(312);
    // LOW till the next interrupt kicks in
    SET_LOW_FINAL();
}

// Отправка команды на  Syma S026
void sendCommand026(uint8_t ya, uint8_t pit, uint8_t thr, uint8_t tri, uint8_t butt) {
    uint8_t data[5];
    data[0] = thr;
    data[1] = ya;
    data[2] = butt;
    data[3] = pit;
    data[4] = tri;

    // SEND HEADER
    SET_HIGH(2000);
    SET_LOW(2000);
    // SEND DATA
    for (uint8_t i = 0; i < 7; i++) { // Throttle - 7bit, MSB first
        if ((data[0] & 0x40) == 0) {  // 0
            SET_HIGH(300);
            SET_LOW(500);
        }
        else {                        // 1
            SET_HIGH(300);
            SET_LOW(900);
        }
    }

    for (uint8_t i = 0; i < 6; i++) { // Yaw - 6bit, MSB first
        if ((data[1] & 0x20) == 0) {  // 0
            SET_HIGH(300);
            SET_LOW(500);
        }
        else {                        // 1
            SET_HIGH(300);
            SET_LOW(900);
        }
    }

    for (uint8_t i = 0; i < 4; i++) { // Buttons - 4bit, MSB first
        if ((data[2] & 0x08) == 0) {  // 0
            SET_HIGH(300);
            SET_LOW(500);
        }
        else {                        // 1
            SET_HIGH(300);
            SET_LOW(900);
        }
    }

    for (uint8_t i = 0; i < 4; i++) { // Pitch - 4bit, MSB first
        if ((data[3] & 0x08) == 0) {  // 0
            SET_HIGH(300);
            SET_LOW(500);
        }
        else {                        // 1
            SET_HIGH(300);
            SET_LOW(900);
        }
    }

    for (uint8_t i = 0; i < 5; i++) { // Trim - 5bit, MSB first
        if ((data[4] & 0x10) == 0) {  // 0
            SET_HIGH(300);
            SET_LOW(500);
        }
        else {                        // 1
            SET_HIGH(300);
            SET_LOW(900);
        }
    }

    // SEND FOOTER
    SET_HIGH(300);
    // LOW till the next interrupt kicks in
    SET_LOW_FINAL();
}

#define battPin A0  // вход измерения напряжения батареи

char pack[24];

void sendState(void) {

    int batt = analogRead(battPin);
    if (batt > 999) {
        batt = 999;
    }

    int h = 0;
    if (copter == 0) {  // S026
        h = 26;
        sprintf(pack, "%03d%03d%03d%02d%1d%3d%04d\n", h, Throttle, Yaw, Pitch, butt026, batt,
          (h + Throttle + Yaw + Pitch + butt026 + batt));
    } else {  // S107G/S111G
        h = 107;
        sprintf(pack, "%03d%03d%03d%03d%3d%04d\n", h, Throttle, Yaw, Pitch, batt,
          (h + Throttle + Yaw + Pitch + batt));      
    }
    if (digitalRead(pinConnected) == HIGH) {
        Serial.print(pack);
    }
}

// Отправка команды по прерыванию от таймера - постоянно
void timerISR() {
    if (copter == 0) {
      sendCommand026(Yaw, Pitch, Throttle, Trim, butt026);
    }
    else {
      sendCommand(Yaw, Pitch, Throttle, Trim);
    }
}

// Смена типа вертолета, используется также для сброса параметров
void setCopter(uint8_t copt) {
   if (copt == 0) {
        ThrottleMax = THR_026 - 1;       
        YawMax = YAW_026 - 1;
        PitchMax = PITCH_026 - 1;
        TrimMax = TRIM_026 - 1;
    }
    else {
        ThrottleMax = THR_107 - 1;
        YawMax = YAW_107 - 1;
        PitchMax = PITCH_107 - 1;
        TrimMax = TRIM_107 - 1;      
    }
    
    copter = copt;
    Throttle = 0;
    YawHalf = YawMax / 2;
    Yaw = YawHalf;
    PitchHalf = PitchMax / 2;
    Pitch = PitchHalf;
    TrimHalf = TrimMax / 2;
    Trim = TrimHalf;
}

#define STAT_INTERVAL   500   // интервал отправки статуса, mS

unsigned long timeStat = 0;
unsigned long timeNow = 0;
int noCmdCounter = 0;


void setup() {
    Serial.begin(115200);
   
    // IR LEDS
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);

    // кнопка сброса 
    pinMode(buttReset,INPUT);
    digitalWrite(buttReset, HIGH);

    // HIGH - BLE connected
    pinMode(pinConnected, INPUT);

    // LED - на Ардуино
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    //setup interrupt interval: 180ms
    Timer1.initialize(180000);  //180000
    Timer1.attachInterrupt(timerISR);

    /*
    setup PWM: f=38Khz PWM=50%
      COM2A = 00: disconnect OC2A
      COM2B = 00: disconnect OC2B; to send signal set to 10: OC2B non-inverted
      WGM2 = 101: phase-correct PWM with OCRA as top
      CS2 = 000: no prescaling
    */
    TCCR2A = _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS20);

    // Timer value
    OCR2A = 8000 / 38;
    OCR2B = OCR2A / 2; // 50% duty cycle 

    timeNow = timeStat = millis();
    noCmdCounter = 0;
}


char stcmd[24];
uint8_t icmd = 0;
char sttt[64];

void loop() {

    uint16_t hh = 0, thr = 0, ya = 0, pit = 0, tri = 0, bt =  0, pib = 0;
    uint16_t sum = 0;
    char val[8];

    // Если нет соединения и параметры не соответствуют начальным - сброс
    if (digitalRead(pinConnected) == LOW) {
        if ((Throttle > 0) || (Pitch != PitchHalf) || 
            (Yaw != YawHalf) || (Trim != TrimHalf) ) {
            setCopter(copter);
        }
    } else {
    
        if (Serial.available() > 0) {      
            char ch = Serial.read();
            if ((ch != '\n') && (icmd < 20)) {
                stcmd[icmd++] = ch;
            } else {
                //Serial.print("rec: ");
                //Serial.println(stcmd);
                if (icmd >= 19) {  // нормальный пакет
                    strncpy(val, &stcmd[0], 3);
                    val[3] = '\0';
                    hh = atoi(val);

                    strncpy(val, &stcmd[3], 3);
                    val[3] = '\0';
                    thr = atoi(val);
                  
                    strncpy(val, &stcmd[6], 3);
                    val[3] = '\0';
                    ya = atoi(val);

                    strncpy(val, &stcmd[9], 3);
                    val[3] = '\0';
                    pib = atoi(val);
                    if (copter == 0) { // S026
                        strncpy(val, &stcmd[9], 2);
                        val[2] = '\0';
                        pit = atoi(val);
                        bt = (uint8_t)(stcmd[11] - 0x30);
                    } else {
                        pit = pib;
                        bt = 0;
                    }

                    strncpy(val, &stcmd[12], 3);
                    val[3] = '\0';
                    tri = atoi(val);

                    strncpy(val, &stcmd[15], 4);
                    val[4] = '\0';
                    sum = atoi(val);
                    
                    // контрольная сумма совпала и тип правильный
                    if ( (sum == (hh + thr + ya + pib + tri)) &&
                      ((hh == 107) || (hh == 111) || (hh == 26)) ) {
                        if ( ((hh == 107) || (hh == 111)) && (copter != 1) ) {
                            copter = 1; // сменился тип вертолета
                            setCopter(copter);                    
                        } else if ( (hh == 26) && (copter != 0) ) {
                            copter = 0; // сменился тип вертолета
                            setCopter(copter);                    
                        }                

                        //sprintf(sttt, "New: %d %d %d %d\n", thr, ya, pit, tri);
                        //Serial.print(sttt);
                        noInterrupts();
                        Throttle = (uint8_t)thr;
                        Yaw = (uint8_t)ya;
                        Pitch = (uint8_t)pit;
                        Trim = (uint8_t)tri;
                        if (copter == 0) {
                            butt026 = (uint8_t)bt;
                        }
                        interrupts();
                    }
                } // нормальный пакет
                                     
                stcmd[icmd] = '\0';
                icmd = 0;
                noCmdCounter = 0; // был принят пакет от пульта
                digitalWrite(13, LOW);         
            } // принят пакет 
        } else {  // нет байтов из BLE
            timeNow = millis();
            if ((timeNow - timeStat) > STAT_INTERVAL) {
                if (++noCmdCounter > 4) { // на 4 статуса не было ответа
                    digitalWrite(13, HIGH);
                    setCopter(copter);  // сбрасываем параметры движения
                    noCmdCounter = 0;  
                }
                
                sendState();
                timeStat = millis();
            }
        }
    } // есть соединение по BLE
}
