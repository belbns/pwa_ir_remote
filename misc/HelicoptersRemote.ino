/*
 * Управление вертолетами Syma S111G(S107G) и S026
 * оригинал: https://github.com/infusion/Fritzing/tree/master/Syma-S107G
 *
 *
 Формат пакета от пульта управления для S107G/S111G:
    HHHTTTYYYPPPMMMSSSS\n
    HHH = 107
    TTT - Throttle          0..127
    YYY - Yaw               0..127
    PPP - Pitch             0..127
    MMM - Trim              0..127
    SSS - контрольная сумма HHH + TTT + YYY + PPP + MMM (последние 4 знака)
    
    для S026:
    HHHTTTYYYPPBMMMSSSS\n
    HHH = 026
    TTT - Throttle          0..127
    YYY - Yaw               0..63
    PP  - Pitch             0..16
    B   - Buttons           0..2
    MMM - Trim              0..31
    SSS - контрольная сумма HHH + TTT + YYY + PP + B + MMM (последние 4 знака)

    запрос состояния:
    9990000000000000999\n

 *
 
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
    
*/
 
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

// Syma S107G parameters
uint8_t Throttle = 0; // 0-127
uint8_t Yaw = 63; // 0-127, center=63
uint8_t Pitch = 63; // 0-127, center=63
uint8_t Trim = 63; // 0-127, center=63
uint8_t butt026 = 0;  // кнопки 026

uint8_t ThrottleMax = THR_107 - 1;
uint8_t YawMax = YAW_107 - 1;
uint8_t PitchMax = PITCH_107 - 1;
uint8_t TrimMax = TRIM_107 - 1;

// кнопка сброса
int buttReset = 7;   // D7
int pinConnected = 12; // D12 - HIGH - connected

int copter = 1; // 1 - S111/107, 0 - S026

// Syma S107G send command
//void sendCommand(uint8_t yaw, uint8_t pitch, uint8_t throttle, uint8_t trimc, uint8_t channel) {
void sendCommand(uint8_t yaw, uint8_t pitch, uint8_t throttle, uint8_t trimc) {
    uint8_t data[4];

    data[3] = yaw;
    data[2] = pitch;
    data[1] = throttle;// | 0x80; // | (channel << 7);
    data[0] = trimc;

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

// Syma S026 send command
void sendCommand026(uint8_t yaw, uint8_t pitch, uint8_t throttle, uint8_t trimc, uint8_t butt) {
    uint8_t data[5];

    data[0] = throttle;
    data[1] = yaw;
    data[2] = butt;
    data[3] = pitch;
    data[4] = trimc;

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

void sendState(void) {
    char pack[24];
    int batt = analogRead(battPin);
    if (batt > 999) {
        batt = 999;
    }
    int h = 0;
    if (copter == 0) {
        h = 26;
        sprintf(pack, "%03d%03d%03d%02d%1d%3d%04d\n", h, Throttle, Yaw, Pitch, butt026, batt,
          (h + Throttle + Yaw + Pitch + butt026 + batt));
    } else {
        h = 107;
        sprintf(pack, "%03d%03d%03d%03d%3d%04d\n", h, Throttle, Yaw, Pitch, batt,
          (h + Throttle + Yaw + Pitch + batt));      
    }
    if (digitalRead(pinConnected) == HIGH) {
        Serial.print(pack);
    }
}

void timerISR() {
    if (copter == 0) {
      sendCommand026(Yaw, Pitch, Throttle, Trim, butt026);
    }
    else {
      sendCommand(Yaw, Pitch, Throttle, Trim);
    }
}

void setCopter(uint8_t copt) {
   if (copt == 0) {
        ThrottleMax = THR_026 - 1;       
        YawMax = YAW_026 - 1;
        Yaw = YAW_026 / 2 - 1;
        PitchMax = PITCH_026 - 1;
        Pitch = PITCH_026 / 2 - 1;
        TrimMax = TRIM_026 - 1;
        Trim = TRIM_026 / 2 - 1;
    }
    else {
        ThrottleMax = THR_107 - 1;
        YawMax = YAW_107 - 1;
        Yaw = YAW_107 / 2 - 1;
        PitchMax = PITCH_107 - 1;
        Pitch = PITCH_107 / 2 - 1;
        TrimMax = TRIM_107 - 1;      
        Trim = TRIM_107 / 2 - 1;
    }
    copter = copt;
    Throttle = 0;
}

#define STAT_INTERVAL   1000   // интервал отправки статуса, mS
#define NO_CMD_INTERVAL 4000  // останов при отсутствии команд, mS

unsigned long timeLastCmd = 0;
unsigned long timeStat = 0;
unsigned long timeNow = 0;



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

    // LED
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

    timeLastCmd = timeNow = timeStat = millis();
}


char stcmd[24];
uint8_t icmd = 0;
char sttt[64];

void loop() {
    
    int t = 0;
    bool stateReq = false;

    uint8_t thr = 0, ya = 0, pit = 0, tri = 0, bt =  0, pib = 0, sum = 0;
    char val[8];

    if ((digitalRead(pinConnected) == LOW) && (Throttle > 0)) {
        setCopter(copter);  
    }
    
    timeNow = millis();
    
    if (Serial.available() > 0) {
        char ch = Serial.read();

        if ((ch != '\n') && (icmd < 20)) {
            stcmd[icmd++] = ch;
        } else {
            //Serial.print("rec: ");
            //Serial.println(stcmd);
            if (icmd < 19) {  // плохой пакет
                //timeLastCmd -= STAT_INTERVAL; // отдаляем время последней команды
                                              // 4 потери - сброс
            } else {
                // HHH  
                strncpy(val, &stcmd[0], 3);
                val[3] = '\0';
                t = atoi(val);            
                if ((t == 107) || (t == 111)) {
                    if (copter != 1) {  // сменился тип вертолета
                        copter = 1; // S107/111
                        setCopter(copter);                    
                    }
                } else if (t == 26) {
                    if (copter != 0) {  // сменился тип вертолета
                        copter = 0; // S026
                        setCopter(copter);                    
                    }                
                } else {  // остальное интерпретируем как запрос состояния
                    stateReq = true;  
                }
                
                if (stateReq) {
                    sendState();
                    stateReq = false;
                    timeStat = millis();          
                } else {  // разбираем остальные параметры
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
                    }

                    strncpy(val, &stcmd[12], 3);
                    val[3] = '\0';
                    tri = atoi(val);

                    strncpy(val, &stcmd[15], 4);
                    val[4] = '\0';
                    sum = atoi(val);
                    
                    sprintf(sttt, "New: %d %d %d %d\n", thr, ya, pit, tri);
                    Serial.print(sttt);
                    noInterrupts();
                    Throttle = thr;
                    Yaw = ya;
                    Pitch = pit;
                    Trim = tri;
                    if (copter == 0) {
                        butt026 = bt;
                    }
                    interrupts();

/*
                    if (sum == (t + thr + ya + pib + tri)) {
                        noInterrupts();
                        Throttle = thr;
                        Yaw = ya;
                        Pitch = pit;
                        Trim = tri;
                        if (copter == 0) {
                            butt026 = bt;
                        }
                        interrupts();
                        timeLastCmd = millis();
                        digitalWrite(13, LOW);
                    }
*/
                }            
            }                         
            stcmd[icmd] = '\0';
            icmd = 0;         
        } 
    } else {  // нет данных от BLE
      /*
        if ((timeNow - timeLastCmd) > NO_CMD_INTERVAL) {
            // долго нет команд - останов
            setCopter(copter);
            digitalWrite(13, HIGH);
        }
    */
        if ((timeNow - timeStat) > STAT_INTERVAL) {
            sendState();
            timeStat = millis();
        }
    }
}
