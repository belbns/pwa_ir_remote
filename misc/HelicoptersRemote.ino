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
  A - Trim
  
  Timings:
  HH - Header:
      2ms HIGH
      2ms LOW
  32 bit Command:
      "0": 312us HIGH, 288us LOW
      "1": 312us HIGH, 688us LOW
  F - Footer:
      312us HIGH

  Формат пакета к S026 - вариант 1:
     0     6 7    12 13  16 17  20 21   26
  HH TTTTTTT YYYYYY   BBBB   PPPP   MMMMM  F
  T - Throttle 0-127, (63=50%, 127=100%)
  Y - Yaw 0-63, (0=left, 31=center, 63=right)
  P - Pitch 0-16, (0=backwards, 7=hold, 15=forward)
  B - buttons 22-LEFT, 26-RIGHT
  M - TRIM 0-31

  Формат пакета к S026 - вариант 2:
  https://www.rcgroups.com/forums/showpost.php?p=15007427&postcount=47
     0     6 7    12 13 14 15 16 17 20 21  26 27
  HH TTTTTTT YYYYYY  С1 LB RB C2  PPPP MMMMMM  0 F
  T - Throttle 0-127, (63=50%, 127=100%)
  Y - Yaw 0-63, (111111=left, 100000=neutral, 000001=right)
  P - Pitch 0-15, (0001=backwards, 1000=neutral, 1111=forward)
  LB, RB - trim buttons
  C1,C2 - channel (A=0,1 B=1,1 C=0,0)
  M - TRIM 0-63, (111111=left, 100000=neutral, 000001=right
  27 - always 0
********************************************************* */
 
#include <TimerOne.h>

#define STAT_INTERVAL         500     // интервал отправки статуса, mS
#define NO_ACTIVITY_INTERVAL  180000  // 3 min - отключение при отсутствии соединения по BLE
#define NO_COMMAND_INTERVAL   2000    // 2 sec - сброс параметров при отсутствии ответа по BLE

#define AFTER_COUNT 11  // 22 раза послать стоп после обнуления Throttle ~ 4 сек.

// IR Timing
// set pwm active on pin 3
#define SET_HIGH(t)    TCCR2A |= _BV(COM2B1); delayMicroseconds(t)
 // set pwm inactive on pin3
#define SET_LOW(t)     TCCR2A &= ~_BV(COM2B1); delayMicroseconds(t)

#define SET_LOW_FINAL()   TCCR2A &= ~_BV(COM2B1)
// длина ИК пакетов в интервалах
#define LEN107    67
#define LEN026    59

#define COPT107   1
#define COPT026   0

#define THR_107   128
#define THR_026   128
#define YAW_107   128
#define YAW_026   64
#define PITCH_107 128
#define PITCH_026 16
#define TRIM_107  128
#define TRIM_026  64  //V1 - 32
#define CH_107    0   // A
//#define CH_107    1   // B
// канал A
#define CH1_026 1
#define CH2_026 1
// канал B
//#define CH1_026 1
//#define CH2_026 1
// канал C
//#define CH1_026 0
//#define CH2_026 0

struct ir_piece {
  bool high;
  unsigned int tm;
};

// Начальные значения параметров
uint8_t copter = COPT107;       // Тип: 1 - S111/107, 0 - S026

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

uint8_t after_count = 0; // до запуска ничего не посылаем

uint8_t pack_len = LEN107;

// D4 - удержание питания
int pwrHold = 4;

// D7 - кнопка аварийного сброса
// D12 - вход индикации установленного соединения
// от HM-10: HIGH - connected, LOW - disconnected
int pinConnected = 12;

ir_piece ir_cmd[100];


void sendCommand(void) {
  for (uint8_t i = 0; i < pack_len; i++) {
    if (ir_cmd[i].high) {
      SET_HIGH(ir_cmd[i].tm);  
    } else {
      SET_LOW(ir_cmd[i].tm);
    }
  }
  SET_LOW_FINAL();
}

void setCommand(void) {

  uint8_t cnt = 0;
  uint8_t tmp = 0;
  
  if (copter == COPT026) { // 026
    
    ir_cmd[0].high = true;
    ir_cmd[0].tm = 470;
    
    ir_cmd[1].high = false;
    ir_cmd[1].tm = 350;
    
    ir_cmd[2].high = true;
    ir_cmd[2].tm = 1700;

    uint8_t cnt = 3;
    uint8_t tmp = Throttle << 1;
    // 7..1 - throttle
    for (uint8_t i = 0; i < 7; i++) {
        ir_cmd[cnt].high = false;
        ir_cmd[cnt].tm = 350;
        cnt++;
        ir_cmd[cnt].high = true;
        if (tmp & 0x80) {  // 1
          ir_cmd[cnt].tm = 880;
        }
        else {                  // 0
          ir_cmd[cnt].tm = 460;
        }
        tmp = tmp << 1;
        cnt++;
    }

    // Yaw - 6 бит
    tmp = Yaw << 2; 
    for (uint8_t i = 0; i < 6; i++) {
        ir_cmd[cnt].high = false;
        ir_cmd[cnt].tm = 350;
        cnt++;
        ir_cmd[cnt].high = true;
        if (tmp & 0x80) {  // 1
          ir_cmd[cnt].tm = 880;
        }
        else {                  // 0
          ir_cmd[cnt].tm = 460;
        }
        tmp = tmp << 1;
        cnt++;
    }
    
    tmp = Pitch;       // 3..0 - pitch
    tmp |= (CH1_026 << 7);   // CH1
    tmp |= (CH2_026 << 4);   // CH2
    tmp |= 0x60;             // LB, RB ???

    for (uint8_t i = 0; i < 8; i++) {
        ir_cmd[cnt].high = false;
        ir_cmd[cnt].tm = 350;
        cnt++;
        ir_cmd[cnt].high = true;
        if (tmp & 0x80) {  // 1
          ir_cmd[cnt].tm = 880;
        }
        else {                  // 0
          ir_cmd[cnt].tm = 460;
        }
        tmp = tmp << 1;
        cnt++;
    }

    //Trim + 0
    tmp = Trim << 2;
    for (uint8_t i = 0; i < 7; i++) {
        ir_cmd[cnt].high = false;
        ir_cmd[cnt].tm = 350;
        cnt++;
        ir_cmd[cnt].high = true;
        if (tmp & 0x80) {  // 1
          ir_cmd[cnt].tm = 880;
        }
        else {                  // 0
          ir_cmd[cnt].tm = 460;
        }
        tmp = tmp << 1;
        cnt++;
    }
    
  } else {  // COPT107

    ir_cmd[0].high = true;
    ir_cmd[0].tm = 2000;
    ir_cmd[1].high = false;
    ir_cmd[1].tm = 2000;
    
    cnt = 2;
    tmp = Yaw & 0x7F;
    for (uint8_t i = 0; i < 8; i++) {
        ir_cmd[cnt].high = true;
        ir_cmd[cnt].tm = 312;
        cnt++;
        ir_cmd[cnt].high = false;
        if (tmp & 0x80) {  // 1
          ir_cmd[cnt].tm = 688;
        }
        else {                  // 0
          ir_cmd[cnt].tm = 288;
        }
        tmp = tmp << 1;
        cnt++;
    }

    tmp = Pitch & 0x7F;
    for (uint8_t i = 0; i < 8; i++) {
        ir_cmd[cnt].high = true;
        ir_cmd[cnt].tm = 312;
        cnt++;
        ir_cmd[cnt].high = false;
        if (tmp & 0x80) {  // 1
          ir_cmd[cnt].tm = 688;
        }
        else {                  // 0
          ir_cmd[cnt].tm = 288;
        }
        tmp = tmp << 1;
        cnt++;
    }

    tmp = Throttle | (CH_107 << 7);
    for (uint8_t i = 0; i < 8; i++) {
        ir_cmd[cnt].high = true;
        ir_cmd[cnt].tm = 312;
        cnt++;
        ir_cmd[cnt].high = false;
        if (tmp & 0x80) {  // 1
          ir_cmd[cnt].tm = 688;
        }
        else {                  // 0
          ir_cmd[cnt].tm = 288;
        }
        tmp = tmp << 1;
        cnt++;
    }

    tmp = Trim & 0x7F;
    for (uint8_t i = 0; i < 8; i++) {
        ir_cmd[cnt].high = true;
        ir_cmd[cnt].tm = 312;
        cnt++;
        ir_cmd[cnt].high = false;
        if (tmp & 0x80) {  // 1
          ir_cmd[cnt].tm = 688;
        }
        else {                  // 0
          ir_cmd[cnt].tm = 288;
        }
        tmp = tmp << 1;
        cnt++;
    }
    
    // SEND FOOTER
    ir_cmd[cnt].high = true;
    ir_cmd[cnt].tm = 312;
    cnt++;    
  }

  ir_cmd[cnt].high = false;
  ir_cmd[cnt].tm = 8000;

}

#define battPin A0  // вход измерения напряжения батареи

char pack[64];

void sendState(void) {

    int batt = analogRead(battPin);
    if (batt > 999) {
        batt = 999;
    }

    int h = 0;
    if (copter == COPT026) {  // S026
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
    if (after_count > 0) {  // посылаем команды только при наличии управления и 4 сек. после
      sendCommand();
        if (Throttle == 0) {
            after_count--;
        }
    }       
}

// Смена типа вертолета, используется также для сброса параметров
void setCopter(uint8_t copt) {

    noInterrupts();
    after_count = 0;    
    Throttle = 0;

    if (copt == COPT026) {
        ThrottleMax = THR_026 - 1;       
        YawMax = YAW_026 - 1;
        PitchMax = PITCH_026 - 1;
        TrimMax = TRIM_026 - 1;
        pack_len = LEN026;
    }
    else {
        ThrottleMax = THR_107 - 1;
        YawMax = YAW_107 - 1;
        PitchMax = PITCH_107 - 1;
        TrimMax = TRIM_107 - 1;      
        pack_len = LEN107;
    }    
    copter = copt;
    YawHalf = YawMax / 2;
    Yaw = YawHalf;
    PitchHalf = PitchMax / 2;
    Pitch = PitchHalf;
    TrimHalf = TrimMax / 2;
    Trim = TrimHalf;
    
    setCommand();
    interrupts();
}

unsigned long timeStat = 0;
unsigned long timeCmd = 0;
unsigned long timeNow = 0;

void setup() {
    Serial.begin(115200);

    // Удержание питания
    pinMode(pwrHold, INPUT);
    
    // IR LEDS
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);

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

    timeNow = timeStat = timeCmd = millis();
}


char stcmd[64];
uint8_t icmd = 0;
char sttt[64];

void loop() {

    uint16_t hh = 0, thr = 0, ya = 0, pit = 0, tri = 0, pib = 0, bt = 0;
    uint16_t sum = 0;
    char val[8];
    bool set_copt = false;

    if (digitalRead(pwrHold) == LOW) {  // нажата кнопка питания - выключение
        digitalWrite(3, LOW);
        pinMode(pwrHold, OUTPUT);
        digitalWrite(pwrHold, LOW);
        while(true) {};
    }

    // Если нет соединения и параметры не соответствуют начальным - сброс
    if (digitalRead(pinConnected) == LOW) {

        if ((Throttle > 0) || (Pitch != PitchHalf) || 
            (Yaw != YawHalf) || (Trim != TrimHalf) ) {
            setCopter(copter);
        }
        
        timeNow = millis();
        if ((timeNow - timeStat) > NO_ACTIVITY_INTERVAL) { // долго нет соединения по BLE - отключаемся
            digitalWrite(13, HIGH);
            pinMode(pwrHold, OUTPUT);
            digitalWrite(pwrHold, LOW);
            while(true) {};
        }
        
    } else {  // есть соединение по BLE
    
        if (Serial.available() > 0) {      
            char ch = Serial.read();
            if ((ch != '\n') && (icmd < 20)) {
                stcmd[icmd++] = ch;
            } else {
                digitalWrite(13, HIGH);
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
                    if (hh == 26) { // S026
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
                    /*
                    sprintf(sttt, "New: %d %d %d %d %d %d %d\n", hh, thr, ya, pit, tri, sum,
                      hh + thr + ya + pit + tri);
                    Serial.print(sttt);
                    */
                    set_copt = false;
                    // контрольная сумма совпала и тип правильный
                    if ( (sum == (hh + thr + ya + pit + bt + tri)) &&
                      ((hh == 107) || (hh == 111) || (hh == 26)) ) {

                        if (hh == 26) {                          
                          if (copter == COPT107) {
                            copter = COPT026; // сменился тип вертолета
                            set_copt = true;
                          }    
                        } else {
                          if (copter == COPT026) {
                            copter = COPT107; // сменился тип вертолета
                            set_copt = true;                            
                          }
                        }      
                                  
                        //sprintf(sttt, "New: %d %d %d %d\n", thr, ya, pit, tri);
                        //Serial.print(sttt);
                        
                        if (set_copt) { // при смене типа остальные значения игнорируются
                            setCopter(copter);  
                        } else {
                            noInterrupts();
                            Yaw = (uint8_t)(YawMax - ya);
                            if (copter == COPT026) {
                                Pitch = (uint8_t)(pit + 1);
                                if (Pitch > PitchMax) {
                                    Pitch = PitchMax;                              
                                }
                            } else {
                                Pitch = (uint8_t)(PitchMax - pit);
                            }
                            Trim = (uint8_t)(TrimMax - tri);
                            Throttle = (uint8_t)thr;
                            if (Throttle > 0) { // включаем передачу ИК пакетов
                                after_count = AFTER_COUNT;
                            }
                            setCommand();
                            interrupts();
                        }
                        
                    }
                    timeCmd = millis(); // был принят пакет от пульта
                } // нормальный пакет
                                     
                stcmd[icmd] = '\0';
                icmd = 0;
                digitalWrite(13, LOW);         
            } // принят пакет
             
        } else {  // нет байтов из BLE
            timeNow = millis();
            
            if ((timeNow - timeCmd) > NO_COMMAND_INTERVAL) { // нет ответа от пульта
                digitalWrite(13, HIGH);
                setCopter(copter);  // сбрасываем параметры движения
                timeCmd = timeNow;                
            }

            if ((timeNow - timeStat) > STAT_INTERVAL) {
                sendState();
                timeStat = timeNow;
            }

        }
        
    }
}
