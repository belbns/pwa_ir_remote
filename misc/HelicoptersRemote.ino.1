/*
 * Nikolay Belov
 * 
 * Idea: https://github.com/infusion/Fritzing/tree/master/Syma-S107G,
 *       https://www.rcgroups.com/forums/showpost.php?p=15007427&postcount=47
 *
 * Arduino based IR transmitter for helicopters Syma S111G(S107G) and S026.
 * Transmitter receives command packets from smartphone (PWA)
 * via BLE module HM-10 and send IR packages to helicopters.
 * 
 * If connection with BLE is established Arduino send packets to 
 * remote control via BLE with interval 500 uS.
 * Remote control must response with control packet. 
 * If Arduino didn't receive 4 packets in sequence, it interprets it 
 * as a lost control and resets control parameters.
 *
 ************************************************************************
 Packet from BLE for S107G/S111G:
 --------------------------------
    HHH TTT YYY PPP AAA SSSS\n
    HHH = 107
    TTT - Throttle          0..127
    YYY - Yaw               0..127
    PPP - Pitch             0..127
    AAA - Trim              0..127
    SSSS - control sum (HHH + TTT + YYY + PPP + AAA)
    
 Packet from BLE for S026:
 -------------------------
    HHH TTT YYY PP B AAA SSSS\n
    HHH = 026
    TTT - Throttle          0..127
    YYY - Yaw               0..63
    PP  - Pitch             0..16
    B   - Buttons           0..3
    AAA - Trim              0..63
    SSSS - control sum (HHH + TTT + YYY + PP + B + AAA)

 Packet from Arduino to BLE:
 ---------------------------
    Current values:
    107 TTT YYY PPP VVV SSSS\n - 20 байт
    or
    026 TTT YYY PP B VVV SSSS\n

    VVV - battery (512 -> 5V, 999 - max)
 ***************************************************************************
 
  IR packet for S107:
  -------------------
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

  IR packet for S026:
  -------------------
     0     6 7    12 13 14 15 16 17 20 21  26 27
  HH TTTTTTT YYYYYY  С1 LB RB C2  PPPP AAAAAA  0 F
  T - Throttle 0-127, (63=50%, 127=100%)
  Y - Yaw 0-63, (111111=left, 100000=neutral, 000001=right)
  P - Pitch 0-15, (1111=backwards, 1000=neutral, 0000=forward)
  LB, RB - trim buttons, "1" - OFF
  C1,C2 - channel ( A=1,1 ???(B=1,0 C=0,0) )
  A - TRIM 0-63, (111111=left, 100000=neutral, 000001=right
  27 - always 0

  Timings:
  HH - Header:
    470uS  HIGH
    350uS  LOW
    1700uS HIGH
  28 bit Command:
    "0": 350uS LOW, 460uS HIGH
    "1": 350uS LOW, 880uS HIGH
  F - LOW
********************************************************* */
 
#include <TimerOne.h>

// IR Timing
// set pwm active on pin 3
#define SET_HIGH(t)    TCCR2A |= _BV(COM2B1); delayMicroseconds(t)
 // set pwm inactive on pin3
#define SET_LOW(t)     TCCR2A &= ~_BV(COM2B1); delayMicroseconds(t)

#define SET_LOW_FINAL()   TCCR2A &= ~_BV(COM2B1)

// IR packages length (in intervals)
#define LEN107    67
#define LEN026    59
// Helicopter type
#define COPT107   1
#define COPT026   0
// Helicopter parameters
#define THR_107   128
#define THR_026   128
#define YAW_107   128
#define YAW_026   64
#define PITCH_107 128
#define PITCH_026 16
#define TRIM_107  128
#define TRIM_026  64
#define CH_107    0   // channel A
//#define CH_107    1   // B
// S026 channel A
#define CH1_026 1
#define CH2_026 1


#define AFTER_COUNT 11  // after Throttle is zero send IR packets 22 times (~4 sec).

// interval structure
struct ir_piece {
  bool high;
  unsigned int tm;
};

// Start parameters
uint8_t copter = COPT107;
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

uint8_t after_count = 0; // do not send IR packets

uint8_t pack_len = LEN107;

// D7 - Reset button
int buttReset = 7;

// D12 - HM-10 BLE connection HIGH - connected, LOW - disconnected
int pinConnected = 12;

// IR packet intervals array
ir_piece ir_cmd[72];

// Send IR packet
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

// Fill IR packet intervals array
void setCommand(void) {

  uint8_t cnt = 0;
  uint8_t tmp = 0;
  
  if (copter == COPT026) { // S026
    
    ir_cmd[0].high = true;  // 470 HIGH
    ir_cmd[0].tm = 470;
    
    ir_cmd[1].high = false; // 350 LOW
    ir_cmd[1].tm = 350;
    
    ir_cmd[2].high = true;  // 1700 HIGH
    ir_cmd[2].tm = 1700;

    uint8_t cnt = 3;
    uint8_t tmp = Throttle << 1;
    // 0..6 - Throttle - 7bit
    for (uint8_t i = 0; i < 7; i++) {
        ir_cmd[cnt].high = false;     // 350 LOW
        ir_cmd[cnt].tm = 350;
        cnt++;
        ir_cmd[cnt].high = true;
        if (tmp & 0x80) {           // 1
          ir_cmd[cnt].tm = 880;     // 880 HIGH
        }
        else {                      // 0
          ir_cmd[cnt].tm = 460;     // 460 HIGH
        }
        tmp = tmp << 1;
        cnt++;
    }

    // 7..12 Yaw - 6bit
    tmp = Yaw << 2; 
    for (uint8_t i = 0; i < 6; i++) {
        ir_cmd[cnt].high = false;
        ir_cmd[cnt].tm = 350;
        cnt++;
        ir_cmd[cnt].high = true;
        if (tmp & 0x80) {       // 1
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
    // 13..20 = CH1 LB RB CH2 PPPP - 8bit
    for (uint8_t i = 0; i < 8; i++) {
        ir_cmd[cnt].high = false;
        ir_cmd[cnt].tm = 350;
        cnt++;
        ir_cmd[cnt].high = true;
        if (tmp & 0x80) {       // 1
          ir_cmd[cnt].tm = 880;
        }
        else {                  // 0
          ir_cmd[cnt].tm = 460;
        }
        tmp = tmp << 1;
        cnt++;
    }

    // 21..26 Trim + 27=0
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
    
  } else {  // S107/S111

    ir_cmd[0].high = true;  // 2000 HIGH
    ir_cmd[0].tm = 2000;
    ir_cmd[1].high = false; // 2000 LOW
    ir_cmd[1].tm = 2000;
    
    cnt = 2;
    
    tmp = Yaw & 0x7F;
    for (uint8_t i = 0; i < 8; i++) {
        ir_cmd[cnt].high = true;
        ir_cmd[cnt].tm = 312;
        cnt++;
        ir_cmd[cnt].high = false;
        if (tmp & 0x80) {       // 1
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
        if (tmp & 0x80) {       // 1
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
        if (tmp & 0x80) {       // 1
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
  ir_cmd[cnt].tm = 0;

}

#define battPin A0  // ADC input for battery

// Status package (to BLE)
char pack[64];

// Send package with device status
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

// Timer interrupt
void timerISR() {
    if (after_count > 0) {  // send IR packets only if connection is active
        sendCommand();
        if (Throttle == 0) {
            after_count--;
        }
    }       
}

// Change helicopter type, can be used for resetting control parameters
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

#define STAT_INTERVAL   500   // staus sending interval, mS

unsigned long timeStat = 0;
unsigned long timeNow = 0;
int noCmdCounter = 0;


void setup() {
    Serial.begin(115200);
   
    // IR LEDS
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);

    // reset button 
    pinMode(buttReset,INPUT);
    digitalWrite(buttReset, HIGH);

    // HIGH - BLE connected
    pinMode(pinConnected, INPUT);

    // LED
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    //Timer interrupt interval: 180ms
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


char stcmd[64];
uint8_t icmd = 0;
char sttt[64];

void loop() {

    uint16_t hh = 0, thr = 0, ya = 0, pit = 0, tri = 0, pib = 0, bt = 0;
    uint16_t sum = 0;
    char val[8];
    bool set_copt = false;

    // If BLE connection is not established and Throttle is not zero - resetting
    if ((digitalRead(pinConnected) == LOW) && (Throttle > 0)) {
        setCopter(copter);
    } else {    
        if (Serial.available() > 0) {      
            char ch = Serial.read();
            if ((ch != '\n') && (icmd < 20)) {
                stcmd[icmd++] = ch;
            } else {
                if (icmd >= 19) {  // right packet length
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

                    set_copt = false;
                    // control sum and type are OK
                    if ( (sum == (hh + thr + ya + pit + bt + tri)) &&
                      ((hh == 107) || (hh == 111) || (hh == 26)) ) {

                        if (hh == 26) {                          
                          if (copter == COPT107) {
                            copter = COPT026; // helicopter type is changed
                            set_copt = true;
                          }    
                        } else {
                          if (copter == COPT026) {
                            copter = COPT107; // helicopter type is changed
                            set_copt = true;                            
                          }
                        }      
                        
                        if (set_copt) { 
                            // changing helicopter type and resetting parameters
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
                            if (Throttle > 0) { 
                                // enable sending IR packages
                                after_count = AFTER_COUNT;
                            }
                            setCommand();
                            interrupts();
                        }                        
                    }
                } // right packet length
                // reset input package
                icmd = 0;                                     
                stcmd[icmd] = '\0';
                noCmdCounter = 0; // packet from BLE was received
                digitalWrite(13, LOW);         
            } // принят пакет 
        } else {  // нет байтов из BLE
            timeNow = millis();
            if ((timeNow - timeStat) > STAT_INTERVAL) {
                if (++noCmdCounter > 4) { // 4 packets from BLE were lost
                    digitalWrite(13, HIGH);
                    setCopter(copter);  // resetting parameters
                    noCmdCounter = 0;  
                }
                
                sendState();
                timeStat = millis();
            }
        }
    } // BLE connection is established
}
