/*
 * Управление вертолетами Syma S111G(S107G) и S026
 * оригинал: https://github.com/infusion/Fritzing/tree/master/Syma-S107G
 *
  Формат пакета S107:
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

  Формат пакета S026:
     0     6 7    12 13  16 17  20 21   26
  HH TTTTTTT YYYYYY   BBBB   PPPP   MMMMM  F
  Y - Yaw 0-63, (0=left, 31=center, 63=right)
  P - Pitch 0-16, (0=backwards, 7=hold, 15=forward)
  T - Throttle 0-127, (63=50%, 127=100%)
  B - buttons 22-LEFT, 26-RIGHT
  M - TRIM 0-31
    
*/
 
#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// MPU9250/MPU6500
#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

// JoyStick
#define joyX A0
#define joyY A1

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
#define TRIM_026  31

// Syma S107G parameters
uint8_t Throttle = 0; // 0-127
uint8_t Yaw = 63; // 0-127, center=63
uint8_t Pitch = 63; // 0-127, center=63
uint8_t Trim = 63; // 0-127, center=63

// Syma S026 parameters
uint8_t Throttle026 = 0; // 0-127
uint8_t Yaw026 = 31; // 0-63, center=31
uint8_t Pitch026 = 7; // 0-15, center=7
uint8_t Trim026 = 15; // 0-31, center=15


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


// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();

    // Read Nbytes
    Wire.requestFrom(Address, Nbytes);
    uint8_t index=0;
    while (Wire.available())
    Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}
      

bool cmdSended = false;
uint8_t ThrottleMax = THR_107 - 1;
uint8_t YawMax = YAW_107 - 1;
uint8_t PitchMax = PITCH_107 - 1;
uint8_t TrimMax = TRIM_107 - 1;
uint8_t butt026 = 0;  // кнопки 026

// кнопка джойстика
int buttJoy = 7;   // D7
int copterSw = 6; // D6
// тип вертолета
int copter = 1; // 1 - S111/107, 0 - S026


void timerISR() {
    if (copter == 0) {
      sendCommand026(Yaw, Pitch, Throttle, Trim, butt026);
    }
    else {
      sendCommand(Yaw, Pitch, Throttle, Trim);
    }
    cmdSended = true;
}

LiquidCrystal_I2C lcd(0x27, 16, 2);


void setup() {
    Serial.begin(115200);
    Wire.begin();
   
    // IR LEDS
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);

    // кнопка джойстика 
    pinMode(buttJoy,INPUT);
    digitalWrite(buttJoy, HIGH);

    // тип вертолета
    pinMode(copterSw,INPUT);
    digitalWrite(copterSw, HIGH);

    if (digitalRead(copterSw) == LOW) {
        copter = 0;
        ThrottleMax = THR_026 - 1;
        YawMax = YAW_026 - 1;
        PitchMax = PITCH_026 - 1;
        TrimMax = TRIM_026 - 1;
    }

    // LED
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    // Set accelerometers low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,29,0x06);
    // Set gyroscope low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,26,0x06);

    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

    // Request continuous magnetometer measurements in 16 bits
    I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

    //setup interrupt interval: 180ms
    Timer1.initialize(180000);  //180000
    Timer1.attachInterrupt(timerISR);

    //setup PWM: f=38Khz PWM=50%
    // COM2A = 00: disconnect OC2A
    // COM2B = 00: disconnect OC2B; to send signal set to 10: OC2B non-inverted
    // WGM2 = 101: phase-correct PWM with OCRA as top
    // CS2 = 000: no prescaling
    TCCR2A = _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS20);

    // Timer value
    OCR2A = 8000 / 38;
    OCR2B = OCR2A / 2; // 50% duty cycle

    lcd.init();                      // initialize the lcd 
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Remote init...");

    
}


uint8_t BufAcc[14];

void loop() {

    if (cmdSended) {  // обрабатываем гироскоп и джойстик после отправки пакета ?
        // гироскоп
        I2Cread(MPU9250_ADDRESS,0x3B,14,BufAcc);
        // Create 16 bits values from 8 bits data
        // Accelerometer
        int16_t ax = -(BufAcc[0]<<8 | BufAcc[1]); // Yaw
        int16_t ay = -(BufAcc[2]<<8 | BufAcc[3]); // Pitch
        //int16_t az = BufAcc[4]<<8 | BufAcc[5];

        uint8_t ya = Yaw;
        uint8_t ya2 = (YawMax - 1) / 2; // 63 || 31
        if ((ax > -2000) && (ax < 2000)) {
            ya = ya2;
        }
        else {
            int8_t axt = (int8_t)(ax / 100);
            if (axt > 0) {
              axt -=20;
            }
            else {
              axt += 20;
            }
            if (axt > (ya2 + 1)) {
                axt = ya2 + 1;
            }
            else if (axt < -ya2) {
                axt = -ya2;
            }
            ya = (uint8_t)(ya2 + axt);
        }
        noInterrupts();
        Yaw = ya;
        interrupts();
    
        uint8_t p = Pitch;
        uint8_t p2 = (PitchMax - 1) / 2; // 63 || 7
        if ((ay > -2000) && (ay < 2000)) {
            p = p2;
        }
        else {
            int8_t ayt = (int8_t)(ay / 100);
            if (ayt > 0) {
              ayt -=20;
            }
            else {
              ayt += 20;
            }
            if (ayt > (p2 + 1)) {
                ayt = p2 + 1;
            }
            else if (ayt < -p2) {
                ayt = -p2;
            }
            p = (uint8_t)(p2 + ayt);
        }
        noInterrupts();
        Pitch = p;
        interrupts();
    
        // джойстик
        int xValue = analogRead(joyX);  // Throttle
        int yValue = analogRead(joyY);  // Trim

        uint8_t th = Throttle;
        if (xValue > 810) {
            if (th < (ThrottleMax - 9)) {
                th += 10;
            }
            else {
                th = ThrottleMax;
            }
        }
        else if (xValue > 610) {
            if (th < ThrottleMax) {
                th += 1;
            }
        }
        else if (xValue < 212) {
            if (th > 9) {
                th -= 10;
            }
            else {
                th = 0;
            }
        } else if (xValue < 412) {
            if (th > 0) {
                th -= 1;
            }
        }
        noInterrupts();
        Throttle = th;
        interrupts();

        uint8_t tm = Trim;
        if (yValue > 810) {   // Trim
            if (tm < (TrimMax - 9)) {
                tm += 10;
            }
            else {
                tm = TrimMax;
            }
        }
        else if (yValue > 610) {
            if (tm < TrimMax) {
                tm += 1;
            }
        }
        else if (yValue < 212) {
            if (tm > 9) {
                tm -= 10;
            }
            else {
                tm = 0;
            }
        }
        else if (yValue < 412) {
            if (tm > 0) {
                tm -= 1;
            }
            else {
                tm = 0;
            }
        }
        noInterrupts();
        Trim = tm;
        interrupts();

        if (digitalRead(buttJoy) == LOW) {
            noInterrupts();
            Throttle = 0;
            Yaw = 63;
            Pitch = 63;
            Trim = 63;
            interrupts();
        }

        char st[16];
        sprintf(st, "Thr:%3d Trm:%3d", Throttle, Trim);
        lcd.setCursor(0,0);
        lcd.print(st);
        Serial.println(st);
        sprintf(st, "Yaw:%3d Pit:%3d",Yaw - YawMax / 2, Pitch - PitchMax / 2);
        lcd.setCursor(0,1);
        lcd.print(st);
        Serial.println(st);
        cmdSended = false;
    }
}
