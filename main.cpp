// All Rights to SonOfAnton

//#include <Arduino.h>
#include <LiquidCrystal.h>
#include "./test.cpp"
// #include <avr/interrupt.h>
float dt = 0.050;

unsigned int top = 39999;

PID pid_x(0.1, 0.1, 0.1, 0, 100, dt);
PID pid_y(0.1, 0.1, 0.1, 0, 100, dt);

// lcd is just for test, uncomment and alter pins if needed
// LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define pservoy PB2
#define pservox PB1

// put function declarations here:
void setServoAngleX(int angle);
void setServoAngleY(int angle);
void setupPWM();
void setupTimer0();
void run();

int setPoint[2] = {0, 0};
float angleX;
float angleY;
int counter;

void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);

  // using timer 1 for PWMs
  setupPWM();
  // Timer0 is used when we want to consider a constant time step for our system,
  // here for example we can use Timer0 overflow interrupt to create a 50ms time step
  // uncomment all the code related to timer 0 if  you need a constant time step
  // it causes the system to send and recieve data and calculate pwm every 50 ms or any specified time step
  //setupTimer0();

  // lcd.begin(16, 2);
  
  // calibration
  setServoAngleX(10);
  setServoAngleY(10);

  delay(1000);

  setServoAngleX(45);
  setServoAngleY(45);

  delay(5000);

}

byte buff[20]; // buffer to store serial data

void get_pos(){
Serial.write(1);
Serial.readBytes(buff, 32);
}

float getFloatFromBuffer(byte *buffer, int startIndex) {
    return *(float *)(buffer + startIndex);
}

void loop(){
  run();
}

void run() { 

  get_pos();
 
      int       x = buff[0] << 8 | buff[1];
      int       y = buff[2] << 8 | buff[3];
      int mouse_x = buff[4] << 8 | buff[5];
      int mouse_y = buff[6] << 8 | buff[7];

     // Use pointer method to read floats
      float Kd_x = getFloatFromBuffer(buff, 8);
      float Ki_x = getFloatFromBuffer(buff, 12);
      float Kc_x = getFloatFromBuffer(buff, 16);

      float Kd_y = getFloatFromBuffer(buff, 20);
      float Ki_y = getFloatFromBuffer(buff, 24);
      float Kc_y = getFloatFromBuffer(buff, 28);

      pid_x.setTunings(Kc_x, Ki_x, Kd_x);
      pid_y.setTunings(Kc_y, Ki_y, Kd_y);
      setPoint[0] = mouse_x;
      setPoint[1] = mouse_y;

      // lcd commands are just for test
      // lcd.clear();
      // lcd.setCursor(0, 0);
      // lcd.print(x);
      // lcd.setCursor(0, 1);
      // lcd.print(y);
      // lcd.setCursor(5, 0);
      // lcd.print(mouse_x);
      // lcd.setCursor(5, 1);
      // lcd.print(mouse_y);
      // lcd.setCursor(0, 1);
      // lcd.print(Cd_y);
      // lcd.setCursor(8, 0);
      // lcd.print(Ci_y);
      // lcd.setCursor(8, 1);
      // lcd.print(Cc_y);
      
      // reset condition
      if (x == 2000 && y == 2000){
        angleX = 45;
        angleY = 45;

        pid_x.reset();
        pid_y.reset();
      }
      else{
        angleX =  pid_x.compute(setPoint[0], x, false);
        angleY =  pid_y.compute(setPoint[1], y, true);
      }

      setServoAngleX(angleX);
      setServoAngleY(angleY);

}

void setupTimer0() {
    TCCR0A = 0;  // Normal mode
    TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler = 64
    TIMSK0 |= (1 << TOIE0);  // Enable Timer0 Overflow Interrupt
    TCNT0 = 6;  // Preload for 50ms overflow
    sei();  // Enable global interrupts
}


// using timer 1 of microcontroller to setup PWMs
void setupPWM(){

  DDRB  |= (1 << pservoy) | (1 << pservox); 
  PORTB |= (0 << pservoy)| (0 << pservox);

  ICR1 = top;  // Set TOP value for 50Hz PWM (20ms period)
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);  // Fast PWM, non-inverting
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler = 8, Fast PWM

  // F_clk = 16 MHz 
  // prescale = 8
  // PWM frequency 50 Hz --> top = 25000 = 0x61A8
  OCR1A = 0;
  OCR1B = 0;
}

// put function definitions here:
void setServoAngleX(int angle) {
  // Map angle (0°–180°) to pulse width (1 ms–2 ms)
  unsigned int pulseWidth = map(angle, 0, 180, 0.03*ICR1, 0.13*ICR1); 
  OCR1A = pulseWidth;
}

// put function definitions here:
void setServoAngleY(int angle) {
  // Map angle (0°–180°) to pulse width (1 ms–2 ms)
  unsigned int pulseWidth = map(angle, 0, 180, 0.03*ICR1, 0.13*ICR1); 
  OCR1B = pulseWidth;
}