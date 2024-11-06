// NOTE: requires the Encoder library.
// 1) open Tools -> Manage Libraries...
// 2) install "Encoder" by Paul Stoffregen v1.4.1
#include <Encoder.h>
#include <Servo.h>
// NOTE: Requires the PS2X_lib installed.
// 1) open Sketch -> Include Library -> Add .ZIP Library
// 2) select "PS2X_lib.zip"
#include <PS2X_lib.h>
 
//PS2
#define PS2_DAT        52  //14    
#define PS2_CMD        51  //15
#define PS2_SEL        53  //16
#define PS2_CLK        50  //17

// #define pressures   true
#define pressures   false
#define rumble      true
//#define rumble      false
PS2X ps2x; // create PS2 Controller Class
Servo s1;
Servo s2;
int error = 0;
byte type = 0;
byte vibrate = 0;

void (* resetFunc) (void) = 0;

// --- SPD Motor ---

class SPDMotor {
  public:
  SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 );

  /// Set the PWM speed and direction pins.
  /// pwm = 0, stop (no active control)
  /// pwm = 1 to 255, proportion of CCW rotation
  /// pwm = -1 to -255, proportion of CW rotation
  void speed( int pwm );

  /// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
  void hardStop();

  /// Get the current speed.
  int getSpeed();

  /// Get the current rotation position from the encoder.
  long getEncoderPosition();

  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;

    // Current speed setting.
    int _speed;
};

SPDMotor::SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 ) {
  _encoder = new Encoder(encoderA, encoderB);
  _encoderReversed = encoderReversed;

  _motorPWM = motorPWM;
  pinMode( _motorPWM, OUTPUT );
  _motorDir1 = motorDir1;
  pinMode( _motorDir1, OUTPUT );
  _motorDir2 = motorDir2;
  pinMode( _motorDir2, OUTPUT );
}

/// Set the PWM speed and direction pins.
/// pwm = 0, stop (no active control)
/// pwm = 1 to 255, proportion of CCW rotation
/// pwm = -1 to -255, proportion of CW rotation
void SPDMotor::speed( int speedPWM ) {
  _speed = speedPWM;
  if( speedPWM == 0 ) {
    digitalWrite(_motorDir1,LOW);
    digitalWrite(_motorDir2,LOW);
    analogWrite( _motorPWM, 255);
  } else if( speedPWM > 0 ) {
    digitalWrite(_motorDir1, LOW );
    digitalWrite(_motorDir2, HIGH );
    analogWrite( _motorPWM, speedPWM < 255 ? speedPWM : 255);
  } else if( speedPWM < 0 ) {
    digitalWrite(_motorDir1, HIGH );
    digitalWrite(_motorDir2, LOW );
    analogWrite( _motorPWM, (-speedPWM) < 255 ? (-speedPWM): 255);
  }
}

/// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
void SPDMotor::hardStop() {
    _speed = 0;
    digitalWrite(_motorDir1,HIGH);
    digitalWrite(_motorDir2,HIGH);
    analogWrite( _motorPWM, 0);
}

/// Get the current speed.
int SPDMotor::getSpeed() {
    return _speed;
}

/// Get the current rotation position from the encoder.
long SPDMotor::getEncoderPosition() {
  long position = _encoder->read();
  return _encoderReversed ? -position : position;
}

SPDMotor *motorLF = new SPDMotor(18, 31, true, 11, 34, 35); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, false, 7, 36, 37); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor *motorLR = new SPDMotor( 3, 49, true,  6, 43, 42); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor( 6, A1, false, 4, A5, A4); // <- NOTE: Motor Dir pins reversed for opposite operation

void setup()
{
  Serial.begin(9600);
  delay(300) ;//added delay to give wireless ps2 module some time to startup, before configuring it
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0) {
    Serial.println("Found Controller, configuration successful ");
    Serial.println();
    Serial.println("SPDMotor control by Aaron Hilton of Steampunk Digital");
    Serial.println("=====================================================");
    Serial.println("Holding L1 or R1 will activate analog joystick control.");
    Serial.println("Left analog stick for forward/back and turning.");
    Serial.println("Right analog stick for sideways movement.");
    Serial.println("Hold both L1 and R1 for full-speed.");
  }
  else if (error == 1)
  {
    Serial.println("No controller found, check PS2 receiver is inserted the correct way around.");
    resetFunc();
  }
  else if (error == 2)
    Serial.println("Controller found but not accepting commands.");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  type = ps2x.readType();
  switch (type) {
  case 0:
    Serial.println("Unknown Controller type found ");
    break;
  case 1:
    Serial.println("DualShock Controller found ");
    break;
  case 2:
    Serial.println("GuitarHero Controller found ");
    break;
  case 3:
    Serial.println("Wireless Sony DualShock Controller found ");
    break;
  }
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11,INPUT);
  s1.attach(46);
  s2.attach(45);
}

int pos1=60;
int pos2=60;
int R=0;
int omega=0;
int a=0;
void loop() {
  s1.write(pos1);
      s2.write(pos2);
      
      while(a==0){
        
      int v=140;
      motorLF->speed(-0.866*v);//R1
     motorRF->speed(0);
     motorLR->speed(0);//b
     motorRR->speed(v*0.866);//L1
     delay(2000);
       motorLF->speed(0);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(0);//L1
        delay(500);
        s1.write(110);
        s2.write(10);
        motorLF->speed(-0.866*v);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(v*0.866);//L1
        delay(500);
        motorLF->speed(v);//R1
        motorRF->speed(v);
        motorLR->speed(v);//b
        motorRR->speed(v);//L1
        delay(540);
        motorLF->speed(-0.866*v);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(v*0.866);//L1
        delay(1500);
        s1.write(60);
        s2.write(60);
        motorLF->speed(-0.866*v);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(v*0.866);//L1
        delay(1000);
        motorLF->speed(0.866*v);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(-v*0.866);//L1
         delay(1500);
         motorLF->speed(-0.866*v*2);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(v*2*0.866);//L1
        delay(3000);
//       if(digitalRead(9)==LOW&&digitalRead(10)==LOW){
//        motorLF->speed(-0.866*v);//R1
//        motorRF->speed(0);
//        motorLR->speed(0);//b
//        motorRR->speed(v*0.866);//L1
//        }else if(digitalRead(9)==HIGH&&digitalRead(10)==LOW){
//           motorLF->speed(-0.5*v);//R1
//        motorRF->speed(0);
//        motorLR->speed(v);//b
//        motorRR->speed(-v*0.5);//L1
//          }else if(digitalRead(9)==LOW&&digitalRead(10)==HIGH){
//            motorLF->speed(0.5*v);//R1
//        motorRF->speed(0);
//        motorLR->speed(-v);//b
//        motorRR->speed(v*0.5);//L1
//            }else if(digitalRead(9)==LOW&&digitalRead(10)==LOW){
//              motorLF->speed(0);//R1
//        motorRF->speed(0);
//        motorLR->speed(0);//b
//        motorRR->speed(0);//L1
//        delay(500);
//        s1.write(110);
//        s2.write(10);
//        motorLF->speed(-0.866*v);//R1
//        motorRF->speed(0);
//        motorLR->speed(0);//b
//        motorRR->speed(v*0.866);//L1
//        delay(500);
//        motorLF->speed(v);//R1
//        motorRF->speed(v);
//        motorLR->speed(v);//b
//        motorRR->speed(v);//L1
//        delay(500);
//        motorLF->speed(-0.866*v);//R1
//        motorRF->speed(0);
//        motorLR->speed(0);//b
//        motorRR->speed(v*0.866);//L1
//        delay(500);
        a++;
              /*
         motorLF->speed(-0.866*150);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(150*0.866);//L1
        delay(1800);
        motorLF->speed(0);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(0);//L1
        s1.write(110);
        s2.write(10);
        delay(3000);
         motorLF->speed(-0.5*150);//R1
        motorRF->speed(0);
        motorLR->speed(150);//b
        motorRR->speed(-150*0.5);//L1
        delay(2400);
        motorLF->speed(0);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(0);//L1
        motorLF->speed(100);//R1
        motorRF->speed(0);
        motorLR->speed(100);//b
        motorRR->speed(100);//L1
        delay(800);
        motorLF->speed(-0.866*150);//R1
        motorRF->speed(0);
        motorLR->speed(0);//b
        motorRR->speed(150*0.866);//L1
         delay(800);*/
        
         }
        if (error == 1) //skip loop if no controller found
    return;

  if (type == 2) { //Guitar Hero Controller
    return;
  }
  else  { //DualShock Controller
    static long oldSumPosition = 0;
    long sumPosition = motorLF->getEncoderPosition() + motorRF->getEncoderPosition() + motorLR->getEncoderPosition() + motorRR->getEncoderPosition();
    long deltaPosition = sumPosition - oldSumPosition;
    oldSumPosition = sumPosition;
    Serial.print("∆pos: ");
    Serial.print(deltaPosition);
    long posVibrate = abs(deltaPosition);
    posVibrate = posVibrate > 255 ? 255 : 0;
    vibrate = (byte)posVibrate;
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    if(ps2x.Button(PSB_L2)){
      
      
      pos1++;
      pos2--;
      s1.write(pos1);
      s2.write(pos2);
      }else if(ps2x.Button(PSB_R2)){
        
      
      pos1--;
      pos2++;
      s1.write(pos1);
      s2.write(pos2);
        }
    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
      int LY = ps2x.Analog(PSS_LY);
      int LX = ps2x.Analog(PSS_LX);
      int RX = ps2x.Analog(PSS_RX);
      float forwardNormalized = (float)(-LY + 128)/127.f;

      forwardNormalized = constrain( forwardNormalized, -1.f, 1.f );
      float multiplier = (ps2x.Button(PSB_L1) && ps2x.Button(PSB_R1)) ? 255.f : 80.f;
      int forward = (int)(pow(forwardNormalized, 2.0) * multiplier);
  
      // Preserve the direction of movement.
      if( forwardNormalized < 0 ) {
        forward = -forward;
      }
   
      int right = -RX + 127;
      int ccwTurn = (LX - 127)/2;
      Serial.print( " fwd:" );
      Serial.print( forward );
      Serial.print( " r:" );
      Serial.print( right );
      Serial.print( " ∆°:" );
      Serial.print( ccwTurn );
      Serial.print( " LF:" );
      Serial.print( motorLF->getEncoderPosition() );
      Serial.print( " RF:" );
      Serial.print( motorRF->getEncoderPosition() );
      Serial.print( " LR:" );
      Serial.print( motorLR->getEncoderPosition() );
      Serial.print( " RR:" );
      Serial.println( motorRR->getEncoderPosition() );
   
      
    } else {
      // If there's motor power, try to hard-stop briefly.
      if( motorLF->getSpeed() != 0
      || motorRF->getSpeed() != 0
      || motorLR->getSpeed() != 0
      || motorRR->getSpeed() != 0 )
      {
          motorLF->hardStop(); motorRF->hardStop();
          motorLR->hardStop(); motorRR->hardStop();
          delay(500);
          motorLF->speed(0); motorRF->speed(0);
          motorLR->speed(0); motorRR->speed(0);
      }
    }
  }
  if (ps2x.Button(PSB_L1) ) {
    int LY = ps2x.Analog(PSS_LY);
    int LX = ps2x.Analog(PSS_LX);
   
    
    int VX=(-LX+125)*2.4;
    int VY=(LY-125)*2.4;
    
    // 添加死区
   
    
        
        motorLF->speed(-0.5*VX+0.866*VY+omega*R);//R1
        motorRF->speed(0);
        motorLR->speed(VX+omega*R);//b
        motorRR->speed(-VX*0.5-VY*0.866+omega*R);//L1
    
}
if(ps2x.Button(PSB_R1)){
   int RX = ps2x.Analog(PSS_RX);int ww=(-RX+125)*2.4;
        motorLF->speed(-ww);//R1
        motorRF->speed(0);
        motorLR->speed(-ww);//b
        motorRR->speed(-ww);//L1
  }
  
/*
// 检测 A15 和 A14 引脚的状态
  int pinA15 = digitalRead(9);
  int pinA14 = digitalRead(10);

  // 根据引脚状态决定动作
  if (pinA15 == HIGH && pinA14 == HIGH) {
    // 直走
    motorLF->speed(155); // 向前速度
    motorRF->speed(155);
    motorLR->speed(155);
    motorRR->speed(155);
  } else if (pinA15 == LOW && pinA14 == LOW) {
    // 停止
    motorLF->speed(0);
    motorRF->speed(0);
    motorLR->speed(0);
    motorRR->speed(0);
  } else if (pinA15 == HIGH && pinA14 == LOW) {
    // 左转
    motorLF->speed(0); // 左侧停止
    motorRF->speed(155); // 右侧前进
    motorLR->speed(0);
    motorRR->speed(155);
  } else if (pinA15 == LOW && pinA14 == HIGH) {
    // 右转
    motorLF->speed(155); // 左侧前进
    motorRF->speed(0); // 右侧停止
    motorLR->speed(155);
    motorRR->speed(0);
  }*/
}
