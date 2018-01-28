
/*
This code is used for connecting arduino to serial mpu6050 module, and test in arduino uno R3 board.
connect map:
arduino   mpu6050 module
VCC    5v/3.3v
TX     RX<-0
TX     TX->1
GND    GND
note:
because arduino download and mpu6050 are using the same serial port, you need to un-connect 6050 module when you want to download program to arduino.
 Created 14 Nov 2013
 by Zhaowen

 serial mpu6050 module can be found in the link below:
 http://item.taobao.com/item.htm?id=19785706431
 */
#define encoder0PinA 2
#define encoder0PinB 4
#define encoder1PinA 3
#define encoder1PinB 5
#define encoder2PinA 21
#define encoder2PinB 6


double encoder0Pos = 0;
double encoder1Pos = 0;
double encoder2Pos = 0;
double test = 0;
unsigned char Re_buf[11], counter = 0;
unsigned char Acc_buf[4];
unsigned char w_buf[2];
unsigned char angle_buf[2];
unsigned char sign = 0;
float a[3] = {0, 0, 0}, w[3] = {0, 0, 0}, angle[3] = {0, 0, 0}, T;
byte angle_data[3];
byte command = 0;
int led = 0;
double X = 0, Y = 0, X_temp = 0, Y_temp = 0, enc_temp = 0;
double last_angle = 0;
double pi = 3.1415926;
double first_heading = 0;
double enc_angle = 0;
bool is_initialized = false;
double K1 = 0.97;//0.95
double filtered_angle, relative_angle;
double lasttime = 0;

double Enc0list[] = {0,0,0,0,0,0,0,0,0,0};
double Enc1list[] = {0,0,0,0,0,0,0,0,0,0};
double Enc2list[] = {0,0,0,0,0,0,0,0,0,0};
double timelist[] = {0,0,0,0,0,0,0,0,0,0};

double enc0_update_time = 0,enc1_update_time = 0,enc2_update_time = 0;



void setup() {
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH); // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH); // turn on pullup resistor
  attachInterrupt(0, doEncoder, RISING); // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoder2, RISING); // encoder pin on interrupt 1 - pin 3
  attachInterrupt(2, doEncoder3, RISING); // encoder pin on interrupt 2 - pin 21
  // initialize serial:
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  //Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  lasttime = micros();
  is_initialized = false;
}

void loop() {
  //Serial.println(Y);
  digitalWrite(12, 1);

  
  while (Serial3.available() > 0) {
    command = Serial3.read();
    switch (command) {
      case 10:
        Serial3.print((char)angle_buf[0]);
        Serial3.print((char)angle_buf[1]);
        Serial3.print((char)Acc_buf[0]);
        Serial3.print((char)Acc_buf[1]);
        Serial3.print((char)Acc_buf[2]);
        Serial3.print((char)Acc_buf[3]);
        break;
      case 9:
        sendfloat((float)-X);
        break;
      case 8:
        sendfloat((float)-Y);
        break;
      case 7:
        first_heading = angle[2] * pi / 180;
        enc_angle = 0;
        X = 0;
        Y = 0;
        break;
    }
    command = 0;
  }
  /*test only
  if (sending >= 10) {
    sending = 0;
    Serial.println(X / 10, DEC);
    Serial.println(Y / 10, DEC);
  }
  */

  while (Serial2.available() > 0) {
    Re_buf[counter] = (unsigned char)Serial2.read();
    if (counter == 0 && Re_buf[0] != 0x55) {
      digitalWrite(13, 1);
    } else
    {
      digitalWrite(13, 0);
    }
    counter++;
    if (counter == 11)          //接收到11个数据
    {
      counter = 0;             //重新赋值，准备下一帧数据的接收
      sign = 1;
    }
  }
  if (sign)
  {
    sign = 0;
    if (Re_buf[0] == 0x55)   //检查帧头
    {

      switch (Re_buf [1])
      {
        case 0x51:
          a[0] = (short(Re_buf [3] << 8 | Re_buf [2])) / 32768.0 * 16;
          a[1] = (short(Re_buf [5] << 8 | Re_buf [4])) / 32768.0 * 16;
          a[2] = (short(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 16;
          T = (short(Re_buf [9] << 8 | Re_buf [8])) / 340.0 + 36.25;

          Acc_buf[0] = Re_buf[3];
          Acc_buf[1] = Re_buf[2];
          Acc_buf[2] = Re_buf[5];
          Acc_buf[3] = Re_buf[4];
          break;
        case 0x52:
          w[0] = (short(Re_buf [3] << 8 | Re_buf [2])) / 32768.0 * 2000;
          w[1] = (short(Re_buf [5] << 8 | Re_buf [4])) / 32768.0 * 2000;
          w[2] = (short(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 2000;
          T = (short(Re_buf [9] << 8 | Re_buf [8])) / 340.0 + 36.25;

          w_buf[0] = Re_buf[7];
          w_buf[0] = Re_buf[6];

          break;
        case 0x53:
          angle[0] = (short(Re_buf [3] << 8 | Re_buf [2])) / 32768.0 * 180;
          angle[1] = (short(Re_buf [5] << 8 | Re_buf [4])) / 32768.0 * 180;
          angle[2] = (short(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 180;
          test = angle[2];
          T = (short(Re_buf [9] << 8 | Re_buf [8])) / 340.0 + 36.25;
          if (!is_initialized) {
            first_heading = angle[2] * pi / 180.0;
            last_angle = first_heading;
            is_initialized = true;
          }
          //Serial.println(angle[2]);
          angle_buf[0] = Re_buf[7];
          angle_buf[1] = Re_buf[6];
          DoIntegrate();
          break;
      }
    }
  }
}

void doEncoder() {
  enc0_update_time = micros();
  if (!digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void doEncoder2() {
  enc1_update_time = micros();
  if (!digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

void doEncoder3() {
  enc2_update_time = micros();
  if (!digitalRead(encoder2PinB)) {
    encoder2Pos++;   
  } else {
    encoder2Pos--;
  }
}

void DoIntegrate() {
  double enc0_inc = 0,enc1_inc = 0,enc2_inc = 0;//increment
  double E0 = 0,E1 = 0,E2 = 0,t0 = 0,t1 = 0,t2 = 0;
  E0 = encoder0Pos;
  E1 = encoder1Pos;
  E2 = encoder2Pos;
  t0 = enc0_update_time;
  t1 = enc1_update_time;
  t2 = enc2_update_time;

  
  for (int i = 0; i < 1 ; i ++){
        Enc0list[1-i] = Enc0list[0 - i];
        Enc1list[1-i] = Enc1list[0 - i];
        Enc2list[1-i] = Enc2list[0 - i];
        timelist[1-i] = timelist[0 - i];
    }
    
    Enc0list[0] = E0;
    Enc1list[0] = E1;
    Enc2list[0] = E2;
    
    
  double Enc0_sum = 0,Enc1_sum = 0,Enc2_sum = 0;
  for (int i = 0; i < 2 ;i++){
      Enc0_sum += Enc0list[i];
      Enc1_sum += Enc1list[i];
      Enc2_sum += Enc2list[i];
    }
    //Serial.print(Enc0_sum);
    //Serial.print("      ");
    Enc0_sum = Enc0_sum / (micros() - timelist[1] + 10404);
    Enc1_sum = Enc1_sum / (micros() - timelist[1] + 10404);
    Enc2_sum = Enc2_sum / (micros() - timelist[1] + 100404);
  
    enc0_inc = (micros() - t0) * Enc0_sum;
    enc1_inc = (micros() - t1) * Enc1_sum;
    enc2_inc = (micros() - t2) * Enc2_sum;
/*
    E0 += enc0_inc;
    E1 += enc1_inc;
    E2 += enc2_inc;
  */
  X_temp = (0.50000 * E0 - 1.00000 * E1 + 0.50000 * E2);
  Y_temp = (-0.50000 * E0 + 0 * E1 + 0.50000 * E2);
  X_temp = X_temp * pi/250*30*1.40;
  Y_temp = Y_temp * pi/250*30*1.40;

  
  
  //TO-DO
  enc_temp = (0.09700 * 3.0000 * E0 * pi / 250.0000 + 0.09700 * 3.0000 * E2 * pi / 250.0000);
  //TO-DO
  enc_angle += enc_temp;
  double relative_angle2 =  0;
  relative_angle2 = atan2(sin((angle[2] * pi / 180.0) - first_heading), cos((angle[2] * pi / 180.0) - first_heading));
  filtered_angle = K1 * atan2(sin(relative_angle2), cos(relative_angle2)) + (1 - K1) * (atan2(sin(enc_angle), cos(enc_angle)));
  relative_angle = filtered_angle;
  X += X_temp * cos(relative_angle) - Y_temp * sin(relative_angle);
  Y += X_temp * sin(relative_angle) + Y_temp * cos(relative_angle);
  /*
  Serial.print("X: ");
  Serial.print(-X);
  Serial.print("Y: ");
  Serial.println(-Y);
  */
  enc_angle = filtered_angle;
  encoder_reset();
  last_angle = filtered_angle;
  timelist[0] = micros();
}
void encoder_reset() {
  encoder0Pos = 0;
  encoder1Pos = 0;
  encoder2Pos = 0;
}

