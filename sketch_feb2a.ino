
#include <Wire.h>


const int MPU = 0x68;                                            // MPU6050 I2C address
float AccX, AccY, AccZ;                                          //linear acceleration
float GyroX, GyroY, GyroZ;                                       //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;  //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
const int maxSpeed = 255;    //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 160;    //min PWM value at which motor moves
float angle;                 //due to how I orientated my MPU6050 on my car, angle = roll
int equilibriumSpeed = 248;  //rough estimate of PWM at the speed pin of the stronger motor, while driving straight

bool isDriving = false;     //it the car driving forward OR rotate/stationary
bool prevIsDriving = true;  //equals isDriving in the previous iteration of void loop()
bool paused = false;        //is the program paused
#define t_top 2
#define e_top 4
#define t_right 10
#define e_right 11
#define t_left 7
#define e_left 8
#define r_f 6
#define r_b 9
#define l_f 3
#define l_b 5
#define e_d 12
#define t_d 13
#define ir_r A1
#define ir_l A0
float d_top, d_right, d_left, p, i, d, last_diff, diff, d_right_junc, d_m;
float kp, ki, kd;
int def_t = 200;
int ir_val_r;
int ir_val_l;
int top_distance = 18;  //minimum top distance
bool straight_road = true;
bool cornering = false;
bool ir_stop = false;
bool ir_blocked = false;
unsigned long current_time, time_limit_d, time_limit_c, prev_time1, prev_time2, start_t,ir_time;

void setup() {
  pinMode(t_top, OUTPUT);
  pinMode(e_top, INPUT);
  pinMode(t_right, OUTPUT);
  pinMode(e_right, INPUT);
  pinMode(t_left, OUTPUT);
  pinMode(r_f, OUTPUT);
  pinMode(r_b, OUTPUT);
  pinMode(l_f, OUTPUT);
  pinMode(l_b, OUTPUT);
  pinMode(e_left, INPUT);
  pinMode(t_d, OUTPUT);
  pinMode(e_d, INPUT);
  pinMode(ir_r,INPUT);
  pinMode(ir_l,INPUT);
  Serial.begin(9600);
  // put your setup code here, to run once:
  kp = 0.14;
  kd = 0.00009;
  ki = 0.0000008;
  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  delay(20);
  currentTime = micros();
}


void loop() {

  time_limit_c = 5;
  time_limit_d = 2;
  current_time = millis();
  if (current_time - prev_time1 >= time_limit_d) {
   ir_val_r = digitalRead(ir_r);
   ir_val_l = digitalRead(ir_l);
  
    

    if (get_d(t_d, e_d) < top_distance) {
      stop();
      delay(50);
    }

    if (straight_road && !(d_right >= 46 && d_right < 100 && d_left <= 50 && d_top > 50) && !ir_stop) {
      forward();
    }
   ir_scan();
    prev_time1 = current_time;
  }

  if (current_time - prev_time2 >= time_limit_c) {

    d_left = get_d(t_left, e_left);
    d_top = get_d(t_top, e_top);
    d_right = get_d(t_right, e_right);


    //to identify th right side path;
    if (d_right >= 45 && d_right < 150 && d_top >= 40 && (d_right > d_left || d_top > d_left) && !ir_stop) {
      
      stop();
      delay(50);
      
  
        int n_ang = get_angle();
      int ag = get_angle();

      while (ag > n_ang - 56) {
        right();
        ag = get_angle();
      }
      stop();
      delay(def_t);
      Serial.println("good");

      after_turn();
      stop();
      delay(def_t);
      cornering = false;
      straight_road = true;
      }
    


    //to identify the left side path;
    else if (d_left > 40 && d_left+1 > ((d_right+ d_top) / 2) && get_d(t_d, e_d) < top_distance && !ir_stop) {
      stop();
      delay(50);
 
        int n_ang2 = get_angle();
      int ag2 = get_angle();

      while (ag2 < n_ang2 + 49) {
        left();
        ag2 = get_angle();
      }

      stop();
      delay(def_t);
      after_turn();
      stop();
      delay(def_t);

      cornering = false;
      straight_road = true;
     
      
    }
     if (get_d(t_d, e_d) < top_distance) {
      stop();
      delay(50);
      if((get_d(t_right, e_right)<35 || get_d(t_top, e_top)<35) && get_d(t_left, e_left)<40 && !ir_stop){
        int ang_now = get_angle();
        int ang_c = get_angle();
          while (ang_c > ang_now - 100) {
            right();
            ang_c = get_angle();
          }
        delay(100);
        stop();
      }
    }

    prev_time2 = current_time;
  }
}
void right_turn(){
    
}
//scan ir sensors and stop
void ir_scan(){
  if(ir_val_l==1 && ir_val_r==1 && !ir_blocked){
    ir_stop=true;
      stop();
      delay(5000);
      ir_time = millis();
      ir_stop=false;
      int loop_;
     while(millis()<ir_time+3000){
       ir_blocked=true;
       loop_=1;
       if(loop_==1){
         loop();
       }
       loop_=+1;
     }
     ir_blocked=false;
     
    }
 

}
// go forward to pass the junction
void after_turn() {
  start_t = millis();
  bool finished = false;
 
    while ((millis() < start_t + 930) && !finished) {
      d_m = get_d(t_d, e_d);
      if (d_m <= top_distance) {
        stop();
        finished = true;
      } else {
        straight();
      }
    }
  
  stop();
  finished = true;
}
void straight() {
  analogWrite(r_f, 85);
  analogWrite(l_f, 85);
  analogWrite(r_b, 0);
  digitalWrite(l_b, 0);
}



void forward() {


  d_left = get_d(t_left, e_left);
  d_right = get_d(t_right, e_right);

  if (d_left > 500) {
    d_left = 18;
  }
  if (d_right > 500) {
    d_right = 15;
  }
  diff = (d_right - d_left);
  p = diff;
  i = diff + last_diff;
  d = diff - last_diff;
  last_diff = diff;
  int err = kp * p + ki * i + kd * d;




  analogWrite(r_f, 85 + err);
  analogWrite(l_f, 85 - err);
  analogWrite(r_b, 0);
  digitalWrite(l_b, 0);
}

void backword() {
  digitalWrite(r_f, 0);
  digitalWrite(l_f, 0);
  digitalWrite(r_b, 1);
  analogWrite(l_b, 180);
}
void stop() {
  digitalWrite(r_f, 0);
  digitalWrite(l_f, 0);
  digitalWrite(r_b, 0);
  digitalWrite(l_b, 0);
}
void left() {
  digitalWrite(r_f, 0);
  analogWrite(l_f, 85);
  analogWrite(r_b, 85);
  digitalWrite(l_b, 0);
}
void right() {
  analogWrite(r_f, 85);
  digitalWrite(l_f, 0);
  digitalWrite(r_b, 0);
  analogWrite(l_b, 85);
}
float get_d(int tri, int echo) {
  float fin_d = 0;
  float d = 0;
  float t = 0;
  int n = 3;
  int valx = 0;
  for (int q = 0; q < n; q++) {
    digitalWrite(tri, 0);
    delayMicroseconds(2);
    digitalWrite(tri, 1);
    delayMicroseconds(10);
    digitalWrite(tri, 0);
    t = pulseIn(echo, 1);
    d = (t * .0343) / n;
    fin_d = d + fin_d;
  }

  return (fin_d / 2);
}


void calculateError() {
  //When this function is called, ensure the car is stationary. See Step 2 for more info

  // Read accelerometer values 200 times
  c = 0;
  while (c < 200) {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  // Read gyro values 200 times
  while (c < 200) {
    readGyro();
    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}

void readAcceleration() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}

int get_angle() {
  readAcceleration();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;  //AccErrorX is calculated in the calculateError() function
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  // === Read gyroscope (on the MPU6050) data === //
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000;  // Divide by 1000 to get seconds
  readGyro();
  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX;  //GyroErrorX is calculated in the calculateError() function
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX += GyroX * elapsedTime;  // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  angle = roll;  //if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three
  //for me, turning right reduces angle. Turning left increases angle.
  return yaw;
  // Print the values on the serial monitor
}




/* 


    09/02/2024 5:55
    Gateway Competition


*/
