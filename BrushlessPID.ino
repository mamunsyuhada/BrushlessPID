// Include LCD and I2C library

#include <Wire.h>

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

#include "ESC.h"
#define LED_PIN (13)              // Pin for the LED 

ESC myESC (9, 1000, 2000, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Arm Value)

unsigned long lastTime;
double Input, Output;
double Iterm, lastErr;
double Setpoint;
double error;
double dErr;
int SampleTime = 10; //0.01 sec
double offset = 88;
double SampleTimeInSec = ((double)SampleTime) / 1000;
double  kp = 2.04;   // Ku = 3.4 Tu = 1.24
double  ki =  3.41 * SampleTimeInSec;   //3.62
double  kd = 0.306 / SampleTimeInSec;  //0.325
double  ue = 40;
unsigned long current_millis;
unsigned long previous_millis = 0;
unsigned long interval = 50;
long lastDebounceTime = 0;
long debounceDelay = 1000;

void setup() {
  pinMode(4, INPUT);
  pinMode(LED_PIN, OUTPUT);       // LED Visual Output (can be removed)
  digitalWrite(LED_PIN, HIGH);    // LED High while signal is High (can be removed)
  myESC.calib();                  // Calibration of the Max and Min value the ESC is expecting
  myESC.stop();                   // Stop the ESC to avoid damage or injuries
  digitalWrite(LED_PIN, LOW);     // LED Low when the calibration is done (can be removed)
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(57600);                                               //Use only for debugging

  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro


  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  loop_timer = micros();                                               //Reset the loop timer
}

void loop() {

  current_millis = millis();

  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if (set_gyro_angles) {                                               //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else {                                                               //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
  //Input = angle_pitch_output + offset;
  //Setpoint = 45;
  Compute();
  command();

  //   if ((current_millis - previous_millis) >= interval)
  //{
  //
  //    Serial.print(kd * dErr);
  //     Serial.print(" ");
  //     Serial.print(kp * error);
  //     Serial.print(" ");
  //     Serial.print(Iterm/100);
  //     Serial.print(",");
  //     Serial.println(Setpoint - (angle_pitch_output + offset));
  //     previous_millis = millis();
  //}


}

void command() {
  myESC.speed(Output);
}
