#include<Arduino.h>
#include<LiquidCrystal.h>
#include<Wire.h>

int gyro_X, gyro_Y, gyro_Z;
long acc_X, acc_Y, acc_Z, acc_total_vector;
long gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
int lcd_loop_counter;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void calibrate_mpu6050();
void read_mpu6050_data();
void setup_mpu6050_registers();
void write_LCD();

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(16, 2);
  lcd.setCursor(1,0);
  lcd.print("   MPU6050 IMU");
  delay(1500);
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Calibrating Gyro");
  // lcd.setCursor(0,1);
  // for(int cal_int = 0; cal_int < 2000; cal_int++)
  // {
  //   if(cal_int % 125 == 0)
  //     lcd.print(".");
  //   read_mpu6050_data();
  //   gyro_X_cal += gyro_X;
  //   gyro_Y_cal += gyro_Y;
  //   gyro_X_cal += gyro_Z;
  //   delay(3);
  // }
  // gyro_X_cal /= 2000; 
  // gyro_Y_cal /= 2000; 
  // gyro_Z_cal /= 2000;
  calibrate_mpu6050();
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Pitch:");
  lcd.print("Roll :");

  loop_timer = micros();
}

void loop()
{
  read_mpu6050_data(); 

  gyro_X -= gyro_X_cal;
  gyro_Y -= gyro_Y_cal;
  gyro_Z -= gyro_Z_cal;

  angle_pitch += gyro_X * 0.0000611;
  angle_roll += gyro_Y * 0.0000611;

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_Z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_Z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_X*acc_X)+(acc_Y*acc_Y)+(acc_X*acc_X));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_Y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_X/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  
  write_LCD();                                                         //Write the roll and pitch values to the LCD display

  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();      

}

void calibrate_mpu6050()
{
  for(int cal_int = 0; cal_int < 2000; cal_int++)
  {
    if(cal_int % 125 == 0)
      lcd.print(".");
    read_mpu6050_data();
    gyro_X_cal += gyro_X;
    gyro_Y_cal += gyro_Y;
    gyro_X_cal += gyro_Z;
    delay(3);
  }
  gyro_X_cal /= 2000; 
  gyro_Y_cal /= 2000; 
  gyro_Z_cal /= 2000;
}

void read_mpu6050_data()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  Serial.println("Here");
  while(Wire.available() < 14);
  Serial.println("Past");
  acc_X = Wire.read() << 8 | Wire.read();
  acc_Y = Wire.read() << 8 | Wire.read();
  acc_Z = Wire.read() << 8 | Wire.read();

  gyro_X = Wire.read() << 8 | Wire.read();
  gyro_Y = Wire.read() << 8 | Wire.read();
  gyro_X = Wire.read() << 8 | Wire.read();
}

void setup_mpu6050_registers()
{
  // Activate the MPU6050:
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure the accelerometer (+/- 8 G):
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Configure the gyro (500dps full scale):
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void write_LCD()
{                                                      //Subroutine for writing the LCD
  Serial.println("Write LCD");
  //To get a 250Hz program loop (4us) it's only possible to write one character per loop
  //Writing multiple characters is taking to much time
  if(lcd_loop_counter == 14)lcd_loop_counter = 0;                      //Reset the counter after 14 characters
  lcd_loop_counter ++;                                                 //Increase the counter
  if(lcd_loop_counter == 1){
    angle_pitch_buffer = angle_pitch_output * 10;                      //Buffer the pitch angle because it will change
    lcd.setCursor(6,0);                                                //Set the LCD cursor to position to position 0,0
  }
  if(lcd_loop_counter == 2){
    if(angle_pitch_buffer < 0)lcd.print("-");                          //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 3)lcd.print(abs(angle_pitch_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 4)lcd.print((abs(angle_pitch_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 5)lcd.print((abs(angle_pitch_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 6)lcd.print(".");                             //Print decimal point
  if(lcd_loop_counter == 7)lcd.print(abs(angle_pitch_buffer)%10);      //Print decimal number

  if(lcd_loop_counter == 8){
    angle_roll_buffer = angle_roll_output * 10;
    lcd.setCursor(6,1);
  }
  if(lcd_loop_counter == 9){
    if(angle_roll_buffer < 0)lcd.print("-");                           //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 10)lcd.print(abs(angle_roll_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 11)lcd.print((abs(angle_roll_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 12)lcd.print((abs(angle_roll_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 13)lcd.print(".");                            //Print decimal point
  if(lcd_loop_counter == 14)lcd.print(abs(angle_roll_buffer)%10);      //Print decimal number
}




