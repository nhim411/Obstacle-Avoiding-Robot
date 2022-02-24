//lcd V0, Rẽ Trái/Phải(0/1) V2,Dừng/Chạy(0/1) V3, terminal V4
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include<Wire.h>
#include <Servo.h>
Servo myservo;  // khởi tạo đối tượng Servo với tên gọi là myservo
int pos = 0;    // biến pos dùng để lưu tọa độ các Servo
char auth[] = "08e3387f0f104a849f2f14109d359a98";
char ssid[] = "WIFI";
char pass[] = "PASSWORD";

#define PIN_RIGHT D1
#define PIN_RUN D0
#define PIN_BARRIER D2
#define trigPin D4
#define echoPin D3
#define PIN_SCL D6
#define PIN_SDA D7
#define PIN_SERVO D5

long duration;
int distance;
int isRight = 0;
int isRun = 0;
String incommingdata = "";
int received = 0;
String zone = "";
int barrier = 0;
int temp_barrier = 0;

//MPU6050 config
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;
// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 4;
const uint16_t GyroScaleFactor = 131;
// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t GyroX, GyroY, GyroZ;

WidgetLCD lcd(V0);
WidgetTerminal terminal(V1);
BLYNK_WRITE(V1)
{
  if (String("NAM") == param.asStr()) {
    terminal.println("You said: 'NAM'") ;
    terminal.println("I said: 'DEP TRAI'") ;
  } else {
    // Send it back
    terminal.print("You said:");
    terminal.write(param.getBuffer(), param.getLength());
    terminal.println();
  }
  terminal.flush();
}
//Nút cấu hình rẽ Trái/Phải khi gặp vật cản
BLYNK_WRITE(V2)
{
  isRight = param.asInt();
  digitalWrite(PIN_RIGHT, isRight);
}
//Nút Dừng/Chạy
BLYNK_WRITE(V3)
{
  isRun = param.asInt();
  digitalWrite(PIN_RUN, isRun);
  // String i = param.asStr();
  // double d = param.asDouble();
  //int Run = param.asInt();
}
//Đọc UART
void SerialEvent()
{
  if (Serial.available())
  {
    while (Serial.available())
    {
      // get the new byte:
      char inChar = (char)Serial.read();
      incommingdata += inChar;
      if (inChar == '\n') {
        //stringComplete = true;
        received = 1;
      }
    }
  }
}
void lcdPrint() {
  switch (zone.toInt()) {
    case 0:
      lcd.print(0, 0, "               ");
      break;
    case 1:
      lcd.print(0, 0, "Xe den Zone 1");
      break;
    case 2:
      lcd.print(0, 0, "Xe den Zone 2");
      break;
    case 3:
      lcd.print(0, 0, "Xe den Zone 3");
      break;
    case 4:
      lcd.print(0, 0, "Xe den Zone 4");
      break;
    case 5:
      lcd.print(0, 0, "Xe den dich  ");
      break;
  }
  switch (barrier) {
    case 1:
      lcd.print(0, 1, "Xe gap vat can");
      break;
    case 0:
      lcd.print(0, 1, "               ");
      break;
  }
}

void delay_ms(unsigned long t)
{
  unsigned long a;
  a = millis();
  while ((millis() - a) < t) {}
}

//Đồng bộ lại trạng thái khi mất nguồn
BLYNK_CONNECTED() {
  Blynk.syncAll();
}

void setup()
{
  Serial.begin(9600);
  Wire.begin(PIN_SDA, PIN_SCL);
  myservo.attach(14);
  MPU6050_Init();
  //Serial.setTimeout(10);
  Blynk.begin(auth, ssid, pass, "gith.cf", 8442);//18.140.56.47
  //Blynk.begin(auth, ssid, pass);
  lcd.clear(); //Use it to clear the LCD Widget
  terminal.clear();
  lcd.print(0, 0, "Connected");
  pinMode(PIN_RUN, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
  digitalWrite(PIN_RUN, isRun);
  digitalWrite(PIN_RIGHT, isRight);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(PIN_BARRIER, OUTPUT);
  digitalWrite(PIN_RUN, 0);
  terminal.println(F("Connect to Shipping Robot"));
  terminal.println(F("-------------"));
  terminal.flush();
}

void loop()
{
  Blynk.run();
  //timer.run();
  digitalWrite(PIN_RUN, isRun);
  digitalWrite(PIN_RIGHT, isRight);
  SerialEvent();
  if (received == 1) {
    //Incoming data: "Zone(0->4)|Barrier(0/1)"
    //incommingdata = Serial.readString();
    terminal.print(F("PIC: "));
    terminal.println(incommingdata);
    terminal.flush();
    zone = incommingdata.substring(0, 1);
    //barrier = incommingdata.substring(2, 3);

    //Gửi thông báo vị trí Zone lên App
    lcdPrint();
    incommingdata = "";
    received = 0;
  }
  //MPU6050
  double Gx, Gy, Gz;
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  //divide each with their sensitivity scale factor
  Gx = (double)GyroX / GyroScaleFactor;
  Gy = (double)GyroY / GyroScaleFactor;
  Gz = (double)GyroZ / GyroScaleFactor;
  //Serial.print(" Gx: "); Serial.print(Gx);
  //Serial.print(" Gy: "); Serial.print(Gy);
  //Serial.print(" Gz: "); Serial.println(Gz);
  Serial.print('g');
  Serial.print(Gy);
  Serial.print('\r');
  Blynk.virtualWrite(V5, Gx);
  Blynk.virtualWrite(V6, Gy);
  Blynk.virtualWrite(V7, Gz);

  //SRF04
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = (duration / 2) / 29.1; //cm
  if (distance < 20) {
    digitalWrite(PIN_BARRIER, HIGH);
    barrier = 1;
  }
  else {
    digitalWrite(PIN_BARRIER, LOW);
    barrier = 0;
  }
  if (barrier != temp_barrier) {
    lcdPrint();
    temp_barrier = barrier;
  }
  // Prints the distance on the Serial Monitor
  for (pos = 0; pos < 180; pos += 1) // cho servo quay từ 0->179 độ
  { // mỗi bước của vòng lặp tăng 1 độ
    myservo.write(pos);              // xuất tọa độ ra cho servo
    delay(3);                       // đợi 15 ms cho servo quay đến góc đó rồi tới bước tiếp theo
  }
  for (pos = 180; pos >= 1; pos -= 1) // cho servo quay từ 179-->0 độ
  {
    myservo.write(pos);              // xuất tọa độ ra cho servo
    delay(3);                       // đợi 15 ms cho servo quay đến góc đó rồi tới bước tiếp theo
  }
  Blynk.virtualWrite(V4, distance);
}
//-----------------------------------------------------------------------------

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init() {
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
