
//Inicializacion de Mision principal bpm 280
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

int paquete = 0;

//Inicializacion de aceletrometro MPU 6050
//#include <Adafruit_MPU6050.h>


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


//Inicializacion del medidor de particulas dsm501a
#include<string.h>
byte buff[2];
int pin = 8;//DSM501A input D8
unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 1000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

int i=0;



void setup() {

  //Setup de Mision principal bpm 280
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test2"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

//Setup del acelerometro MPU 6050
Serial.begin(9600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:        
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

//Setup medidor de paticulas dsm501a
//Serial.begin(9600);
  pinMode(8,INPUT);
  starttime = millis(); 


}

void loop() {


  //loop de Mision principal bpm 280
    Serial.print (paquete);
    Serial.print(" , ");
    Serial.print(bmp.readTemperature());
    Serial.print(" , ");
    Serial.print(bmp.readPressure());
   Serial.print(" , ");
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    
    
    
    
    
   
  //loop acelerometro MPU 6050
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
 // Serial.print("Acceleration X: ");
  Serial.print(" , ");
  Serial.print(a.acceleration.x);
  //Serial.print(", Y: ");
  Serial.print(" , ");
  Serial.print(a.acceleration.y);
 // Serial.print(", Z: ");
  Serial.print(" , ");
  Serial.print(a.acceleration.z);
  //Serial.println(" m/s^2");

  //Serial.print("Rotation X: ");
  Serial.print(" , ");
  Serial.print(g.gyro.x);
 // Serial.print(", Y: ");
  Serial.print(" , ");
  Serial.print(g.gyro.y);
  //Serial.print(", Z: ");
  Serial.print(" , ");
  Serial.print(g.gyro.z);
  //Serial.println(" rad/s");

 // Serial.print("Temperature: ");
 // Serial.print(temp.temperature);
 // Serial.println(" degC");

  //Serial.println("");
 // delay(500);


  //loop del detector de particulas dsm501a
   duration = pulseIn(pin, LOW);
  lowpulseoccupancy += duration;
  endtime = millis();
  if ((endtime-starttime) > sampletime_ms)
  {
    ratio = (lowpulseoccupancy-endtime+starttime + sampletime_ms)/(sampletime_ms*10.0);  // Integer percentage 0=>100
   concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve

   // Serial.print("lowpulseoccupancy:");
   // Serial.print(lowpulseoccupancy);
   // Serial.print("\n");
   // Serial.print("ratio:");
   // Serial.print("\n");
   // Serial.print(ratio);
   // Serial.print("DSM501A:");
   Serial.print(" , ");
    Serial.print(concentration);
   // Serial.print(";\n\n");

    lowpulseoccupancy = 0;
    starttime = millis();
  }

 Serial.print(" , ");
    Serial.println("Salzillo");
    //Serial.println("");

    paquete++;
    delay(1000);


}
