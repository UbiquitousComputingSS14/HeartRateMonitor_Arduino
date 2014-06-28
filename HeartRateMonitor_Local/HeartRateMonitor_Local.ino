#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

#define LIN_OUT 1 // use the log output function
//#define FFT_N 256 // set to 256 point fft
#include <FFT.h>
int fftIndex = 0;

#define DEFAULT_INTERVAL 125
uint8_t sampleInterval = DEFAULT_INTERVAL;

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

struct Led {
  int pin;
  int status;
};

struct Led redLed = {3, LOW};
struct Led irLed = {2, LOW};

void configureSensor()
{
  tsl.setGain(TSL2561_GAIN_16X);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_MANUAL);
}

void configureLeds()
{
  pinMode(redLed.pin, OUTPUT);
  pinMode(irLed.pin, OUTPUT);
}

void toggleLed(struct Led led)
{
  if (led.status == HIGH)
    led.status = LOW;
  else if (led.status == LOW)
    led.status = HIGH;
    
  digitalWrite(led.pin, led.status);
}

void printSensorDetails()
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  
  Serial.print("settings: ");
  
  Serial.print("sensor "); Serial.print(sensor.name); Serial.print(" ");
  Serial.print("id "); Serial.print(sensor.sensor_id); Serial.print(" ");
  Serial.print("maxValue "); Serial.print(sensor.max_value); Serial.print(" ");
  Serial.print("minValue "); Serial.print(sensor.min_value); Serial.print(" ");
  Serial.print("resolution "); Serial.print(sensor.resolution); Serial.print(" ");
  Serial.print("sampleInterval "); Serial.print(sampleInterval); Serial.print(" ");
  
  Serial.println("");
}

void printLuminosity(uint16_t broadband, uint16_t ir)
{
  Serial.print("data: ");
  
  Serial.print("broadband");
  Serial.print(" ");
  Serial.print(broadband);
  Serial.print(" ");
  Serial.print("ir");
  Serial.print(" ");
  Serial.print(ir);
  Serial.println();
}

void luminosity()
{
  uint16_t broadband, ir;
  tsl.getLuminosity(&broadband, &ir);
  
  printLuminosity(broadband, ir);
}

void luminosityManualTiming()
{
  uint16_t broadband, ir;
  
  tsl.beginIntegrationCycle();
  delay(sampleInterval);
  tsl.stopIntegrationCycle(&broadband, &ir);
  
  //fft_input[fftIndex++] = broadband;
  //fft_input[fftIndex++] = 0;
  
  printLuminosity(broadband, ir);
}

void readData()
{
  char data[30];
  
  if (Serial.available() > 0) {
    int bytesRead = Serial.readBytesUntil('\n', data, 30);
    if (bytesRead == 0)
      return;
    
    String readData = String(data);
    readData = readData.substring(0, bytesRead);
    readData.trim();
    
    if (readData == "get settings") {
      printSensorDetails();
    } else if (readData.startsWith("set sampleInterval", 0)) {
      readData.replace("set sampleInterval", "");
      readData.trim();
      
      sampleInterval = readData.toInt();
    }
  }
}

void setup()
{
  Serial.begin(9600);

  if (!tsl.begin()) {
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  configureSensor();
  configureLeds();
  
  toggleLed(redLed);
}

void loop()
{ 
  luminosityManualTiming();
  
  /*if (fftIndex == 512) {
    fft_window();
    fft_run();
    //fft_mag_lin();
    
    Serial.println("==========");
    
    for (int i = 0; i < 128; ++i) {
      Serial.print(i); Serial.print(": ");
      Serial.println(fft_lin_out[i]);
    }
    
    fftIndex = 0; 
    
    Serial.println("==========");
  }*/
  
  readData();
}
