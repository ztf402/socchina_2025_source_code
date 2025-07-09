#pragma once
#include <Arduino.h>
#include <ch32yyxx.h>
#define LED_PIN GPIO_Pin_7
#define LED_GPIO GPIOD
#define TempSensorPin      PA_2
#define PhSensorPin        PA_3
#define TdsSensorPin       PA_4
#define TurbiditySensorPin PA_5

//#define DEBUG_Serial_ENABLE
void Led_init(){
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED_PIN; // PD7
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(LED_GPIO,LED_PIN);
}
void Led_toggle(){
    uint8_t status_led=GPIO_ReadOutputDataBit(LED_GPIO,LED_PIN);
    GPIO_WriteBit(LED_GPIO,LED_PIN,status_led?Bit_SET:Bit_RESET);
}
void init_sensor(){
    pinMode(TempSensorPin, INPUT_ANALOG); // 温度传感器
    pinMode(PhSensorPin, INPUT_ANALOG); // pH传感器
    pinMode(TdsSensorPin, INPUT_ANALOG); // TDS传感器
    pinMode(TurbiditySensorPin, INPUT_ANALOG); // 浊度传感
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

#define VREF_TDS 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;


float get_tds()
{
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF_TDS / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
  }
  #ifdef DEBUG_Serial_ENABLE
  Serial.print("TDS Value: ");
    Serial.print(tdsValue);
    Serial.println(" ppm");
    Serial.print("Average Voltage: ");
    Serial.print(averageVoltage);
    Serial.println(" V");
  #endif
  return tdsValue; //return the tds value
}

float TU=0.0;
float TU_value=0.0;
float TU_calibration=0.0;
float temp_data=25.0;
float K_Value=3347.19;
float get_turbidity() {
  int sensorValue = analogRead(TurbiditySensorPin);// read the input on analog pin 0:
  float TU = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  TU_calibration=-0.0192*(temp_data-25)+TU;  
  TU_value=-865.68*TU_calibration + K_Value;
  
  if(TU_value<=0){TU_value=0;}
  if(TU_value>=3000){TU_value=3000;}
    #ifdef DEBUG_Serial_ENABLE
    Serial.print("Turbidity Value: ");
    Serial.print(TU_value);
    Serial.println(" NTU");
    Serial.print("Sensor Value: ");
    Serial.print(sensorValue);
    Serial.println(" ADC");
    Serial.print("Voltage: ");
    Serial.print(TU_calibration);
    Serial.println(" V");
    #endif
    TU_value/=30.0;
  return TU_value; // 返回浊度值
}


#define Offset 0.00            //deviation compensate
unsigned long int avgValue;     //Store the average value of the sensor feedback
float get_ph()
{
  int buf[10];                //buffer for read analog
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(PhSensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        int temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue+Offset;                      //convert the millivolt into pH value
  #ifdef DEBUG_Serial_ENABLE
  Serial.print("pH Value: ");
    Serial.print(phValue);
    Serial.println(" pH");
  Serial.print("Average Voltage: ");
    Serial.print(avgValue * 5.0 / 1024 / 6);
    Serial.println(" V");
    #endif
  if(phValue>0&&phValue<14){
    return phValue; 
  }
  else{
    return -1; 
  }
}

float get_temp()
{
  int temp_value = analogRead(TempSensorPin); // 读取温度传感器的模拟值
  float voltage = (temp_value * 3.3) / 1024.0; // 将模拟值转换为电压
  float temperature = (voltage - 0.5) * 100.0; // 将电压转换为摄氏度
  return temperature; // 返回温度值
}