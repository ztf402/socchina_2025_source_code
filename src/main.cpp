#include <Arduino.h>
#include <U8g2lib.h>
#include <uart.hpp>
#include <ch32yyxx_rtc.h>
#include "terminal.hpp"
#include <Wire.h>
#include <my_peripheral.hpp>
UART uart5_my(UART5, GPIOC, GPIO_Pin_12, GPIOD, GPIO_Pin_2); // UART5 TX: PC12, RX: PD2
enum class Status { OK, ERR, UNKNOWN };
namespace at {
namespace parser {
namespace detail {

inline void assignOne(const std::string& s, Status& out) {
    std::string t;
    for (char c : s) {
        if (!std::isspace((unsigned char)c) && c != '"')
            t.push_back(c);
    }
    if (t == "OK")       out = Status::OK;
    else if(t=="READY")   out=Status::OK;
    else if (t == "ERROR") out = Status::ERR;
    else                 out = Status::UNKNOWN;
}

} // namespace detail
} // namespace parser
} // namespace at
#include <at_parser.hpp>
#include <rtc_32bit.hpp>

Terminal terminal;
std::string ipaddress;
std::string rxdata,termial_data;
char buffer[128]={0};
U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
long temp=0,flag=0;
bool network_status=false;
bool at_command_status=false,sim_status=false;
uint8_t signal_quality=0,temp_short=0,network_type=0;

void readserial(std::string *data,uint32_t time){
  data->clear();
  uint64_t temp=millis();
  bool isFirstRead = true;
  while(millis()-temp<time){
    std::string chunk=uart5_my.readAll();
    if (!chunk.empty()) {
        *data += chunk;
        temp = millis(); 
    }
    if(isFirstRead && !chunk.empty()) {
        isFirstRead = false; // 第一次读取后不再清空
    }
  }
}

void ip_address_init(){
  uart5_my.println("AT+MIPCALL?"); // 检查拨号状态
  readserial(&rxdata,150);
  at::parser::parseAtResponse(rxdata,"+MIPCALL",temp_short,ipaddress);
  Serial.println(rxdata.c_str());

  if((ipaddress.compare("0.0.0.0")||ipaddress.length()>8)&&sim_status&&temp_short) {
    Terminal_setFontSize(terminal,1);
    Terminal_print(terminal,"IP address:");
    termial_data.clear();
    Terminal_print(terminal,ipaddress.c_str());
    Terminal_draw(terminal);
    Terminal_draw(terminal);
    network_status=true;
  }
  else{
    ipaddress.clear();
    Terminal_print(terminal,"Tring to get IP address...");
    Terminal_draw(terminal);
    uart5_my.println("AT+MIPCALL=1"); // 尝试拨号
    readserial(&rxdata,350);
    at::parser::parseAtResponse(rxdata,"+MIPCALL",ipaddress);
    Terminal_print(terminal,ipaddress.c_str());
    Serial.println(rxdata.c_str());
    Serial.println(ipaddress.c_str());

    if((ipaddress.compare("0.0.0.0")||ipaddress.length()>8)&&sim_status) {
      Terminal_setFontSize(terminal,1);
      Terminal_print(terminal,"IP address:");
      Terminal_print(terminal,ipaddress.c_str());
      Terminal_draw(terminal);
      network_status=true;
    }
    else{
      Terminal_print(terminal,"No IP address");
      Terminal_draw(terminal);
    }
  }//IP addres
}

void ip_off(){
  uart5_my.println("AT+MIPCALL=0"); // 关闭拨号
  readserial(&rxdata,150);
  at_command_status=at::parser::isOkResponse(rxdata);
  if(at_command_status) {
    Terminal_print(terminal,"IP off");
    Terminal_draw(terminal);
    network_status=false;
  }
  else{
    Terminal_print(terminal,"IP off failed");
    Terminal_draw(terminal);
  }
}

void mqtt_open(){
  if(!network_status) return;
  else{
    uart5_my.println("AT+MQTTOPEN=1,\"(Your IP)\",1883,0,60"); // 打开MQTT连接
    readserial(&rxdata,550);
    at_command_status=at::parser::isOkResponse(rxdata);
    if(at_command_status) {
      Terminal_print(terminal,"MQTT open success");
      Terminal_draw(terminal);
    }
    else{
      Terminal_print(terminal,"MQTT open failed");
      Terminal_draw(terminal);
    }//MQTT open
  }
}

void mqtt_sub(const char * topic){
  if(!network_status) return;
  else{
    sprintf(buffer,"AT+MQTTSUB=1,\"%s\",0",topic); // 订阅主题
    uart5_my.println(buffer); // 订阅主题
    readserial(&rxdata,150);
    at_command_status=at::parser::isOkResponse(rxdata);
    if(at_command_status) {
      sprintf(buffer,"MQTT sub: \n %s",topic);
      Terminal_print(terminal,buffer);
      Terminal_draw(terminal);
    }
    else{
      Terminal_print(terminal,"MQTT sub failed");
      Terminal_draw(terminal);
    }//MQTT sub
  }
}

void mqtt_pub(const char * topic, const char * message){
  if(!network_status) return;
  else{
    sprintf(buffer,"AT+MQTTPUB=1,\"%s\",0,1,\"%s\"",topic,message); // 发布消息
    uart5_my.println(buffer); // 发布消息
    readserial(&rxdata,150);
    at_command_status=at::parser::isOkResponse(rxdata);
    if(at_command_status) {
      sprintf(buffer,"MQTT pub: \n %s",topic);
      Terminal_print(terminal,buffer);
      Terminal_draw(terminal);
    }
    else{
      Terminal_print(terminal,"MQTT pub failed");
      Terminal_draw(terminal);
    }//MQTT pub
  }
}
void mqtt_close(){
  if(!network_status) return;
  else{
    uart5_my.println("AT+MQTTCLOSE=1"); // 关闭MQTT连接
    readserial(&rxdata,150);
    at_command_status=at::parser::isOkResponse(rxdata);
    if(at_command_status) {
      Terminal_print(terminal,"MQTT close success");
      Terminal_draw(terminal);
    }
    else{
      Terminal_print(terminal,"MQTT close failed");
      Terminal_draw(terminal);
    }//MQTT close
  }
}
std::string extract_quoted(const std::string& src) {
    size_t first = src.find('\"');
    if (first == std::string::npos) return "";
    size_t second = src.find('\"', first + 1);
    if (second == std::string::npos) return "";
    return src.substr(first + 1, second - first - 1);
}

void setup(void) {
  Led_init();
  pinMode(9, OUTPUT);
  digitalWrite(9, 0);
  
  init_sensor();

  Serial.begin(115200);
  uart5_my.begin(115200);
  Serial.println("initing...");
  Led_toggle();
  Serial.println("LED_TOGGLE");
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_10x20_tr);
  Serial.println("U8g2_Inited");
  Terminal_init(terminal,u8g2,0,0,128,128,12);
  Terminal_setFontSize(terminal,4);
  Terminal_print(terminal,"Welcome!");
  Terminal_draw(terminal);
  Terminal_setFontSize(terminal,1);
  Terminal_print(terminal,"Using L610 AT");
  Terminal_draw(terminal);
  Terminal_print(terminal,"Waiting 4G...");
  Terminal_draw(terminal);
  temp=0;
  Terminal_draw(terminal);

  uart5_my.println("AT");
  readserial(&rxdata,6500);
  at_command_status=at::parser::isOkResponse(rxdata);
  if(at_command_status!=true) {
    Terminal_print(terminal,"No AT devices");
    Terminal_draw(terminal);
  }
  else{
    Terminal_print(terminal,"AT command ready");
    Terminal_draw(terminal);
  }//at command

  uart5_my.println("AT+CPIN?");
  readserial(&rxdata,150);
  at::parser::parseAtResponse(rxdata,"+CPIN",sim_status);
  if(sim_status&&at_command_status) {
    Terminal_print(terminal,"SIM ready");
    Terminal_draw(terminal);
  }
  else{
    Terminal_print(terminal,"No SIM card");
    Terminal_draw(terminal);
  }//sim status

  uart5_my.println("AT+CSQ");
  readserial(&rxdata,150);
  at::parser::parseAtResponse(rxdata,"+CSQ",signal_quality,temp_short);
  if((signal_quality>0||signal_quality<32)&&sim_status) {
    termial_data.clear();
    Terminal_setFontSize(terminal,1);
    Terminal_print(terminal,"Signal quality:");
    //Terminal_print(terminal,rxdata.c_str());
    if(signal_quality==0) termial_data="No signal";
    else if(signal_quality==99) termial_data="No service";
    else if(signal_quality<10) termial_data="Weak signal";
    else if(signal_quality<20) termial_data="Good signal";
    else if(signal_quality<30) termial_data="Strong signal";
    else termial_data="Excellent signal!";
    Terminal_print(terminal,termial_data.c_str());
    Terminal_draw(terminal);
  }
  else{
    Terminal_print(terminal,"No signal");
    Terminal_draw(terminal);
  }//signal quality

  uart5_my.println("AT+CREG?");
  readserial(&rxdata,150);
  at::parser::parseAtResponse(rxdata,"+CREG",temp_short,network_type);
  if((network_type==0||network_type==4||network_type==3)&&(temp_short>-1&&temp_short<3)&&sim_status) {
    Terminal_setFontSize(terminal,1);
    Terminal_print(terminal,"Network type:");
    termial_data.clear();
    if(network_type==0) termial_data="GSM";
    else if(network_type==1) termial_data="GPRS";
    else if(network_type==2) termial_data="EDGE";
    else if(network_type==3) termial_data="LTE";
    else if(network_type==4) termial_data="NR";
    else termial_data="Unknown";
    Terminal_print(terminal,termial_data.c_str());
    Terminal_draw(terminal);
  }
  else{
    Terminal_print(terminal,"No network");
    Terminal_draw(terminal);
  }//network type

  ip_address_init();
  
  uart5_my.println("AT+MQTTUSER=1,\"L610\",\"L610142536\""); // 设置MQTT用户名和密码
  readserial(&rxdata,150);
  at_command_status=at::parser::isOkResponse(rxdata);
  if(at_command_status) {
    Terminal_print(terminal,"MQTT user set");
    Terminal_draw(terminal);
  }
  else{
    Terminal_print(terminal,"MQTT user set failed");
    Terminal_draw(terminal);
  }//MQTT user
  
  rtc_init();
  Terminal_print(terminal,"rtc initing...");
  Terminal_draw(terminal);
  temp=millis();
  while(RTC_GetCounter()==0||millis()-temp<1000);
  if(millis()-temp>999&&RTC_GetCounter()==0) Terminal_print(terminal,"rtc init failed");
  else Terminal_print(terminal,"rtc inited");
  Terminal_draw(terminal);
  rxdata.clear();
  mqtt_open();

  uart5_my.println("AT+CCLK?");
  readserial(&rxdata,150);
  uint8_t hour_offset=0;
  int year=0, month=0, day=0, hour=0, min=0, sec=0;
  std::string time_str = extract_quoted(rxdata);
  Serial.printf("Time string: %s\n", time_str.c_str());
  if(sscanf(time_str.c_str(), "%2d/%2d/%2d,%2d:%2d:%2d%3s",
    &year, &month, &day, &hour, &min, &sec, buffer) == 7) {
    year += 2000;
    Serial.printf("RTC set to: %04d-%02d-%02d %02d:%02d:%02d tz:%d\n", year, month, day, hour, min, sec, hour_offset);
    rtc_setTime(year, month, day, hour, min, sec, 8); // 设置RTC时间，假设时区为东八区
    rxdata.clear();
    Terminal_print(terminal,"RTC set to:");
    Terminal_draw(terminal);
    char buf1[30], buf2[30];
    timestamp_to_string(rtc_get_epoch(), buf1,buf2);
    Terminal_print(terminal, buf1);
    Terminal_print(terminal, buf2);
    Terminal_draw(terminal);
  } else {
    Serial.println("Failed to parse time string!");
  }
  mqtt_sub("poweron_time");
  mqtt_pub("poweron_time", time_str.c_str()); // 发布消息
  mqtt_close();
  ip_off();

  delay(500);
  u8g2.clearBuffer();
  Terminal_deinit(terminal);
  u8g2.sendBuffer();
}

void loop(void) {
  char buf1[45],buf2[20],buf3[80],buf_ph[20],buf_temp[20],buf_tds[20],buf_turbidity[20];
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_8x13_mf);
  float ph_value=0,temp_value=0,tds_value=0,turbidity_value=0;
  ph_value=get_ph();
  temp_value=get_temp();
  tds_value=get_tds();
  turbidity_value=get_turbidity();
  char ph_str[10], temp_str[10], tds_str[10], turbidity_str[10];

  int ph_int = (int)ph_value;
  int ph_frac = (int)((ph_value - ph_int) * 100); // 保留2位小数
  if (ph_frac < 0) ph_frac = -ph_frac;
  snprintf(ph_str, sizeof(ph_str), "%d.%02d", ph_int, ph_frac);
  // ...同理处理 temp_value, tds_value, turbidity_value ...
  int temp_int = (int)temp_value;
  int temp_frac = (int)((temp_value - temp_int) * 100);
  if (temp_frac < 0) temp_frac = -temp_frac;
  snprintf(temp_str, sizeof(temp_str), "%d.%02d", temp_int, temp_frac);

  int tds_int = (int)tds_value;
  int tds_frac = (int)((tds_value - tds_int) * 100);
  if (tds_frac < 0) tds_frac = -tds_frac;
  snprintf(tds_str, sizeof(tds_str), "%d.%02d", tds_int, tds_frac);

  int turbidity_int = (int)turbidity_value;
  int turbidity_frac = (int)((turbidity_value - turbidity_int) * 100);
  if (turbidity_frac < 0) turbidity_frac = -turbidity_frac;
  snprintf(turbidity_str, sizeof(turbidity_str), "%d.%02d", turbidity_int, turbidity_frac);

  sprintf(buf_ph, "PH:%s", ph_str);
  sprintf(buf_temp, "Temp:%s", temp_str);
  sprintf(buf_tds, "TDS:%s", tds_str);
  sprintf(buf_turbidity, "Tur:%s%", turbidity_str);
  
  Serial.printf("%s%s%s%s",ph_str,temp_str,tds_str,turbidity_str);
  u8g2.drawStr(0, 15, buf_ph);
  u8g2.drawStr(0, 30, buf_temp);
  u8g2.drawStr(0, 45, buf_tds);
  u8g2.drawStr(0, 60, buf_turbidity);
  timestamp_to_string(rtc_get_epoch(), buf1,buf2);
  uint16_t second=rtc_get_second();
  uint16_t minute=rtc_get_minute();
  if(second==0&&minute%2==0){
    Terminal_init(terminal,u8g2,0,0,128,128,12);
    Terminal_setFontSize(terminal,1);
    ip_address_init();
    mqtt_open();
    mqtt_sub("last_update_time");
    mqtt_sub("ph_value");
    mqtt_sub("temp_value");
    mqtt_sub("tds_value");
    mqtt_sub("turbidity_value");
    mqtt_pub("ph_value", buf_ph); // 发布消息
    mqtt_pub("temp_value", buf_temp); // 发布消息
    mqtt_pub("tds_value", buf_tds); // 发布消息
    mqtt_pub("turbidity_value", buf_turbidity); // 发布消息
    sprintf(buf3,"%s %s",buf1,buf2); // 发布消息
    mqtt_pub("last_update_time", buf3); // 发布消息
    mqtt_close();
    ip_off();
    delay(500);
    Terminal_deinit(terminal);
    u8g2.clearBuffer();
  }
  u8g2.setFont(u8g2_font_7x13_me);
  u8g2.drawStr(0, 85, buf1);
  u8g2.drawStr(0, 98, buf2);
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 110, "last update time:");
  u8g2.drawStr(0, 118, buf3);
  u8g2.sendBuffer();
  delay(150);
}

