/*
Group 38
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <esp_now.h>
//#include <vl53l4cx_class.h>
#include <stdio.h>
#include <string.h>
#include "html510.h"
#include "vive510.h"
#include "webpage.h"

#define TEAM_NUM 38

// assumes m1 on left, m2 on right from back of bot
#define M1_PWM_PIN 18
#define M1_H1_PIN 0
#define M2_PWM_PIN 19
#define M2_H1_PIN 1

#define VIVE1_PIN 4
#define VIVE2_PIN 5
#define IR1_PIN 6
#define IR2_PIN 7
#define WALL_PIN 10

// #define TOF_SCL 11
// #define TOF_SDA 10
// #define TOF_XSHUT 9

#define M_FREQ 2000
#define RES_BITS 10

unsigned int GAME_UDPPORT = 2510;

#define ESPNOW_CHANNEL 1
#define MAC_RCV \
  { 0xC8, 0xF0, 0x9E, 0xF6, 0xE0, 0xC8 }

const char* ssid = "TP-Link_E0C8";
const char* pass = "52665134";
IPAddress src_IP(192, 168, 1, 154);
IPAddress broadcast_IP(192, 168, 0, 255);
HTML510Server h(80);
WiFiUDP UDPServer;

Vive510 vive1(VIVE1_PIN);
Vive510 vive2(VIVE2_PIN);

// vive coords + derived heading of bot
uint16_t v1x, v1y, v2x, v2y;
float heading;

//VL53L4CX sensor_vl53l4cx_sat(&Wire, TOF_XSHUT);
// closest tof obj dist (mm)
int closest_tof_dist;

// police car vive coords
int pc_x, pc_y;

hw_timer_t* timer = NULL;

esp_now_peer_info_t peer1 = {
  .peer_addr = MAC_RCV,
  .channel = ESPNOW_CHANNEL,
  .encrypt = false,
};
uint8_t espnow_msg[1];

// sliding window avg of 5 ir freq measurements
int ir1_freq[5] = { 0, 0, 0, 0, 0 }, ir2_freq[5] = { 0, 0, 0, 0, 0 };
int ir1_freq_idx = 0, ir2_freq_idx = 0;
int ir1_freq_sum = 0, ir2_freq_sum = 0;
unsigned long ir1_rise, ir2_rise;

/*
 * 0: stop
 * 1: wall follow
 * 2: police car push
 * 3: fake trophy nav
 * 4: real trophy nav
 */
short mode = 0;

// movement setting functions
void stop() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void xmlTurn() {
  int dir = h.getVal();
  turn(dir, 1023);
}

void turn(int dir, int duty) {
  // m1 backwards, m2 forwards for left, vice versa for right
  // val param = 0 is left, 1 is right
  digitalWrite(M1_H1_PIN, 1 - dir);
  digitalWrite(M2_H1_PIN, dir);

  ledcWrite(0, duty);
  ledcWrite(1, duty);
}

void xmlStraight() {
  int dir = h.getVal();
  straight(dir, 1023);
}

void straight(int dir, int duty) {
  // val param 0 forwards, 1 backwards
  digitalWrite(M1_H1_PIN, 1 - dir);
  digitalWrite(M2_H1_PIN, 1 - dir);

  ledcWrite(0, duty);
  ledcWrite(1, duty);
}

// comm handlers
void handleRoot() {
  h.sendhtml(body);
}

//handle all udp server actions(rcv and transmit vive coords)
void handleUDPServer() {
  const int UDP_PACKET_SIZE = 14;  // can be up to 65535          
  uint8_t packetBuffer[UDP_PACKET_SIZE];

  int cb = UDPServer.parsePacket();  // if there is no message cb=0
  while (cb) {
    packetBuffer[13] = 0;  // null terminate string
    //rcv police car
    UDPServer.read(packetBuffer, UDP_PACKET_SIZE);
    packetBuffer[2] = 0;
    int x = atoi((char*)packetBuffer + 3);  // ##,####,#### 2nd indexed char
    int y = atoi((char*)packetBuffer + 8);  // ##,####,#### 7th indexed char
    if (strcmp((char*)packetBuffer, "00") == 0) {
      pc_x = x;
      pc_y = y;
    }
    cb = UDPServer.parsePacket();
  }
  // TODO FIX UDP SENDING BELOW
  // currently segfaults at packet sending

  // char outPBuffer[13];
  // //memset(outPBuffer, 0, sizeof (outPBuffer));
  // int wx = v1x < 10000 ? v1x : 0;
  // int wy = v1y < 10000 ? v1y : 0;
  // sprintf(outPBuffer, "%2d:%4d,%4d", TEAM_NUM, wx, wy);
  // UDPServer.beginPacket(broadcast_IP, GAME_UDPPORT);
  // UDPServer.write((uint8_t*) outPBuffer, 13);
  // UDPServer.endPacket();
}

// would really like to merge these...
// ir interrupts on digital read change
void ir1_onchange() {
  unsigned long now = micros();
  double diff = (now - ir1_rise);
  if (diff < 500 || diff > 1000000) return;
  diff /= 1000.f;
  ir1_freq_sum -= ir1_freq[ir1_freq_idx];
  ir1_freq[ir1_freq_idx] = (int)(1000.f / diff);
  ir1_freq_sum += ir1_freq[ir1_freq_idx];
  ir1_freq_idx = (ir1_freq_idx + 1) % 5;
  ir1_rise = now;
}

void ir2_onchange() {
  unsigned long now = micros();
  double diff = (now - ir2_rise);
  if (diff < 500 || diff > 1000000) return;
  diff /= 1000.f;
  ir2_freq_sum -= ir2_freq[ir2_freq_idx];
  ir2_freq[ir2_freq_idx] = (int)(1000.f / diff);
  ir2_freq_sum += ir2_freq[ir2_freq_idx];
  ir2_freq_idx = (ir2_freq_idx + 1) % 5;
  ir2_rise = now;
}

// timer interrupt for 1hz vive coord report and rcv
void IRAM_ATTR one_hz_timer() {
  handleUDPServer();
}

// change mode based on user web req, log packet
void switchMode() {
  mode = h.getVal();
  //esp_now_send(peer1.peer_addr, espnow_msg, sizeof(espnow_msg));
}

void setup() {
  // setup ledc with channel 0 and 1, 2khz, 10 bits res for both motors
  ledcSetup(0, M_FREQ, RES_BITS);
  ledcAttachPin(M1_PWM_PIN, 0);
  ledcSetup(1, M_FREQ, RES_BITS);
  ledcAttachPin(M2_PWM_PIN, 1);
  // setup dir control pins for both motors
  pinMode(M1_H1_PIN, OUTPUT);
  pinMode(M2_H1_PIN, OUTPUT);
  // setup vive
  vive1.begin();
  vive2.begin();
  // setup IR
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  // setup TOF
  // Wire.setPins(TOF_SDA, TOF_SCL);
  // Wire.begin();
  // sensor_vl53l4cx_sat.begin();
  // sensor_vl53l4cx_sat.VL53L4CX_Off();
  // sensor_vl53l4cx_sat.InitSensor(0x12);
  // sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  // ir frequency measurements
  attachInterrupt(digitalPinToInterrupt(IR1_PIN), ir1_onchange, RISING);
  attachInterrupt(digitalPinToInterrupt(IR2_PIN), ir2_onchange, RISING);

  pinMode(WALL_PIN, INPUT_PULLUP);

  // setup udp send/receive for vive send and police car coord rcv + web server
  Serial.begin(9600);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.config(src_IP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("Use this URL to connect: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");

  UDPServer.begin(GAME_UDPPORT);

  // setup timer autofire for vive reporting at 1hz
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &one_hz_timer, false);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
  // setup ESPNOW for msg rcv emits
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
  }
  if (esp_now_add_peer(&peer1) != ESP_OK) {
    Serial.println("Failed to add peer");
  }
  // 1 byte with team num
  espnow_msg[0] = TEAM_NUM;
  // FIXME not integrated with movement keys since they should be temporary?

  // html page handlers(only called on transition states)
  h.begin();
  h.attachHandler("/ ", handleRoot);
  // left in for movement testing
  h.attachHandler("/straight?val=", xmlStraight);
  h.attachHandler("/turn?val=", xmlTurn);
  h.attachHandler("/stop", stop);
  h.attachHandler("/switchmode?val=", switchMode);
}

unsigned long last_turn=0;

// the loop function runs over and over again forever
void loop() {
  // vive coord reading
  if (vive1.status() == VIVE_RECEIVING) {
    v1x = vive1.xCoord();
    v1y = vive1.yCoord();
  } else {
    vive1.sync(15);
  }
  if (vive2.status() == VIVE_RECEIVING) {
    v2y = vive2.xCoord();
    v2x = vive2.yCoord();
  } else {
    vive2.sync(15);
  }
  Serial.print("vive 1 x: ");
  Serial.println(v1x);
  Serial.print("vive 1 y: ");
  Serial.println(v1y);
  Serial.print("vive 2 x: ");
  Serial.println(v2x);
  Serial.print("vive 2 y: ");
  Serial.println(v2y);
  // heading calculation
  heading = atan2(v2y - v1y, v2x - v1x);
  // TOF reading
  // VL53L4CX_MultiRangingData_t MultiRangingData;
  // VL53L4CX_MultiRangingData_t* pMultiRangingData = &MultiRangingData;
  // uint8_t NewDataReady = 0;
  // int no_of_object_found = 0, j;
  // int status =
  //     sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  // if (!status && NewDataReady != 0) {
  //   status =
  //       sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
  //   if (!status) {
  //     no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
  //     for (j = 0; j < no_of_object_found; j++) {
  //       if (pMultiRangingData->RangeData[j].RangeStatus == 0) {
  //         // TODO filter and find closest obj dist, update closest_tof_dist
  //         //Serial.println(pMultiRangingData->RangeData[j].RangeMilliMeter);
  //         // int16_t dist = pMultiRangingData->RangeData[j].RangeMilliMeter;
  //       }
  //     }
  //     status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
  //   }
  // }
  // ir readings for left and right beacons for use in below task behavior
  int ir1_freq_avg = ir1_freq_sum / 5;
  int ir2_freq_avg = ir2_freq_sum / 5;

  Serial.print("ir1: ");
  Serial.println(ir1_freq_avg);
  Serial.print("ir2: ");
  Serial.println(ir2_freq_avg);

  //clear measurements if long time
  int ms_ir1_rise = ir1_rise / 1000;
  int ms_ir2_rise = ir2_rise / 1000;
  if (millis() - ms_ir1_rise > 1000 && millis() - ms_ir2_rise > 1000) {
    ir1_freq_sum = 0;
    ir2_freq_sum = 0;
    int i;
    for (i = 0; i < 5; i++) {
      ir2_freq[i] = 0;
      ir1_freq[i] = 0;
    }
  }
  // comms stuff
  h.serve();

  // depending on current task mode (wall follow, police car push, trophy
  // locate) change behavior
  switch (mode) {
    case 0:
      {
        stop();
        break;
      }
    case 1:
      {
        // wall follow
        //TOF value = tof_val
        //minimum distance to wall = min_dist
        straight(0, 1023);
        // delay(10);
        

        if (digitalRead(WALL_PIN) == 0 && millis() - last_turn > 1000) {
          straight(1, 1023);
          delay(250);
          turn(0 ,1023);
          delay(440);
          straight(0, 1023);
          last_turn = millis();
        }
        break;
      }
    case 2:
      {
        // police car push
        break;
      }
    case 3:
      {
        // fake trophy locate
        if (ir1_freq_avg > 0 && ir1_freq_avg < 100 && ir2_freq_avg > 0 && ir2_freq_avg < 100) {
          straight(0, 850);
        } else if (ir1_freq_avg > 0 && ir1_freq_avg < 100) {
          turn(0, 850);
        } else {
          turn(1, 850);
        }
        break;
      }
    case 4:
      {
        // real trophy locate
        if (ir1_freq_avg > 250 && ir2_freq_avg > 250) {
          straight(0, 850);
        } else if (ir1_freq_avg > 250) {
          turn(0, 850);
        } else {
          turn(1, 850);
        }
        break;
      }
  }
  delay(500);
}
