/*
Group 38
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <esp_now.h>
#include <vl53l4cx_class.h>
#include "html510.h"
#include "vive510.h"
#include "webpage.h"

#define TEAM_NUM 38

// assumes m1 on left, m2 on right from back of bot
#define M1_PWM_PIN 45
#define M1_H1_PIN 40
#define M1_H2_PIN 41
#define M2_PWM_PIN 35
#define M2_H1_PIN 36
#define M2_H2_PIN 37

#define VIVE1_PIN 1
#define VIVE2_PIN 2

#define IR1_PIN 3
#define IR2_PIN 4

#define TOF_SCL 11
#define TOF_SDA 10
#define TOF_XSHUT 9

#define M_FREQ 2000
#define RES_BITS 10

#define PC_UDPPORT 2510
#define VIVE_TGT_UDPPORT 2808

#define ESPNOW_CHANNEL 1
// FIXME fill in mac address of tgt
#define MAC_RCV \
  { 0x84, 0xF7, 0x03, 0xA8, 0xBE, 0x30 }

const char* ssid = "TP-Link_E0C8";
const char* pass = "52665134";
IPAddress src_IP(192, 168, 0, 154);
// FIXME fill in tgt_IP with dest for vive coords
IPAddress tgt_IP(192, 168, 0, 0);
HTML510Server h(80);
WiFiUDP PC_UDPServer;
WiFiUDP Vive_UDPServer;

Vive510 vive1(VIVE1_PIN);
Vive510 vive2(VIVE2_PIN);

// vive coords + derived heading of bot
uint16_t v1x, v1y, v2x, v2y;
float heading;

VL53L4CX sensor_vl53l4cx_sat(&Wire, TOF_XSHUT);
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
uint8_t espnow_msg[200];

// sliding window avg of 5 ir freq measurements
// to get avg divide sums by 5
int ir1_freq[5] = {0}, ir2_freq[5] = {0}, ir1_freq_idx, ir2_freq_idx,
    ir1_freq_sum, ir2_freq_sum;
unsigned long ir1_rise, ir2_rise;

/*
 * 0: stop
 * 1: wall follow
 * 2: police car push
 * 3: trophy locate
 */
short mode = 0;

// movement setting functions
void stop() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void turn() {
  // m1 backwards, m2 forwards for left, vice versa for right
  // val param = 0 is left, 1 is right
  int dir = h.getVal();
  digitalWrite(M1_H1_PIN, 1 - dir);
  digitalWrite(M1_H2_PIN, dir);
  digitalWrite(M2_H1_PIN, dir);
  digitalWrite(M2_H2_PIN, 1 - dir);

  ledcWrite(0, 1023);
  ledcWrite(1, 1023);
}

void straight() {
  // val param 0 forwards, 1 backwards
  int dir = h.getVal();
  digitalWrite(M1_H1_PIN, 1 - dir);
  digitalWrite(M1_H2_PIN, dir);
  digitalWrite(M2_H1_PIN, 1 - dir);
  digitalWrite(M2_H2_PIN, dir);

  ledcWrite(0, 1023);
  ledcWrite(1, 1023);
}

// comm handlers
void handleRoot() {
  h.sendhtml(body);
}

// udp receiver for reading police car vive coords
void rcv_pc_udp() {
  const int UDP_PACKET_SIZE = 14;  // can be up to 65535          
  uint8_t packetBuffer[UDP_PACKET_SIZE];

  int cb = PC_UDPServer.parsePacket();  // if there is no message cb=0
  if (cb) {
    packetBuffer[13] = 0;  // null terminate string

    // format of buffer(14 bytes)
    // team #(2 byte), 0, x(4 byte), 0, y(4 byte), 0, 0
    PC_UDPServer.read(packetBuffer, UDP_PACKET_SIZE);
    String s = (char*)packetBuffer;
    // police car from team 00
    if (s == "00") {
      pc_x = atoi((char*)packetBuffer + 3);  // ##,####,#### 2nd indexed char
      pc_y = atoi((char*)packetBuffer + 8);  // ##,####,#### 7th indexed char
    }
  }
}

// marshal and send own vive coords to tgt_IP
void send_vive_udp() {
  const int UDP_PACKET_SIZE = 14;  // can be up to 65535
  uint8_t packetBuffer[UDP_PACKET_SIZE] = {0};
  // same format as above
  itoa(TEAM_NUM, (char*)packetBuffer, 10);
  itoa(v1x, (char*)packetBuffer + 3, 10);
  itoa(v1y, (char*)packetBuffer + 8, 10);

  Vive_UDPServer.beginPacket(tgt_IP, VIVE_TGT_UDPPORT);
  Vive_UDPServer.write(packetBuffer, UDP_PACKET_SIZE);
  Vive_UDPServer.endPacket();
}

// would really like to merge these...
// ir interrupts on digital read change
void ir1_onchange() {
  if (digitalRead(IR1_PIN) == HIGH) {
    ir1_rise = millis();
  } else {
    unsigned long now = millis();
    ir1_freq_sum -= ir1_freq[ir1_freq_idx];
    ir1_freq[ir1_freq_idx] = 1000 / (now - ir1_rise);
    ir1_freq_sum += ir1_freq[ir1_freq_idx];
    ir1_freq_idx = (ir1_freq_idx + 1) % 5;
  }
}

void ir2_onchange() {
  if (digitalRead(IR2_PIN) == HIGH) {
    ir2_rise = millis();
  } else {
    unsigned long now = millis();
    ir2_freq_sum -= ir2_freq[ir2_freq_idx];
    ir2_freq[ir2_freq_idx] = 1000 / (now - ir2_rise);
    ir2_freq_sum += ir2_freq[ir2_freq_idx];
    ir2_freq_idx = (ir2_freq_idx + 1) % 5;
  }
}

// timer interrupt for 1hz vive coord reporting
void IRAM_ATTR one_hz_timer() {
  send_vive_udp();
}

// change mode based on user web req, log packet
void switchMode() {
  mode = h.getVal();
  esp_now_send(peer1.peer_addr, espnow_msg, sizeof(espnow_msg));
}

void setup() {
  // setup ledc with channel 0 and 1, 2khz, 10 bits res for both motors
  ledcSetup(0, M_FREQ, RES_BITS);
  ledcAttachPin(M1_PWM_PIN, 0);
  ledcSetup(1, M_FREQ, RES_BITS);
  ledcAttachPin(M2_PWM_PIN, 0);
  // setup dir control pins for both motors
  pinMode(M1_H1_PIN, OUTPUT);
  pinMode(M1_H2_PIN, OUTPUT);
  pinMode(M2_H1_PIN, OUTPUT);
  pinMode(M2_H2_PIN, OUTPUT);
  digitalWrite(2, HIGH);
  // setup vive
  vive1.begin();
  vive2.begin();
  // setup IR
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  // setup TOF
  Wire.setPins(TOF_SDA, TOF_SCL);
  Wire.begin();
  sensor_vl53l4cx_sat.begin();
  sensor_vl53l4cx_sat.VL53L4CX_Off();
  sensor_vl53l4cx_sat.InitSensor(0x12);
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  // ir frequency measurements
  attachInterrupt(digitalPinToInterrupt(IR1_PIN), ir1_onchange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR2_PIN), ir2_onchange, CHANGE);

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

  PC_UDPServer.begin(PC_UDPPORT);
  // FIXME if same tgt as above can merge
  Vive_UDPServer.begin(VIVE_TGT_UDPPORT);
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
  // FIXME with actual espnow msg format
  sprintf((char*)espnow_msg, "Hello from team %d", TEAM_NUM);
  // FIXME not integrated with movement keys since they should be temporary?

  // html page handlers(only called on transition states)
  h.begin();
  h.attachHandler("/ ", handleRoot);
  // left in for movement testing
  h.attachHandler("/straight?val=", straight);
  h.attachHandler("/turn?val=", turn);
  h.attachHandler("/stop", stop);
  h.attachHandler("/switchmode?val=", switchMode);
}

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
    v2x = vive2.xCoord();
    v2y = vive2.yCoord();
  } else {
    vive2.sync(15);
  }
  // heading calculation
  heading = atan2(v2y - v1y, v2x - v1x);
  // TOF reading
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t* pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  int status =
      sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  if (!status && NewDataReady != 0) {
    status =
        sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    if (!status) {
      no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
      for (j = 0; j < no_of_object_found; j++) {
        if (pMultiRangingData->RangeData[j].RangeStatus == 0) {
          // TODO filter and find closest obj dist, update closest_tof_dist
          // int16_t dist = pMultiRangingData->RangeData[j].RangeMilliMeter;
        }
      }
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

  // comms stuff
  rcv_pc_udp();
  h.serve();

  // depending on current task mode (wall follow, police car push, trophy
  // locate) change behavior
  switch (mode) {
    case 0: {
      stop();
      break;
    }
    case 1: {
      // TODO wall follow
      break;
    }
    case 2: {
      // TODO police car push
      break;
    }
    case 3: {
      // TODO fake trophy locate
      break;
    }
    case 4: {
      // TODO real trophy locate
      break;
    }
  }
}
