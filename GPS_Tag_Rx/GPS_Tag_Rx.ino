// This code is the vehicle side of the GPS tracking tag
// Mike Holden 2025
// It receives the GPS tag location and other stuff over the lora radio
// It communicates to the ardupilot flight controller over mavlink
// *Processes:
//  **Both the lora data and mavlink data are polled in loop
//  **The lora data is captured and processed by the function cbk(), 
//    then a data structure is filled with the info
//  **Lora outgoing info is also sent to the tag periodically, 
//    for display to the user 
//  **The mavlink incoming data is processed with function comm_receive() in loop
//  **Mavlink outgoing messages are sent with functions from mavlinkfun.ino 
//    which are mostly wrappers around the mavlink library functions
//  **Loop also updates the display and serial.prints debug information periodically
// *board is Lilygo Lora32 display (T-Display) or T-beam #2 

#include <SPI.h>
#include <LoRa.h> // Sandeep Mistry
#include <Wire.h>  
#include <TinyGPSPlus.h> // Mikal Hart
#include "SSD1306.h" // ESP32 version by ThingPulse Fabrice Weinberg

// Oleg's mavlink library installed to src in this folder https://forum.arduino.cc/t/how-do-i-include-a-library-in-a-sketch-directory/1079812/3
#include "src/mavlink/MAVLink.h"


#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     23   // GPIO14 -- SX1278's RESET (14 for T-Beam, 23 for lora32)
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    915E6

struct FC_info_t {
  float volts;
  float range_m;
  uint8_t sum;
} __attribute__((packed));
FC_info_t FC_info;

struct GPS_mode_t {
  long mode;
  double lat;
  double lon;
  float vel_mps;
  float heading_deg;
  uint8_t sum;
} __attribute__((packed));
GPS_mode_t GPS_mode, GPS_temp;

TinyGPSPlus gps_plus;  // this library is used to calculate distance but no GPS connection
unsigned long GPS_mode_time;
//MavStuff_t mav,mav_old;
int current_mode, commanded_mode, last_commanded_mode;

// Initialize the OLED display using Wire library
SSD1306  display(0x3c, 21, 22);

//String rssi = "RSSI --";
String packSize = "--";
String packet ;

unsigned long printtime=0, mavlinktime=0, lorabacktime=0;

// HardwareSerial GPS(1);
// TinyGPSPlus GPShome,GPSaway;

uint8_t data,done=0, send_heart=1;

uint8_t msgbuffy[200];

// global variables- MavLink
uint64_t timegps_usec;
uint64_t timesys_usec;
uint64_t timegps_old=0;
uint64_t timegps_boot=0;
uint8_t sysid_from;
uint8_t compid_from;
uint8_t gotmsg=0;
uint8_t notrequested=1;
mavlink_gps_raw_int_t gps_raw;
mavlink_mission_item_reached_t mission_reached;
uint64_t lastprint=0;
uint8_t gotGPS=0;
uint8_t base_mode, custom_mode;
//uint8_t ccmm;

HardwareSerial mySerial(2); // https://randomnerdtutorials.com/esp32-uart-communication-serial-arduino/#esp32-custom-uart-pins
 // Define the RX and TX pins for Serial 2
#define RXD2 4
#define TXD2 25
#define MAVLINK_BAUD 57600

void setupscreen(void) {
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.clear();
  display.drawString(0, 0, "Hello!\r\noff we go\r\n....."); 
  display.display();  
}


uint8_t xor_checksum(const void *data, size_t size) {
  // ugh written by AI: google search "xor checksum c structures example"
  // had a bug where it included the checksum in the calculations.  Of course.  
  // changed for loop to skip last byte.
  // Tested by M Holden to seem to work now.
    uint8_t checksum = 0;
    const uint8_t *bytes = (const uint8_t *)data;

    for (size_t i = 0; i < size-1; i++) {
        checksum ^= bytes[i];
    }

    return checksum;
}


void cbk(int packetSize) {
  // packet ="";
  // packSize = String(packetSize,DEC);
  for (int i = 0; i < packetSize; i++) { 
    msgbuffy[i] = LoRa.read(); 
  }

  memcpy(&GPS_temp, &msgbuffy, sizeof(GPS_temp));

  //GPS_mode.sum = (long) GPS_temp.mode + (long)(GPS_temp.lat*100000.0) + (long)(GPS_temp.lon*100000.0); // checksomething
  GPS_mode.sum = xor_checksum(&GPS_temp, sizeof(GPS_mode_t)); // checksomething

  if (GPS_temp.sum == GPS_mode.sum) {
    GPS_mode = GPS_temp;
    GPS_mode_time = millis();
  } else {
    Serial.print("ERROR ");
    Serial.print(GPS_temp.sum);
    Serial.print(" not ");
    Serial.print(GPS_mode.sum);
    Serial.print(" mode:");
    Serial.print(GPS_temp.mode);
    Serial.print(" Lat:");
    Serial.print(GPS_temp.lat);
    Serial.print(" Lon:");
    Serial.println(GPS_temp.lon);
    
  }
  // //rssi = "RSSI " + String(LoRa.packetRssi(), DEC) ;
  // loraData();
  //LoRa.read((uint8_t*)&GPS_mode, sizeof(GPS_mode));

}

void setup() {
  Serial.begin(115200);
  // GPS.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX
  while (!Serial);

  // Initialise LED display
  setupscreen();

  Serial.println();
  Serial.println("LoRa Receiver Callback");
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //LoRa.onReceive(cbk);
  LoRa.receive();
  Serial.println("init ok");
  delay(1500);

  /// setup mavlink
  mySerial.begin(MAVLINK_BAUD, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
  display.clear();
  display.drawString(0, 0, "Waiting\r\nfor mavlink"); 
  display.display();
  // wait for heartbeat
  while (notrequested) {

    // process messages
    comm_receive(); // poll until gotmsg

    // request messages (beyond heartbeat)
    if ((gotmsg) && (notrequested)) {
      mavlink_request_comm(); // this sets notrequested=0
    }

  }

  display.clear();
  display.drawString(0, 0, "Done\r\nintitializing\r\ngo go!"); 
  display.display();

  Serial.println("Done with setup");

}

void loop() {


  // Process LoRa messages received
  int packetSize = LoRa.parsePacket();
  // Serial.print("packet: ");
  // Serial.print(packetSize);
  if (packetSize) { 
    // Got lora data
    Serial.print("Getting lora..");
    cbk(packetSize);  
    Serial.print("got M=");
    Serial.print(GPS_mode.mode);
    Serial.print(" Lat:");
    Serial.print(GPS_mode.lat,8);
    Serial.print(" Lon:");
    Serial.print(GPS_mode.lon,8);
    Serial.print(" sum:");
    Serial.println(GPS_mode.sum);

    commanded_mode = GPS_mode.mode;

    lorabacktime = millis()+100; // send back to handset soon
    send_heart = 1; // only send 2nd veh heartbeat if receiving lora comms too so GCS complains on lost link

  }

  // process mavlink messages received
  comm_receive(); // poll until gotmsg

  // this uses mavlink and lora data so just calc here, used in a couple of places.
  FC_info.range_m = gps_plus.distanceBetween(
      GPS_mode.lat,
      GPS_mode.lon,
      (double)gps_raw.lat/1e7,
      (double)gps_raw.lon/1e7);

  // send boat command mode updates (if needed)
  if (millis() > mavlinktime) {

    // want to change modes only once from handheld so that groundstation can also change modes
    // keep trying until command=current, then stop until command!=last_command again.
    if (commanded_mode != last_commanded_mode) { 
      // changed command
      if (commanded_mode != current_mode) { 
        // need to change vehicle mode
        mavlink_set_mode();
      } else {
        // vehicle is good stop trying even if vehicle changes later
        last_commanded_mode = commanded_mode;
      }

    }

    // send guided positions if in that mode too.
    // but if it is too close guide it to its current location instead
    if ((commanded_mode==3)&&(current_mode==3)) {
      
      if (FC_info.range_m > 3.0) {
        mavlink_set_guidepts(GPS_mode.lat, GPS_mode.lon);
      } else {
        mavlink_set_guidepts((double)gps_raw.lat/1e7, (double)gps_raw.lon/1e7);
      }
    }

    mavlinktime += 2000;
  }

  // To use follow mode (rather than guided), the mavlink GLOBAL_POSITION_INT message should be fed into the flight controller.
  // https://ardupilot.org/copter/docs/follow-mode.html#follow-mode
  // https://discuss.ardupilot.org/t/follow-mode-copter/34782
  // sysid must be different for tag: FOLL_SYSID: MAVLink system id of lead vehicle (“0” means follow the first vehicle “seen”)
  // send when received lora message so lost lora link makes GCS complain about lost heartbeat
  if (send_heart) {
    mavlink_send_2ndveh_heartbeat();
    mavlink_set_2ndveh_inertialpos();
    Serial.println("sent HB+Tag GPS to FC");
    send_heart = 0;
  }


  if (millis() > lorabacktime) {
    // send lora data back to handset
    lora_stuff();

    lorabacktime = millis()+5000; // worst case- trying for 100 ms after lora received to minimize interference
  }

  if (millis() > printtime) {

    // // hack to try changing modes
    //   GPS_mode.mode++;
    //   if (GPS_mode.mode>3) {GPS_mode.mode=0;}

    //  Serial.print("===================================changing to ");
    //  Serial.println(GPS_mode.mode);

    double dist_m = gps_plus.distanceBetween(
      GPS_mode.lat,
      GPS_mode.lon,
      (double)gps_raw.lat/1e7,
      (double)gps_raw.lon/1e7);

    Serial.print("Distance = ");
    Serial.println(dist_m);

    // update display
    display.clear();
    display.drawString(0, 0, "M" + String(GPS_mode.mode+1) 
      + "\r\n"+String(FC_info.range_m,0) + "m " + String(FC_info.volts,1)+"V" ); 
    display.display();  

    printtime = millis()+3000;
    
  }
}

void lora_stuff() {
      //FC_info.sum = FC_info.volts + FC_info.range_m; // checksomething
      FC_info.sum = xor_checksum(&FC_info, sizeof(FC_info_t)); // checksomething

      // send packet
      LoRa.beginPacket();
      //LoRa.print(packet);  
      LoRa.write((uint8_t*)&FC_info, sizeof(FC_info));
      LoRa.endPacket();
      Serial.print("sent to tag: ");
      Serial.print(FC_info.volts);
      Serial.print(" V, ");
      Serial.print(FC_info.range_m);
      Serial.println("m");
}

