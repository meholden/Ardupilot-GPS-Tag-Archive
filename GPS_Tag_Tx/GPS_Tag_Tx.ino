// This code is the tag side of the GPS tracking tag
// Mike Holden 2025
// It receives the GPS location and mode button and sends that plus other stuff over the lora radio
// It communicates to the vehicle's lora processor which does all the mavlink.  No mavlink done on this code
// *Processes:
//  **The button matrix is wired with a voltage divider so it can be polled with the ADC
//  **Both the lora data and GPS data are polled in loop
//  **The lora data is captured and processed by the function cbk(), 
//    then a data structure is filled with the info
//  **Lora outgoing info is also sent to the vehicle periodically, to tell it what to do
//  **Loop also updates the display and serial.prints debug information periodically
// *board is Lilygo Lora32 display (T-Display) or T-beam #2 


//Notes to myself:
//GET gps on T-Beam and send on LORA
// T-Beam board
// some code from SerialToSerial.ino example
// and ttgo gps example https://github.com/LilyGO/TTGO-T-Beam/tree/master/GPS
// Had to reset my board with this first: https://github.com/eriktheV-king/TTGO_T-beam_GPS-reset/blob/master/T22-GPS-reset-v3/T22-GPS-reset-v3.ino
// maybe in the future use radiohead library instead of LoRa library for structured data not nmea repeater?
// https://www.airspayce.com/mikem/arduino/RadioHead/packingdata.html 

// updated for lilygo lora32 board with external GPS as T-beam GPS sucked.
// one pin on the lora changes and added ssd1306 display for UI

#include <SPI.h>
#include <LoRa.h>  // tried the one by Sandeep Mistry
#include <TinyGPSPlus.h>
#include "SSD1306.h" // ESP32 version by ThingPulse Fabrice Weinberg or is it Alexey Dynda's IDK

// lora pinout
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     23   // GPIO14 -- SX1278's RESET  (14 for T-Beam, 23 for lora32)
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  915E6

// keypad pinout (4 buttons in voltage divider ladder)
// 3.3v-R1-R2-R3-R4-Gnd
//        |  |  |  |
//       B1 B2  B3 B4
//   keypad common to ADC pin 
// 10k pullup on ADC

#define KEYADC  34 

// battery monitoring when switched on
// sends to serial as that is how it charges
// could add to display if use smaller font
// voltage divider 4.20 fully charged so gain of 2/3 works 
//  Vbatt--1k--ADC--2k--Gnd
#define BATTADC 35

// GPS Pins
#define GPTX  25 // Tx to GPS Rx
#define GPRX  4 // Rx to GPS Tx (35 is input only)
#define GPBAUD 115200

struct GPS_mode_t {
  long mode;
  double lat;
  double lon;
  float vel_mps;
  float heading_deg;
  uint8_t sum;
} __attribute__((packed));

GPS_mode_t GPS_mode;

struct FC_info_t {
  float volts;
  float range_m;
  uint8_t sum;
} __attribute__((packed));

FC_info_t FC_info, FC_temp;
uint8_t msgbuffy[200];



HardwareSerial GPS(1);
TinyGPSPlus GPSloc;
int Status_GPS=0, Status_LORA=0, Status_button=0, dtPrint=1000, dtLora=2000;;
unsigned long print_time;
unsigned long lora_time;
int isValid, isUpdated;

char data,done=0;

String rssi = "RSSI --";
String packSize = "--";
//String packet ;

// Initialize the OLED display using Wire library
SSD1306  display(0x3c, 21, 22);

void setupscreen(void) {
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.clear();
  display.drawString(0, 0, "Hello!\r\noff we go"); 
  display.display();  
}



void setup() {
  Serial.begin(115200);
  GPS.begin(GPBAUD, SERIAL_8N1, GPRX, GPTX);   

  // Initialise LED display
  setupscreen();

  delay(100);
  
  Serial.println("LoRa Sender Test");
  
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //LoRa.onReceive(cbk);
//  LoRa.receive();
  Serial.println("init ok");

  Serial.println("The device started, go go!");
}

void read_GPS() {
  if (GPS.available()) {
    data=GPS.read();
    GPSloc.encode(data); // parses NMEA
    //Serial.write(data);
    isUpdated = GPSloc.location.isUpdated();
    isValid = GPSloc.location.isValid();

    if (isValid && (GPSloc.location.age() < 2000)) { 
      // fill struct
      GPS_mode.lat = GPSloc.location.lat();
      GPS_mode.lon = GPSloc.location.lng();
      GPS_mode.vel_mps = GPSloc.speed.mps();
      GPS_mode.heading_deg = GPSloc.course.deg();
      Status_GPS=2;

    }  else {
      // This is not an error until a full sentence comes in
      if (data==13) { // EOL
        // Serial.print("waiting for GPS: ");
        // Serial.print(GPSloc.time.value());
        // Serial.print(" Nsats=");
        // Serial.println(GPSloc.satellites.value());
        // GPS_mode.mode = (millis()/1000)%4 + 999;
        // GPS_mode.lat = 0.0;
        // GPS_mode.lon = 0.0;

        Status_GPS=1;

      }
    }
  } // GPS Available test
}

uint8_t xor_checksum(const void *data, size_t size) {
  // ugh written by AI: google search "xor checksum c structures example"
  // Of course had a bug where it included the checksum in the calculations 
  // changed for loop to skip last byte.
  // Tested by M Holden to seem to work now.
    uint8_t checksum = 0;
    const uint8_t *bytes = (const uint8_t *)data;

    for (size_t i = 0; i < size-1; i++) {
        checksum ^= bytes[i];
    }

    return checksum;
}

void lora_stuff() {
      //GPS_mode.sum = (long) GPS_mode.mode + (long)(GPS_mode.lat*100000.0) + (long)(GPS_mode.lon*100000.0); // checksomething
      GPS_mode.sum = xor_checksum(&GPS_mode, sizeof(GPS_mode_t)); // checksomething

      // send packet
      LoRa.beginPacket();
      //LoRa.print(packet);  
      LoRa.write((uint8_t*)&GPS_mode, sizeof(GPS_mode));
      LoRa.endPacket();
      Serial.print("sent: ");
      Serial.print(GPS_mode.mode);
      Serial.print(" Lat:");
      Serial.print(GPS_mode.lat,8);
      Serial.print(" Lon:");
      Serial.print(GPS_mode.lon,8);
      Serial.print(" sum:");
      Serial.println(GPS_mode.sum);

}

void cbk(int packetSize) {
  for (int i = 0; i < packetSize; i++) { 
    msgbuffy[i] = LoRa.read(); 
  }

  memcpy(&FC_temp, &msgbuffy, sizeof(FC_temp));

  //FC_info.sum = FC_temp.volts + FC_temp.range_m; // checksomething
  FC_info.sum = xor_checksum(&FC_temp, sizeof(FC_info_t)); // checksomething

  if (FC_temp.sum == FC_info.sum) {
    FC_info = FC_temp;
  } else {
    Serial.print("ERROR ");
    Serial.print(FC_temp.sum);
    Serial.print(" not ");
    Serial.print(FC_info.sum);
    
  }
  // //rssi = "RSSI " + String(LoRa.packetRssi(), DEC) ;
  // loraData();
  //LoRa.read((uint8_t*)&GPS_mode, sizeof(GPS_mode));

}


void loop() {

  // Get mode info from UI
  //GPS_mode.mode = (millis()/10000)%4;
  // 4 button matrix (label 0-3) wired with voltage dividers so that:
  // no press = 3.3V
  // 3 = 2.53V
  // 2 = 1.8V
  // 1 = 0.999V
  // 0 = 0V
  float butV = analogRead(KEYADC)*3.3/4095.0;
  if (butV < 0.5) {
    GPS_mode.mode=1; //1
  } else {
    if (butV < 1.4) {
      GPS_mode.mode=0; //0
    } else {
      if (butV < 2.2) {
        GPS_mode.mode=3; //3
      } else {
        if (butV < 3.0) {
          GPS_mode.mode=2; //2
        } 
      }
    }
  }

  // 0.66 voltage divider gain so 5V = 3.3/0.66 Max
  float battV = analogRead(BATTADC)*5.0/4095.0; 

  // poll and handle lora
  int packetSize = LoRa.parsePacket();
  // Serial.print("packet: ");
  // Serial.print(packetSize);
  if (packetSize) { 
    Serial.print("Getting..");
    cbk(packetSize);  
    Serial.print("got: ");
    Serial.print(FC_info.volts);
    Serial.print(" volts, ");
    Serial.print(FC_info.range_m);
    Serial.println(" m");
    // display.clear();
    // display.drawString(0, 0, String(millis()) + " Mode " + String(GPS_mode.mode)); 
    // display.drawString(0, 16, String(GPS_mode.lat,5)); 
    // display.drawString(0, 32, String(GPS_mode.lon,5)); 
    // display.display();
  }

  // Poll and handle GPS
  read_GPS();

  // update display periodically
  if (millis() >= print_time) {
    // update display
    display.clear();
    
    if (Status_GPS ==1) {
      display.drawString(0, 0, "M" + String(GPS_mode.mode+1) + "   -----"
        + "\r\n"+String(FC_info.range_m,0) + "m " + String(FC_info.volts,1)+"V" ); 
    } else {
      display.drawString(0, 0, "+++      M" + String(GPS_mode.mode+1)
        + "\r\n"+String(FC_info.range_m,0) + "m  " + String(FC_info.volts,1)+"V" ); 
    }
    display.display();  

    if (Status_GPS ==1) {
      Serial.print("waiting for GPS: ");
    } else {
      if (Status_GPS==2) {
        Serial.print("Good GPS: ");
      } else {
        Serial.print("No GPS: ");
      }
    }

    Serial.print(GPSloc.time.value());
    Serial.print(" Nsats=");
    Serial.print(GPSloc.satellites.value());
    Serial.print(", Upd: ");
    Serial.print(isUpdated);
    Serial.print(", Val: ");
    Serial.print(isValid);
    Serial.print(", Batt:");
    Serial.print(battV);
    Serial.println("");

    print_time += dtPrint;
  }

  // transmit to lora periodically
  if (millis() >= lora_time) {
    lora_stuff();
    lora_time += dtLora;
  }

}
