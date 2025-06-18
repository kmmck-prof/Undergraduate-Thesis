#include <WiFi.h>
#include <Wire.h>
#include "esp_wifi.h"
#include <TimeLib.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define RXD2 17 // orange wire
#define TXD2 16 // yellow wire

//START: Declaration of List and Variables

//------mac address variables-----------------------------------------
String maclist[200][4];//list of MAC Addresses 
int listcount = 0; //index count of list
int onlinecount = 0; //counts the number of people

String defaultTTL = "60"; // Maximum time (Apx seconds) elapsed before device is considered offline


//------integers for buffer time------------------------------------------
int on_buffer=5;

//CHANGE buffer based on travel speed
int low_buffer=5;
int high_buffer=5;
int rssi_filter=-60;

//------integers for capacity---------------------------------------
int max_capacity=50;//seating capacity (number of seats)
int percent=0;//percent capacity


//------integers for speed-------------------------------------------
int gps_threshold=10; //km per hour
int gps_speed=5; //placeholder default


//-------more declarations----------------------------------------------------------

//wifi struct object
const wifi_promiscuous_filter_t filt={ 
    .filter_mask=WIFI_PROMIS_FILTER_MASK_MGMT|WIFI_PROMIS_FILTER_MASK_DATA
};

typedef struct { 
  uint8_t mac[6];
} __attribute__((packed)) MacAddr;

typedef struct { 
  int16_t fctl;
  int16_t duration;
  MacAddr da;
  MacAddr sa;
  MacAddr bssid;
  int16_t seqctl;
  unsigned char payload[];
} __attribute__((packed)) WifiMgmtHdr;


//max Channel -> US = 11, EU = 13, Japan = 14
#define maxCh 13 
int curChannel = 1;


//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------


//====================== Wifi scanner function =========================================================
void sniffer(void* buf, wifi_promiscuous_pkt_type_t type) { //This is where packets end up after they get sniffed
  wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t*)buf; 
  int len = p->rx_ctrl.sig_len;
  WifiMgmtHdr *wh = (WifiMgmtHdr*)p->payload;
  len -= sizeof(WifiMgmtHdr);
  if (len < 0){
    Serial.println("Received 0");
    return;
  }
  String packet;
  String mac;
  int fctl = ntohs(wh->fctl);
  for(int i=8;i<=8+6+1;i++){ // This reads the first couple of bytes of the packet. This is where you can read the whole packet replaceing the "8+6+1" with "p->rx_ctrl.sig_len"
     packet += String(p->payload[i],HEX);
  }
  for(int i=4;i<=15;i++){ // This removes the 'nibble' bits from the stat and end of the data we want. So we only get the mac address.
    mac += packet[i];
  }
  mac.toUpperCase();

  int rssi = p->rx_ctrl.rssi; // NEW: get RSSI of signal
  
  int added = 0;

  for(int i=0;i<=199;i++){ // checks if the MAC address has been added before    

    if(mac == maclist[i][0]){
      //"if added: set timer to 60. Set offline to 0"
      maclist[i][1] = defaultTTL;
      if(maclist[i][2] == "OFFLINE"){
        maclist[i][2] = "0";
      }
      maclist[i][3] = String(rssi); // NEW: update RSSI of wifi
      added = 1;
    }
  }
  
  if(added == 0){ // If its new. add it to the array.
    maclist[listcount][0] = mac;
    maclist[listcount][1] = defaultTTL;
    maclist[listcount][3] = String(rssi); // NEW: Store RSSI value
    //Serial.println(mac);
    listcount ++;
    if(listcount >= 200){
      Serial.println("Too many addresses");
      listcount = 0;
    }
  }
}



//========================SD Card Functions ===========================================================
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

void SD_setup(){
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    listDir(SD, "/", 0);
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt");
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));


    //bus setup
    appendFile(SD, "/Occupancy_Log.txt", "Data Logging Begins Now:\n");
}

//========================SD Card END ===========================================================//


//========================Wifi Scanner Functions ===========================================================//

/*
Purge Function
- checks if not empty
- if offline, ignore
- if active, subtract 1 second from TTL
- store back to list as string

*/

void purge(){ // This manages the TTL
  for(int i=0;i<=199;i++){
    if(!(maclist[i][0] == "")){ //if not empty, subtract time
      int ttl = (maclist[i][1].toInt());
      ttl --;
      if(ttl <= 0){
        Serial.println("OFFLINE: " + maclist[i][0]);
        maclist[i][2] = "OFFLINE";
        maclist[i][1] = defaultTTL;
      }else{
        maclist[i][1] = String(ttl);
      }
    }
  }
}

/*
Update Function
- if not online, increase alive time "timehere"
- return as string

*/

void updatetime(){ // This updates the time the device has been online for
  for(int i=0;i<=199;i++){
    if(!(maclist[i][0] == "")){
      if(maclist[i][2] == "")maclist[i][2] = "0";
      if(!(maclist[i][2] == "OFFLINE")){
          int timehere = (maclist[i][2].toInt());
          timehere ++;
          maclist[i][2] = String(timehere);
      }
      
      //Serial.println(maclist[i][0] + " : " + maclist[i][2]);
      
    }
  }
}

/*
Update Function
- lots of Print Debugging
- "onlinecount" counts how many people
- online_buffer checks how many seconds alive
- lots of print debugging

*/


void showpeople(){ // This checks if the MAC is in the reckonized list and then displays it on the OLED and/or prints it to serial.
  String forScreen = "";
  Serial.print("\n reset \n");
  Serial.print("------------- \n");
  Serial.print("\n \n \n \n \n");
  onlinecount = 0;
  for(int i=0;i<=199;i++){
    String tmp1 = maclist[i][0];
    if(!(tmp1 == "")){
      if(!(maclist[i][2]== "OFFLINE")){
        //new if condition: buffer time
        //count how many seconds the device has been active before considering it as ONLINE
        
        //extract integer
        int timehere = (maclist[i][2].toInt());

        //compare with on_buffer
        if(timehere>=on_buffer){
          int rssi_check = (maclist[i][3].toInt());
          if(rssi_check >= rssi_filter){
            Serial.print(maclist[i][0] + "    RSSI:");
            Serial.print(maclist[i][3]);
            Serial.print("   (DEBUG) Time Alive:");
            Serial.print(timehere);
            Serial.print("\n");
            onlinecount++;
          }
          
        }
        
        //Serial.print(maclist[i][0] + "\n");
        //onlinecount++;

      }
    }
  }

  //calculate seating capacity % !!disable for now
  percent = onlinecount*100 / max_capacity;

  // update_screen_text(forScreen);

  //print current Online Buffer Time and current Bus Speed
  Serial.print("\nBuffer Time:");
  Serial.print(on_buffer);
  Serial.println(" seconds");
  Serial.print("Bus Speed:");
  Serial.print(gps_speed);
  Serial.println("km/h");

  //print the number of passengers on the bus
  Serial.print("Passengers on Board: ");
  Serial.println(onlinecount);

  //print the seating status in percent !!disable for now
  Serial.print("Seating Status:" );
  Serial.print(String(percent));
  Serial.print("% \n");
}

//========================Wifi Scanner Functions END ===========================================================//

//====================SETUP======================================================================//
void setup() {

  /* start Serial */
  Serial.begin(115200);

  // setupUART();
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //setup SD Card
  SD_setup();
  

  /* setup wifi */
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&sniffer);
  esp_wifi_set_channel(curChannel, WIFI_SECOND_CHAN_NONE);
  
  //start time
  setTime(0, 0, 0, 1, 1, 2024);

  Serial.println("starting!");
}



//==============occupancy SD Card log=====================================================================================================================================================================
char currentTimeString[9];  // HH:MM:SS\0
char occupancy_payload[500];

void occupancy_log(){
  time_t t = now();
  sprintf(currentTimeString, "%02d:%02d:%02d", hour(t), minute(t), second(t));
  
  sprintf(occupancy_payload, "Time:  %s, Passengers: %d, Seating Status: %d%%, Bus Speed: %d km/h, Buffer Time: %d seconds\n", currentTimeString, onlinecount, percent, gps_speed, on_buffer);
  Serial.print(occupancy_payload);
  appendFile(SD, "/Occupancy_Log.txt", occupancy_payload);
}

//====================speed checker================================================================//
int UARTholder = 0;

//Buffer time is adjusted based on bus speed
void speed_check(){
  // Debugging output
  //Serial.print("Bytes available in Serial2 buffer: ");
  //Serial.println(Serial2.available());

  //check current speed from border router
  if (Serial2.available() > 0) {
    Serial.print("!---------Buffer Detected--------------!\n");
    //Serial.print();
    UARTholder=Serial2.parseInt();

    if(UARTholder>0){
      Serial.print("!---------NON ZERO DETECTED--------------!");
      Serial.println(UARTholder);
      gps_speed=UARTholder;
    }

  }

  if(gps_speed<gps_threshold){
    //if bus is slow or stopped, increase buffer time in order to minimize false positives
    //change to high buffer time
    on_buffer=high_buffer;
  } else {
    //if bus is moving above the required speed, return back to original buffer time
    //change to low buffer time
    on_buffer=low_buffer;
  }

}

void send_occupancy(){
  //send as string
  Serial2.print(String(onlinecount));
  Serial2.print("\n");

}

//===== LOOP =====//
void loop() {
    //Serial.println("Changed channel:" + String(curChannel));
    if(curChannel > maxCh){ 
      curChannel = 1;
    }
    esp_wifi_set_channel(curChannel, WIFI_SECOND_CHAN_NONE);
    delay(1000);
    speed_check();
    updatetime();
    purge();
    showpeople();
    occupancy_log();
    send_occupancy();
    curChannel++;
    
}
