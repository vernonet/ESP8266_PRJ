#include <ESP_WiFiManager.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <NTPClient.h>
#include "sample_wav.h"   //test wav  file

// I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S
// SPH0645, INMP441 MEMS MICROPHONE
//insert in vlc  "http:\\wifi-mic.local:8080\rec.wav"  or "http:\\ip:8080\rec.wav"   ip - ip address of mic

//#define NO_WIFI            //testing the microphone using the "serial_audio.exe" program.
#define USE_SPH0645          

#define SERVER_PORT          8080
#define BAUDRATE             921600//921600//921600    //500000 //baud one of 300, 600, 1200, 2400, 4800, 9600, 19200, 31250, 38400, 57600, 74880, 115200, 230400, 256000, 460800, 921600, 1843200, 3686400
#define I2S_CLK_FREQ         80000000  // Hz
#define I2S_SAMPLE_RATE      (33085)//(37400) (22050)  (33085) (16029)  frequency of mcu not accurate //write to stream
#define I2S_SAMPLE_RATE_SET  (32000)//(36000) (21400)  (32000) (15500)   //21400 optimal for 22050    //set on I2S module
#define BITS_PER_SAMPLE      (16)  // 16 or 24        //for 24bit & SAMPLE_RATE <= 33085 need baudrate > 921600
#define SIGNAL_GAIN          (16384)//(65536)//(16384)  //65536 no gain, if < 65536 -> gain+   //8192;//16384;//32768;//65536; //130000   262144 131072
#define REC_TIME             (6000) //sec
#define NUM_CPY              ((I2S_SAMPLE_RATE * BITS_PER_SAMPLE / 8 * REC_TIME)/SLC_BUF_LEN)//500000
#define I2S_24BIT            3     // I2S 24 bit half data
#define I2S_16BIT            1     // I2S 16 bit half data
#define I2S_LEFT             2     // I2S RX Left channel
#define I2S_DUAL             0     // I2S RX dual channel mode

#define I2SI_DATA            12    // I2S data on IO12
#define I2SI_BCK             13    // I2S clk on IO13
#define I2SI_WS              14    // I2S select on IO14

#define SLC_BUF_CNT          8    // Number of buffers in the I2S circular buffer   8
#define SLC_BUF_LEN          96    // Length of one buffer, in 32-bit words.         64

#define WIFI_CONNECT_TIMEOUT        30000L
#define WHILE_LOOP_DELAY            200L
#define WHILE_LOOP_STEPS            (WIFI_CONNECT_TIMEOUT / ( 3 * WHILE_LOOP_DELAY ))

#define reverse_bytes(val)  ((val & 0x000000FFU) << 24 | (val & 0x0000FF00U) << 8 |(val & 0x00FF0000U) >> 8 | (val & 0xFF000000U) >> 24)
#define reverse_halfword(val)  ((val & 0x00FFU) << 24 |  (val & 0xFF000000U) >> 24)
#define reverse_sample16(val)  (((val >> 8)&0xFF) | ((val << 8)&0xFF00))


typedef struct {
  uint32_t blocksize      : 12;
  uint32_t datalen        : 12;
  uint32_t unused         : 5;
  uint32_t sub_sof        : 1;
  uint32_t eof            : 1;
  volatile uint32_t owner : 1;

  uint32_t *buf_ptr;
  uint32_t *next_link_ptr;
} sdio_queue_t;

static sdio_queue_t i2s_slc_items[SLC_BUF_CNT];  // I2S DMA buffer descriptors
static uint32_t *i2s_slc_buf_pntr[SLC_BUF_CNT];  // Pointer to the I2S DMA buffer data
static volatile uint32_t rx_buf_cnt = 0;
static volatile uint32_t rx_buf_idx = 0;
static volatile bool rx_buf_flag = false;

void launchWeb(int webtype);
void i2s_init();
void slc_init();
void i2s_set_dividers(uint8_t div1, uint8_t div2);
float i2s_get_real_rate();
void i2s_set_rate(uint32_t rate);
void i2s_set_rate_(uint32_t rate);
void slc_isr(void *para);
void i2s_adc_data_scale(int16_t * d_buff, uint32_t* s_buff, uint32_t len);
void prepare_sample(int16_t * d_buff, uint32_t* s_buff, uint32_t len);
void prepare_sample_24bit(uint8_t * d_buff, uint32_t* s_buff, uint32_t len);


// I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S


#define LED_ON            LOW
#define LED_OFF           HIGH
#define PIN_LED           2         // Pin D4 mapped to pin GPIO2/TXD1 of ESP8266, NodeMCU and WeMoS, control on-board LED
#define PIN_D7            13        // Pin D7 mapped to pin GPIO13/RXD2/HMOSI of ESP8266
#define PIN_D3            0         // Pin D3 mapped to pin GPIO0/FLASH of ESP8266
#define ESP_getChipId()   (ESP.getChipId())



uint8_t temp_buf_f[SLC_BUF_LEN*(BITS_PER_SAMPLE/8)] __attribute__((aligned(4))); //4000
volatile bool  sample_ready = false;
 
MDNSResponder mdns;


char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String ssid = "wifi_mic_ap";
const char* passphrase = "testtest";
const char* password = "testtest";

//const char* www_username = "admin";
//const char* www_password = "esp8266";

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;
String st;
String content;
int statusCode;
const int TRIGGER_PIN =  PIN_D3; // 
const int TRIGGER_PIN2 = PIN_D7; // 
const int TRIGGER_PIN3 = 5;      // IO5 , short  contact to ground to enter config portal
bool con_flag = false;
volatile bool cl_authenticated = false;
volatile unsigned int tme = 0;
int16_t val_tmp;
 
ESP8266WebServer server(SERVER_PORT);
 
String webPage = "";
 
int gpio0_pin = 0;
int gpio2_pin = 2;
bool initialConfig = false;

// Define NTP Client to get time
const long utcOffsetInSeconds = 2*60*60; //+2;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
 
void setup(void){
 
  pinMode(gpio0_pin, OUTPUT);
  //digitalWrite(gpio0_pin, LOW);
  pinMode(gpio2_pin, OUTPUT);
  digitalWrite(gpio2_pin, LED_OFF);

  //pinMode(TRIGGER_PIN,  INPUT_PULLUP);
  //pinMode(TRIGGER_PIN2, INPUT_PULLUP);
  pinMode(TRIGGER_PIN3, INPUT_PULLUP);

  rx_buf_cnt = 0;

  pinMode(I2SI_WS, OUTPUT);
  pinMode(I2SI_BCK, OUTPUT);
  pinMode(I2SI_DATA, INPUT);

#ifdef NO_WIFI
  WiFi.forceSleepBegin();
#endif
  delay(500);

  Serial.begin(BAUDRATE);//(115200);  //230400
  Serial.println("\nStarting");

  slc_init();
  i2s_init();
 
  
#ifndef NO_WIFI
  unsigned long startedAt = millis();
  
  ESP_WiFiManager ESP_wifiManager("wifi-mic");
  ESP_wifiManager.setMinimumSignalQuality(-1);
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
  // ESP_wifiManager.setSTAStaticIPConfig(IPAddress(192,168,2,114), IPAddress(192,168,2,1), IPAddress(255,255,255,0), IPAddress(192,168,2,1), IPAddress(8,8,8,8));
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();
  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);
  // SSID to uppercase 
  ssid.toUpperCase();  
  if (Router_SSID == "")
  {
    Serial.println("We haven't got any access point credentials, so get them now");        
    digitalWrite(PIN_LED, LED_ON); // Turn led on as we are in configuration mode.
    
    //it starts an access point 
    //and goes into a blocking loop awaiting configuration
    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password)) 
      Serial.println("Not connected to WiFi but continuing anyway.");
    else {
      Serial.println("WiFi connected...yeey :)");    
      //launchWeb(0);
    }
  }

  digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.
  
  startedAt = millis();
  
  while ( (WiFi.status() != WL_CONNECTED) && (millis() - startedAt < WIFI_CONNECT_TIMEOUT ) )
  {   
    WiFi.mode(WIFI_STA);
    WiFi.hostname("wifi-mic");
    WiFi.persistent (true);
    // We start by connecting to a WiFi network
  
    Serial.print("Connecting to ");
    Serial.println(Router_SSID);
  
    WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());

    int i = 0;
    while((!WiFi.status() || WiFi.status() >= WL_DISCONNECTED) && i++ < WHILE_LOOP_STEPS)
    {
      delay(WHILE_LOOP_DELAY);
    }    
  }

  Serial.print("After waiting ");
  Serial.print((millis()- startedAt) / 1000);
  Serial.print(" secs more in setup(), connection result is ");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("connected. Local IP: ");
    Serial.println(WiFi.localIP());
    con_flag = true;
    timeClient.begin();
    // GMT +1 = 3600
    // GMT +8 = 28800
    // GMT -1 = -3600
    // GMT 0 = 0
    timeClient.setTimeOffset(3600*3);  //for GMT+3
    launchWeb(0);
  }
  else
    Serial.println(ESP_wifiManager.getStatus(WiFi.status()));

#endif
}
 
void loop(void){
  int32_t value;
  char buff[9];

#ifndef NO_WIFI
  if (con_flag) {
    server.handleClient();
    MDNS.update();
    if (tme < 1) {
      Serial.print(daysOfTheWeek[timeClient.getDay()]);
      Serial.print(", ");
      Serial.printf("%02d", timeClient.getHours());
      Serial.print(":");
      Serial.printf("%02d", timeClient.getMinutes());
      Serial.print("\n\r");
      timeClient.update();
	  if (WiFi.status() != WL_CONNECTED){
        Serial.println("NO WIFI, try to connect");
        WiFi.mode(WIFI_STA);
        WiFi.hostname("wifi-mic");
        WiFi.persistent (true);
        //WiFi.setOutputPower(0);
        WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());
     }
    }
    tme++;
    if (tme > 1000000) tme = 0;
  }

  // is configuration portal requested?
  //if ((digitalRead(TRIGGER_PIN) == LOW) || (digitalRead(TRIGGER_PIN2) == LOW)) 
  if (digitalRead(TRIGGER_PIN3) == LOW) 
  {
    Serial.println("\nConfiguration portal requested.");
    digitalWrite(PIN_LED, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.
    
    //Local intialization. Once its business is done, there is no need to keep it around
    ESP_WiFiManager ESP_wifiManager;
    
    //Check if there is stored WiFi router/password credentials.
    //If not found, device will remain in configuration mode until switched off via webserver.
    Serial.print("Opening configuration portal. ");
    Router_SSID = ESP_wifiManager.WiFi_SSID();
    if (Router_SSID != "")
    {
      ESP_wifiManager.setConfigPortalTimeout(60); //If no access point name has been previously entered disable timeout.
      Serial.println("Got stored Credentials. Timeout 60s");
    }
    else
      Serial.println("No stored Credentials. No timeout");
    
    //it starts an access point 
    //and goes into a blocking loop awaiting configuration
    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password)) 
    {
      Serial.println("Not connected to WiFi but continuing anyway.");
    } 
    else 
    {
      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :)");
    }
    
    digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.
  }
#else
   uint32_t temp;
   int16_t  temp_S16;
   if (rx_buf_flag) {
     for (int x = 0; x < SLC_BUF_LEN; x++) {
//        Serial.printf("%08x   ", i2s_slc_buf_pntr[rx_buf_idx][x]);
          temp = i2s_slc_buf_pntr[rx_buf_idx][x]<<1;
//        value = *(int32_t*)&i2s_slc_buf_pntr[rx_buf_idx][x];
          value =  *(int32_t*)&temp;  
          value = value/SIGNAL_GAIN;
          temp_S16 = (int16_t)(value);
//        val_tmp = (int16_t)(value/65536);
//      Serial.print(i2s_slc_buf_pntr[rx_buf_idx][x], HEX);
//      Serial.print(value, HEX);
//         Serial.printf("   %08X",  value);
//         Serial.printf("   %08X",  (value<<1)/65536);
//        Serial.printf("   %04X",  val_tmp);
//         sprintf (buff, "%08X", value);
//        
//#if BITS_PER_SAMPLE == 24 
//          Serial.write(&buff[0], 6);       //for terminal
//#else
//          Serial.write(&buff[0], 4);       //for terminal
//#endif
//          Serial.write(0x20);              //for terminal

#if BITS_PER_SAMPLE == 16           
	         Serial.write((temp_S16)   &0xFF);   //for serial audio  0
           Serial.write((temp_S16>>8)&0xFF);   //for serial audio  8
          
#else
           //24-bits 
           Serial.write((value)&0xFF);    //for serial audio  16
           Serial.write((value>>8)&0xFF);    //for serial audio  16
           Serial.write((value>>16)&0xFF);    //for serial audio  16
#endif                     


//         Serial.printf("    %d   ",  value);
//         Serial.printf("  int_16 -> %05d", value>>16);
//         Serial.print("\n\r");
     }
    
    rx_buf_flag = false;
//    Serial.print("\n\r");
   }
#endif
  
}

void createWebServer(int webtype)
{
  if ( webtype == 1 ) {  //softAP
    //
    
  } else if (webtype == 0) {  //are configured

    server.onNotFound(handleNotFound);
 
    if (MDNS.begin("wifi-mic", WiFi.localIP())) {
    Serial.println("MDNS responder started");
    }
    MDNS.addService("http", "tcp", 8080);  //orig 80
    MDNS.addService("osc","udp",4500);

   server.on("/rec.wav", []() {   

   // Start I2S receiver
   I2SC |= I2SRXS;
   delay(500);                      //remove noise at the beginning 
   
   server.setContentLength(SLC_BUF_LEN*4*NUM_CPY + 44);
   memcpy(&temp_buf_f[0], &test_wav[0],  44);
   *(uint32_t *)&temp_buf_f[4]  = (SLC_BUF_LEN)*NUM_CPY +36;  //long size
   *(uint32_t *)&temp_buf_f[24]  = I2S_SAMPLE_RATE; 
   *(uint32_t *)&temp_buf_f[28]  = I2S_SAMPLE_RATE*(BITS_PER_SAMPLE/8);
   *(uint32_t *)&temp_buf_f[32]  = (uint32_t)(BITS_PER_SAMPLE>>3) + ((BITS_PER_SAMPLE<<16)&0xFF0000);
   *(uint32_t *)&temp_buf_f[40] = (SLC_BUF_LEN)*NUM_CPY;      //long chunkSize
   
   server.send_P(200, "audio/x-wav", (const char*)&temp_buf_f[0], 44);
   //server.sendContent_P(test_wav, (sizeof test_wav));  
   uint32_t c = 0;
   while (c < ((SLC_BUF_LEN)*NUM_CPY)) {
      digitalWrite(gpio2_pin, !digitalRead(gpio2_pin));      
      if (rx_buf_flag && !(sample_ready)) {
//           Serial.print("  prepare_sample\n\r");
#if BITS_PER_SAMPLE == 16
           prepare_sample((int16_t*)&temp_buf_f[0], &i2s_slc_buf_pntr[rx_buf_idx][0], SLC_BUF_LEN);
#else 
           prepare_sample_24bit(&temp_buf_f[0], &i2s_slc_buf_pntr[rx_buf_idx][0], SLC_BUF_LEN);
#endif           
           rx_buf_flag = false;  
           sample_ready = true;
        }
        else if (sample_ready) {
              server.sendContent_P((const char*)&temp_buf_f[0], SLC_BUF_LEN*(BITS_PER_SAMPLE/8));
//              Serial.print("  server.sendContent_P\n\r");
              sample_ready = false;
              c += SLC_BUF_LEN;
              }
             
      }
   }); 
 
  //server.begin();
  //Serial.println("HTTP server started");
  }
}


void launchWeb(int webtype) {
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
  createWebServer(webtype);
  // Start the server
  server.begin();
  Serial.println("Server started"); 
}



void handleNotFound(){
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

// I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S

/**
 * Initialise I2S as a RX master.
 */
void
i2s_init()
{
  // Config RX pin function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_I2SI_BCK);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);

  // Enable a 160MHz clock
  I2S_CLK_ENABLE();

  // Reset I2S
  I2SC &= ~(I2SRST);
  I2SC |= I2SRST;
  I2SC &= ~(I2SRST);

  // Reset DMA
  I2SFC &= ~(I2SDE | (I2SRXFMM << I2SRXFM));

  // Enable DMA
  I2SFC |= I2SDE | (I2S_24BIT << I2SRXFM); //I2S_24BIT  I2S_16BIT

  // Set RX single channel (left)
  I2SCC &= ~((I2STXCMM << I2STXCM) | (I2SRXCMM << I2SRXCM));
  I2SCC |= (I2S_LEFT << I2SRXCM);  //I2SCC |= (I2S_LEFT << I2SRXCM);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  i2s_set_rate_(I2S_SAMPLE_RATE_SET); //
  Serial.printf("RealFreq %u\n\r", (uint32_t)(i2s_get_real_rate()));

  // Set RX data to be received
  I2SRXEN = SLC_BUF_LEN;

  // Bits mode
  I2SC |= (15 << I2SBM);

#ifdef NO_WIFI
  // Start receiver
  I2SC |= I2SRXS;
#endif  
}

float i2s_get_real_rate(){
  return (float)I2S_CLK_FREQ/32/((I2SC>>I2SBD) & I2SBDM)/((I2SC >> I2SCD) & I2SCDM);
}

/**
 * Set I2S clock.
 * I2S bits mode only has space for 15 extra bits,
 * 31 in total. The
 */
void
i2s_set_rate(uint32_t rate)
{
  uint32_t i2s_clock_div = (I2S_CLK_FREQ / (rate * 31 * 2)) & I2SCDM;
  uint32_t i2s_bck_div = (I2S_CLK_FREQ / (rate * i2s_clock_div * 31 * 2)) & I2SBDM;


  Serial.printf("\n\rRate %u Div %u Bck %u Freq %u\n\r", rate, i2s_bck_div, I2S_CLK_FREQ / (i2s_clock_div * i2s_bck_div * 31 * 2));


  // RX master mode, RX MSB shift, right first, msb right
  I2SC &= ~(I2STSM | I2SRSM | (I2SBMM << I2SBM) | (I2SBDM << I2SBD) | (I2SCDM << I2SCD));
  I2SC |= I2SRF | I2SMR | I2SRMS | (i2s_bck_div << I2SBD) | (i2s_clock_div << I2SCD);
}

void i2s_set_rate_(uint32_t rate) { //Rate in HZ
//  if (rate == _i2s_sample_rate) {
//    return;
//  }
//  _i2s_sample_rate = rate;

 

  uint32_t scaled_base_freq = I2S_CLK_FREQ/32;
  float delta_best = scaled_base_freq;

  uint8_t sbd_div_best=1;
  uint8_t scd_div_best=1;
  for (uint8_t i=1; i<64; i++) {
    for (uint8_t j=i; j<64; j++) {
      float new_delta = fabs(((float)scaled_base_freq/i/j) - rate);
      if (new_delta < delta_best){
        delta_best = new_delta;
        sbd_div_best = i;
        scd_div_best = j;
      }
    }
  }

  i2s_set_dividers( sbd_div_best, scd_div_best );

  Serial.printf("\n\rRate %u  ", rate);
}

void i2s_set_dividers(uint8_t div1, uint8_t div2) {
  // Ensure dividers fit in bit fields
  div1 &= I2SBDM;
  div2 &= I2SCDM;

  // trans master(active low), recv master(active_low), !bits mod(==16 bits/chanel), clear clock dividers
  I2SC &= ~(I2STSM | I2SRSM | (I2SBMM << I2SBM) | (I2SBDM << I2SBD) | (I2SCDM << I2SCD));

  // I2SRF = Send/recv right channel first (? may be swapped form I2S spec of WS=0 => left)
  // I2SMR = MSB recv/xmit first
  // I2SRMS, I2STMS = 1-bit delay from WS to MSB (I2S format)
  // div1, div2 = Set I2S WS clock frequency.  BCLK seems to be generated from 32x this
  I2SC |= I2SRF | I2SMR | I2SRMS | I2STMS | (div1 << I2SBD) | (div2 << I2SCD);
}

/**
 * Initialize the SLC module for DMA operation.
 * Counter intuitively, we use the TXLINK here to
 * receive data.
 */
void
slc_init()
{
  for (int x = 0; x < SLC_BUF_CNT; x++) {
    i2s_slc_buf_pntr[x] = (uint32_t *)malloc(SLC_BUF_LEN * 4);
    for (int y = 0; y < SLC_BUF_LEN; y++) i2s_slc_buf_pntr[x][y] = 0;

    i2s_slc_items[x].unused = 0;
    i2s_slc_items[x].owner = 1;
    i2s_slc_items[x].eof = 0;
    i2s_slc_items[x].sub_sof = 0;
    i2s_slc_items[x].datalen = SLC_BUF_LEN * 4;
    i2s_slc_items[x].blocksize = SLC_BUF_LEN * 4;
    i2s_slc_items[x].buf_ptr = (uint32_t *)&i2s_slc_buf_pntr[x][0];
    i2s_slc_items[x].next_link_ptr = (uint32_t *)((x < (SLC_BUF_CNT - 1)) ? (&i2s_slc_items[x + 1]) : (&i2s_slc_items[0]));
  }

  // Reset DMA
  ETS_SLC_INTR_DISABLE();
  SLCC0 |= SLCRXLR | SLCTXLR;
  SLCC0 &= ~(SLCRXLR | SLCTXLR);
  SLCIC = 0xFFFFFFFF;

  // Configure DMA
  SLCC0 &= ~(SLCMM << SLCM);      // Clear DMA MODE
  SLCC0 |= (1 << SLCM);           // Set DMA MODE to 1
  SLCRXDC |= SLCBINR | SLCBTNR;   // Enable INFOR_NO_REPLACE and TOKEN_NO_REPLACE

  // Feed DMA the 1st buffer desc addr
  SLCTXL &= ~(SLCTXLAM << SLCTXLA);
  SLCTXL |= (uint32_t)&i2s_slc_items[0] << SLCTXLA;

  ETS_SLC_INTR_ATTACH(slc_isr, NULL);

  // Enable EOF interrupt
  SLCIE = SLCITXEOF;
  ETS_SLC_INTR_ENABLE();

  // Start transmission
  SLCTXL |= SLCTXLS;
}

/**
 * Triggered when SLC has finished writing
 * to one of the buffers.
 */
void ICACHE_RAM_ATTR
slc_isr(void *para)
{
  uint32_t status;

  status = SLCIS;
  SLCIC = 0xFFFFFFFF;

  if (status == 0) {
    return;
  }

  if (status & SLCITXEOF) {
    // We have received a frame
    ETS_SLC_INTR_DISABLE();
    sdio_queue_t *finished = (sdio_queue_t*)SLCTXEDA;

    finished->eof = 0;
    finished->owner = 1;
    finished->datalen = 0;

    for (int i = 0; i < SLC_BUF_CNT; i++) {
      if (finished == &i2s_slc_items[i]) {
        rx_buf_idx = i;
      }
    }
    rx_buf_cnt++;
    rx_buf_flag = true;
    ETS_SLC_INTR_ENABLE();
  }
}




void prepare_sample(int16_t * d_buff, uint32_t* s_buff, uint32_t len)   //len - count of full word
{
    int32_t temp;
    int16_t temp_16;
    uint32_t tempU32;
    
    for (int i = 0; i < len; i ++) {
        //temp = *(int32_t *)&s_buff[i];
        tempU32 = s_buff[i]<<1;
        temp = *(int32_t *)&tempU32;

//        Serial.printf("  %08x", temp);
//        Serial.printf("\n\r");
//       temp = temp>>16;
        temp = temp/SIGNAL_GAIN;                            //8192;//16384;//32768;//65536;
        temp_16 = (int16_t)(temp);
        //temp_16 = reverse_sample16(temp_16);   //gain++     need 65536; //130000   262144   

        //Serial.printf("  %08x", temp);
       // d_buff[i] = (int16_t)(temp);
        d_buff[i] = temp_16; 
        //Serial.printf("  %04x\n\r", d_buff[i]);  
    }
}

void prepare_sample_24bit(uint8_t * d_buff, uint32_t* s_buff, uint32_t len)   //len - count of full word
{
    //int32_t temp;
    
    for (int i = 0; i < len; i ++) {
#ifndef USE_SPH0645     
        *d_buff = (uint8_t)((s_buff[i]>>8)&0xFF); 
        d_buff++;
        *d_buff = (uint8_t)((s_buff[i]>>16)&0xFF); 
        d_buff++;
        *d_buff = (uint8_t)((s_buff[i]>>24)&0xFF);  
        d_buff++; 
#else
        //for SPH0645
        //temp = *(int32_t *)&s_buff[i];
        *d_buff = (uint8_t)(((s_buff[i]<<1)>>8)&0xFF); 
        d_buff++;
        *d_buff = (uint8_t)(((s_buff[i]<<1)>>16)&0xFF); 
        d_buff++;
        *d_buff = (uint8_t)(((s_buff[i]<<1)>>24)&0xFF);  
        d_buff++; 
#endif             
    }
}


//int32_t prepare_sample(uint32_t x)
//{
//    int32_t temp;
//    temp = *(int32_t *)&x;
//    temp = temp>>8;  // 32 to 24 bit
////    x = x>>1;  // 
////    x = (x & 0xFF) <<  8 | (x >> 8) & 0xFF;   //reverse
////    x = (x & 0x00FF00FF) <<  8 | (x & 0xFF00FF00) >>  8;
////    x = (x & 0x0000FFFF) << 16 | (x & 0xFFFF0000) >> 16;
////    temp = (int32_t)(x - 8388607);
////    temp = *(int32_t *)&x;
////    temp = (temp / 0x100) ; 
//    
//    return (temp);
//}

// I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S
