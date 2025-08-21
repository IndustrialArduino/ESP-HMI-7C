

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "RTClib.h"
#include "FS.h"
#include "SD.h"
#include <Adafruit_ADS1X15.h>
#include <PCA9538.h>



#define SDA 19
#define SCL 20

#define RS485_RXD 46
#define RS485_TXD 2
#define RS485_FC  0

#define ETHERNET_RESET -1
#define ETH_CS 10

#define MISO 12
#define MOSI 13
#define SCLK 11

#define DSP_CS 47

// I2C address of PCA9538
#define PCA9538_ADDR 0x73

#define INPUT1 1
#define INPUT2 2
#define INPUT3 3
#define INPUT4 4
#define OUTPUT1  5
#define OUTPUT2  6
#define OUTPUT3  7
#define OUTPUT4  8

// SD Paramerters
#define SD_chipSelect 48

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

unsigned int localPort = 8888;       // local port to listen for UDP packets

const char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

EthernetUDP Udp;// A UDP instance to let us send and receive packets over UDP

unsigned long int timer1 = 0;
unsigned long int millis_start = 0;
unsigned long total = 0;
unsigned long tn = 0;

Adafruit_ADS1115 ads1;
float current_multiplier = 1.274;

#define USE_UI    //if you want to use the ui export from Squareline, please do not annotate this line.
#if defined USE_UI
#include <lvgl.h>
#include "ui.h"
#endif

#include <Arduino_GFX_Library.h>
#define TFT_BL -1
#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin
#define TFT_RST 39

/******Please define a corresponding line based on your development board.************/
//#define Display_50
#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *gfx = create_default_Arduino_GFX();
#else /* !defined(DISPLAY_DEV_KIT) */


Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
  4 /* DE */, 5 /* VSYNC */, 6 /* HSYNC */, 7 /* PCLK */,
  1 /* R0 */, 41 /* R1 */, 40 /* R2 */, 38 /* R3 */, 45 /* R4 */,
  47 /* G0 */, 21 /* G1 */, 14 /* G2 */, 9 /* G3 */, 3 /* G4 */, 3 /* G5 */,
  8 /* B0 */, 18 /* B1 */, 17 /* B2 */, 16 /* B3 */, 15 /* B4 */
);
Arduino_RPi_DPI_RGBPanel *lcd = new Arduino_RPi_DPI_RGBPanel(
  bus,
  1024 /* width */, 0 /* hsync_polarity */, 210 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
  600 /* height */, 0 /* vsync_polarity */, 22 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
  1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

#endif

/*******************************************************************************
   Screen Driver Configuration  end
*******************************************************************************/


/*******************************************************************************
   Please config the touch panel in touch.h
 ******************************************************************************/
#include "touch.h"

#ifdef USE_UI
/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[1024 * 600 / 10];      //5,7inch: lv_color_t disp_draw_buf[800*480/10]            4.3inch: lv_color_t disp_draw_buf[480*272/10]
//static lv_color_t disp_draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  lcd->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  lcd->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      Serial.print( "Data x :" );
      Serial.println( touch_last_x );

      Serial.print( "Data y :" );
      Serial.println( touch_last_y );
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
  delay(15);
}
#endif

PCA9538 io(PCA9538_ADDR);

void setup()
{
  Serial.begin(115200);
  Serial.println("LVGL Widgets Demo");

  /*#if defined(Display_50) || defined(Display_70)
    //IO Port Pins
    pinMode(38, OUTPUT);
    digitalWrite(38, LOW);
    pinMode(17, OUTPUT);
    digitalWrite(17, LOW);
    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);
    pinMode(42, OUTPUT);
    digitalWrite(42, LOW);
    #elif defined(Display_43)
    pinMode(20, OUTPUT);
    digitalWrite(20, LOW);
    pinMode(19, OUTPUT);
    digitalWrite(19, LOW);
    pinMode(35, OUTPUT);
    digitalWrite(35, LOW);
    pinMode(38, OUTPUT);
    digitalWrite(38, LOW);
    pinMode(0, OUTPUT);//TOUCH-CS
    #endif*/

  Serial2.begin(115200, SERIAL_8N1, RS485_RXD, RS485_TXD);
  delay(1000);



  pinMode(TFT_RST, OUTPUT);
  delay(1000);
  digitalWrite(TFT_RST, LOW);
  delay(1000);
  digitalWrite(TFT_RST, HIGH);
  delay(1000);


  Wire.begin(SDA, SCL);
  delay(100);

  //I2C_SCAN();
  //delay(1000);

  pinMode(RS485_FC, OUTPUT);
  //  pinMode(GSM_RESET, OUTPUT);

  //  digitalWrite(GSM_RESET, LOW);
  //  delay(1000);
  //  digitalWrite(GSM_RESET, HIGH);

  SPI.begin(SCLK, MISO, MOSI); // Ensure these pin numbers are correct // SCLK, MISO, MOSI
  delay(1000);

  //RTC_Check();
  //delay(1000);

  //SD_CHECK();
  // delay(1000);

  //ETHERNET_CHECK();

  //WiFi.mode(WIFI_MODE_APSTA);
  // delay(300);
  // wifi_test();

  io.pinMode(INPUT1, INPUT);
  io.pinMode(INPUT2, INPUT);
  io.pinMode(INPUT3, INPUT);
  io.pinMode(INPUT4, INPUT);

  io.pinMode(OUTPUT1, OUTPUT);
  io.pinMode(OUTPUT2, OUTPUT);
  io.pinMode(OUTPUT3, OUTPUT);
  io.pinMode(OUTPUT4, OUTPUT);

  if (!ads1.begin(0x49)) {
    Serial.println("Failed to initialize ADS 1 .");
    // while (1);
  }
  ads1.setGain(GAIN_ONE);

  // Init Display
  lcd->begin();
  lcd->fillScreen(BLACK);
  lcd->setTextSize(2);
  delay(200);

#ifdef USE_UI
  lv_init();

  delay(100);
  touch_init();

  screenWidth = lcd->width();
  screenHeight = lcd->height();

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 10);
  //  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, 480 * 272 / 10);
  /* Initialize the display */
  lv_disp_drv_init(&disp_drv);
  /* Change the following line to your display resolution */
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /* Initialize the (dummy) input device driver */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
#endif

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

#ifdef USE_UI
  ui_init();//ui from Squareline or GUI Guider
#else
  lcd->fillScreen(RED);
  delay(800);
  lcd->fillScreen(BLUE);
  delay(800);
  lcd->fillScreen(YELLOW);
  delay(800);
  lcd->fillScreen(GREEN);
  delay(800);
#endif

  lv_timer_create(GPIOCallback, 1000, NULL);
  Serial.println( "Setup done" );



}

void loop()
{
  //while(1)
  //{
#ifdef USE_UI
  lv_timer_handler();
  delay(5);
#endif
  // }
}

void GPIOCallback(lv_timer_t * timer) {
  // ----------- DIGITAL INPUTS -----------
  int di1 = io.digitalRead(INPUT1);
  int di2 = io.digitalRead(INPUT2);
  int di3 = io.digitalRead(INPUT3);
  int di4 = io.digitalRead(INPUT4);

  Serial.printf("DI1:%d DI2:%d DI3:%d DI4:%d\n", di1, di2, di3, di4);

  lv_label_set_text_fmt(ui_DI1, "%d", di1);
  lv_label_set_text_fmt(ui_DI2, "%d", di2);
  lv_label_set_text_fmt(ui_DI3, "%d", di3);
  lv_label_set_text_fmt(ui_DI4, "%d", di4);

  // ----------- TRANSISTOR OUTPUT TEST (Non-blocking) -----------
  static uint8_t step = 0;          // Tracks which output to toggle
  static uint32_t lastTime = 0;     // Timestamp for non-blocking timing
  uint32_t now = millis();

  if (now - lastTime > 300) {       // 300ms per step
    switch (step) {
      case 0: io.digitalWrite(OUTPUT1, HIGH); lv_label_set_text(ui_TR1, "ON"); break;
      case 1: io.digitalWrite(OUTPUT2, HIGH); lv_label_set_text(ui_TR2, "ON"); break;
      case 2: io.digitalWrite(OUTPUT3, HIGH); lv_label_set_text(ui_TR3, "ON"); break;
      case 3: io.digitalWrite(OUTPUT4, HIGH); lv_label_set_text(ui_TR4, "ON"); break;
      case 4:
        io.digitalWrite(OUTPUT1, LOW);
        io.digitalWrite(OUTPUT2, LOW);
        io.digitalWrite(OUTPUT3, LOW);
        io.digitalWrite(OUTPUT4, LOW);
        lv_label_set_text(ui_TR1, "OFF");
        lv_label_set_text(ui_TR2, "OFF");
        lv_label_set_text(ui_TR3, "OFF");
        lv_label_set_text(ui_TR4, "OFF");
        break;
    }
    step = (step + 1) % 5;
    lastTime = now;
  }

  // ----------- ANALOG INPUTS -----------
  int16_t adc0 = ads1.readADC_SingleEnded(0);
  int16_t adc1 = ads1.readADC_SingleEnded(1);
  int16_t adc2 = ads1.readADC_SingleEnded(2);
  int16_t adc3 = ads1.readADC_SingleEnded(3);

  float current0 = (current_multiplier * adc0) / 1000.0;
  float current1 = (current_multiplier * adc1) / 1000.0;
  float current2 = (current_multiplier * adc2) / 1000.0;
  float current3 = (current_multiplier * adc3) / 1000.0;

  Serial.printf("Input Current 0: %.1f mA\n", current0);
  Serial.printf("Input Current 1: %.1f mA\n", current3);
  Serial.printf("Input Current 2: %.1f mA\n", current1);
  Serial.printf("Input Current 3: %.1f mA\n", current2);

  char buf[16];

  // Convert each float to string with 1 decimal
  snprintf(buf, sizeof(buf), "%.1f mA", current0);
  lv_label_set_text(ui_AI1, buf);

  snprintf(buf, sizeof(buf), "%.1f mA", current3);
  lv_label_set_text(ui_AI2, buf);

  snprintf(buf, sizeof(buf), "%.1f mA", current1);
  lv_label_set_text(ui_AI3, buf);

  snprintf(buf, sizeof(buf), "%.1f mA", current2);
  lv_label_set_text(ui_AI4, buf);

}





void I2C_SCAN() {
  byte error, address;
  int deviceCount = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      deviceCount++;
      delay(1);  // Wait for a moment to avoid overloading the I2C bus
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("Scanning complete\n");
  }
}

void SD_CHECK() {
  Serial.print("\nInitializing SD card...");
  uint8_t cardType = SD.cardType();

  if (SD.begin(SD_chipSelect))
  {
    Serial.println("Card Mount: success");
    Serial.print("Card Type: ");

    if (cardType == CARD_MMC) {
      Serial.println("MMC");
    } else if (cardType == CARD_SD) {
      Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
      Serial.println("SDHC");
    } else {
      Serial.println("Unknown");
    }

    int cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("Card Size: %lluMB\n", cardSize);
  }

  if (!SD.begin(5))
  {
    Serial.println("NO SD card");
  }
}

void displayTime(void) {
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);

  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  delay(1000);

}

void RTC_Check() {
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }
  else {
    if (rtc.lostPower()) {

      Serial.println("RTC lost power, lets set the time!");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    }


    int a = 1;
    while (a < 6)
    {
      displayTime();   // printing time function for oled
      a = a + 1;
    }
  }
}

void ETHERNET_CHECK() {
  Ethernet.init(ETH_CS);  // CS PIN OF THE ETHERNET

  // start Ethernet and UDP
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");

    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    }

    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
  }

  else {
    Udp.begin(localPort);

    sendNTPpacket(timeServer); // send an NTP packet to a time server

    // wait to see if a reply is available
    delay(1000);
    if (Udp.parsePacket()) {
      // We've received a packet, read the data from it
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      // the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, extract the two words:

      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      Serial.print("Seconds since Jan 1 1900 = ");
      Serial.println(secsSince1900);

      // now convert NTP time into everyday time:
      Serial.print("Unix time = ");
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      // print Unix time:
      Serial.println(epoch);


      // print the hour, minute and second:
      Serial.print("The UTC time is ");
      Serial.print((epoch  % 86400L) / 3600);
      // print the hour (86400 equals secs per day)
      Serial.print(':');
      if (((epoch % 3600) / 60) < 10) {
        // In the first 10 minutes of each hour, we'll want a leading '0'
        Serial.print('0');
      }

      Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
      Serial.print(':');
      if ((epoch % 60) < 10) {
        // In the first 10 seconds of each minute, we'll want a leading '0'
        Serial.print('0');
      }

      Serial.println(epoch % 60); // print the second
    }
    // wait ten seconds before asking for the time again
    delay(3000);
    Ethernet.maintain();
  }
}


void sendNTPpacket(const char * address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void wifi_test() {
  Serial.println("");
  String str_macAddress;
  byte mac[6];

  WiFi.macAddress(mac);

  str_macAddress = (String(mac[0] >> 4,  HEX) + String(mac[0] & 0x0F, HEX)) + (":") +
                   (String(mac[1] >> 4,  HEX) + String(mac[1] & 0x0F, HEX)) + (":") +
                   (String(mac[2] >> 4,  HEX) + String(mac[2] & 0x0F, HEX)) + (":") +
                   (String(mac[3] >> 4,  HEX) + String(mac[3] & 0x0F, HEX)) + (":") +
                   (String(mac[4] >> 4,  HEX) + String(mac[4] & 0x0F, HEX)) + (":") +
                   (String(mac[5] >> 4,  HEX) + String(mac[5] & 0x0F, HEX));
  str_macAddress.toUpperCase();

  Serial.println("MAC Address: " + str_macAddress);

  String ssid =  str_macAddress;
  const char* password = "12345678";

  Serial.println("Setting up the Wi-Fi Access Point...");
  WiFi.softAP(ssid.c_str(), password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(IP);

  Serial.print("Access Point SSID: ");
  Serial.println(ssid);

  Serial.println("Wi-Fi Access Point is active!");
  Serial.println("");
}
