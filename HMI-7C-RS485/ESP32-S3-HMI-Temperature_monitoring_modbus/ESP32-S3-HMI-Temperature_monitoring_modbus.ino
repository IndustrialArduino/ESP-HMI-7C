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
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <PCA9536D.h>

#define SDA 19
#define SCL 20

#define RS485_RXD 2
#define RS485_TXD 9
//#define RS485_FC  0

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

#define UART_SEL 3
#define GSM_RESET 2

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
  47 /* G0 */, 21 /* G1 */, 14 /* G2 */, 46 /* G3 */, 3 /* G4 */, 3 /* G5 */,
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
}

#endif

PCA9538 io(PCA9538_ADDR);

PCA9536 PortIO;

ModbusMaster node;

//void preTransmission() {
//  digitalWrite(RS485_FC, 1);
//}
//void postTransmission() {
//  digitalWrite(RS485_FC, 0);
//}

void setup() {
  Serial.begin(9600);
  Serial.println("LVGL Widgets Demo");

  Serial1.begin(9600, SERIAL_8N1, RS485_RXD, RS485_TXD);
  delay(1000);

  pinMode(TFT_RST, OUTPUT);
  delay(1000);
  digitalWrite(TFT_RST, LOW);
  delay(1000);
  digitalWrite(TFT_RST, HIGH);
  delay(1000);

//  pinMode(RS485_FC, OUTPUT);

  Wire.begin(SDA, SCL);
  delay(100);

  //I2C_SCAN();
 // delay(1000);

  if (PortIO.begin() == false)
  {
    Serial.println("PCA9536 not detected. Please check wiring. Freezing...");
    while (1);
  }
  delay(100);
  PortIO.pinMode(UART_SEL, OUTPUT);
  PortIO.pinMode(GSM_RESET, OUTPUT);
  
  PortIO.digitalWrite(GSM_RESET, HIGH); 
  delay(1000); 
  
  PortIO.digitalWrite(UART_SEL, LOW); 
  delay(100);  
 

  node.begin(1, Serial1); // XY-MD02 default ID = 1
//  node.preTransmission(preTransmission);
//  node.postTransmission(postTransmission);

  touch_init();
  delay(50);

  // Init Display
  lcd->begin();
  lcd->fillScreen(BLACK);
  lcd->setTextSize(2);
  delay(200);

#ifdef USE_UI
  lv_init();

  delay(100);

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

void loop() {
#ifdef USE_UI
  lv_timer_handler();
  delay(5);
#endif

}

void GPIOCallback(lv_timer_t * timer) {
  // PortIO.digitalWrite(UART_SEL, LOW); 
    delay(100);
  // === Read Modbus from XY-MD02 ===
  uint8_t result = node.readInputRegisters(0x0001, 2);// read Temp + Hum
  float temperature = NAN, humidity = NAN;

  if (result == node.ku8MBSuccess) {
    int16_t tempRaw = (int16_t)node.getResponseBuffer(0);   // signed
    uint16_t humRaw = node.getResponseBuffer(1);            // unsigned
    temperature = tempRaw / 10.0;
    humidity    = humRaw / 10.0;

    Serial.print("Temperature: "); Serial.println(temperature);
    Serial.print("Humidity: "); Serial.println(humidity);

    // --- Show numeric values ---
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f °C", temperature);
    lv_label_set_text(ui_temperaturevalue, buf);

    snprintf(buf, sizeof(buf), "%.1f %%", humidity);
    lv_label_set_text(ui_Humidityvalue, buf);

    // --- Temperature Bar Fill (0–50 °C mapped to 0–full height) ---
    int fullHeight = lv_obj_get_height(ui_Tempempty);  // height of background
    int fillHeight = (int)((temperature / 50.0) * fullHeight);
    if (fillHeight < 0) fillHeight = 0;
    if (fillHeight > fullHeight) fillHeight = fullHeight;

    // Resize ui_Tempfull to match value
    lv_obj_set_height(ui_Tempfull, fillHeight);

    // Align fill to bottom of the thermometer
    lv_obj_align_to(ui_Tempfull, ui_Tempempty, LV_ALIGN_BOTTOM_MID, 0, 0);

    // --- Humidity Arc (0–100%) ---
    int arc_val = (int)humidity;
    if (arc_val < 0) arc_val = 0;
    if (arc_val > 100) arc_val = 100;
    lv_arc_set_value(ui_Arc, arc_val);


    // --- Temperature Zone (0–50 °C) ---
    if (temperature < 20) {
      lv_label_set_text(ui_temperaturezone, "COLD");
      lv_obj_set_style_text_color(ui_temperaturezone, lv_color_hex(0x00FF51), 0); // green
    } else if (temperature < 35) {
      lv_label_set_text(ui_temperaturezone, "WARM");
      lv_obj_set_style_text_color(ui_temperaturezone, lv_color_hex(0xFFF200), 0); // yellow
    } else {
      lv_label_set_text(ui_temperaturezone, "HOT");
      lv_obj_set_style_text_color(ui_temperaturezone, lv_color_hex(0xFF0000), 0); // red
    }

    // --- Humidity Zone (0–100%) ---
    if (humidity < 40) {
      lv_label_set_text(ui_Humidityzone, "DRY");
      lv_obj_set_style_text_color(ui_Humidityzone, lv_color_hex(0x00FF51), 0); // green
    } else if (humidity < 70) {
      lv_label_set_text(ui_Humidityzone, "COMFORT");
      lv_obj_set_style_text_color(ui_Humidityzone, lv_color_hex(0xFFF200), 0); // yellow
    } else {
      lv_label_set_text(ui_Humidityzone, "HUMID");
      lv_obj_set_style_text_color(ui_Humidityzone, lv_color_hex(0xFF0000), 0); // red
    }


  } else {
    Serial.println("Modbus read failed");
    Serial.println(result, HEX);
  }

}
