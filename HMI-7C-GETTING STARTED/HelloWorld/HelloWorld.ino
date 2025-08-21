/*******************************************************************************
 ******************************************************************************/
#include <Arduino_GFX_Library.h>

#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin
#define TFT_BL -1
#define TFT_RST 39
/* More dev device declaration: https://github.com/moononournation/Arduino_GFX/wiki/Dev-Device-Declaration */
#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *gfx = create_default_Arduino_GFX();
#else /* !defined(DISPLAY_DEV_KIT) */

/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
//Arduino_DataBus *bus = create_default_Arduino_DataBus();

/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
//Arduino_GFX *gfx = new Arduino_ILI9341(bus, 39,, 0 /* rotation */, false /* IPS */);

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    4 /* DE */, 5 /* VSYNC */, 6 /* HSYNC */, 7 /* PCLK */,
    1 /* R0 */, 41 /* R1 */, 40 /* R2 */, 38 /* R3 */, 45 /* R4 */,
    47 /* G0 */, 21 /* G1 */, 14 /* G2 */, 9 /* G3 */, 3 /* G4 */, 3 /* G5 */,
    8 /* B0 */, 18 /* B1 */, 17 /* B2 */, 16 /* B3 */, 15 /* B4 */
);
Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
    bus,
    1024 /* width */, 0 /* hsync_polarity */, 210 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    600 /* height */, 0 /* vsync_polarity */, 22 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

#endif /* !defined(DISPLAY_DEV_KIT) */
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

void setup(void)
{
    pinMode(TFT_RST, OUTPUT);
    delay(1000);
    digitalWrite(TFT_RST, LOW);
    delay(1000);
    digitalWrite(TFT_RST, HIGH);
    delay(1000);
    
    gfx->begin();
    //delay(2000);
    gfx->fillScreen(BLACK);

//#ifdef TFT_BL
//    pinMode(TFT_BL, OUTPUT);
//    digitalWrite(TFT_BL, HIGH);
//#endif

    gfx->setCursor(10, 10);
    gfx->setTextColor(RED);
    gfx->println("NORVI CONTROLLER");

    gfx->setCursor(180, 250);
    gfx->setTextColor(GREEN, BLACK);
    gfx->setTextSize(7/* x scale */, 12 /* y scale */, random(2) /* pixel_margin */);
    gfx->println("NORVI CONTROLLERS");

    delay(5000); // 5 seconds
}

void loop()
{
    //gfx->setCursor(200, 400);
    //gfx->setTextColor(GREEN, BLACK);
    //gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */, random(2) /* pixel_margin */);
    //gfx->println("NORVI CONTROLLERS");

    delay(2000); // 1 second
}
