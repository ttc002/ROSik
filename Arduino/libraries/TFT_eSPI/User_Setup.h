
#define USER_SETUP_INFO "User_Setup"
#define ST7735_DRIVER // Define additional parameters below for this display
#define TFT_WIDTH 128
#define TFT_HEIGHT 160
#define ST7735_BLACKTAB
#define TFT_CS PIN_D8 // Chip select control pin D8
#define TFT_DC PIN_D2 // Data Command control pin
#define TFT_RST PIN_D4 // Reset pin (could connect to NodeMCU RST, see next line)

// this will save ~20kbytes of FLASH
#define SMOOTH_FONT
#define SPI_FREQUENCY 27000000
#define SPI_READ_FREQUENCY 20000000
#define SPI_TOUCH_FREQUENCY 2500000

