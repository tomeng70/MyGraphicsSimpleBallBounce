#include <Elegoo_GFX.h>    // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Elegoo_TFTLCD tft;

int xpos;
int ypos;
int r;
int xvel;
int yvel;

int leftbound;
int rightbound;
int topbound;
int bottombound;

void setup(void) {
  Serial.begin(9600);
  Serial.println(F("TFT LCD test"));

#ifdef USE_Elegoo_SHIELD_PINOUT
  Serial.println(F("Using Elegoo 2.8\" TFT Arduino Shield Pinout"));
#else
  Serial.println(F("Using Elegoo 2.8\" TFT Breakout Board Pinout"));
#endif

  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());

  tft.reset();

  uint16_t identifier = tft.readID();
   if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x4535) {
    Serial.println(F("Found LGDP4535 LCD driver"));
  }else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else if(identifier==0x0101)
  {     
      identifier=0x9341;
       Serial.println(F("Found 0x9341 LCD driver"));
  }else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Elegoo 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_Elegoo_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Elegoo_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    identifier=0x9341;
  
  }

  tft.begin(identifier);

  // set to landscape mode.
  tft.setRotation(1);
  tft.fillScreen(BLACK);

  leftbound = 0;
  rightbound = tft.width() - 1;
  topbound = 0;
  bottombound = tft.height() - 1;

  xpos = tft.width() / 2;
  ypos = tft.height() / 2;
  r = 6;

  Serial.print("xpos, ypos = ");
  Serial.print(xpos);
  Serial.print(", ");
  Serial.println(ypos);
  tft.fillCircle(xpos,ypos, r, GREEN);

  randomSeed(analogRead(15));
  xvel = random(5, 15);
  yvel = random(5, 15);
}

void loop() {
  // put your main code here, to run repeatedly:
  checkCollision();
  erasePrevious();
  updatePosition();
  delay(60);
}

void checkCollision() {
  // x direction.
  if ((xpos - r) < leftbound) {
    xvel = -xvel;
  } else if ((xpos + r) > rightbound) {
    xvel = -xvel;
  }

  // y direction.
  if ((ypos - r) < topbound) {
    yvel = -yvel;
  } else if ((ypos + r) > bottombound) {
    yvel = -yvel;
  }
}

void erasePrevious() {
  tft.fillCircle(xpos, ypos, r, BLACK);
}

void updatePosition() {
  xpos += xvel;
  ypos += yvel;
  tft.fillCircle(xpos,ypos, r, GREEN);
}
