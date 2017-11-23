/*************************************************** 
DaskeTaske for julebord 2017

Uses Adafruit 1.5" OLED https://www.adafruit.com/product/1431


Requires:
* Adafruit SSD1351 Library - https://github.com/adafruit/Adafruit-SSD1351-library/archive/master.zip
* Adafruit GFX Library - https://github.com/adafruit/Adafruit-GFX-Library/archive/master.zip
* 

Stay kyb!

*****************/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>


// If we are using the hardware SPI interface, these are the pins (for future ref)
#define sclk 13
#define mosi 11
#define cs   5
#define rst  6
#define dc   4

// SDCS 10
// MISO 12


// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

// to draw images from the SD card, we will share the hardware SPI interface
Adafruit_SSD1351 tft = Adafruit_SSD1351(cs, dc, rst);

// For Arduino Uno/Duemilanove, etc
//  connect the SD card with MOSI going to pin 11, MISO going to pin 12 and SCK going to pin 13 (standard)
//  Then pin 10 goes to CS (or whatever you have set up)
#define SD_CS 10    // Set the chip select line to whatever you use (10 doesnt conflict with the library)

// the file itself
File bmpFile;

// information we extract about the bitmap file
int bmpWidth, bmpHeight;
uint8_t bmpDepth, bmpImageoffset;
float t=0;
unsigned long lastAction = 1;

float zeta=0.1; 
float old_zeta;

float w;
float old_w;


#define DT 0.1
#define MAX_IMAGE 2
#define MAX_SCREEN 4
#define SCREEN_SELECT_PIN 3
#define POT1 1

#define BODE_X_MIN 0.1
#define BODE_X_MAX 10 

#define BODE_Y_MAX 40
#define BODE_Y_MIN -40

#define XRES 128
#define YRES 128

int image=0;

int screen=2;
int new_screen = 1;

int buttonEdge = 0;

void setup(void) {
  
  Serial.begin(9600);
   
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
     
  // initialize the OLED
  tft.begin();
  tft.setRotation(3);
  Serial.println("init");
  
  tft.fillScreen(BLUE);
  
  delay(500);
  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
    return;
  }
  Serial.println("SD OK!");
  tft.fillScreen(BLACK);

  pinMode(SCREEN_SELECT_PIN,INPUT_PULLUP);
  
}

void loop() {
    if (millis()-lastAction > 60000) {
      screen = screen+1;
      Serial.println("Timeout to new screen");
      screen = capInt(screen,MAX_SCREEN);
      lastAction = millis();
      new_screen = 1;
    }
    
    switch (screen) {
      case 0:
        if (new_screen == 1) {
          drawBitmaps(0);
        }
        new_screen = 0;
        break;
      case 1:
        if (new_screen == 1) {
          tft.fillScreen(BLACK);
          new_screen = 0;        
        }
        drawSine();
        
        break;
      case 2:
       if (new_screen == 1) {
          drawBitmaps(1);
        }
        new_screen = 0;        
        break;        
      case 3:
        if (new_screen == 1) {
          tft.fillScreen(BLACK);
          new_screen = 0;                  
        }      
        drawBode();
        break;        
      case 4:
       if (new_screen == 1) {
          drawBitmaps(2);
        }
        new_screen = 0;        
        break;         
    }
    

    int val = digitalRead(SCREEN_SELECT_PIN);

    if ((val == LOW) && (buttonEdge==0)) {
      
      screen = screen+1;
      screen = capInt(screen,MAX_SCREEN);
      tft.fillScreen(BLACK);
     
      Serial.print("Switching to screen:");
      Serial.println(screen);
      buttonEdge = 1;
      
      new_screen = 1;
      lastAction = millis();

    }
    else if (( buttonEdge==1 ) && (val == HIGH) ) {
      buttonEdge = 0;
    }
     

}

void drawBode() {
  float w0=1; 

  float gain;
  float old_gain;
  float w;

  int x;
  int y;
  int potread;
  potread = analogRead(POT1);
  Serial.println(potread);
  old_zeta = zeta;
  zeta = map(potread,0,1023,0.01*1000,1*1000)/1000.0;
  Serial.println(zeta);

  for (int x=0;x<XRES;x++) {
    w = pow(10,((float)x/XRES)*(log10(BODE_X_MAX)-log10(BODE_X_MIN))+log10(BODE_X_MIN));
    gain = 20*log10(w0/sqrt(square(-1*square(w0) + w) + square(2*zeta*w*w0)));
    old_gain = 20*log10(w0/sqrt(square(-1*square(w0) + w) + square(2*old_zeta*w*w0)));
    

    y = map(old_gain,BODE_Y_MIN,BODE_Y_MAX,0,YRES);
    tft.drawPixel(x,YRES-y,BLACK);

    y = map(gain,BODE_Y_MIN,BODE_Y_MAX,0,YRES);
    tft.drawPixel(x,YRES-y,RED);
    
  }
}

void drawBitmaps(int image) {
  switch (image) {
    case 0:
      bmpDraw("c1.bmp", 0, 0);
      break;
    case 1:
      bmpDraw("c2.bmp", 0, 0);
      break;
    case 2:
      bmpDraw("c3.bmp", 0, 0);
      break;
  }
}

void drawSine() {
      int potread;
      potread = analogRead(POT1);
      Serial.println(potread);
      old_w = w;
      w = map(potread,0,1023,0.5*1000,3*1000)/1000.0;
      Serial.println(zeta);  
  
      for (int y=0;y<128;y++) {
      int x1 = floor(sin(t-DT+(old_w*3.14*y/128.0))*64)+64;
      
      tft.drawPixel(y,x1,BLACK);
      int x2 = floor(sin(t+(w*3.14*y/128.0))*64)+64;
      tft.drawPixel(y,x2,YELLOW);
    }    
    t=t+DT;
}

int capInt(int i, int cap) {
  if (i > cap) {
    return 0;
  }
  return i;
}

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 60  

void bmpDraw(char *filename, uint8_t x, uint8_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print("Loading image '");
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print("File not found");
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print("File size: "); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print("Image Offset: "); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print("Header size: "); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print("Bit Depth: "); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print("Image size: ");
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        for (row=0; row<h; row++) { // For each scanline...
          tft.goTo(x, y+row);

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          // optimize by setting pins now
          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];

            tft.drawPixel(x+col, y+row, tft.Color565(r,g,b));
            // optimized!
            //tft.pushColor(tft.Color565(r,g,b));
          } // end pixel
        } // end scanline
        Serial.print("Loaded in ");
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println("BMP format not recognized.");
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}
