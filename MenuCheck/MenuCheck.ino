#include <GEM_adafruit_gfx.h>
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#define SS_SWITCH        24
#define SS_NEOPIX        6

#define SEESAW_ADDR          0x36

Adafruit_seesaw ss;
seesaw_NeoPixel sspixel = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);

int32_t encoder_position;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Create variables that will be editable through the menu and assign them initial values
int interval = 500;
char label[GEM_STR_LEN] = "Blink!"; // Maximum length of the string should not exceed 16 characters
                                    // (plus special terminating character)!

// Supplementary variable used in millis based version of Blink routine
unsigned long previousMillis = 0;

// Variable to hold current label state (visible or hidden)
boolean labelOn = false;

// Create two menu item objects of class GEMItem, linked to interval and label variables
// with validateInterval() callback function attached to interval menu item,
// that will make sure that interval variable is within allowable range (i.e. >= 50)
void validateInterval(); // Forward declaration
GEMItem menuItemInterval("Interval:", interval, validateInterval);
GEMItem menuItemLabel("Label:", label);

// Create menu button that will trigger blinkDelay() function. It will blink the label on the screen with delay()
// set to the value of interval variable. We will write (define) this function later. However, we should
// forward-declare it in order to pass to GEMItem constructor
void blinkDelay(); // Forward declaration
GEMItem menuItemDelayButton1("Blink v1", blinkDelay);
// Likewise, create menu button that will trigger blinkMillis() function. It will blink the label on the screen with millis based
// delay set to the value of interval variable. We will write (define) this function later. However, we should
// forward-declare it in order to pass to GEMItem constructor
void blinkMillis(); // Forward declaration
GEMItem menuItemDelayButton2("Blink v2", blinkMillis);

// Create menu page object of class GEMPage. Menu page holds menu items (GEMItem) and represents menu level.
// Menu can have multiple menu pages (linked to each other) with multiple menu items each
GEMPage menuPageMain("Main Menu"); // Main page
GEMPage menuPageSettings("Settings"); // Settings submenu

// Create menu item linked to Settings menu page
GEMItem menuItemMainSettings("Settings", menuPageSettings);

// Create menu object of class GEM_adafruit_gfx. Supply its constructor with reference to tft object we created earlier
GEM_adafruit_gfx menu(tft, GEM_POINTER_ROW, GEM_ITEMS_COUNT_AUTO);

void setup() {
  // Serial communications setup
  Serial.begin(115200);

  if (! ss.begin(SEESAW_ADDR) || ! sspixel.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while(1) delay(10);
  }
  Serial.println("seesaw started");

  uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
  if (version  != 4991){
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
    while(1) delay(10);
  }
  Serial.println("Found Product 4991");

  // set not so bright!
  sspixel.setBrightness(20);
  sspixel.show();
  
  // use a pin for the built in encoder switch
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  encoder_position = ss.getEncoderPosition();

  Serial.println("Turning on interrupts");
  delay(10);
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();

    pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // Use this initializer if using a 1.8" TFT screen:
  tft.init(135, 240);      // Init ST7735S chip, black tab
  // OR use this initializer if using a 1.8" TFT screen with offset such as WaveShare:
  // tft.initR(INITR_GREENTAB);   // Init ST7735S chip, green tab
  // See more options in Adafruit GFX library documentation

  // Optionally, rotate display
  tft.setRotation(3); // See Adafruit GFX library documentation for details

  // Menu init, setup and draw
  menu.init();
  setupMenu();
  menu.drawMenu();
}

void setupMenu() {
  // Add menu items to Settings menu page
  menuPageSettings.addMenuItem(menuItemInterval);
  menuPageSettings.addMenuItem(menuItemLabel);

  // Add menu items to Main Menu page
  menuPageMain.addMenuItem(menuItemMainSettings);
  menuPageMain.addMenuItem(menuItemDelayButton1);
  menuPageMain.addMenuItem(menuItemDelayButton2);

  // Specify parent menu page for the Settings menu page
  menuPageSettings.setParentMenuPage(menuPageMain);
  
  // Add Main Menu page to menu and set it as current
  menu.setMenuPageCurrent(menuPageMain);
}

bool last_button = false;

void loop() {
  if (menu.readyForKey()) {
    bool button = !ss.digitalRead(SS_SWITCH);
    if (button != last_button) {
      if (button) menu.registerKeyPress(GEM_KEY_OK);
      last_button = button;
    }

    int32_t new_position = ss.getEncoderPosition();
    // did we move arounde?
    if (encoder_position != new_position) {
      if (new_position - encoder_position < 0) {
        menu.registerKeyPress(GEM_KEY_UP);
      } else {
        menu.registerKeyPress(GEM_KEY_DOWN);
      }
      encoder_position = new_position;      // and save for next round
    }
  }
  delay(50);

  // // If menu is ready to accept button press...
  // if (menu.readyForKey()) {
  //   // ...detect key press using KeyDetector library
  //   myKeyDetector.detect();
  //   // Pass pressed button to menu
  //   // (pressed button ID is stored in trigger property of KeyDetector object)
  //   menu.registerKeyPress(myKeyDetector.trigger);
  // }
}

// ---

// Validation routine of interval variable
void validateInterval() {
  // Check if interval variable is within allowable range (i.e. >= 50)
  if (interval < 50) {
    interval = 50;
  }
  // Print interval variable to Serial
  Serial.print("Interval set: ");
  Serial.println(interval);
}

// --- Common Blink routines

// Clear screen and print label at the center
void printLabel() {
  tft.setTextColor(0xFFFF);
  tft.fillScreen(0x0000);
  byte x = tft.width() / 2 - strlen(label) * 3;
  byte y = tft.height() / 2 - 4;
  tft.setCursor(x, y);
  tft.print(label);
}

// Toggle label on screen
void toggleLabel() {
  labelOn = !labelOn;
  if (labelOn) {
    printLabel();
  } else {
    tft.fillScreen(0x0000);
  }
}

// --- Delay based Blink context routines

// Setup context for the delay based Blink routine
void blinkDelay() {
  menu.context.loop = blinkDelayContextLoop;
  menu.context.enter = blinkDelayContextEnter;
  menu.context.exit = blinkDelayContextExit;
  menu.context.enter();
}

// Invoked once when the button is pressed
void blinkDelayContextEnter() {
  Serial.println("Delay based Blink is in progress");
}

// Invoked every loop iteration
void blinkDelayContextLoop() {
  // Blink the label on the screen.
  // Delay based Blink makes it harder to exit the loop
  // due to the blocking nature of the delay() function - carefully match timing of
  // exit key presses with the blink cycles; millis based blink has no such restriction
  toggleLabel();
  delay(interval);
}

// Invoked once when the GEM_KEY_CANCEL key is pressed
void blinkDelayContextExit() {
  // Reset variables
  labelOn = false;
  
  // Draw menu back on screen and clear context
  menu.reInit();
  menu.drawMenu();
  menu.clearContext();

  Serial.println("Exit delay based Blink");
}

// --- Millis based Blink context routines

// Setup context for the millis based Blink routine
void blinkMillis() {
  menu.context.loop = blinkMillisContextLoop;
  menu.context.enter = blinkMillisContextEnter;
  menu.context.exit = blinkMillisContextExit;
  menu.context.allowExit = false; // Setting to false will require manual exit from the loop
  menu.context.enter();
}

// Invoked once when the button is pressed
void blinkMillisContextEnter() {
  Serial.println("Millis based Blink is in progress");
}

// Invoked every loop iteration
void blinkMillisContextLoop() {
  // // Detect key press manually using KeyDetector library
  // myKeyDetector.detect();
  // if (myKeyDetector.trigger == GEM_KEY_CANCEL) {
  //   // Exit Blink routine if GEM_KEY_CANCEL key was pressed
  //   menu.context.exit();
  // } else {
  //   // Test millis timer and toggle label accordingly.
  //   // Program flow is not paused and key press allows to exit Blink routine immediately
  //   unsigned long currentMillis = millis();
  //   if (currentMillis - previousMillis >= interval) {
  //     previousMillis = currentMillis;
  //     toggleLabel();
  //   }
  // }
}

// Invoked once when the GEM_KEY_CANCEL key is pressed
void blinkMillisContextExit() {
  // Reset variables
  previousMillis = 0;
  labelOn = false;
  
  // Draw menu back on screen and clear context
  menu.reInit();
  menu.drawMenu();
  menu.clearContext();

  Serial.println("Exit millis based Blink");
}
