/*  Creative Engineering with Mark Rober
 *    Final submission for Project #3 
 *   
 *    PROJECT 3B
 *    BEST - Bicycle Etiquette made SmarT
 *   
 *   
 *  USE AT YOUR OWN RISK
 *  
 *  This code comes with no warrantees. 
 *  It is probably not fit for use. 
 *  There’s probably a better way to do it. 
 *  You accept all consequences of downloading and/or running this code. 
 *  Seriously, think what you’re doing. Use at your own risk. 
 *  Don’t blame me if your house burns down.
 *  
*/

// uncomment the following line to enable Serial prints

//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print (x)
#else
  #define DEBUG_PRINT(x)
#endif
#ifdef DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
  #define DEBUG_PRINTLN(x)
#endif




#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define PIXEL_PIN_LEFT        5
#define PIXEL_PIN_RIGHT        6
#define PIXEL_COUNT_LEFT 12
#define PIXEL_COUNT_RIGHT 12
Adafruit_NeoPixel strip_right (PIXEL_COUNT_RIGHT, PIXEL_PIN_RIGHT, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel strip (PIXEL_COUNT_RIGHT, PIXEL_PIN_RIGHT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_left (PIXEL_COUNT_LEFT, PIXEL_PIN_LEFT, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 50 // Time (in milliseconds) to pause between pixels



// uint32 colour is defined as:
//  red*256*256 + green*256 + blue
#define COLOUR_RED    16711680
#define COLOUR_BLACK  0
#define COLOUR_GREEN  65280
#define COLOUR_BLUE   255
#define COLOUR_YELLOW 16776960
#define COLOUR_ORANGE 16753920


// timer for the heartbeat signal (in multiples of TIMER2 cycles). 
#define BLINKINGTIMER 32
#define BLINK_DURATION 4
bool timeToBlink = false; 


#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpuLeft;
MPU6050 mpuRight;


#define MPURIGHT  0x68
#define MPULEFT   0x69

boolean leftEnabled = false;
boolean rightEnabled = false;

boolean leftAnimation = false;
boolean rightAnimation = false;

boolean isthisthefirstrun = true;
boolean heartBeatToggle = false;

// size of the moving average window
#define MOVINGAVERAGE  16

// how many cycles your hand needs to be up to activate the turning animation
#define ARM_UP_COUNTER  4


// number of animations after you put your hand down
// this numbers requires more experimental validation
#define NUM_ANIMATIONS  10


// experimentally determined thresholds
// a more sophisticate algorithm is needed but for the time being these seems to work well enough. 
#define ARM_UP_ROLL_THRESHOLD_LEFT 15
#define ARM_UP_ROLL_THRESHOLD_RIGHT -15



unsigned char animationCounter = 0;


// vectors used to store the last MOVINGAVERAGE samples
float maRollRight[MOVINGAVERAGE];
float maRollLeft[MOVINGAVERAGE];



//storage variables
boolean toggle0 = 0;
boolean toggle1 = 0;
boolean toggle2 = 0;
boolean newDataAvailable = false;

boolean timeToPlot = false;
boolean stripBlink = false;



boolean isLeftArmUp = false;
unsigned int leftArmUpCounter = 0;

boolean isRightArmUp = false;
unsigned int rightArmUpCounter = 0;

// Timers
unsigned long timer = 0;
unsigned long blinkingCounter = 0;
float timeStep = 0.01;


// Pitch, Roll and Yaw values
float pitchR = 0;
float rollR = 0;
float yawR = 0;
float pitchL = 0;
float rollL = 0;
float yawL = 0;

Vector normLeft;
Vector normRight;
Vector normAccelLeft;
Vector normAccelRight;

unsigned long counter = 0;



void setup() {

  // Serial output
  Serial.begin(115200);

  DEBUG_PRINTLN("LED strip initialisation");
  setupLEDStrip(&strip_right);
  setupLEDStrip(&strip_left);

  DEBUG_PRINTLN("MPU initialisation");
  leftEnabled = setupMPU(&mpuLeft, MPULEFT);
  rightEnabled = setupMPU(&mpuRight, MPURIGHT);


  DEBUG_PRINTLN("Timer initialisation");
  setupTimer();


  DEBUG_PRINTLN("Output pins");

  //set pins as outputs
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);

  DEBUG_PRINTLN("Ready to go!");

}//end setup




void loop() {
  // acquire the data whenever there is a new one available
  if (newDataAvailable == true) {
    newDataAvailable = false;


    // read the new data every 10msec (timer2 /10 = 100Hz)
    if (leftEnabled) {
      normLeft = mpuLeft.readNormalizeGyro();
      normAccelLeft = mpuLeft.readNormalizeAccel();
    }

    if (rightEnabled) {
      normRight = mpuRight.readNormalizeGyro();
      normAccelRight = mpuRight.readNormalizeAccel();
    }

    // calculate the roll (angle of the arm on the sagittal plane) and store it for later calculations
    // not the best approach, but it works. 
    maRollRight [counter] = (atan2(normAccelRight.YAxis, normAccelRight.ZAxis) * 180.0) / M_PI;;
    maRollLeft [counter] = (atan2(normAccelLeft.YAxis, normAccelLeft.ZAxis) * 180.0) / M_PI;;

    counter = (counter + 1) % MOVINGAVERAGE;

    // plot only every MOVINGAVERAGE new data
    if (!counter) {
      timeToPlot = true;
    }
  
  }

  if (timeToPlot) {
    timeToPlot = false;


    // Print the stored data on the serial port for debugging. 
    for (int i = 0; i < MOVINGAVERAGE; i++) {
      DEBUG_PRINT(maRollLeft[i]);
      DEBUG_PRINT("\t");
    }
    for (int i = 0; i < MOVINGAVERAGE; i++) {
      DEBUG_PRINT(maRollRight[i]);
      DEBUG_PRINT("\t");
    }
    DEBUG_PRINT("\n");

    /*  This section checks which arm is up.
         Not the best code ever, but somehow it works.
         Good enough for now
    */

    float tempArmAngleLeft = movingAverage(maRollLeft);
    float tempArmAngleRight = movingAverage(maRollRight);

    if (tempArmAngleLeft < ARM_UP_ROLL_THRESHOLD_LEFT) {
      isRightArmUp = false;
      isLeftArmUp = true;
      leftArmUpCounter++;
      rightArmUpCounter = 0;
    } else if (tempArmAngleRight > ARM_UP_ROLL_THRESHOLD_RIGHT) {
      isRightArmUp = true;
      isLeftArmUp = false;
      rightArmUpCounter++;
      leftArmUpCounter = 0;
    }
    else {
      isRightArmUp = false;
      isLeftArmUp = false;
      leftArmUpCounter = 0;
      rightArmUpCounter = 0;
    }


    /*  If one arm is up, it starts the direction indication

    */

    if (leftArmUpCounter > ARM_UP_COUNTER) {
      leftAnimation = true;
      rightAnimation = false;
      animationCounter = 0;
    }

    if (rightArmUpCounter > ARM_UP_COUNTER) {
      rightAnimation = true;
      leftAnimation = false;
      animationCounter = 0;
    }

  } // timeToPlot

  // LED animations are plot here.
  // note: each animation actually stops everything else. 

  if ((animationCounter < NUM_ANIMATIONS) && leftAnimation) {
    // this stops everything else, but at least it's only once per cycle...
    animateStrip(&strip_left, COLOUR_ORANGE, DELAYVAL);
    if (!isLeftArmUp) {
      animationCounter++;
    }
  }

  if ((animationCounter < NUM_ANIMATIONS) && rightAnimation) {
    // this stops everything else, but at least it's only once per cycle...
    animateStrip(&strip_right, COLOUR_ORANGE, DELAYVAL);
    if (!isRightArmUp) {
      animationCounter++;
    }
  }

  
  // clears everything up after displaying NUM_ANIMATIONS 
  if (animationCounter == NUM_ANIMATIONS) {
    animationCounter = 0;
    leftAnimation = false;
    rightAnimation = false;
    strip_left.clear(); // Set all pixel colors to 'off'
    strip_left.show();
    strip_right.clear(); // Set all pixel colors to 'off'
    strip_right.show();
  }

  // DEBUG_PRINTLN(blinkingCounter);
  if (timeToBlink && !leftAnimation && !rightAnimation) {
    // for future use
    // heartBeatToggle = !heartBeatToggle;
    /*    if (heartBeatToggle) {
          blinkStrip(&strip_right, COLOUR_BLUE, 0);
        }
        else {
          blinkStrip(&strip_left, COLOUR_GREEN, 0);
        }
    */
    blinkStrip(&strip_right, COLOUR_RED, BLINK_DURATION);
    blinkStrip(&strip_left, COLOUR_RED, BLINK_DURATION);
    
  }



} // loop ()






/* calculate the average of the input vector
    useful to remove some noise in the measurements
*/
float movingAverage (float* inputData) {
  float temp = 0;
  for (int i = 0; i < MOVINGAVERAGE; i++) {
    temp = temp + inputData[i];
  }
  return temp / MOVINGAVERAGE;
}




/* Blinking direction lights
    The animation could be better, but for the time being it should be fine.
*/
bool blinkLED = false;

void animateStrip (Adafruit_NeoPixel* strip, uint32_t color, int wait) {
  blinkLED = !blinkLED;
  if (blinkLED ) {
    symmetricalPartialFill (strip, color, wait);
    delay(wait * 10);
  } else {
    symmetricalPartialFill (strip, COLOUR_BLACK, wait);
  }
}

void animateStripReverse (Adafruit_NeoPixel* strip, uint32_t color, int wait) {
  blinkLED = !blinkLED;
  if (blinkLED ) {
    symmetricalFillReverse (strip, color, wait);
    delay(wait * 10);
  } else {
    symmetricalFillReverse (strip, COLOUR_BLACK, wait);
  }
}

void symmetricalFill (Adafruit_NeoPixel* strip, uint32_t color, int wait) {
  for (int i = 0; i < (strip->numPixels() / 2); i++) { // For each pixel in strip->..
    strip->setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip->setPixelColor(strip->numPixels() - i - 1, color);     //  Set pixel's color (in RAM)
    strip->show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void symmetricalFillReverse (Adafruit_NeoPixel* strip, uint32_t color, int wait) {
  for (int i = strip->numPixels() / 2; i >= 0; i--) { // For each pixel in strip->..
    strip->setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip->setPixelColor(strip->numPixels() - i - 1, color);     //  Set pixel's color (in RAM)
    strip->show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}



// numbver of LEDs to be active at all times
int nNumeroLed = 3;

void symmetricalPartialFill (Adafruit_NeoPixel* strip, uint32_t color, int wait) {
  for (int i = 0; i < (strip->numPixels() / 2); i++) { // For each pixel in strip->..
    strip->setPixelColor(i - 1, COLOUR_BLACK);       //  Set pixel's color (in RAM)
    strip->setPixelColor(strip->numPixels() - i, COLOUR_BLACK);       //  Set pixel's color (in RAM)

    for (int j = 0; j < nNumeroLed; j++) {
      strip->setPixelColor(i + j, color);       //  Set pixel's color (in RAM)
      strip->setPixelColor(strip->numPixels() - i - j - 1, color);   //  Set pixel's color (in RAM)
      strip->show();                          //  Update strip to match
    }
    delay(wait);                           //  Pause for a moment
  }
}

void symmetricalPartialFillReverse (Adafruit_NeoPixel* strip, uint32_t color, int wait) {
  for (int i = strip->numPixels() / 2; i >= 0; i--) { // For each pixel in strip->..
    strip->setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip->setPixelColor(strip->numPixels() - i - 1, color);     //  Set pixel's color (in RAM)
    strip->show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}



void blinkStrip(Adafruit_NeoPixel* strip, uint32_t color, int wait) {
  for (int i = 0; i < strip->numPixels(); i++) { // For each pixel in strip->..
    strip->setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip->show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
  for (int i = 0; i < strip->numPixels(); i++) { // For each pixel in strip->..
    strip->setPixelColor(i, 0);         //  Set pixel's color (in RAM)
    strip->show();                          //  Update strip to match
  }

}



void colorWipe(Adafruit_NeoPixel* strip, uint32_t color, int wait) {
  for (int i = 0; i < strip->numPixels(); i++) { // For each pixel in strip->..
    strip->setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip->show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}




ISR(TIMER0_COMPA_vect) {
  // void for now

}


unsigned long t2counter = 0;
unsigned long t1counter = 0;
unsigned long t0counter = 0;



ISR(TIMER1_COMPA_vect) {
  // Timer 1 : 1 second
  //DEBUG_PRINTLN("Time to plot");
  // timeToPlot = true;

    // heartbeat to confirm everything is working 
    // doubles up as alert for drivers coming from behind
//    blinkingCounter = (blinkingCounter + 1) % BLINKINGTIMER;
  if ((t1counter++ % BLINK_DURATION) == 0)
  {
    timeToBlink = true; 

  }
  else {
        timeToBlink = false; 
    }

}

ISR(TIMER2_COMPA_vect) {
  //Timer 2 at 1kHz
  // data acquisition at 100 Hz
  // -> division by 10 of the counter
  if ((t2counter++ % 10) == 0)
  {
    //DEBUG_PRINTLN(millis());
    //DEBUG_PRINTLN(t2counter);
    newDataAvailable = true;
  }
}




void setupTimer() {
  // calculations from http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  
  // TIMER 1 for interrupt frequency 10 Hz:
  cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 10 Hz increments
  OCR1A = 24999; // = 16000000 / (64 * 10) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Clear registers
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  // 1000 Hz (16000000/((124+1)*128))
  OCR2A = 124;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS20);
  TIMSK2 |= (1 << OCIE2A);
  sei();
}



// all DEBUG_PRINT() instructions could be removed without problems. 
boolean setupMPU (MPU6050* mpu, int MPU_address) {
  // Initialize MPU6050
  unsigned int counter = 0;
  //DEBUG_PRINTLN (MPU_address, HEX);
  while (!mpu->begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, MPU_address))
  {
    DEBUG_PRINT(counter++);
    DEBUG_PRINT(": Could not find a valid MPU6050 sensor at 0x");
    DEBUG_PRINT (String(MPU_address, HEX));
    DEBUG_PRINTLN(", check wiring!");
    delay(500);
  }
  DEBUG_PRINTLN("Calibration!");

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu->calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu->setThreshold(3);

  DEBUG_PRINT("MPU6050 sensor at 0x");
  DEBUG_PRINT (String(MPU_address, HEX));
  DEBUG_PRINTLN(" ready!");
  return true;
}



boolean setupLEDStrip(Adafruit_NeoPixel* strip) {
  // TODO: accept different strips as input
  // useful when there will be 2
  strip->begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  // quick flash to confirm that the LEDs are connected and working
  for (int i = 0; i < 2; i++) {
    colorWipe(strip, stripColor(  0,   5,   0), 0);
    delay(100);
    colorWipe(strip, COLOUR_BLACK, 0);
    delay(100);
  }

  strip->clear(); // Set all pixel colors to 'off'
  strip->show();
  return true;
}




uint32_t stripColor (char red, char green, char blue) {
  return ((uint32_t)red << 16) | ((uint32_t)green <<  8) | blue;
}
