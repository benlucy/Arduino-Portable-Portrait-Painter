
//This code originally created by Indrek Luuk to display OV7670 video on a TFT screen.
//Modified by @SouravSh25 on 22.09.2020 to include a readGRAM  function, to read pixel data from the screen.
//Modified by Ben Lucy, to control a portable portait painting machine 18/12/2020.
//Code in this tab written by Ben Lucy, very minor edits have been made to the other tabs of Indrek's code e.g. edited variables to mirror the image.

// change setup.h to switch between buffered and pixel-by-pixel processing
#include "setup.h"
#include "Adafruit_ST7735_mod.h"
#include "readGRAM.h"
extern Adafruit_ST7735_mod tft;

#include <AccelStepper.h>
#include <Servo.h>


//declare all arduino pins inputs outputs

//TMC2208 stepper motor driver pins
#define RightStepperSTEP 40
#define RightStepperDIR 36
#define RightStepperENABLE 39

#define LeftStepperSTEP 42
#define LeftStepperDIR 41
#define LeftStepperENABLE 43

//limit switch inputs
#define Xlim 18
#define Ylim 19

//buttons
#define CalibrateButton 37
#define PaintButton 34
#define CameraButton 35

//other
#define ServoSignalPin 3
#define BatteryMonitor A0
#define OrientationSensor 30
#define BackLight 32


//define global variables
#define ZeroingSpeed 600
#define ServoMinHeight 158
#define ServoMaxHeight 30
#define ServoStartPosition 94
#define MinInterval 210 //(large = lower max speed. Too fast affects acceleration (crudely coded acc), causing skipped steps)
#define IntervalChange 100 //larger = faster acceleration which may cause skipped steps
int StepsTotal;
unsigned int CurrentTime;
unsigned int SavedTime;
int interval = MinInterval;
int TargetInterval = interval;
unsigned long previousMicros = 0;
byte StepPins = 0;
int MinContactHeight;
int MaxBrushPress = 22;
float ServoPos = 90;
int RowCounter = 0;
bool RunCamera = true;
float BrushArray[165];
float PixelValueFrequency[256];
float PixelValueCumulativeFrequency[256];
float PixelLookUpTable[256];
extern uint32_t GRAM_buffer[165]; //GRAM-buffer is declared in another tab, but I need it to have global scope to work here. 'Extern' makes it a data declaration, not a data definition.

//create instances for the AccelStepper class & Servo class:
AccelStepper LeftStepper = AccelStepper(1, LeftStepperSTEP, LeftStepperDIR);
AccelStepper RightStepper = AccelStepper(1, RightStepperSTEP, RightStepperDIR);
Servo BrushServo;

//conversion for brush change (degrees) to stepper interval (us) to travel at a speed that allows brush to change position within 1mm travel (1 pixel). Calculated via: 1000000(convert units to us)/ 25(no.steps)*300(deg/sec servo speed)*brush change(deg). Or: 1000000/(25*300*DegChange). Or: 66.7* DegChange

//if brush changes by 0 degree between one pixel and the next, stepper interval should be MinIntervalus (max speed). 3deg = 200us.
//interval is time between step pin changing state, so one step takes 2 intervals
//(1deg/2deg/3deg...etc
//{MinInterval, MinInterval, MinInterval, 200, 267, 333, 400, 467, 533, 600, 667, 733, 800, 867, 933, 1000, 1067, 1135, 1200, 1267, 1353, 1400, 1467};


void setup() {

  BrushServo.attach(ServoSignalPin);
  BrushServo.write(ServoPos);
  LeftStepper.setMaxSpeed(4000); //15000
  LeftStepper.setAcceleration(100000);
  BrushServo.attach(ServoSignalPin);
  BrushServo.write(ServoPos);
  RightStepper.setMaxSpeed(4000);
  RightStepper.setAcceleration(100000); //250000

  pinMode(BatteryMonitor, INPUT);
  pinMode(OrientationSensor, INPUT);


  pinMode(Xlim, INPUT_PULLUP);
  pinMode(Ylim, INPUT_PULLUP);

  pinMode(CalibrateButton, INPUT_PULLUP);
  pinMode(PaintButton, INPUT_PULLUP);
  pinMode(CameraButton, INPUT_PULLUP);

  pinMode(RightStepperENABLE, OUTPUT);
  pinMode(LeftStepperENABLE, OUTPUT);

  digitalWrite(LeftStepperENABLE, 1);
  digitalWrite(RightStepperENABLE, 1);
  
  BrushServo.detach();
  initializeScreenAndCamera();
  //  Serial.begin(9600);

  TIMSK0 = 0;



  //display battery voltage on start-up
  //fill screen to show battery charge condition
  digitalWrite(BackLight, 1);
  switch (analogRead(BatteryMonitor)) {
    case 0 ... 749:                       // <11.0V
      tft.fillScreen(ST7735_RED);
      break;
    case 750 ... 790:                     // 11.0 - 11.6V
      tft.fillScreen(ST7735_YELLOW);
      break;
    case 791 ... 1023:                    // >11.6V
      tft.fillScreen(ST7735_GREEN);
      break;
  }
  TIMSK0 = 1;
  delay(1000);
  tft.fillScreen(ST7735_BLACK);
  TIMSK0 = 0;

}

void loop() {

  tft.setRotation(2); //sets the screen the right way up. The values in Indrek's code have been edited so that the image shown is mirrored.

  //If CameraButton is pressed, start or stop the camera.
  if (RunCamera == true) processFrame(); // runs camera

  if (digitalRead(CameraButton) == LOW) {
    if (RunCamera == true) {
      RunCamera = false;                        //freezes camera/screen
      TIMSK0 = 1;                               //enable timer0 which enables delay() function (and millis function for elsewhere in the code).
      delay(500);                               //button debounce / long press prevention
    }
    else {
      RunCamera = true;                         //resumes camera
      delay(500);                               //button debounce / long press prevention
      TIMSK0 = 0;                               //disables timer0 which removes artifacts from the screen
    }
  }



  //If calibrate button is pressed:
  //Axes are zeroed.
  //Gantry starts drawing lines while the brush moves lower and lower.
  //When the brush makes contact with the page and starts drawing a line, the user should press calibrate again.
  //The system will save the MinimumContactHeight of the servo (servo position when calibrate was pressed 2nd time).
  //The system will stop drawing lines and re-zero the axes.

  if (digitalRead(CalibrateButton) == LOW) {
    digitalWrite(LeftStepperENABLE, 0);
    digitalWrite(RightStepperENABLE, 0);
    RunCamera = false;
    TIMSK0 = 1;
    BrushServo.attach(ServoSignalPin);
    BrushServo.write(ServoPos - 25);
    ZeroAxes();
    CalibrateRoutine();
    BrushServo.detach();
    digitalWrite(LeftStepperENABLE, 1);
    digitalWrite(RightStepperENABLE, 1);
  }


  //If paint button is pressed:
  //Axes are zeroed
  //Brush moves up to start position
  // (a) Image processing happens
  // (b) 128 pixels are read from the screen
  // (c) 128 pixels are painted and the brush moves to the next row position
  // a & b repeat for 80 rows
  //the brush is lifted
  //the axes are zeroed.


  if (digitalRead(PaintButton) == LOW) {  //paint the image currently frozen on the screen.
    digitalWrite(LeftStepperENABLE, 0);
    digitalWrite(RightStepperENABLE, 0);

    RunCamera = false;
    TIMSK0 = 1;
    BrushServo.attach(ServoSignalPin);

    ZeroAxes();

    //move up 1375 steps (55mm)
    LeftStepper.move(1375);
    RightStepper.move(-1375);
    while (LeftStepper.currentPosition() != 1375) {
      LeftStepper.run();
      RightStepper.run();
    }

    ImageProcessing();

    RowCounter = 3;  //first few rows of pixels might be nothing? Maybe they're not actual pixels and just exist in the screen register?? Maybe they are legit pixels but I can't bother to upload and verify.
    while (RowCounter < 160) {

      //read the screen GRAM
      initiate_readGRAM();
      readGRAM_Row(GRAM_buffer, RowCounter);


      //converts the pixel array into brush positions. //Using data-type 'byte' for PixelLookUpTable might shorten the pause before painting each line.
      for (int i = 0; i < 128; i++) {
        GRAM_buffer[i] = GRAM_buffer[i] << 16;                                    //GRAM_buffer contains 32bit numbers. The first 8x bits are zero. The next 3 bytes are similar (because RGB/greyscale). G has the most info because RGB565, so use G byte. Outputs number from 0-255 after <<16 & >>24.
        GRAM_buffer[i] = GRAM_buffer[i] >> 24;
        GRAM_buffer[i] = PixelLookUpTable[GRAM_buffer[i]]; //Am I getting away with putting float into long here??  ??  //Revises the pixels according to Histogram Equalisation. E.g. the 5th pixel has a value 106 (GRAM_buffer[5] == 106). The 106th element of the PixelLookUpTable is 122, so the fifth pixel is updated with this value (GRAM_buffer[5]=122).
        BrushArray[i] = (float)GRAM_buffer[i];                                    //cast to float array
        BrushArray[i] = MinContactHeight + MaxBrushPress - (BrushArray[i] * (MaxBrushPress / 255.0));    //Scales BrushArray into servo-range friendly values. Weird formula is because BrushArray seems to be inverted.
      }

      Paint128Pixels();
      ReadyForNextRow();
      RowCounter = RowCounter + 2;
    }
    BrushServo.write(ServoPos - 35);
    delay(250);
    ZeroAxes();
    BrushServo.detach();
    digitalWrite(LeftStepperENABLE, 1);
    digitalWrite(RightStepperENABLE, 1);
    initializeScreenAndCamera();
  }
}

//function to zero axes at the bottom left corner
void ZeroAxes() {

  while (digitalRead(Xlim) == HIGH) {
    LeftStepper.setSpeed(-1 * ZeroingSpeed);
    RightStepper.setSpeed(-1 * ZeroingSpeed);
    LeftStepper.runSpeed();
    RightStepper.runSpeed();
  }

  while (digitalRead(Ylim) == HIGH) {
    LeftStepper.setSpeed(-1 * ZeroingSpeed);
    RightStepper.setSpeed(ZeroingSpeed);
    LeftStepper.runSpeed();
    RightStepper.runSpeed();
  }

  LeftStepper.setCurrentPosition(0);
  RightStepper.setCurrentPosition(0);

  //moves 4mm right and 4mm up.
  while (LeftStepper.currentPosition() != 200)
  {
    LeftStepper.setSpeed(ZeroingSpeed);
    LeftStepper.runSpeed();
  }
  LeftStepper.setCurrentPosition(0);
  RightStepper.setCurrentPosition(0);

  //move the servomotor down a little to stop it making noise
  BrushServo.write(ServoPos + 1);
  delay(200);
  BrushServo.write(ServoPos + 1);
  delay(200);
  BrushServo.write(ServoPos + 1);
}

void CalibrateRoutine() {

  //run the stepper motors in an 80 line pattern
  int LinesDrawn = 0;
  int CalibrateState = 1;
  while (LinesDrawn != 80 && CalibrateState == 1) {

    BrushServo.write(ServoPos);

    //move right 128mm (3200 steps)
    LeftStepper.move(3200);
    RightStepper.move(3200);
    while (LeftStepper.distanceToGo() != 0 && digitalRead(CalibrateButton) == HIGH) {
      LeftStepper.run();
      RightStepper.run();
    }

    BrushServo.write(ServoPos - 35);

    //move up 2mm (50steps)
    LeftStepper.move(50);
    RightStepper.move(-50);
    while (LeftStepper.distanceToGo() != 0 && digitalRead(CalibrateButton) == HIGH) {
      LeftStepper.run();
      RightStepper.run();
    }

    //move left 130mm (3200)
    LeftStepper.move(-3200);
    RightStepper.move(-3200);
    while (LeftStepper.distanceToGo() != 0 && digitalRead(CalibrateButton) == HIGH) {
      LeftStepper.run();
      RightStepper.run();
    }

    if (digitalRead(CalibrateButton) == LOW) {
      CalibrateState = 2;
      MinContactHeight = ServoPos;
      ServoPos = 90;
      BrushServo.write(ServoPos - 25);
      delay(250);
      ZeroAxes();
    }

    LinesDrawn = LinesDrawn + 1;
    ServoPos = ServoPos + 1;

  }
}

void Paint128Pixels() {

  digitalWrite(LeftStepperDIR, 1);
  digitalWrite(RightStepperDIR, 1);

  StepsTotal = 0;
  previousMicros = 0;
  char CurrentPixel = 0;
  char PreviousPixel = 0;
  char BrushChange;
  char PixelStepsTotal = 0;
  long microsAfter;
  long microsBefore;

  while (StepsTotal < 6400) {                                             //64000 = half steps. 25 full steps is 1mm

    //************************************************
    //every pixel (50) check difference in brush position between current and next pixel in GRAM_buffer[]
    //update stepper interval variable according to brush movement distance

    if (PixelStepsTotal >= 50) {                                           //executes every pixel (50 half-steps). (Used to use modulo for this but strange things happened)
      PixelStepsTotal = 0;
      PreviousPixel = CurrentPixel;                                           //indicates which pixel we were on (0 - 127)
      CurrentPixel = CurrentPixel + 1;                                        //indicates which pixel we are currently on (0 - 127)
      BrushChange = BrushArray[PreviousPixel] - BrushArray[CurrentPixel];       //difference between current and previous brush position is the distance the brush must move (in degrees)
      BrushChange = abs(BrushChange);
      if (BrushChange >= 3) TargetInterval = 67 * BrushChange;                      //66.7 converts servo position degree change to an interval controlling the speed the steppers should be going to allow the brush to change position within the width of 1 pixel. Line ~71 comment
      else TargetInterval = MinInterval;
      BrushServo.write(BrushArray[CurrentPixel]);

      if (digitalRead(PaintButton) == LOW) {   //Press paint button again to cancel painting
        StepsTotal = 6401;                     //Exits while loop
        RowCounter = 161;                      //Exits back to loop()
        ServoPos = 90;
        BrushServo.write(ServoPos - 35);
        delay(250);
        ZeroAxes();
        BrushServo.detach();
        digitalWrite(LeftStepperENABLE, 1);
        digitalWrite(RightStepperENABLE, 1);
      }
    }

    //***********************************************
    //Crude way of including acceleration. Acceleration will be non-linear
    //Large changes in interval will cause skipped steps. This code ensures that changes will be 100 max.
    //Will mean that brush may not complete movement during 1mm pixel, but 300deg/sec is quite conservative & it's an arbitrary target anyway.

    if (interval < TargetInterval) {
      if ((TargetInterval - interval) < IntervalChange) interval = interval + (TargetInterval - interval);
      else interval = interval + IntervalChange;
    }

    if (interval > TargetInterval) {
      if ((interval - TargetInterval) < IntervalChange) interval = interval - (interval - TargetInterval);
      else interval = interval - IntervalChange;
    }
    //***********************************************

    //***********************************************
    //every 'interval' if the step pins are high turn them low and vice-versa. This constitutes half a step cycle.

    unsigned long currentMicros = micros();

    if (currentMicros - previousMicros >= interval) {

      previousMicros = currentMicros;


      if (StepPins == 0) {
        PORTL |= 0b10000000;         //makes pin 42 high (left step pin)
        PORTG |= 0b00000010;         //makes pin 40 high (right step pin)
        StepPins = 1;
      } else {
        PORTL &= 0b01111111;         //makes pin 42 low
        PORTG &= 0b11111101;         //makes pin 40 low
        StepPins = 0;
      }
      StepsTotal = StepsTotal + 1;
      PixelStepsTotal = PixelStepsTotal + 1;

    }
    //***********************************************
  }
}

void ImageProcessing() {                      //increase contrast of image according to histogram equalisation technique.

  initiate_readGRAM();
  for (byte a = 0; a < 160; a = a + 2) {      //for 80 rows of pixels on the screen,
    readGRAM_Row(GRAM_buffer, a);             //read the current row of 128 pixels (outputs GRAM_buffer which is a long array of 128 pixels (uint_32))

    for (byte b = 0; b < 128; b++) {          //convert the row of pixels so that we only keep the Green byte.
      GRAM_buffer[b] = GRAM_buffer[b] << 16;
      GRAM_buffer[b] = GRAM_buffer[b] >> 24;
    }

    for (byte c = 0; c < 128; c++) {                                                   //count the frequency of pixel values in the row, for all 80 rows.
      PixelValueFrequency[GRAM_buffer[c]] = PixelValueFrequency[GRAM_buffer[c]] + 1;   //e.g. if GRAM_buffer[c] = 42, then access the 42nd element of PixelValueFrequency, and add one to the count.
    }
  }

  for (int f = 0; f < 256; f++) {                                                      //once the frequency of all pixel values on the screen has been counted (80rows x 128 columns = 10,240 pixels), calculate cumulative frequency:
    if (f == 0 || f == 1) {    //don't want to include 100% black pixels in the calculation, as a relatively large amount of the pixels at the edges of the screen might default black and not updated by the camera code, which will skew the histogram.
      PixelValueCumulativeFrequency[f] = 0;
    }
    else {
      PixelValueCumulativeFrequency[f] = PixelValueFrequency[f] + PixelValueCumulativeFrequency[f - 1];
    }
  }

  //need to account for pixels that are 0 or 255. (eliminate from count)
  //in order to do this, we need to know the total number of pixels counted which fall between the range 1-254
  float CumulativePixels = 0.0;
  for (int d = 1; d < 252; d++) {
    CumulativePixels = CumulativePixels + PixelValueFrequency[d];
  }

  //now that the pixels for the entire image have been counted, produce a look-up table to convert the image according to the Histogram Equalisation method.
  for (int e = 1; e < 252; e++) {
    PixelLookUpTable[e] = (PixelValueCumulativeFrequency[e] / CumulativePixels) * 255;
  }
  PixelLookUpTable[0] = 0;
  PixelLookUpTable[252] = 255;
  PixelLookUpTable[253] = 255;
  PixelLookUpTable[254] = 255;
  PixelLookUpTable[255] = 255;        //because it's 565 colour compression, there are actually only 6 bits & therefore 64 levels of black-grey-white (even though readGRAM_row() gives 3x 8bits). Could have accounted for this and made the code slightly more efficient. Instead I bodged & patched.
}

void ReadyForNextRow() {
  //raises brush
  //moves brush to beginning of next row
  //lowers brush

  //raises brush:
  BrushServo.write(MinContactHeight - 25 );

  //moves brush to beginning of next row:
  //move up 2mm (50steps)
  LeftStepper.move(50);
  RightStepper.move(-50);
  while (LeftStepper.distanceToGo() != 0) {
    LeftStepper.run();
    RightStepper.run();
  }

  //move left 130mm (3200)
  LeftStepper.move(-3200);
  RightStepper.move(-3200);
  while (LeftStepper.distanceToGo() != 0) {
    LeftStepper.run();
    RightStepper.run();
  }

  //lowers brush:
  BrushServo.write(MinContactHeight);
}
