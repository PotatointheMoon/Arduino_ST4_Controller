//*****************************************************************************************************
//
// ************ EQ Mount TFT ASCOM controller (Cream Box) *************
//
// It sends the control lines to the RA, DEC and Focuser motors controller (Black Box)
// It receives the ASCOM commands from the guiding software
//
// All motor control routines should be removed from this code!
//
//*****************************************************************************************************

#include "SPI.h"
#include <Wire.h>
#include "Adafruit_GFX.h"
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>

// The display uses hardware SPI, plus #9 & #10
#define TFT_CS 10
#define TFT_DC 9

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

//#include <TFT.h>

#include <TouchScreen.h>
//#include <SWTFT.h> // Hardware-specific library
#include <EEPROM.h>
//#include "table.h"
//#include "pepe.h"

/*
#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin
*/

// The STMPE610 uses hardware SPI on the shield, and #8
#define STMPE_CS 8
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);

// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

#define MAXX    240
#define MAXY    320

#define button_margin 15

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	MEDRED  0x8800
#define	DARKRED 0x4800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREY    0x79EF

#define PUSHED     1
#define NOT_PUSHED 0

#define LEFT  0
#define RIGHT 1
#define UP    2
#define DOWN  3

#define ARROW  0
#define SQUARE 1
#define CIRCLE 2
#define PAUSE  3

#define EEPROM_flag 0
#define EEPROM_data 1

#define button_delay 200

#define small_incdec 500  //Small speed increment/decrement value
//#define min_period 650    // Smallest allowed period for fast movement of scope 
#define min_period def_interval/10    // Smallest allowed period for fast movement of scope 

#define table_length 100  //Lookup table length

#define speed_normal 1
#define speed_fast_fwd 0.1
#define speed_fast_rev 0.26  // It must be different than fwd for the code to work
#define speed_increment 0.005
#define speed_decrement 0.005

TS_Point p;

int aux_counter =0;

// set pin numbers:
//const int ledPin =  13;      // the number of the LED pin
const int FOC_FW_Pin =  2;      // Focuser FW button pin (Brown cable)
const int FOC_BW_Pin =  A5;     // Focuser BW button pin (#19) (Orange cable)
const int North_Pin =  4;       // North Button pin (Green cable)
const int South_Pin =  3;       // South Button pin (Blue cable)
const int West_Pin =  5;        // West Button pin (Purple cable)
const int East_Pin =  A4;       // East Button pin (#18) (Yellow cable)
//const int Fast_Pin =  13;       // Fast Speed Button pin (Pink cable)
const int Fast_Pin =  14;       // Fast Speed Button pin. //Change pin in PCB!!! (Pink cable)

// Guiding Commands:
//
//   - North: DEC+
//   - South: DEC-
//   - East: RA+
//   - West: RA-

// Variables will change:
int ledState = LOW;             // ledState used to set the LED
long previousMicros = 0;        // will store last time LED was updated

String RXcommand;  //Input serial command
char command [255];
String value;
char command_character;
int command_index = 0;


int rotation = 1; // Rotation direction (1 = forward, 0 = reverse)

// the follow variables is an unsigned long because the time, measured in microseconds,
// will quickly become a bigger number than can be stored in an int.

//const unsigned long def_interval = 140300;           // interval at which to blink (microseconds) (1:2.5 + 1:5 gear) corrected from 1:6 + 1:5 on 14/10/2015

//unsigned long def_interval = 149300;           // interval at which to blink (microseconds) (1:2.5 + 1:5 gear) corrected from 1:6 + 1:5 on 14/10/2015
//unsigned long def_interval = 298600;           // stepper step interval (microseconds) (just 1:5 gear with direct coupling) corrected on 26/09/2018
unsigned long def_interval = 260748;           // stepper step interval (microseconds) (just 1:5 gear with direct coupling) corrected on 26/09/2018

unsigned long interval = def_interval;           // interval variable

unsigned long tmp_interval = def_interval;           // interval variable

unsigned long ref_millis = 0;         // Reference milliseconds read at the index switch rising edge

float next_millis = 0;         // Next milliseconds period update 

unsigned long table_index = 0;        // Lookup table index

float correction_f = 0;                 // Correction factor variable
float speed_factor = speed_normal;      // Speed factor for FF, FR, inc., dec., etc.
float speed_inc_dec = speed_normal;     // Speed increment/decrement factor variable

unsigned long steps_count = 0;        // Steps counter

boolean home_previous = false;
boolean home_actual = false;
boolean switch_fast = false;
boolean PE_correction = false;

float index_speed_correction = -0.8; // Speed correction at index switch to make it visible in the PE recording
float index_speed_duration = 0.5;   // Duration of the speed correction



/*const unsigned int Steps[] = {
				B00000000,	// Bipolar Full-step
				B00001010,
				B00000110,
				B00000101,
				B00001001
//				B00001001,
//				B00000101,
//				B00000110,
//				B00001010
                              };
*/
/* const unsigned int Steps[] = {
				B00000000,	// Bipolar Half-step
				B00001000,
				B00001010,
				B00000010,
				B00000110,
				B00000100,
				B00000101,
				B00000001,
				B00001001
                              };
*/

/*
const unsigned int Steps[] = {
				B00000000,	// Bipolar Half-step
				B00001001,
				B00001000,
				B00001010,
				B00000010,
				B00000110,
				B00000100,
  				B00000101,
				B00000001
                              };
*/

const unsigned int Steps[] = {
				B00000000,	// Bipolar Half-step
				B00100100,
				B00100000,
				B00101000,
				B00001000,
				B00011000,
				B00010000,
  			B00010100,
				B00000100
                              };

unsigned int StepState = 1;

unsigned int HalfStepMode = 1;

boolean button_pusshed = false; // Variable to indicate a button is pressed

int button_border = MEDRED;
int button_body = DARKRED;
int arrow_border = MEDRED;
int arrow_body = RED;

int p_button_border = RED;
int p_button_body = MEDRED;
int p_arrow_border = RED;
int p_arrow_body = DARKRED;

typedef struct b_colours
{
   int but_border;
   int but_body;
   int ar_border;
   int ar_body;
} B_colours;

int button_high = 40;
int button_width = 60;

B_colours button_col = {button_border, button_body, arrow_border, arrow_body};
B_colours p_button_col = {p_button_border, p_button_body, p_arrow_border, p_arrow_body};

typedef struct button
{
  int xpos;
  int ypos;
  int shape;
  int dir; 
} Button;

//Button PEC_enable = {button_margin, 35, SQUARE, LEFT};
Button focus_fw = {button_margin, MAXY - button_margin - button_high*2/3, ARROW, UP};
Button focus_bw = {MAXX - button_width - button_margin, MAXY - button_margin - button_high*2/3, ARROW, DOWN};
Button north = {MAXX/2 - button_width/2, MAXY - button_margin - button_high*6, ARROW, UP};
Button south = {MAXX/2 - button_width/2,  MAXY - button_margin - button_high*2, ARROW, DOWN};
Button east = {MAXX - button_width - button_margin, MAXY - button_margin - button_high*4, ARROW, RIGHT};
Button west = {button_margin, MAXY - button_margin - button_high*4, ARROW, LEFT};
Button fast = {MAXX/2 - button_width/2, MAXY - button_margin - button_high*4, CIRCLE, RIGHT};
//Button mover_fast = {button_margin, MAXY - button_margin - button_high*3, ARROW, LEFT};
//Button load_default = {MAXX/2 - button_width/2, MAXY - button_margin - button_high*3, ARROW, UP};


//SWTFT tft;

#define BOXSIZE 30
int oldcolor, currentcolor;

void Draw_Button (int xpos, int ypos, int shape, int dir, int pushed)
{
  int b_bor, b_bod, a_bor, a_bod;
  
  if (pushed == PUSHED)
    {
      b_bor = p_button_border;
      b_bod = p_button_body;
      a_bor = p_arrow_border;
      a_bod = p_arrow_body;   
    }
  else
    {
      b_bor = button_border;
      b_bod = button_body;
      a_bor = arrow_border;
      a_bod = arrow_body;   
    }
  
  tft.drawRoundRect(xpos, ypos, button_width, button_high, 10, b_bor);
  tft.fillRoundRect(xpos + 1, ypos + 1, button_width - 2, button_high - 2, 10, b_bod);

  switch (shape)
    {
      case ARROW:
        if (dir == LEFT)
         {
           tft.drawTriangle(xpos + button_width/3, ypos + button_high*2/3, xpos + button_width*2/3, ypos + button_high*3/4, xpos + button_width*2/3, ypos + button_high*1/2, a_bor);
           tft.fillTriangle(xpos + button_width/3 + 1, ypos + button_high*2/3, xpos + button_width*2/3 - 2, ypos + button_high*3/4 + 1, xpos + button_width*2/3 - 2, ypos + button_high*1/2 - 2, a_bod);
         }
        else if (dir == RIGHT)
         {
           tft.drawTriangle(xpos + button_width*2/3, ypos + button_high*2/3, xpos + button_width/3, ypos + button_high*3/4, xpos + button_width/3, ypos + button_high*1/2, a_bor);
           tft.fillTriangle(xpos + button_width*2/3 - 2, ypos + button_high*2/3, xpos + button_width/3 + 1, ypos + button_high*3/4 + 1, xpos + button_width/3 + 1, ypos + button_high*1/2 - 2, a_bod);     
         }
        else if (dir == UP)
         {
           tft.drawTriangle(xpos + button_width/3, ypos + button_high*3/4, xpos + button_width*2/3, ypos + button_high*3/4, xpos + button_width/2, ypos + button_high*1/2, a_bor);
           tft.fillTriangle(xpos + button_width/3 + 1, ypos + button_high*3/4 - 1, xpos + button_width*2/3 - 1, ypos + button_high*3/4 - 1, xpos + button_width/2, ypos + button_high*1/2 + 1, a_bod);     
         }
        else if (dir == DOWN)
         {
           tft.drawTriangle(xpos + button_width/3, ypos + button_high*1/2, xpos + button_width*2/3, ypos + button_high*1/2, xpos + button_width/2, ypos + button_high*3/4, a_bor);
           tft.fillTriangle(xpos + button_width/3 + 1, ypos + button_high*1/2 + 1, xpos + button_width*2/3 - 1, ypos + button_high*1/2 + 1, xpos + button_width/2, ypos + button_high*3/4 - 1, a_bod);     
         }
      break;

      case PAUSE:
        if (dir == LEFT)
         {
           tft.drawTriangle(xpos + button_width/3 - 1, ypos + button_high*2/3, xpos + button_width*2/3 - 5, ypos + button_high*1/2, xpos + button_width*2/3 - 5, ypos + button_high*3/4, a_bor);
           tft.fillTriangle(xpos + button_width/3, ypos + button_high*2/3, xpos + button_width*2/3 - 6, ypos + button_high*1/2 + 1, xpos + button_width*2/3 - 6, ypos + button_high*3/4 - 2, a_bod);
           tft.drawRect(xpos + button_width*2/3 - 2, ypos + button_high*1/2, 4, button_high/3 + 1, a_bor);
           tft.fillRect(xpos + button_width*2/3 - 1, ypos + button_high*1/2 + 1, 2, button_high/3 - 1, a_bod);
         }
        else if (dir == RIGHT)
         {
           tft.drawTriangle(xpos + button_width*2/3 + 1, ypos + button_high*2/3, xpos + button_width/3 + 5, ypos + button_high*1/2, xpos + button_width/3 + 5, ypos + button_high*3/4, a_bor);
           tft.fillTriangle(xpos + button_width*2/3 - 1, ypos + button_high*2/3, xpos + button_width/3 + 6, ypos + button_high*1/2 + 1, xpos + button_width/3 + 6, ypos + button_high*3/4 - 2, a_bod);     
           tft.drawRect(xpos + button_width/3 - 1, ypos + button_high*1/2, 4, button_high*1/4 + 1, a_bor);
           tft.fillRect(xpos + button_width/3, ypos + button_high*1/2 + 1, 2, button_high*1/4 - 1, a_bod);
         }
      break;

      case SQUARE:
        tft.drawRect(xpos + button_width/3, ypos + button_high*1/2, button_width/3, button_high*1/4, a_bor);
        tft.fillRect(xpos + button_width/3 + 1, ypos + button_high*1/2 + 1, button_width/3 - 2, button_high*1/4 - 2, a_bod);
      break;

      case CIRCLE:
        tft.drawCircle(xpos + button_width/2, ypos + button_high*2/3, button_width/8, a_bor);
        tft.fillCircle(xpos + button_width/2, ypos + button_high*2/3, button_width/8 - 1, a_bod);
      break;
 
      default:
      break;
   }
}

void setup(void) {

  tmp_interval = interval;
  
  Serial.begin(57600);
//  Serial.begin(9600);
//  Serial.println(F("EQ Mount Control"));

  tft.begin();
  ts.begin();

  DDRD = B00001111;
  PORTD = B00000000;
  // initialize the LED pins:
  for (int thisPin = 0; thisPin < 4; thisPin++)
  {
    pinMode(thisPin, OUTPUT);
  } 

  if ((EEPROM.read(EEPROM_flag))!= 170)
    {
      EEPROM.put(EEPROM_data, def_interval);
      EEPROM.write(EEPROM_flag, 170);
    }
  else
    {
      EEPROM.get(EEPROM_data, def_interval);
      interval = def_interval;
    }

   Serial.println("INITIALIZED#");

//  tft.reset();  //2020/01/17
  
//  uint16_t identifier = tft.readID();

//  identifier = 0x8357;

//  Serial.print(F("LCD driver chip: "));
//  Serial.println(identifier, HEX);
    

//  tft.begin(identifier);  //2020/01/17
//  delay(100);  //2020/01/17
  tft.fillScreen(BLACK);  //2020/01/17


  tft.fillRect(0, 0, BOXSIZE*8, BOXSIZE, DARKRED);  //2020/01/17

  tft.setCursor(15,8);  //2020/01/17
  tft.setTextColor(MEDRED);  //2020/01/17
  tft.setTextSize(2);  //2020/01/17
  tft.println("EQ Mount Control");  //2020/01/17
  tft.drawCircle(MAXX-15, 15, 6, MEDRED);  //2020/01/17
/*
  tft.setCursor(25,40);
  tft.setTextColor(MEDRED);
  tft.setTextSize(2);
  tft.println("SPEED ADJUSTMENT");
*/
//2020/01/17
  tft.setCursor(MAXX/2 - 40, MAXY - button_margin - 10);
  tft.setTextColor(MEDRED);
  tft.setTextSize(2);
  tft.println("FOCUSER");

  Draw_Button (focus_fw.xpos, focus_fw.ypos, focus_fw.shape, focus_fw.dir, 0);
  tft.setCursor(focus_fw.xpos + 10, focus_fw.ypos - button_high*1/2);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("FWD");

  Draw_Button (focus_bw.xpos, focus_bw.ypos, focus_bw.shape, focus_bw.dir, 0);
  tft.setCursor(focus_bw.xpos + 10, focus_bw.ypos - button_high*1/2);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("BCK");

  tft.setCursor(north.xpos + 0, north.ypos - button_high*1/2);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("NORTH");
  Draw_Button (north.xpos, north.ypos, north.shape, north.dir, 0);
  
  tft.setCursor(south.xpos + 0, south.ypos - button_high*1/2);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("SOUTH");
  Draw_Button (south.xpos, south.ypos, south.shape, south.dir, 0);

  tft.setCursor(east.xpos + 10, east.ypos - button_high*1/2);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("EAST");
  Draw_Button (east.xpos, east.ypos, east.shape, east.dir, 0);
 
  tft.setCursor(west.xpos + 10, west.ypos - button_high*1/2);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("WEST");
  Draw_Button (west.xpos, west.ypos, west.shape, west.dir, 0);

  tft.setCursor(fast.xpos + 10, fast.ypos - button_high*1/2);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("FAST");
  Draw_Button (fast.xpos, fast.ypos, fast.shape, fast.dir, 0);
/*
  tft.setCursor(movef_slow.xpos + 10, movef_slow.ypos - button_high*2/3);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("FWD");
  Draw_Button (movef_slow.xpos, movef_slow.ypos, movef_slow.shape, movef_slow.dir, 0);

  tft.setCursor(load_default.xpos + 10, load_default.ypos - button_high*2/3);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("DEF");
  Draw_Button (load_default.xpos, load_default.ypos, load_default.shape, load_default.dir, 0);

  tft.setCursor(stop_motion.xpos + 10, stop_motion.ypos - button_high*2/3);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("STOP");
  Draw_Button (stop_motion.xpos, stop_motion.ypos, stop_motion.shape, stop_motion.dir, 0);
*/
  tft.setCursor(0,0);
  //2020/01/17
  //pinMode(13, OUTPUT);

  //pinMode(ledPin, OUTPUT);      
  pinMode(FOC_FW_Pin, OUTPUT);
  pinMode(FOC_BW_Pin, OUTPUT);
  pinMode(North_Pin, OUTPUT);
  pinMode(South_Pin, OUTPUT);
  pinMode(West_Pin, OUTPUT);
  pinMode(East_Pin, OUTPUT);
  pinMode(Fast_Pin, OUTPUT);

  digitalWrite(FOC_FW_Pin, LOW);
  digitalWrite(FOC_BW_Pin, LOW);
  digitalWrite(North_Pin, LOW);
  digitalWrite(South_Pin, LOW);
  digitalWrite(West_Pin, LOW);
  digitalWrite(East_Pin, LOW);
  digitalWrite(Fast_Pin, LOW);
 
}

#define MINPRESSURE 10
#define MAXPRESSURE 1000

void loop()
{
  uint16_t x, y;
  uint8_t z;
  unsigned long currentMicros = micros();
 
  if(currentMicros - previousMicros > interval) //use this
  {
    
/*    for(aux_counter = 0; aux_counter < table_length; aux_counter++)
      {
        Serial.print((long)correction[0][aux_counter]);
        Serial.print(",");      
        Serial.println((long)correction[1][aux_counter]);
      }
*/
   previousMicros = currentMicros;   
   
//   home_actual = digitalRead(switchPin);
//    Serial.println(interval,DEC);

//    if(home_actual == false)
//     {
//       Serial.println(interval,DEC);
//     }
//    else
//     {
 //      interval = def_interval * (1 + correction_f);
//       Serial.println(interval,DEC);
//       Serial.println(tmp_interval,DEC);
//     }

    if((!home_previous)&&(home_actual)) // Check rising edge on index switch for reseting correction table
     {
       ref_millis = millis();
       table_index = 0;
//       correction_f = correction[1][table_index];
//       next_millis = correction[0][table_index];
//       interval = def_interval * (1 + correction_f);
//       Serial.println(ref_millis,DEC);
//       Serial.println((float)next_millis*1000,0);
/*       
       Serial.print(next_millis,DEC);
       Serial.print("    ");
       Serial.print(correction_f,DEC);
       Serial.println("   --- Index ---");
*/
//       Serial.println(interval,DEC);
//       tmp_interval = def_interval;
//       tmp_interval = interval;
       if(switch_fast == true)
       {
//         correction_f = index_speed_correction;
//         next_millis = index_speed_duration;
/*
       Serial.print(next_millis,DEC);
       Serial.print("    ");
       Serial.print(correction_f,DEC);
       Serial.println("   --- Index2 ---");
*/
       }
//         def_interval = tmp_interval/1.5;
//         interval = def_interval/1.5;
//         interval = tmp_interval/1.5;
//       steps_count = 0;
/*     }
     else
      {
        if (StepState)
          if (rotation == 1)
            steps_count++;
          else
            steps_count--;
      }
*/
     }
     home_previous = home_actual; // Update and record index switch status

//     interval = def_interval - (def_interval * correction[steps_count / 48]);
 //    if (StepState)
 //      Serial.println(correction[steps_count / 48],DEC);

    // save the last time you blinked the LED 
/*
    if (StepState != 0)
      ledState = !ledState;
*/
//    PORTD = (PORTD & 00000000) | Steps[StepState];
//    PORTD = (PORTD & 11000011) | ((Steps[StepState])<<2);

    if (HalfStepMode == 1)
    {
      if (rotation == 1)
       {
        switch (StepState)
         {
//          case 0:// Remove when motor off required
  //          StepState = 1;
   //         break;
          case 1:
            StepState = 8;
            break;
          case 2:
            StepState = 1;
            break;
          case 3:
            StepState = 2;
            break;
          case 4:
            StepState = 3;
            break;
          case 5:
            StepState = 4;
            break;
          case 6:
            StepState = 5;
            break;
          case 7:
            StepState = 6;
            break;
          case 8:
            StepState = 7;
            break;
          default:
            StepState = 0;
         }
        }
       else
        {
         switch (StepState)
          {
//          case 0:// Remove when motor off required
  //          StepState = 1;
   //         break;
          case 1:
            StepState = 2;
            break;
          case 2:
            StepState = 3;
            break;
          case 3:
            StepState = 4;
            break;
          case 4:
            StepState = 5;
            break;
          case 5:
            StepState = 6;
            break;
          case 6:
            StepState = 7;
            break;
          case 7:
            StepState = 8;
            break;
          case 8:
            StepState = 1;
            break;
          default:
            StepState = 0;
         }
        }
    }
    else
    {
      if (rotation == 1)
       {
        switch (StepState)
         {
//          case 0:// Remove when motor off required
//            StepState = 2;
//            break;
          case 1:
            StepState = 7;
            break;
          case 3:
            StepState = 1;
            break;
          case 5:
            StepState = 3;
            break;
          case 7:
            StepState = 5;
            break;
          default:
            StepState = 0;
        }
       }
      else
       {
        switch (StepState)
         {
//          case 0:// Remove when motor off required
//            StepState = 2;
//            break;
          case 1:
            StepState = 3;
            break;
          case 3:
            StepState = 5;
            break;
          case 5:
            StepState = 7;
            break;
          case 7:
            StepState = 1;
            break;
          default:
            StepState = 0;
        }
       }
    }

    // set the LED with the ledState of the variable:
//    digitalWrite(ledPin, ledState);

    if (PE_correction == true)
      interval = (unsigned long)((float)def_interval * (1 + correction_f) * speed_factor * speed_inc_dec);
    else
      interval = (unsigned long)((float)def_interval * speed_factor * speed_inc_dec);
      
//    Serial.print("Period: ");
//    Serial.println(interval);

    tmp_interval = interval;
  }

  if ((millis()-ref_millis) >= (next_millis*1000)) // PE correction section (based on ms between corrections)
  {
    if (table_index < table_length-2)
    table_index++;
//    next_millis = (float)correction[0][table_index];
//    correction_f = (float)correction[1][table_index];
//             interval = def_interval * (1 + correction_f);
//             Serial.println((float)next_millis*1000,0);
/*
    Serial.print(next_millis,DEC);
    Serial.print("    ");
    Serial.println(correction_f,DEC);
*/
//             Serial.println(interval,DEC);
  }

  if (Serial.available() > 0)
    {
//      Serial.println("----------------------------");
//      Serial.println("--Received Serial Data--");
/*      home_actual = true;
      if(!home_previous&&home_actual)
       {
         Serial.println(steps_count,DEC);
       }
      home_previous = home_actual;
      steps_count = 0;
*/      
//      Serial.println("----------------------------");
      RXcommand = Serial.readStringUntil('#');
//      Serial.println("----------------------------");
      boolean validOpcode=true;
//      Serial.println("--Command Received--");
      
/*      
      Serial.println("----------------------------");
      Serial.print("Input Command: ");
      Serial.println(RXcommand);
*/
      command_index = 0;
//      tft.print(RXcommand[command_index]);
//      tft.print(RXcommand.substring(command_index, 1));
      while ((RXcommand[command_index] != " ")&&(RXcommand[command_index] != "#")&&(RXcommand[command_index] != NULL))
        {
  //    Serial.println("CCCCC");
          command[command_index] = RXcommand[command_index];
//          tft.print(RXcommand[command_index]);
          command_index++;
//         tft.print(command);
//       tft.print(" ");
        };
      command[command_index] = NULL;
      command_index++;
 //     Serial.println("--Command Extracted--");
//      tft.print(RXcommand[command_index]);

      while ((RXcommand[command_index]!=NULL)&&(RXcommand[command_index] != "#"))
        {
 //     Serial.println("CCCCC");
          value[command_index] = RXcommand[command_index];
          command_index++;
//          tft.print(value);
//          tft.print(" ");
        };
      value[command_index] = NULL;
//      Serial.println("--Value Extracted--");

//      String command = RXcommand.substring(0, 3);
//      String value = RXcommand.substring(4, 10);
//      Serial.println(command + value);
/*   //2020/01/17
      tft.setTextColor(YELLOW);
      tft.setTextSize(1);
      tft.print(command);
      tft.print(" ");
      tft.print(value);
      tft.print(" ");
*/  //2020/01/17
//      Serial.println("--Command Displayed on TFT--");
      
    //Parse RXcommand
    if((RXcommand=="CONNECT")||(RXcommand == "connect"))
     {
//      digitalWrite(ledPin, HIGH);
        tft.fillCircle(MAXX-15, 15, 6, RED);  //2020/01/17
//      resetPins();
     }
    else if ((RXcommand=="DISCONNECT")||(RXcommand=="disconnect"))
     {
//      digitalWrite(ledPin, LOW);
      tft.fillCircle(MAXX-15, 15, 6, DARKRED);  //2020/01/17
      tft.drawCircle(MAXX-15, 15, 6, MEDRED);  //2020/01/17
//      resetPins();
     }
    else if((RXcommand=="RA0")||(RXcommand=="ra0"))
     {
        EEPROM.get(EEPROM_data , def_interval);
        speed_inc_dec = speed_normal;
        speed_factor = speed_normal;
        digitalWrite(West_Pin, LOW);
        digitalWrite(East_Pin, LOW);
        Draw_Button (west.xpos, west.ypos, west.shape, west.dir, NOT_PUSHED);      
        Draw_Button (east.xpos, east.ypos, east.shape, east.dir, NOT_PUSHED);      

//        interval = def_interval;
     }
    else if((RXcommand=="RA+")||(RXcommand=="ra+"))
     {
//        EEPROM.get(EEPROM_data , def_interval);
//        def_interval = def_interval *2;
//        interval = def_interval *2;
//       speed_inc_dec = speed_inc_dec + speed_decrement*100;
        digitalWrite(West_Pin, LOW);
        digitalWrite(East_Pin, HIGH);
        Draw_Button (west.xpos, west.ypos, west.shape, west.dir, NOT_PUSHED);      
        Draw_Button (east.xpos, east.ypos, east.shape, east.dir, PUSHED);      
//        Serial.println(East_Pin);
        speed_factor = 4;
     }
    else if((RXcommand=="RA-")||(RXcommand=="ra-"))
     {
//        EEPROM.get(EEPROM_data , def_interval);
//        def_interval = def_interval /2;
//        interval = def_interval /2;
//       speed_inc_dec = speed_inc_dec - speed_decrement*100;
        speed_factor = 0.1;
        digitalWrite(West_Pin, HIGH);
        digitalWrite(East_Pin, LOW);
        Draw_Button (west.xpos, west.ypos, west.shape, west.dir, PUSHED);      
        Draw_Button (east.xpos, east.ypos, east.shape, east.dir, NOT_PUSHED);      
//        Serial.println(West_Pin);

     }
    else if((RXcommand=="DEC0")||(RXcommand=="dec0"))
     {
//      declination.reset();
        digitalWrite(North_Pin, LOW);
        digitalWrite(South_Pin, LOW);
        Draw_Button (north.xpos, north.ypos, north.shape, north.dir, NOT_PUSHED);      
        Draw_Button (south.xpos, south.ypos, south.shape, south.dir, NOT_PUSHED);      
     }
    else if((RXcommand=="DEC+")||(RXcommand=="dec+"))
     {
        digitalWrite(North_Pin, HIGH);
        Draw_Button (north.xpos, north.ypos, north.shape, north.dir, PUSHED);      
        Draw_Button (south.xpos, south.ypos, south.shape, south.dir, NOT_PUSHED);      
//        Serial.println(North_Pin);
//      declination.plus();
     }
    else if((RXcommand=="DEC-")||(RXcommand=="dec-"))
     {
        digitalWrite(South_Pin, HIGH);
        Draw_Button (north.xpos, north.ypos, north.shape, north.dir, NOT_PUSHED);      
        Draw_Button (south.xpos, south.ypos, south.shape, south.dir, PUSHED);      
//        Serial.println(South_Pin);
//      declination.minus();
     }
    else if ((RXcommand == "def")||(RXcommand == "DEF")) // Sets the default values for switching period and rotation direction
     {
        EEPROM.get(EEPROM_data , def_interval);
        speed_inc_dec = speed_normal;
        speed_factor = speed_normal;
//        Serial.println("default");
//          interval = def_interval;
          rotation = 1;
     }
    else if ((RXcommand == "per")||(RXcommand == "PER")) // Sets the stepper phases switching period
     {
          def_interval = value.toInt();
          interval = value.toInt();
     }
    else if ((RXcommand == "for")||(RXcommand == "FOR")) // Forward direction
     {
          rotation = 1;
     }
    else if ((RXcommand == "rev")||(RXcommand == "REV")) // Reverse direction
     {
          rotation = 0;
     }
    else if ((RXcommand == "f")||(RXcommand == "F")) // Small speed increment
     {
          def_interval = def_interval - small_incdec;
//          interval = interval - small_incdec;
     }
    else if ((RXcommand == "s")||(RXcommand == "S")) // Small speed decrement
     {
          def_interval = def_interval + small_incdec;
//          interval = interval + small_incdec;
     }
    else if (RXcommand == "+") // Doubles the speed
     {
          def_interval = def_interval / 2;
//          interval = interval / 2;
     }
    else if (RXcommand == "-") // Halves the speed
     {
          def_interval = def_interval * 2;
//          interval = interval * 2;
     }
    else if ((RXcommand == "i")||(RXcommand == "I")) // Increases speed by 4
     {
          def_interval = def_interval / 4;
//          interval = interval / 4;
     }
    else if ((RXcommand == "d")||(RXcommand == "D")) // Decreases speed by 4
     {
          def_interval = def_interval * 4;
//          interval = interval * 4;
     }
    else if ((RXcommand == "o")||(RXcommand == "O")) // Sets Half Step Mode
     {
          if (HalfStepMode == 0)
            def_interval = def_interval/2;
          HalfStepMode = 1;
          StepState = 1;
     }
    else if ((RXcommand == "p")||(RXcommand == "P")) // Sets Full Step Mode
     {
          if (HalfStepMode == 1)
            def_interval = def_interval*2;
          HalfStepMode = 0;
          StepState = 1;
     }

    else if ((RXcommand == "e")||(RXcommand == "E")) // Read EEPROM Flag
     {
          Serial.println(EEPROM.read(EEPROM_flag),DEC);
          EEPROM.get(EEPROM_data , tmp_interval);
          Serial.println(tmp_interval,DEC);
     }
    else if ((RXcommand == "z")||(RXcommand == "Z")) // Stop motor
     {
          StepState = 0;
     }
    else if ((RXcommand == "x")||(RXcommand == "X")) // Start motor
     {
          StepState = 1;
     }
    else if ((RXcommand == "swf")||(RXcommand == "SWF")) // Activate fast run index while switch pressed
     {
          switch_fast = true;
     }
    else if ((RXcommand == "swn")||(RXcommand == "SWN")) // Deactivate fast run index while switch pressed
     {
          switch_fast = false;
     }
    else if ((RXcommand == "pe+")||(RXcommand == "PE+")) // Activate fast run index while switch pressed
     {
          PE_correction = true;
     }
    else if ((RXcommand == "pe-")||(RXcommand == "PE-")) // Deactivate fast run index while switch pressed
     {
          PE_correction = false;
     }
    else
     {
      validOpcode=false;
     }
    tmp_interval = interval;
    if(validOpcode)
     {
      //Acknowledge valid command
      Serial.println("OK#");
//          EEPROM.get(EEPROM_data , tmp_interval);
//          Serial.println(tmp_interval,DEC);
     }
//      Serial.println("--Command Proccessed--");
/*
    Serial.print("Period: ");
    Serial.println(interval);
    Serial.print("Rotation Direction: ");
    Serial.println(rotation);
    Serial.print("Half-Step Mode: ");
    Serial.println(HalfStepMode);
*/ 
    }
/*   else
    {
      home_actual = false;
      home_previous = home_actual;
    }
*/
/////////////// TOUCHSCREEN HANDLER ///////////////////

  if (ts.touched())// & ((ts.getPoint()).z == 1))
  {
    button_pusshed = true;
    while (! ts.bufferEmpty())
    {
      ts.readData(&x, &y, &z);
//      Serial.println(z);
    }
    
  // Scale from ~0->4000 to tft.width using the calibration #'s
    x = tft.width() - (map(x, TS_MINX, TS_MAXX, tft.width(), 0));
    y = tft.height() +14 - (map(y, TS_MINY, TS_MAXY, tft.height(), 0));

    if ((x > focus_fw.xpos) & (x < focus_fw.xpos + button_width) & (y > focus_fw.ypos) & (y < focus_fw.ypos + button_high))
      {
        Draw_Button (focus_fw.xpos, focus_fw.ypos, focus_fw.shape, focus_fw.dir, PUSHED);       
        digitalWrite(FOC_FW_Pin, HIGH);
        delay(button_delay);
        if (! ts.touched())
          Draw_Button (focus_fw.xpos, focus_fw.ypos, focus_fw.shape, focus_fw.dir, NOT_PUSHED);
      }
    else if ((x > focus_bw.xpos) & (x < focus_bw.xpos + button_width) & (y > focus_bw.ypos) & (y < focus_bw.ypos + button_high))
      {
        Draw_Button (focus_bw.xpos, focus_bw.ypos, focus_bw.shape, focus_bw.dir, PUSHED);
        digitalWrite(FOC_BW_Pin, HIGH);
        delay(button_delay);
        if (! ts.touched())
          Draw_Button (focus_bw.xpos, focus_bw.ypos, focus_bw.shape, focus_bw.dir, NOT_PUSHED);      
      }
    else if ((x > north.xpos) & (x < north.xpos + button_width) & (y > north.ypos) & (y < north.ypos + button_high))
      {
        Draw_Button (north.xpos, north.ypos, north.shape, north.dir, PUSHED);
        digitalWrite(North_Pin, HIGH);
        delay(button_delay);
        if (! ts.touched())
          Draw_Button (north.xpos, north.ypos, north.shape, north.dir, NOT_PUSHED);      
      }
    else if ((x > south.xpos) & (x < south.xpos + button_width) & (y > south.ypos) & (y < south.ypos + button_high))
      {
        Draw_Button (south.xpos, south.ypos, south.shape, south.dir, PUSHED);
        digitalWrite(South_Pin, HIGH);
        delay(button_delay);
        if (! ts.touched())
          Draw_Button (south.xpos, south.ypos, south.shape, south.dir, NOT_PUSHED);      
      }
    else if ((x > east.xpos) & (x < east.xpos + button_width) & (y > east.ypos) & (y < east.ypos + button_high))
      {
        Draw_Button (east.xpos, east.ypos, east.shape, east.dir, PUSHED);
        digitalWrite(East_Pin, HIGH);
        delay(button_delay);
        if (! ts.touched())
          Draw_Button (east.xpos, east.ypos, east.shape, east.dir, NOT_PUSHED);      
      }
    else if ((x > west.xpos) & (x < west.xpos + button_width) & (y > west.ypos) & (y < west.ypos + button_high))
      {
        Draw_Button (west.xpos, west.ypos, west.shape, west.dir, PUSHED);
        digitalWrite(West_Pin, HIGH);
        delay(button_delay);
        if (! ts.touched())
          Draw_Button (west.xpos, west.ypos, west.shape, west.dir, NOT_PUSHED);      
      }
    else if ((x > fast.xpos) & (x < fast.xpos + button_width) & (y > fast.ypos) & (y < fast.ypos + button_high))
      {
        if(switch_fast == false)
        {
          switch_fast = true;
          digitalWrite(Fast_Pin, HIGH);
          Draw_Button (fast.xpos, fast.ypos, fast.shape, fast.dir, PUSHED);
          delay(button_delay * 2);
        }
        else
        {
          switch_fast = false;
          digitalWrite(Fast_Pin, LOW);
          Draw_Button (fast.xpos, fast.ypos, fast.shape, fast.dir, NOT_PUSHED); 
          delay(button_delay * 2);
        }     
      }
  }
  else if (button_pusshed == true) // Clear control lines only after releasing a button.
  {
    digitalWrite(FOC_FW_Pin, LOW);
    digitalWrite(FOC_BW_Pin, LOW);
    digitalWrite(North_Pin, LOW);
    digitalWrite(South_Pin, LOW);
    digitalWrite(West_Pin, LOW);
    digitalWrite(East_Pin, LOW);
    button_pusshed = false;
  }
    while (! ts.bufferEmpty())
  {
    TS_Point pepe = ts.getPoint();
//    Serial.println(ts.bufferSize());
  }
}
