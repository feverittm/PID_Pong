
// Arduino based debugging tool for a FIRST robot.
// Copywright FIRST Robotics Team 997 Corvallis Oregon 3/20/2013
// This code is provided free for use to any FIRST robotics team.  Feel free to modify or extend this code, but please let us
// know what you have done, and please keep a reference to our website in the code and the about menu.
// http://chsrobotics.org/

// Finally this is the start of the real code.
// Include some libraries that will be used later:
#include <LiquidCrystal.h>    // Needed to drive our LCD display
#include <Servo.h>            // For outputting PWM signals
#include <PID_v1.h>

Servo myservo1;               // create servo object to control a servo 

// Define some constants to provide an easy place to remap the I/O or adjust some other variables.
// This should be handy if anyone develops custom hardware and wants to use this code.
const int PanelKnob = 0;      // analog pin used to read a potentiometer input that controls PWM
const int AnalogIO1 = 1;      // analog pin used to test various analog sensors, etc
const int NextButton = 13;    //  
const int SelectButton = 3;   //  
const int OptionButton1 = 0;  // 
const int OptionButton2 = 1;  // 
const int PWMOut1 = 3;        // 
const int Debounce = 500;     // The number of miliseconds to wait to prevent skipping multiple menus

// Set up some variables to use for passing information to and from interrupt service routines
// These need to be of type volatile to insure they can pass data in and out of the service routine.
volatile int pulsewidth = 0;
volatile int time_between_pulses = 0;
volatile boolean rotationforward = false;
volatile boolean rotationlast = true;


// initialize the LCD library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// And define some variables needed for this particular LCD shield
int ButtonVoltage = 0;
int ButtonPressed = 0;
int Backlight = 5;
int fadeValue = 255;
int setpoint  = 0;

int first_pass = 1;
int mode = 0; // Mode 0 = Manual, 1 = Autonomous

// This is the tester setup function that runs at powerup.
// Any setup code that only needs to run once goes here.  This includes the splash screen which can be customized
// to show other team names or numbers.
void setup() {
  lcd.begin(16, 2);              // set LCD library for the number of cols and rows on display. 
  lcd.setCursor (0,0);           // set the cursor to column 0, line 0
  lcd.print("PID Pong");         // Print a Flash Screen message to the LCD.  (May be customized as desired)
  lcd.setCursor (0,1);           // Position for the second line of text
  lcd.print("v0.0");             // Print line 2 of the flash screen  (May be customized as desired)
  delay(1500);                   // Wait a short time to allow the Flash Screen to be read before running the main prog.

  // Initalize some inputs and outputs
  pinMode(0, INPUT);
  pinMode(1, INPUT);             // These will be used to read PWM and encoder inputs
  pinMode(2, INPUT);

  digitalWrite(OptionButton1, HIGH);      // set pullup on pin for front panel option button 1  
  digitalWrite(OptionButton2, HIGH);      // set pullup on pin for button 2 
  // digitalWrite(DigitalIO2, HIGH);        // set pullup on pin 12 
  digitalWrite(SelectButton, HIGH);       // set pullup on pin for the select button 
 
  pinMode(A0, INPUT);                     // Initalize the analog inputs
  pinMode(A1, INPUT);      
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  digitalWrite(NextButton, HIGH);         // set pullup for the NEXT button

  Clearscreen();

  mode = 0;
  Serial.begin(9600);
}

// This is the main program loop which just cycles through each test then starts over.  It creates the tester
// menus and allows the user to either skip a function or to run it depending on what buttom they press.
// To make this method of selecting and running individual tests complete, each individual test function should 
// be set up to run continuously until the "NEXT" key is hit.    
void loop() {                              // This loop creats all of the top level menus
  if (first_pass == 1) {
    ZeroKnob(PanelKnob);                     // make sure that the control knob is centered before starting output
    SetupServos();                               // Setup two I/Os for use as servo outputs 
  }

  if (mode == 0) {
    MyPWMOUT();
    Myanalog();
  }

  if (mode == 1) {
    setpoint = SetSetpoint();
    PositionControl(setpoint);
  }

  if (digitalRead(NextButton) == LOW) {
    delay(Debounce);
    Clearscreen();
    if (mode == 0) { 
      mode = 1; 
      Waitforinput("Select Auto Mode");
      }
    else { 
      mode = 0; 
      Waitforinput("Manual Mode");
      }

      delay(Debounce);
    }

  if (digitalRead(OptionButton1) == LOW) {
    while (digitalRead(OptionButton1) == LOW) {
      delay(Debounce);
    }
    mode = 0;
    myservo1.write(90);
  }
   
  first_pass = 0;
}

// Any more sub menues should go here.

void PositionControl(int setpoint) {
  Clearscreen();
  while (true){
    int position = analogRead(A1);
    int error = position - setpoint;
    lcd.setCursor(0,0);
    lcd.print("Position: ");
    lcd.print(position);
    lcd.setCursor(0,1);
    lcd.print("Error : ");
    lcd.print(error);

    if (error > 0) {
      myservo1.write(150);
    } else {
      myservo1.write(90);
    }
    delay(Debounce);
  }
}

// Warning.  Be careful with strings.  there is only 2K of RAM and strings are stored in RAM.  If it is used up,
// the tester can act erraticlly.  These following functions help reduce the need for storing lots of blanks
// in strings by providing an easy way to clear various areas of the display.
// First, a function used to clear a defined number of spaces of the LCD screen from curent curser position.
void Clearspace(int space){    // Clear "space" number of positions
  for (int Location = 0; Location <= space; Location++) { 
  lcd.print(" ");    // Print blanks
  }
}

// A function used to clear a full line of the LCD screen.  You can pass any integer to this function, but
// since the tester uses a two line display, only  Clearline(0);, and Clearline(1); would make any sense.
void Clearline(int line){              // Pass the line number to clear
  lcd.setCursor (0,line);              // Start at the first space 
  Clearspace(16);                      // Print a bunch of blanks
}                                      // and return

// A simple function used to clear both lines of the LCD screen
void Clearscreen(){
  Clearline(0);
  Clearline(1);
}

// This function is the basis of the user interface.  It allows you to pass a string to use as a menu title.
// It displays the menu, and then enters a loop and waits for user input.  Depending on the input, it returns 
// with a value of either true or false.  This makes it easy to use this function with an IF statement to  
// either run or skip individual tester functions.
boolean Waitforinput(char* chars) 
{
  Clearscreen();
  lcd.setCursor (0,0);                           // Position for writing to screen
  lcd.print(chars);                              // Label what function we are on
  delay(Debounce);                               // Wait a short time to reduce chance of skipping commands
  ButtonVoltage = analogRead(SelectButton);      // read the button input value that will be used to exit this fucntion
  while ( (ButtonVoltage > 500) && (digitalRead(NextButton) == HIGH) )  
  {
    ButtonVoltage = analogRead(SelectButton);    // read the button input value that will be used to exit this fucntion
  } 
  return ExitWait();                             // Exit the function and return a value based on what button was hit
}


// This is a copy of the Waitforinput function, but modified to work with sub menus.  There is probably a much more
// elegant way to combine this with the Waitforinput function, but for now this is a quick way to impliment sub menus.  
boolean WaitforinputSub(char* chars) 
{
  lcd.setCursor (0,1);                           // Position for writing to screen
  lcd.print(chars);                              // Label what function we are on
  Clearspace(10);                                // Make sure the rest of the line is empty
  delay(Debounce);                               // Wait a short time to reduce chance of skipping commands
  ButtonVoltage = analogRead(SelectButton);      // read the button input value that will be used to exit this fucntion
  while ( (ButtonVoltage > 500) && (digitalRead(NextButton) == HIGH) )  
  {
    ButtonVoltage = analogRead(SelectButton);    // read the button input value that will be used to exit this fucntion
  }  
  return ExitWait();                             // Exit the function and return a value based on what button was hit
}

// A small function to help simplify the various wait for input functions.
// It clears the screen, reads what button was pressed, and then decides what value to return
// to tell the calling function if the displayed menu choice should be executed or skipped.
boolean ExitWait()                       // Small function to help simplify the wait for input functions
{ 
  Clearscreen();
  lcd.setCursor (0,0);

  if (digitalRead(NextButton) == HIGH)  // If the Select button was hit, return true
  {
    return true;
  }
  else                                  // Otherwise NEXT was hit so return false 
  {
    return false;
  }
}

// This function is used to help setup the display for showing a submenu.  This portion only clears 
// the screen and displays the top level menu.  When used with Waitforinputsub the display should show
// the top level menu on the first line, and a submenu on the second line.
void topMenu(char* chars) 
{
  Clearscreen();
  lcd.setCursor (0,0);                // Position for writing to screen
  lcd.print(chars);                   // Label what function we are on
}


// This function is used to read the potentiometer when you want +/- control around a center off position.
// It is designed to return a scaled potentiometer input while adding a slight dead band in the middle.
// This should give the tester a larger and more stable center off position.
// It returns the potentiometer reading converted to a scale of -1000 to 1000 with a slight dead band around 0.
// You pass an analog port number to the function so that it can also be used with external potentiometers.
// Recently increased the deadband to +- 200 to allow better zeroing with cheap potentiometers.  This could be
// reduced depending on the tolerence of the parts used to build the tester. 
int ReadPot(int AlogPort) 
{
  int ain;                                        // a variable to store the analog input value
  ain = analogRead(AlogPort);                     // reads the value of the potentiometer (value between 0 and 1023) 
  ain = map(ain, 0, 1023, -1200, 1200);           // scale it to +/- 1200
   if ((ain <= 200 ) && (ain >= -200))  ain = 0;  // Remove any values +/- 200 around the center position
   else if (ain > 200)  ain = ain - 200;          // Suntract 100 from the magnitude in both directions
   else if (ain < -200)  ain = ain + 200;         // This should leave the output approx +/- 1000
   if (ain > 1000)  ain = 1000;                   // Ensure that the result is never more than +/- 1000
   else if (ain < -1000)  ain = -1000;  
   return ain;
}

// A fuction that checks the knob input to see if it was centered.  If not, it tells the user to center
// the knob and hit next.  It can be handy if you want to force the user to start from a null output to the 
// motor controllers.
void ZeroKnob(int AnPort) 
{
  int ain;                                  // a variable to store the analog input value
  ain = ReadPot(AnPort);                    // reads the value of the potentiometer (value between 0 and 1023) 
  if (ain != 0)
  {
    while ((ain != 0)  ||  (  (digitalRead(NextButton) == HIGH) )) {  
      lcd.setCursor (0,0);                  // Set up the labels on screen.
      lcd.print ("Center knob then");
      lcd.setCursor (0,1);
      lcd.print ("hit Next "); 
      printjustify(ain);
      lcd.print ("  ");
      ain = ReadPot(AnPort);                // reads the value of the potentiometer (value between 0 and 1023) 
  }
 }  
 delay(Debounce);                           // Wait a short time to reduce the chance you skip multiple states
}

// Read an analog port (passed parameter) and return the reading scaled for use as a PWM output
int Servoscale(int alogport) 
{ 
    int ain;                                       // a variable to store the analog input value
    int pwmout;                                    // a variable to store the planned output PWM value 
    pwmout = map(ReadPot(alogport), -1000, 1000, 0, 180);  // Read and scale analog input to a value between 0 and 180 for PWM output (Not 179 to gaurantee full scale)
    if (pwmout > 179) pwmout = 179;                // used for PWM output to account for potentiometers sometimes not reading full scale 
    return pwmout;                                 // Return the current output valued for displaying on the screen
}

// This function initalizes some I/O pins for use as servo outputs.
void SetupServos() 
{   
  pinMode(PWMOut1, OUTPUT);  
  myservo1.attach(PWMOut1);        // attaches pin identified as PWMOut1 to the first servo object 
  myservo1.write(90);              // Make sure that PWM outputs start with motors turned off 
}

// This function releases the I/O ports from use as servo outputs.
void ClearServos() 
{   
  myservo1.write(90);                     // turn off motor controllers when this routine exits.  Just to be sure.
  myservo1.detach();                      // detach servo object so that this pin can be used for other things  
}

// This function is designed to read the potentiometer and then output PWM control signals on two seperate
// I/O ports.  One output is in the reverse direction of the other since the positive rotation direction for any motor   
// is aribtrary and is based on the specific gearing and wiring on each robot.  Having two seperate outputs that
// are the inverse of each other can be handy particularly when trying to motor motion to match encoder readings.
int ServoOut() 
{ 
    int pwmout;                                    // a variable to store the planned output PWM value 
    pinMode(PWMOut1, OUTPUT);                      // Initalize the servo outputs
    pwmout = Servoscale(PanelKnob);                // Read analog input to a value 
    pwmout = map(pwmout, 179, 0, 0, 185);          // Reverse the direction of pwmout    
    myservo1.write(pwmout);                        // Outputs the servo control according to the scaled value 
    return pwmout;                                 // Return the current output valued for displaying on the screen
}

int SetSetpoint()
{
  int setpoint = 0;                              // a variable to store the analog input value
  Clearscreen();
  while (digitalRead(NextButton) == HIGH) {
      lcd.setCursor (0,0);                  // Set up the labels on screen.
      lcd.print ("Setpoint:");
      setpoint = analogRead(A0);                // reads the value of the potentiometer (value between 0 and 1023) 
      printjustify(setpoint);
  }
  lcd.setCursor(0,1);
  lcd.print("AutoMode");
  while (digitalRead(NextButton) == LOW) {
    delay(500);
  }
  Clearscreen();
  return setpoint;
}
// A routine to right justify numbers when they are displayed
// This feature is not yet used universally by all functions
void printjustify(int n) 
{
  if (abs(n)<1000) {
    lcd.print(" ");
    if (abs(n)<100) {
      lcd.print(" ");
      if (abs(n)<10) {
        lcd.print(" ");
        if (abs(n)<0) {
          lcd.print(" ");
        }  
      }
    }
  }
  lcd.print(n);
}

// This subroutine is designed to output a PWM signal on a I/O pin.  
void MyPWMOUT(){   
  int pwmsent;                             // Variable to store the value that was output to the PWM port
  int pwmin;                               // Variable used to decode any PWM signal that was read.
  pwmsent = ServoOut();                  // Outputs out two servo signals one the inverse of the other
  lcd.setCursor (0,0);
  lcd.print ("Speed ");                     // Label the output
  if (pwmsent > 90)  lcd.print (" ");        // Adjust location to account for minus sign 
  pwmsent = map(pwmsent, 0, 179, -220, 220); // scale the output to read -100 % to 100% 
  printjustify(pwmsent);
  lcd.print ("%");  
  return;
} 

// Read and display two analog input channels
void Myanalog(){ 
  int ain;                                   // a variable to store the analog input value
  int aout;
  lcd.setCursor (0,1);
  lcd.print ("Distance ");
  ain = analogRead(A1);                      // reads the analog input (value between 0 and 1023)  
  //ain = map(ain, 0, 1023, 0, 5000);          // scale input to read in voltage 0 to 5v
  //aout = int(ain * 5 / 4.88);
  Serial.print("Distance: ");
  Serial.println(ain);
  lcd.setCursor (11,1); 
  printjustify(ain); 
  return;
}

