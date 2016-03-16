#include <PS2X_lib.h>  //for v1.6

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(8, 9); // RX | TX    //The software serial buffer can hold 64 bytes

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        13  //14 brown  
#define PS2_CMD        11  //15 orange
#define PS2_SEL        10  //16 yellow
#define PS2_CLK        12  //17 blue

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   true
//#define pressures   false
#define rumble      true
//#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;


volatile byte a[21];
int botInt= 0;          //Interrupt from bot to send array                  // Interrupt 0 is on DIGITAL PIN 2!
volatile char fromBot;   //character received from bot, to check?

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
 
  Serial.begin(57600);
  BTSerial.begin(38400);
  attachInterrupt(botInt, sendArray, CHANGE);
  
  delay(500);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
//
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
	if (pressures)
	  Serial.println("true ");
	else
	  Serial.println("false");
	Serial.print("rumble = ");
	if (rumble)
	  Serial.println("true)");
	else
	  Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
    
  type = ps2x.readType(); 
  
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
	case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }

    a[0] = '<';
}
//////////////////////////////////////////////////////////////////////////
void loop() {
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
   */  
  
  if(error == 1) //skip loop if no controller found
    return; 
  /*
  if(type == 2){ //Guitar Hero Controller
    ps2x.read_gamepad();   
    //read controller 
       
    if(ps2x.ButtonPressed(GREEN_FRET))
      Serial.println("Green Fret Pressed");
    if(ps2x.ButtonPressed(RED_FRET))
      Serial.println("Red Fret Pressed");
    if(ps2x.ButtonPressed(YELLOW_FRET))
      Serial.println("Yellow Fret Pressed");
    if(ps2x.ButtonPressed(BLUE_FRET))
      Serial.println("Blue Fret Pressed");
    if(ps2x.ButtonPressed(ORANGE_FRET))
      Serial.println("Orange Fret Pressed"); 

    if(ps2x.ButtonPressed(STAR_POWER))
      Serial.println("Star Power Command");
    
    if(ps2x.Button(UP_STRUM))          //will be TRUE as long as button is pressed
      Serial.println("Up Strum");
    if(ps2x.Button(DOWN_STRUM))
      Serial.println("DOWN Strum");
 
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");
    
    if(ps2x.Button(ORANGE_FRET)) {     // print stick value IF TRUE
      Serial.print("Wammy Bar Position:");
      Serial.println(ps2x.Analog(WHAMMY_BAR), DEC); 
    } 
  }
  else { //DualShock Controller
  */
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
        if (ps2x.Button(PSB_START))
        {
          a[1]=1;
          Serial.println("Start");
        }

	if(ps2x.Button(PSB_SELECT))
        {
	  a[2]=1;
          Serial.println("Select");
        }

	if (ps2x.Button(PSB_L3))
        {
	  a[3]=1;
          Serial.println("L3");
        }

	if (ps2x.Button(PSB_R3))
	{
	  a[4]=1;
          Serial.println("R3");
        }

	if (ps2x.Button(PSB_TRIANGLE))
	{
	  a[5]=1;
          Serial.println("Triangle");
        }

	if (ps2x.Button(PSB_SQUARE))
	{
	  a[6]=1;
          Serial.println("Square");
        }

	if (ps2x.Button(PSB_CIRCLE))
        {
	  a[7]=1;
          Serial.println("Circle");
        }

	if (ps2x.Button(PSB_CROSS))
	{
	  a[8]=1;
          Serial.println("Cross");
        }

	if (ps2x.Button(PSB_L2))
	{
	  a[9]=1;
          Serial.println("L2");
        }

	if (ps2x.Button(PSB_R2))
	{
	  a[10]=1;
          Serial.println("R2");
        }

	if (ps2x.Button(PSB_L1))
	{
	  a[11]=1;
          Serial.println("L1");
        }
        
	if (ps2x.Button(PSB_R1))
	{
	  a[12]=1;
          Serial.println("R1");
        }

        //Analog values
        a[13] = ps2x.Analog(PSS_LX);
        a[14] = ps2x.Analog(PSS_LY);
        a[14] = map(a[14], 0, 255, 255, 0);
        a[15] = ps2x.Analog(PSS_RX);
        a[16] = ps2x.Analog(PSS_RY);
        
//        //Pressure values
//        //if(ps2x.Button(PSB_PAD_UP))
//          a[17] = ps2x.Analog(PSAB_PAD_UP);
//        //if(ps2x.Button(PSB_PAD_DOWN))
//          a[18] = ps2x.Analog(PSAB_PAD_DOWN);
//        //if(ps2x.Button(PSB_PAD_LEFT))
//          a[19] = ps2x.Analog(PSAB_PAD_LEFT);
//        //if(ps2x.Button(PSB_PAD_RIGHT))
//          a[20] = ps2x.Analog(PSAB_PAD_RIGHT);
//          
        a[17] = ps2x.Button(PSB_PAD_UP);
      //if(ps2x.Button(PSB_PAD_DOWN))
        a[18] = ps2x.Button(PSB_PAD_DOWN);
      //if(ps2x.Button(PSB_PAD_LEFT))
        a[19] = ps2x.Button(PSB_PAD_LEFT);
      //if(ps2x.Button(PSB_PAD_RIGHT))
        a[20] = ps2x.Button(PSB_PAD_RIGHT);
        
        for(int i=0; i<=20; i++)
        {
          Serial.print(a[i]);
          Serial.print(" ");
        }
        Serial.println();

        attachInterrupt(botInt, sendArray, CHANGE);
}

void sendArray()
{
  detachInterrupt(botInt);
  if(BTSerial.available())
  {
    fromBot = BTSerial.read();
  }
  
  for(int i=0; i<=20; i++)
  {
    BTSerial.write(a[i]);
  }
  BTSerial.flush();
  for(int i=1; i<=12; i++)    //array values changed to zero after data is sent to ensure that no data is lost
  {
    a[i] = 0;
  }
  
//  attachInterrupt(botInt, sendArray, CHANGE);
}
