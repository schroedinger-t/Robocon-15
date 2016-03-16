#include <SabertoothSimplified.h>
SabertoothSimplified ST1(Serial1);
SabertoothSimplified ST2(Serial2);

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // RX | TX    //The software serial buffer can hold 64 bytes

volatile byte a[21];
int contInt = 0;//Interrupt from controller containing array    // Interrupt 0 is on DIGITAL PIN 2!
int brake = -1;
int Lx_new;
int Ly_new;
//new variables
int x;  //Joystick X
int y;  //Joystick Y
int t;  //Turning L2,R2
boolean line;  //line present or absent

/****************************BOT MOTION********************/
#define brakingButton triangle
#define BotCW R2
#define BotCCW L2

float x_1;
float y_1;
int deadz=46;
float V_x;  //Sideways Velocity
float V_y;  //Forward Velocity
float V_t;  //Turning velocity
float rot=0.2;  //Rotation Factor for turning while line following
//drive motors
float m1=0;
float m2=0;
float m3=0;
float m4=0;
float divisor;  //divisor  for normalizing
/*********************************************************/

/********************************SHUTTLE PUSH*********************/
#define ShuttlePushButton down_button
#define ShuttleReset up_button
int hall_shuttle[7];

int Shuttle_Push_Pos = 1; //hall sensor to choose for shuttle pushing mechanism
int shuttle_overshoot = 0;

//Shuttle Push Piston Solenoids
int Piston_solfor_Shuttle = 9;
int Piston_solbac_Shuttle = 8;

//volatile int mux_var1 = 0;
//volatile int mux_var2 = 0;
//volatile int shuttlepush_reset = 0;
//volatile int shuttle_stop_called = 0;
/*******************************************************************/

/********************************FRAME ROTATION*********************/
#define FrameRotationCW R1
#define FrameRotationCCW L1

//Pin numbers in <void setup()> of Frame Rotation Hall Sensors
int hall_frame[3];
int frame_pos = 1;   //hall sensor to choose for frame rotation mechanism

//Frame Rotation Piston Solenoids
int Piston_solfor_frame = 51;
int Piston_solbac_frame = 49;
/*******************************************************************/


/*******************************Racquet SWING************************/
#define RacquetSwingHalf cross
#define RacquetSwingFull circle

//Pin numbers in <void setup()> of Racquet Swing Hall Sensors
int hall_racquet[3];
int racquet_pos=1;   //hall sensor to choose for racquet swing mechanism

//racquet Swing Piston Solenoids
int Piston_solfor_racquet = 31;
int Piston_solbac_racquet = 29;
/*******************************************************************/

/**************************FLAGS************************************/
#define AllReset select

int Flag_Brake = 0;
int Flag_Shuttle_Pushed = 0;
int Flag_Frame_Rotated = 0;
int Flag_Racquet_Swung = 0;
/*******************************************************************/

/*********************Assigning PS Buttons*****************************************************/
volatile int start = 0;
volatile int select = 0;
volatile int L3 = 0;
volatile int R3 = 0;
volatile int triangle = 0;
volatile int Square = 0;
volatile int circle = 0;
volatile int cross = 0;
volatile int L2 = 0;
volatile int R2 = 0;
volatile int L1 = 0;
volatile int R1 = 0;
volatile int Lx = 127;
volatile int Ly = 127;
volatile int Rx = 127;
volatile int Ry = 127;
//volatile int up_pressure;
//volatile int down_pressure;
//volatile int left_pressure;
//volatile int right_pressure;
volatile int up_button = 0;
volatile int down_button = 0;
volatile int left_button = 0;
volatile int right_button = 0;
/****************************************************************/

/******************************************************************************************************************/

void setup(){
  Serial.begin(38400);
  Serial1.begin(38400);
  Serial2.begin(38400);
  //SabertoothTXPinSerial.begin(9600);
  BTSerial.begin(38400);
  
  attachInterrupt(contInt, receiveArray, CHANGE);
  
  //LED for indicating start
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  
  /********TEMPORARY************/
  pinMode(41, OUTPUT);
  digitalWrite(41, HIGH);
  pinMode(42, OUTPUT);
  digitalWrite(42, LOW);
  /*****************************/
  
  /**********************************SHUTTLE PUSH***********************/
//  pinMode(mux_out,INPUT);
//  pinMode(mux_s2, OUTPUT);
//  pinMode(mux_s1, OUTPUT);
//  pinMode(mux_s0, OUTPUT);
//*********  Not using MUX due to error in reading LOW
  hall_shuttle[0] = 22;
  hall_shuttle[1] = 24;
  hall_shuttle[2] = 26;
  hall_shuttle[3] = 28;
  hall_shuttle[4] = 30;
  hall_shuttle[5] = 32;
  hall_shuttle[6] = 34;
  for(int i=0; i <=6; i++)
  {
    pinMode(hall_shuttle[i], INPUT);
    //digitalWrite(hall_shuttle[i], HIGH);
  }
  pinMode(Piston_solbac_Shuttle, OUTPUT);
  digitalWrite(Piston_solbac_Shuttle, LOW);
  pinMode(Piston_solfor_Shuttle, OUTPUT);
  digitalWrite(Piston_solfor_Shuttle, LOW);
//  digitalWrite(mux_s2,LOW);//select the hall sensor for the first level
//  digitalWrite(mux_s1,LOW);
//  digitalWrite(mux_s0,HIGH);
  /*********************************************************************/
  
  /**********************************FRAME ROTATION**********************/
  hall_frame[0] = 43;
  hall_frame[1] = 45;
  hall_frame[2] = 47;
  for(int i=0; i<3; i++)
    pinMode(hall_frame[i], INPUT);
  
  pinMode(Piston_solfor_frame, OUTPUT);
  digitalWrite(Piston_solfor_frame, LOW);
  pinMode(Piston_solbac_frame, OUTPUT);
  digitalWrite(Piston_solbac_frame, LOW);
  /*********************************************************************/
  
  /***********************************RACQUET SWING**********************/
  hall_racquet[0] = 23;
  hall_racquet[0] = 25;
  hall_racquet[0] = 27;
  for(int i=0; i<3; i++)
    pinMode(hall_racquet[i], INPUT);
    
  pinMode(Piston_solfor_racquet, OUTPUT);
  digitalWrite(Piston_solfor_racquet, LOW);
  pinMode(Piston_solbac_racquet, OUTPUT);
  digitalWrite(Piston_solbac_racquet, LOW);
  /*********************************************************************/
  while(!start)
  {
    BTSerial.write('s');
    attachInterrupt(contInt, receiveArray, CHANGE);
    delay(150);
    Serial.println(start);
  }
  digitalWrite(13, HIGH);
}

/******************************************************************************************************************/

void loop() {
  int start_time = millis();
  for(int i=0; i<=20; i++)
  {
    Serial.print(a[i]);
    Serial.print(" ");
  }
  x = Rx;
  y = Ly;
//  int start = a[1];
//  int select = a[2];
//  int L3 = a[3];
//  int R3 = a[4];
//  int triangle = a[5];
//  int square = a[6];
//  int circle = a[7];
//  int cross = a[8];
//  int L2 = a[9];
//  int R2 = a[10];
//  int L1 = a[11];
//  int R1 = a[12];
//  int Lx = a[13];
//  int Ly = a[14];
//  int Rx = a[15];
//  int Ry = a[16];
//  int up_pressure = a[17];
//  int down_pressure = a[18];
//  int left_pressure = a[19];
//  int right_pressure = a[20];
//  x=Rx;
//  y=Ly;
  if(BotCW)
    t=1;
  else if(BotCCW)
    t=-1;
  else t=0;
  
  /*if(start == 1)
    Serial.print("    Start pressed");
  if(select == 1)
    Serial.print("    Select pressed");
  if(L3 == 1)
    Serial.print("    L3 pressed");
  if(R3 == 1)
    Serial.print("    R3 pressed");
  if(L2 == 1)
    Serial.print("    L2 pressed");
  if(R2 == 1)
    Serial.print("    R2 pressed");
  if(L1 == 1)
    Serial.print("    L1 pressed");
  if(R1 == 1)
    Serial.print("    R1 pressed");
  if(triangle == 1)
    Serial.print("    Triangle pressed");        
  if(circle == 1)
    Serial.print("    Circle pressed");
  if(cross == 1)
    Serial.print("    X pressed");
  if(square == 1)
    Serial.print("    Square pressed");     
  Serial.println();
  //if(L2 || R2) 
  //Serial.print("Stick Values:");
  //Serial.print(Ly); //Left stick, Y axis. Other options: LX, RY, RX  
  //Serial.print(",");
  //Serial.print(Lx); 
  //Serial.print(",");
  //Serial.print(Ry); 
  //Serial.print(",");
  //Serial.println(Rx); 
  

  /*Serial.print("Lx: "); //Left stick, Y axis. Other options: LX, RY, RX  
  Serial.print(Lx);
  Serial.print("    Ly: "); //Left stick, Y axis. Other options: LX, RY, RX  
  Serial.print(Ly);
  Serial.println();*/
//Lx_new = Ly + Lx;
 // Ly_new = Ly - Lx;
  if(x>=0 && x<(128-deadz/2))
  {
    x_1=(x-(127.5-deadz/2))/(127.5-deadz/2);
  }
  else if(x>128+deadz/2 && x<=256)
  {
    x_1=(x-(127.5+deadz/2))/(127.5-deadz/2);
  }
  else
  {
    x_1 = 0;
  }

  if(y >= 0 && y <(128-deadz/2))
  {
    y_1=-((y-(127.5-deadz/2)) / (127.5-deadz/2));
  }
  else if(y > 128+deadz/2 && y<=256)
  {
    y_1=-((y -(127.5+deadz/2)) / (127.5-deadz/2));
  }
  else
  {
    y_1 = 0;
  }

  if(x_1>=0)  
    V_x = (pow(11,x_1)-1);
  else
    V_x = -pow(11,-x_1)+1;

  if(y_1>=0)
    V_y = pow(11,y_1)-1;
  else
    V_y = -pow(11,-y_1)+1;

  V_t = 5*t;

//  if(ps2x.Button(PSB_PAD_LEFT))  //Change the drive if curve selected
 // {
   //if(mode != 2 || mode !=0)
 //  V_x=(abs(x_1)/x_1)*10;
 //     V_t= -V_x*rot;  //rot is the multiplication factor which decides difference in absolute values of left and right wheels  
 // }
//m1(Serial1,2);m2(Serial1,1);m3(Serial2,2),m4(Serial2,1)
  m1 = (V_x + V_y - V_t)*12.7*2;  //Subtract |V_t| from absolute value of linear velocity
  m2 = (V_x - V_y - V_t)*12.7*2;
  m3 = (-V_x - V_y - V_t)*12.7*2; //Add |V_t| to absolute value of linear velocity
  m4 = (-V_x + V_y - V_t)*12.7*2; 

  if(abs(m1)>127 || abs(m2)>127 || abs(m3)>127 || abs(m4)>127)  //Normalizing the values to counter the overflow
  {
    divisor= max(abs(m1),abs(m2));
    divisor= max(abs(divisor),abs(m3));
    divisor= max(abs(divisor),abs(m4));
    //    divisor= divisor;
    m1= m1*127/divisor;
    m2= m2*127/divisor;
    m3= m3*127/divisor;
    m4= m4*127/divisor;
  }

  m1=m1/8;
  m2=m2/8;
  m3=m3/8;
  m4=m4/8;
  if((!Flag_Brake)&&(brakingButton))
  {
    brake = -1*brake;    //brake
    Flag_Brake = 1;
  }
  Serial.println(brake);
//  if(brake == 1)
//    digitalWrite(13, HIGH);
//  else if(brake == -1)
//    digitalWrite(13, LOW);
    
  if(brake!=1)
  {
    if((int)m3 > 0)
    {
      m3 = (m3*17.5)/15;
    }
    if((int)m1 < 0)
    {
      m1 = (m1*17.5)/15;
    }
    ST2.motor(2,(int)-m3);
    ST2.motor(1,(int)-m4);
    ST1.motor(1,(int)-m2);
    ST1.motor(2,(int)m1);
    
    Serial.println();
    Serial.print((int)m1);
    Serial.print("    ");
    Serial.print((int)m2);
    Serial.print("    ");
    Serial.print((int)m3);
    Serial.print("    ");
    Serial.print((int)m4);
    Serial.println();    
//    ST2.motor(2,(int)-m3);
//    ST2.motor(1,(int)-m4);
//    ST1.motor(1,(int)-m2);
//    ST1.motor(2,(int)m1);
 /*   if((Lx<=12 && Lx>=-9)&&(Ly<=12 && Ly>=-9))
    {
      ST1.motor(1, 0);
      ST1.motor(2, 0);
      ST2.motor(1, 0);
      ST2.motor(2, 0);
    }
    else
    { 
      //Lx_new = map(Lx_new, -255, 256, -31, 32);
      //Ly_new = map(Ly_new, -255, 256, 32, -31);
      Lx_new = Lx_new/8;
      Ly_new = Ly_new/8;
      ST1.motor(1, -Lx_new); 
      ST1.motor(2, Lx_new);
      ST2.motor(1, -Ly_new); 
      ST2.motor(2, -Ly_new);
      
      Serial.print("Lx_new: ");     //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(Lx_new);
      Serial.print("    Ly_new: "); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(Ly_new);
      Serial.println();
      Serial.println();
    }*/

  }
  else
  {
    ST1.motor(1, 0);
    ST1.motor(2, 0);
    ST2.motor(1, 0);
    ST2.motor(2, 0);
  }
/**************************************************************************/

/**********************************SHUTTLE PUSH********************************/

  if((!Flag_Shuttle_Pushed)&&(ShuttlePushButton)) //1st push and instead of 'z' keep the serving button
  {
    Flag_Shuttle_Pushed = 1;
    ShuttlePushButton = 0;
    Serial.print("Pushing    ");
    digitalWrite(Piston_solbac_Shuttle,HIGH); //piston should move forward
    digitalWrite(Piston_solfor_Shuttle,LOW);  
    if(Shuttle_Push_Pos<6)
      while((digitalRead(hall_shuttle[Shuttle_Push_Pos]) == 1)&&(digitalRead(hall_shuttle[Shuttle_Push_Pos + 1]) == 1));
    else
      while(digitalRead(hall_shuttle[Shuttle_Push_Pos]) == 1);      
    Serial.println("jai mata di");
    digitalWrite(Piston_solbac_Shuttle, LOW);
    digitalWrite(Piston_solfor_Shuttle, LOW);
    Shuttle_Push_Pos++;
    Serial.println(Shuttle_Push_Pos);
  }
          
  if (ShuttleReset == 1)
  {
    ShuttleReset = 0;
    
    digitalWrite(Piston_solbac_Shuttle,LOW);
    digitalWrite(Piston_solfor_Shuttle,HIGH);

    while(digitalRead(hall_shuttle[0]) == 1);
    digitalWrite(Piston_solfor_Shuttle, LOW);
    digitalWrite(Piston_solbac_Shuttle, LOW);
    Shuttle_Push_Pos = 1;
  }
  
  if(Shuttle_Push_Pos==7)
  {
    digitalWrite(Piston_solbac_Shuttle,LOW);
    digitalWrite(Piston_solfor_Shuttle,HIGH);
    Shuttle_Push_Pos = 0;
    while(digitalRead(hall_shuttle[Shuttle_Push_Pos])==1);
    digitalWrite(Piston_solfor_Shuttle, LOW);
    digitalWrite(Piston_solbac_Shuttle, LOW);
    Shuttle_Push_Pos++;
  }
  //add condition that if shuttle piston is already at 0 position, reset doesn't keep the forward solenoid ON
    
  /**********************************************************************************/

  /**********************************FRAME ROTATION********************************/
  if(!Flag_Frame_Rotated&&((FrameRotationCW == 1)&&(frame_pos != 3)))
  {
    Flag_Frame_Rotated = 1;
    Serial.println("Rotating Clockwise");
    digitalWrite(Piston_solbac_frame, HIGH);
    digitalWrite(Piston_solfor_frame, LOW);
    if(frame_pos == 1)
    {
      while(digitalRead(hall_frame[2]) != 0);
      digitalWrite(Piston_solbac_frame, LOW);
      digitalWrite(Piston_solfor_frame, LOW);
      frame_pos++;
      Serial.print("Next frame position: ");
      Serial.println(frame_pos);
    }
    else if(frame_pos == 2)
    {
      while(digitalRead(hall_frame[3]) != 0);
      digitalWrite(Piston_solbac_frame, LOW);
      digitalWrite(Piston_solfor_frame, LOW);
      frame_pos++;
      Serial.print("Next frame position: ");
      Serial.println(frame_pos);
    }
  }
  
  if(!Flag_Frame_Rotated&&((FrameRotationCCW == 1)&&(frame_pos != 1)))
  {
    Flag_Frame_Rotated = 1;
    Serial.println("Rotating Counter Clockwise");
    digitalWrite(Piston_solbac_frame, LOW);
    digitalWrite(Piston_solfor_frame, HIGH);
    if(frame_pos == 3)
    {
      while(digitalRead(hall_frame[2]) != 0);
      digitalWrite(Piston_solbac_frame, LOW);
      digitalWrite(Piston_solfor_frame, LOW);
      frame_pos--;
      Serial.print("Next frame position: ");
      Serial.println(frame_pos);
    }
    else if(frame_pos == 2)
    {
      while(digitalRead(hall_frame[1]) != 0);
      digitalWrite(Piston_solbac_frame, LOW);
      digitalWrite(Piston_solfor_frame, LOW);
      frame_pos--;
      Serial.print("Next frame position: ");
      Serial.println(frame_pos);
    }
  }
  /************************************************************************/
  
  /************************************RACQUET SWING************************/
  if((!Flag_Racquet_Swung) && RacquetSwingHalf)
  {
    Flag_Racquet_Swung = 1; 
    Serial.println("Half Swing Racquet");
    digitalWrite(Piston_solbac_racquet, HIGH);
    digitalWrite(Piston_solfor_racquet, LOW);
    
    while(hall_racquet[2] != 0);
    digitalWrite(Piston_solbac_racquet, LOW);
    digitalWrite(Piston_solfor_racquet, LOW);
    delay(500);
    digitalWrite(Piston_solbac_racquet, LOW);
    digitalWrite(Piston_solfor_racquet, HIGH);
    
    while(hall_racquet[1] != 0);
    digitalWrite(Piston_solbac_racquet, LOW);
    digitalWrite(Piston_solfor_racquet, LOW);
  }
  
  if((!Flag_Racquet_Swung) && RacquetSwingFull)
  {
    Flag_Racquet_Swung = 1;
    Serial.println("Full Swing Racquet");
    digitalWrite(Piston_solbac_racquet, HIGH);
    digitalWrite(Piston_solfor_racquet, LOW);
    
    while(hall_racquet[3] != 0);
    digitalWrite(Piston_solbac_racquet, LOW);
    digitalWrite(Piston_solfor_racquet, LOW);
    delay(500);
    digitalWrite(Piston_solbac_racquet, LOW);
    digitalWrite(Piston_solfor_racquet, HIGH);
    
    while(hall_racquet[1] != 0);
    digitalWrite(Piston_solbac_racquet, LOW);
    digitalWrite(Piston_solfor_racquet, LOW);
  }
  /************************************************************************/
  if(AllReset)
  {         
    Flag_Brake = 0;
    Flag_Shuttle_Pushed = 0;
    Flag_Frame_Rotated = 0;
    Flag_Racquet_Swung = 0;
  }
  int time_elapsed = millis() - start_time;
  if(time_elapsed < 150)
    delay(150-time_elapsed);  //loop should execute for atleast 150 ms to avoid messing up received data
  
  BTSerial.write('s');
  attachInterrupt(contInt, receiveArray, CHANGE);
}


void receiveArray()
{
  detachInterrupt(contInt);
  if(BTSerial.available())
  {
    a[0] = BTSerial.read();
    if(a[0] == 60)
    {
      //Serial.println("Rec.");
      for(int i=1; i<=20; i++)
      {
        a[i] = BTSerial.read();
      }
    }
    BTSerial.flush();  //clear the buffer
  }
  
  start = a[1];
  select = a[2];
  L3 = a[3];
  R3 = a[4];
  triangle = a[5];
  Square = a[6];
  circle = a[7];
  cross = a[8];
  L2 = a[9];
  R2 = a[10];
  L1 = a[11];
  R1 = a[12];
  Lx = a[13];
  Ly = a[14];
  Rx = a[15];
  Ry = a[16];
//  up_pressure = a[17];
//  down_pressure = a[18];
//  left_pressure = a[19];
//  right_pressure = a[20];
  up_button = a[17];
  down_button = a[18];
  left_button = a[19];
  right_button = a[20];
  x=Rx;
  y=Ly;

  //attachInterrupt(contInt, receiveArray, CHANGE);
}

/*void shuttlepush_pistonstop()
{
  detachInterrupt(1);
  //digitalWrite(13, HIGH);
  shuttle_stop_called = 1;
  //digitalWrite(Piston_solbac_Shuttle,LOW);
  //digitalWrite(Piston_solfor_Shuttle,LOW);
  if(shuttlepush_reset != 1)
  {
    Shuttle_Push_Pos++;
//    Serial.print("Next Position: ");
//    Serial.print(Shuttle_Push_Pos);
//    Serial.println();
  }
  else
  {
     Shuttle_Push_Pos=1;
//    Serial.print("Next Position: ");
//    Serial.print(Shuttle_Push_Pos);
//    Serial.println()   
//    Serial.println(digitalRead(mux_out));
    shuttlepush_reset = 0;
  }
    
}*/
