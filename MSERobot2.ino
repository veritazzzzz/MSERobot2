//hibby

#include <uSTimer2.h>

#include <I2CEncoder.h>

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>

#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_FrontRightMotor;
Servo servo_FrontLeftMotor;
Servo servo_BackLeftMotor;
Servo servo_BackRightMotor;

Servo servo_BaseArmMotor;
Servo servo_TopArmMotor;
Servo servo_GripMotor;

Servo left_grip_servo; //these two are for the grip to open and close
Servo right_grip_servo;

I2CEncoder encoder_FrontRightMotor;
I2CEncoder encoder_FrontLeftMotor;
I2CEncoder encoder_BackRightMotor;
I2CEncoder encoder_BackLeftMotor;
I2CEncoder encoder_GripMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION

boolean bt_Motors_Enabled = true;

//port pin constants
const int ci_Mode_Button = 7000;                                     //place the port pins here (ultrasonic, motors)
const int ci_FrontRight_Motor = 12;
const int ci_FrontLeft_Motor = 13;
const int ci_BackRight_Motor = 10;
const int ci_BackLeft_Motor = 11;


//problem pins are 4 and 5
const int ci_Front_Ultrasonic_Data = 3;
const int ci_Back_Ultrasonic_Data = 100; //DONT CHANGE
const int ci_FrontLeft_Ultrasonic_Data = 7;
const int ci_BackLeft_Ultrasonic_Data = 2000;
const int ci_FrontRight_Ultrasonic_Data = 8;
const int ci_BackRight_Ultrasonic_Data = 800;

const int ci_Front_Ultrasonic_Ping = 2;
const int ci_Back_Ultrasonic_Ping = 200; //DONT CHANGE
const int ci_FrontLeft_Ultrasonic_Ping = 6;
const int ci_BackLeft_Ultrasonic_Ping = 600;
const int ci_FrontRight_Ultrasonic_Ping = 9;
const int ci_BackRight_Ultrasonic_Ping = 900;

const int ci_Grip_Motor = 11;
//const int ci_Motor_Enable_Switch = 12;
const int ci_Right_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Left_Line_Tracker = A2;
const int ci_Light_Sensor = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow
//<<< <<< < HEAD
//== == == =



//might have to go on seperate board
int ISRPin = 13;




// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 10;
/*const int ci_Right_Line_Tracker_LED = 6;
  const int ci_Middle_Line_Tracker_LED = 9;
  const int ci_Left_Line_Tracker_LED = 12;*/

//constants

// EEPROM addresses

const int ci_Front_Left_Motor_Offset_Address_L = 12;
const int ci_Front_Left_Motor_Offset_Address_H = 13;
const int ci_Front_Right_Motor_Offset_Address_L = 14;
const int ci_Front_Right_Motor_Offset_Address_H = 15;

const int ci_Back_Left_Motor_Offset_Address_L = 16;
const int ci_Back_Left_Motor_Offset_Address_H = 17;       //not sure if 16-19 are correct
const int ci_Back_Right_Motor_Offset_Address_L = 18;
const int ci_Back_Right_Motor_Offset_Address_H = 19;

//used for line tracking code (mode 1)
boolean pauseHere;

int stageCounter = 0;
const int ci_Front_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Front_Right_Motor_Stop = 1500;
const int ci_Back_Left_Motor_Stop = 1500;
const int ci_Back_Right_Motor_Stop = 1500;

const int ci_Grip_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 180;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Zero = 80;          //  "
const int ci_Grip_Motor_Closed = 80;       //  "

const int ci_BaseArm_Servo_Retracted = 55;      //  "
const int ci_BaseArm_Servo_Extended = 120;      //  "

const int ci_TopArm_Servo_Retracted = 55;      //  "
const int ci_TopArm_Servo_Extended = 120;

const int ci_Display_Time = 500;
/*const int ci_Line_Tracker_Calibration_Interval = 100;
  const int ci_Line_Tracker_Cal_Measures = 20;
  const int ci_Line_Tracker_Tolerance = 100; // May need to adjust this
  const int ci_Motor_Calibration_Time = 5000;*/

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;
/*unsigned int ui_Left_Line_Tracker_Data;
  unsigned int ui_Middle_Line_Tracker_Data;
  unsigned int ui_Right_Line_Tracker_Data;*/
unsigned int ui_Motors_Speed = 1900;        // Default run speed

unsigned int ui_Front_Left_Motor_Speed;
unsigned int ui_Front_Right_Motor_Speed;
unsigned int ui_Back_Right_Motor_Speed;
unsigned int ui_Back_Left_Motor_Speed;

unsigned long ul_Front_Left_Motor_Position;
unsigned long ul_Front_Right_Motor_Position;
unsigned long ul_Back_Left_Motor_Position;
unsigned long ul_Back_Right_Motor_Position;

unsigned long ul_Grip_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;

unsigned long ui_Front_Left_Motor_Offset;
unsigned long ui_Front_Right_Motor_Offset;
unsigned long ui_Back_Left_Motor_Offset;
unsigned long ui_Back_Right_Motor_Offset;

/*unsigned int ui_Cal_Count;
  unsigned int ui_Left_Line_Tracker_Dark;
  unsigned int ui_Left_Line_Tracker_Light;
  unsigned int ui_Middle_Line_Tracker_Dark;
  unsigned int ui_Middle_Line_Tracker_Light;
  unsigned int ui_Right_Line_Tracker_Dark;
  unsigned int ui_Right_Line_Tracker_Light;
  unsigned int ui_Line_Tracker_Tolerance;*/

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536
};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;
boolean grab_object_mode = false; //used for lab04, sets robot into target aquisition mode
boolean target_aquired = false; //usef for lab04, to determine which stage of target aquisition bot is in
boolean coarse_target_direction = false; //used for lab04, coarse grain direction to target

int x_degrees_turn_position = 10; //value associated with reading a 90 degree turn
int light_sensor_data = 0; //globabl variable to hold light sensor output data
int light_tolerance = 0; //might need to change this
int light_sensor_bright = 300;
int distance_to_object = 10; //EXPERIMENTAL VALUE
int arm_target_length = 20; //value of arm length
unsigned long max_light_position = 0;
int which_case = 0;
unsigned long current_position = 0;


/////////////////////////////////////////////////////////////
//JULIAN'S GLOBAL VARIABLES
/////////////////////////////////////////////////////////////
//for opening and closing the grip
int open_pos = 40; //angle associated with claw open
int closed_pos = 160; //angle associated with claw closed

//for navigation
int current_pos[3]; //0th position is x coordinate, 1st is y, 2nd is directionality register
int last_known_cube_pos [3];
int home_pos[2]; //no directuionality b/c we know always pointing in the positive direction
//just x and y coordinates {[x][y]}
int cubesCollected; //keeps track of how many cubes have been collected
const int sideLength = 200; //total side length of track in cm


//for pinging:
int cmFront, cmBack, cmFrontLeft, cmBackLeft, cmFrontRight, cmBackRight; //variables hold distances of ultrasonic sensors
int frontrightDuration, backrightDuration, frontleftDuration, backleftDuration, frontDuration, backDuration; //used to send a length of pings out

const int delayTime = 10; //milisecond time

//for checking cube:
const int NOFIELD = 0; //have to change this


int numberOfPasses; //keeps track of how many times we've driven in the y-direction


//array to hold distances obtained from this baord and sent to board 1
int valuesToSend[7];




void setup() {


  ///////////////////////
  //setting up ISR
  //////////////////////
  pinMode(ISRPin, OUTPUT);
  // attachInterrupt(digitalPinTOInterrupt(ISRPin), CheckCube(), RISING); //setting up ISR from LOW to HIGH on ISRPin



  pauseHere = true;

  /////////////////////////////////////////////////
  //initializing navigational functions and variables
  cubesCollected = 0;
  numberOfPasses = 0;
  //initPos(); //initialze the home position only when the robot starts each round
  current_pos[2] = 0; //sets the direction to positive y orientation
  // TURN 180 FUNCTION SHOULD FLIP THIS VALUE TO 1
  //TO INDICATE TRAVELLING IN THE NEGATIVE DIRECTION
  //cubeFound = false;
  /////////////////////////////////////////////////////




  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);



  // set up ultrasonic
  pinMode(ci_Front_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Front_Ultrasonic_Data, INPUT);

  pinMode(ci_Back_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Back_Ultrasonic_Data, INPUT);

  pinMode(ci_FrontLeft_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_FrontLeft_Ultrasonic_Data, INPUT);

  pinMode(ci_BackLeft_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_BackLeft_Ultrasonic_Data, INPUT);

  pinMode(ci_FrontRight_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_BackRight_Ultrasonic_Data, INPUT);

  pinMode(ci_BackRight_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_BackRight_Ultrasonic_Data, INPUT);

  // set up drive motors, need to reinitialize names
  pinMode(ci_FrontRight_Motor, OUTPUT);
  servo_FrontRightMotor.attach(ci_FrontRight_Motor);
  pinMode(ci_FrontLeft_Motor, OUTPUT);
  servo_FrontLeftMotor.attach(ci_FrontLeft_Motor);
  pinMode(ci_BackRight_Motor, OUTPUT);
  servo_BackRightMotor.attach(ci_BackRight_Motor);
  pinMode(ci_BackRight_Motor, OUTPUT);
  servo_BackLeftMotor.attach(ci_BackLeft_Motor);


  ///////////////////////////////////////////////////////
  // setting up grip servos
  left_grip_servo.attach(4); //the pins here are arbitrary and can be changed
  right_grip_servo.attach(5);

  ///////////////////////////////////////////////////////  ///////////////////////////////////////////////////////  ///////////////////////////////////////////////////////
  //NOTE FOR WHOEEVR SETS UP TEH ARM SERVOS:
  //THE PINS NEED TO BE SET FOR EACH SERVO LINK

  // set up arm motors
  // pinMode(ci_Base_Arm_Motor, OUTPUT);
  // servo_ArmMotor.attach(ci_Base_Arm_Motor);
  //pinMode(ci_Top_Arm_Motor, OUTPUT);
  //servo_ArmMotor.attach(ci_Top_Arm_Motor);

  //pinMode(ci_Grip_Motor, OUTPUT);
  // servo_GripMotor.attach(ci_Grip_Motor);
  //servo_GripMotor.write(ci_Grip_Motor_Zero);

  // set up motor enable switch
  //pinMode(ci_Motor_Enable_Switch, INPUT);
  //digitalWrite(ci_Motor_Enable_Switch, HIGH); //pullup resistor

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_FrontLeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_FrontLeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_FrontRightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_FrontRightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_BackLeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_BackLeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_BackRightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_BackRightMotor.setReversed(false);  // adjust for positive count when moving forward

  //encoder_GripMotor.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);

  // read saved values from EEPROM
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //EEPROM NEEDS TO BE SORTED OUT ASAP SO WE CAN CALIBRATE THE 2 IR SENSORS
  /*
    b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
    <<<<<<< HEAD
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
    ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
    b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
    ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
  */
  /*
     b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
     ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
     b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
     b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
     ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
     b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
     b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
     ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
     b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
     b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
     ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
     b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
     b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
     ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
     b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
     b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
     ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
     b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
     b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
     ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
     b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
     b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
     ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
  */




  ///////////////////////////////////////////////
  //ADDING THIS TO SEE IF THE CODE IS BEING KEPT IN THE SETUP FUNCTION
  Serial.println("code reaching this point");

}

void loop()
{




  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (ci_Mode_Button) //chanmged from: CharliePlexM::ui_Btn

  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  //bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  // modes
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.  -
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.      - Won't need
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level. - Won't need
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight. - Might need
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        //readLineTrackers();
        //Ping();
        //servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        //servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        //servo_ArmMotor.write(ci_Arm_Servo_Retracted);
        //servo_GripMotor.writeMicroseconds(ci_Grip_Motor_Stop);
        //encoder_LeftMotor.zero();
        //encoder_RightMotor.zero();
        //encoder_GripMotor.zero();
        ui_Mode_Indicator_Index = 0;
        Serial.print("light: ");
        Serial.println(analogRead(A3));
        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          //not declared in this scope
          //readLineTrackers();

#ifdef DEBUG_ENCODERS
          ul_Front_Left_Motor_Position = encoder_FrontLeftMotor.getPosition();
          ul_Front_Right_Motor_Position = encoder_FrontRightMotor.getPosition();
          ul_Back_Left_Motor_Position = encoder_BackLeftMotor.getPosition();
          ul_Back_Right_Motor_Position = encoder_BackRightMotor.getPosition();
          ul_Grip_Motor_Position = encoder_GripMotor.getPosition();

          Serial.print("Encoders FL: ");
          Serial.print(encoder_FrontLeftMotor.getPosition());
          Serial.print(", FR: ");
          Serial.print(encoder_FrontRightMotor.getPosition());
          Serial.print(", BR: ");
          Serial.print(encoder_BackRightMotor.getPosition());
          Serial.print(", BL: ");
          Serial.print(encoder_BackLeftMotor.getPosition());
          Serial.print(", G: ");
          Serial.println(ul_Grip_Motor_Position, DEC);
#endif

          // set motor speeds
          ui_Front_Left_Motor_Speed = constrain(ui_Motors_Speed - ui_Front_Left_Motor_Offset, 1600, 2100);
          ui_Front_Right_Motor_Speed = constrain(ui_Motors_Speed - ui_Front_Right_Motor_Offset, 1600, 2100);
          ui_Back_Left_Motor_Speed = constrain(ui_Motors_Speed - ui_Back_Left_Motor_Offset, 1600, 2100);
          ui_Back_Right_Motor_Speed = constrain(ui_Motors_Speed - ui_Back_Right_Motor_Offset, 1600, 2100);


          /***************************************************************************************
             Add line tracking code here.
             Adjust motor speed according to information from line tracking sensors and
             possibly encoder counts.
             Line tracking code added Jan 19, 2016 by Julian Zane
            /*************************************************************************************/


          pingFront();
          pingBack();
          pingFrontLeft();
          pingBackLeft();
          pingFrontRight();
          pingBackRight();

          //setting array values to their correct distance values in cm
          //these are what we send to board 1
          valuesToSend[0] = cmFront;
          valuesToSend[1] = cmBack;
          valuesToSend[2] = cmFrontLeft;
          valuesToSend[3] = cmBackLeft;
          valuesToSend[4] = cmFrontRight;
          valuesToSend[5] = cmBackRight;
          valuesToSend[6] = -1; //indicates the very end of the array
          //sending the values to baord 1
          //they get written to the serial buffer of board 1
          for (int n = 0; n < 7; n++)
          {
            Serial.write(valuesToSend[n]);
          }

          //loop only pings ultrasonic data and sends it to the board





#ifdef DEBUG_MOTORS
          Serial.print("Motors: Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(" , Front Left = ");
          Serial.print(ui_Front_Left_Motor_Speed);
          Serial.print(" . Front Right = ");
          Serial.println(ui_Front_Right_Motor_Speed);
          Serial.print(" . Back Right = ");
          Serial.println(ui_Back_Right_Motor_Speed);
          Serial.print(" . Back Left = ");
          Serial.println(ui_Back_Left_Motor_Speed);
#endif
          // ui_Mode_Indicator_Index = 1; // remember to chage this back to 1 for logiacal flow
          which_case = 0;
        }
        break;
      }






    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //NOTE FOR WHOEVER LOOKS INTO CALIBRATING MOTORS:
    //THE CODE FROM LAB04 (HERE) NEEDS TO BE MODIFIED TO USE 4 MOTORS
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    case 4:    //Calibrate motor straightness after 3 seconds.
      {
        /*
          if (bt_3_S_Time_Up)
          {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            ul_Calibration_Time = millis();
            servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
          }
          else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
          {
            servo_FrontLeftMotor.writeMicroseconds(ci_Front_Left_Motor_Stop);
            servo_FrontRightMotor.writeMicroseconds(ci_Front_Right_Motor_Stop);
            servo_BackLeftMotor.writeMicroseconds(ci_Back_Left_Motor_Stop);
            servo_BackRightMotor.writeMicroseconds(ci_Back_Right_Motor_Stop);
            ul_Front_Left_Motor_Position = encoder_FrontLeftMotor.getRawPosition();
            ul_Front_Right_Motor_Position = encoder_FrontRightMotor.getRawPosition();
            ul_Back_Left_Motor_Position = encoder_BackLeftMotor.getRawPosition();
            ul_Back_Right_Motor_Position = encoder_BackRightMotor.getRawPosition();
            if (ul_Left_Motor_Position > ul_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = (ul_Left_Motor_Position - ul_Right_Motor_Position) / 3.43;
              ui_Left_Motor_Offset = 0;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = 0;
              ui_Left_Motor_Offset = (ul_Right_Motor_Position - ul_Left_Motor_Position) / 3.43;
            }
          #ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Right = ");
            Serial.print(ui_Right_Motor_Offset);
            Serial.print(", Left = ");
            Serial.println(ui_Left_Motor_Offset);
          #endif
            EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));
            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
          #ifdef DEBUG_ENCODERS
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
          #endif
          ui_Mode_Indicator_Index = 4;
          }
          break;
        */
      }

    case 5:    //Light Sensor mode
      {
        /*
           experimenting if using false zeros will keep encoders working when control is passed to case 5 code
           this means not using encoder.zero()
           list of variables used in case 5:
           start_case5_position
          //encoder_RightMotor.zero(); //zeros the tick count of right encoder
          delay(100);
          unsigned long start_case5_position = encoder_RightMotor.getRawPosition; //start_case5_position holds current encoder pos
          /*
          while loop to hold code here to make full left turn
          holds until right encoder reads
          looks at relative position between current pos and starting pos (false zero)
          while ((encoder_RightMotor.getRawPosition() - start_case5_position) <= 1.37)
          {
          servo_LeftMotor.writeMicroseconds(1600); //left turn
          servo_RightMotor.writeMicroseconds(200);
          }
          //if here, now pointing left
          servo_LeftMotor.writeMicroseconds(200); //stop
          servo_RightMotor.writeMicroseconds(200);
          which_case = 1;//set to 1 so next time breaks out of case 1, goes into case 7
          ui_Robot_State_Index = 1;  //when breaks from this case, it will go back into 1
          break;
        */
      }








    case 6:
    /*
      //case 6 turns left at end of course, locates the platform and drops it off
      {
        ul_Right_Motor_Position = 0; //set right encoder to zero
        //might have to change the way I used ul_Right_Motor_Position
        while (ul_Right_Motor_Position <= x_degrees_turn_position)
        {
          //turn left until right encoder registers 90 degree turn
          // turnLeft();
          ul_Right_Motor_Position = encoder_RightMotor.getPosition(); //this should be cumulative
        }
        //now generally facing platform
        //stop_motors(); //stops motors from spinning contimuously
        while ((ul_Echo_Time / 58) >= arm_target_length)
        { //while driving towards target platform
          Ping();
          servo_LeftMotor.writeMicroseconds(1600); //driving straight
          servo_RightMotor.writeMicroseconds(1600);
        }//end while
        //  stop_motors();
        //extend arm our towards platform
        for (int pos = 60; pos < 121; pos++)
        {
          servo_ArmMotor.write(pos);
          delay(10);
        }
        //let go of grip motor slowly
        for (int pos = 40; pos <= 180; pos++)
        {
          servo_ArmMotor.write(pos);
          delay(10);
        }
        break;
      }//end case 5
      /*
       case 7 starts when bot is on the yellow line, we know the target is somewhere in front of us
      start by moving bot to the extreme right edge
    */
    case 7:
      {

        /*
            left_turn_while_scanning();
          servo_ArmMotor.write(ci_Arm_Servo_Retracted);
          delay(1000);
          servo_GripMotor.write(ci_Grip_Motor_Open);
          delay(1000);
          for (int ArmMotorAngle = ci_Arm_Servo_Retracted; ArmMotorAngle < 120; ArmMotorAngle = ArmMotorAngle + 1)
          {
          servo_ArmMotor.write(ArmMotorAngle);
          delay(100);
          }
          servo_ArmMotor.write(ci_Arm_Servo_Extended);
          delay(1000);
          servo_GripMotor.write(ci_Grip_Motor_Closed);
          delay(1000);
          for (int ArmMotorAngle = ci_Arm_Servo_Extended; ArmMotorAngle > 55; ArmMotorAngle = ArmMotorAngle - 1)
          {
          servo_ArmMotor.write(ArmMotorAngle);
          delay(100);
          }
        */
        /*
                //start by moving to the right
                encoder_RightMotor.zero();
                while (encoder_RightMotor.getRawPosition() >= -0.15) //arbitrary value to get robot to turn right to some value
                {
                  servo_LeftMotor.writeMicroseconds(200); //right turn
                  servo_RightMotor.writeMicroseconds(1350);
                }
                servo_LeftMotor.writeMicroseconds(200); //full stop
                servo_RightMotor.writeMicroseconds(200);
                //now robot in "full right" position
                     extend the arm a little so the light sensor is at the same height as the target
                  initially using angle of 60 MAY HAVE TO CHANGE THIS
                for (int ArmMotorAngle = ci_Arm_Servo_Retracted; ArmMotorAngle < 60; ArmMotorAngle++)
                {
                  servo_ArmMotor.write(ArmMotorAngle);
                  delay(10);
                }
                //arm is half extended and ready to sweep for light sensor values
                encoder_RightMotor.zero(); //zero encoder again
                light_sensor_data = analogRead(A3); //read value from light sensor, store it in global variable
                while (encoder_RightMotor.getRawPosition() <= 0.3) //arbitrary value to get robot to turn to full left position
                {
                  current_position = encoder_RightMotor.getRawPosition(); //sets current_position as the current encoder 'tick reading'
                  while (encoder_RightMotor.getRawPosition() - current_position <= 0.1) //moves the bot a little bit to the left each time
                  {
                    servo_LeftMotor.writeMicroseconds(1000); //full speed left
                    servo_RightMotor.writeMicroseconds(2000);
                  }
                  //stop motors to take another reading of light sensor and compare it with the current max value
                  servo_LeftMotor.writeMicroseconds(200); //full stop
                  servo_RightMotor.writeMicroseconds(200);
                  //bot has rotated a little bit to the left, corresponding to a number of encoder ticks
                  //this 'if statement' finds at what encoder position corresponds to a maximum light sensor reading (ie: on target)
                  if (analogRead(A3) < light_sensor_data) //if current light sensor reading is greater than the
                  {
                    max_light_position = encoder_RightMotor.getRawPosition(); //stores this encoder position
                  }
                }
                //here, robot is in full left position, with encoder values quite high, and the position of
                //max light sensor value stored in 'max_light_position'
                //turn right until the encoder is set to max_light_position
                //this corresponds to the arm beingon target
                while (encoder_RightMotor.getRawPosition() >= max_light_position) //this is assuming the encoders decrease when moving in reverse
                {
                  servo_LeftMotor.writeMicroseconds(2000); //full speed right
                  servo_RightMotor.writeMicroseconds(1000);
                }
                servo_LeftMotor.writeMicroseconds(200); //full stop
                servo_RightMotor.writeMicroseconds(200);
                //now robot is pointing at the target
                /*
                     JUSTIN'S CODE FOR GRABBING THE TARGET, will have to re-write for top and base arms (will need to build then test)
                servo_ArmMotor.write(ci_Arm_Servo_Retracted);
                delay(1000);
                servo_GripMotor.write(ci_Grip_Motor_Open);
                delay(1000);
                for (int ArmMotorAngle = ci_Arm_Servo_Retracted; ArmMotorAngle < 120; ArmMotorAngle = ArmMotorAngle + 1)
                {
                  servo_ArmMotor.write(ArmMotorAngle);
                  delay(10);
                }
                servo_ArmMotor.write(ci_Arm_Servo_Extended);
                delay(1000);
                servo_GripMotor.write(ci_Grip_Motor_Closed);
                delay(1000);
                for (int ArmMotorAngle = ci_Arm_Servo_Extended; ArmMotorAngle > 55; ArmMotorAngle = ArmMotorAngle - 1)
                {
                  servo_ArmMotor.write(ArmMotorAngle);
                  delay(20);
                }
                //now we have the target in the grip
                //then we have to back up, turn to the right, then bump back into case 1
                //servo_LeftMotor.writeMicroseconds(1000); //full reverse
                //servo_RightMotor.writeMicroseconds(1000);
                /*
                     NOTE: MIGHT NOT BE REVERSING STRAIGHT, TO SOLVE THIS WE COULD
                  RECORD ENCODER POSITION WHEN WE INITIALLY GET UP TO THE LINE, THEN USE FALSE ZEROS INSTEAD OF ZEROING THE ENCODERS A LOT
                  FIRST MAKE SURE WE CAN GET THE ABOVE CODE TO WORK BEFORE WE START USING FALSE ZEROS
                encoder_RightMotor.zero(); //zero encoders again
                /*
                     reverse a small amount
                while (encoder_RightMotor.getRawPosition() <= 1.5) //NEED TO CHANGE 1.5 TO A SUITABLE VALUE
                {
                  servo_LeftMotor.writeMicroseconds(1000); //full speed reverse
                  servo_RightMotor.writeMicroseconds(1000);
                }
                encoder_RightMotor.zero(); //zero encoders again
                /*
                     turn right 90 degrees
                  use the same encoder value from above (when we turned left 90 degrees
                  but it will have to be the negative of that value b/c right motor is reversing)
                while (encoder_RightMotor.getRawPosition() >= -1.5) //NEED TO CHANGE 1.5 TO A SUITABLE VALUE
                {
                  servo_LeftMotor.writeMicroseconds(1000); //full speed right
                  servo_RightMotor.writeMicroseconds(2000);
                }
                /*
                     here we want to be pointed to the left of the track
                   sweeping from left to right until the right line tracking sensor registers light
                   then we pop back into case 1 to follow the path until the drop off point
                encoder_RightMotor.zero();
                while (encoder_RightMotor.getRawPosition() <= 0.2) //MAY NEED TO CHANGE THIS VALUE
                {
                  servo_LeftMotor.writeMicroseconds(1000); //full speed right
                  servo_RightMotor.writeMicroseconds(2000);
                }
                servo_LeftMotor.writeMicroseconds(200); //full stop
                servo_RightMotor.writeMicroseconds(200);
                //read line trackers
                readLineTrackers();
                /*
                  holds code here until the right line tracker reads light
                  sweeps bot from left to right searching for light line
                while (!(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)))
                {
                  readLineTrackers();
                  encoder_RightMotor.zero(); //zero encoder again
                  current_position = encoder_RightMotor.getRawPosition();
                  while (encoder_RightMotor.getRawPosition() - current_position <= 0.05) //moves the bot a little bit to the left each time
                  {
                    servo_LeftMotor.writeMicroseconds(2000); //full speed right
                    servo_RightMotor.writeMicroseconds(1000);
                  }
                  //stop motors to take another reading of line trackers
                  servo_LeftMotor.writeMicroseconds(200); //full stop
                  servo_RightMotor.writeMicroseconds(200);
                }
                //breaks into case 1 again to drive the rest of the course
                ui_Robot_State_Index = 1;
                break;
        */
      }




  }//end switch

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY
    Serial.print("Mode: ");
    Serial.println(ui_Robot_State_Index, DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;

    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
}

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led

  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

#ifdef DEBUG_LINE_TRACKERS
Serial.print("Trackers: Left = ");
Serial.print(ui_Left_Line_Tracker_Data, DEC);
Serial.print(", Middle = ");
Serial.print(ui_Middle_Line_Tracker_Data, DEC);
Serial.print(", Right = ");
Serial.println(ui_Right_Line_Tracker_Data, DEC);
#endif



//*****Driving Functions*****

void forward(int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_FrontRightMotor.writeMicroseconds(1500 - speed); //forward
  servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackRightMotor.writeMicroseconds(1500 - speed); //forward

  //////////////////////////////////
  //added from searchForCubes()
  //THIS CODE ALLOWS THE ROBOT TO TRAVEL IN A STRAIGHT LINE
  //////////////////////////////////////////////////////////////////////
  //NOTE: MIGHT HAVE TO USE PINGFORWARD HALFWAY THROUGH OUR INCREASE OF Y COORDINATE
  //////////////////////////////////////////////////////////////////////

  //if travelling in positive y-direction
  if (current_pos[2] == 0)
  {
    pingBack();
    pingFrontLeft();
    current_pos[1] = cmBack; //update y coordinate (might not even need this)
    current_pos[0] = cmFrontLeft; //updates current x-coordinate

    //veerLeft() and veerRight() keep us driving relatively straight in y-direction
    //brings robot towards left wall if drifting right
    if ( current_pos[0] > ((10 * numberOfPasses) + 3)) //comparative value is standard robot width * number of passes
    {
      veerLeft(150, 10); //new function to steer slightly to the left
      //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER LEFT FOR A LITTLE AMOUNT
      //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
      //TO THIS PART OF THE CODE
    }

    //brings robot away from left wall if drifting left
    if ( current_pos[0] < ((10 * numberOfPasses)) - 3)
    {
      veerRight(150, 10); //new function to steer slightly to the left
      //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER RIGHT FOR A LITTLE AMOUNT
      //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
      //TO THIS PART OF THE CODE
    }
  }//end if(current_pos[2] == 0)

  ///////////////////////////////////////////////////////////////////
  //HAVE TO REWRITE ALL THE ABOVE CODE FOR WHEN WE TRAVEL IN NEGATIVE Y-DIRECTION
  ///////////////////////////////////////////////////////////////////

  //if travelling in negative y-direction
  if (current_pos[2] == 1)
  {
    pingBack();
    pingFrontRight();
    current_pos[1] = cmBack; //update y coordinate (might not even need this)
    current_pos[0] = cmFrontRight + 20; //the added value accounts for width of robot MIGHT NOT NEED TO ACCOUTN FOR ROBOT WIDTH

    //veerLeft() and veerRight() keep us driving relatively straight in y-direction
    //robot drifting left away from wall
    if ( current_pos[0] > (10 * numberOfPasses))
    {
      //FUNCTION NOT WRITTEN YET, AS OF MARCH 27, 2016
      veerRight(150, 10); //new function to steer slightly to the left
      //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER LEFT FOR A LITTLE AMOUNT
      //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
      //TO THIS PART OF THE CODE
    }

    //brings drifting towards wall
    if ( current_pos[0] < (10 * numberOfPasses))
    {
      veerLeft(150, 10); //new function to steer slightly to the left
      //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER RIGHT FOR A LITTLE AMOUNT
      //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
      //TO THIS PART OF THE CODE
    }
  }//end if(current_pos[2] == 1)


}//END FORWARD()

void reverse(int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_FrontRightMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackRightMotor.writeMicroseconds(1500 - speed); //reverse
}

void moveLeft(int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_FrontRightMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackRightMotor.writeMicroseconds(1500 - speed); //reverse
}

void moveRight(int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_FrontRightMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackRightMotor.writeMicroseconds(1500 + speed); //forward
}

void rotateClockwise(int speed, int angle) {
  //change the numbers accordingly
  encoder_FrontRightMotor.zero();
  //MIGHT HAVE TO AVERAGE THE 4 ENCODER VALUES AND COMPARE THEM TO ANGLE PASSED
  while (encoder_FrontRightMotor.getRawPosition() > -(9 * angle)) //9 encoder ticks per degree
  {
    Serial.print("right encoder:");
    Serial.println(encoder_FrontRightMotor.getRawPosition());
    servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
    servo_FrontRightMotor.writeMicroseconds(1500 + speed); //forward
    servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
    servo_BackRightMotor.writeMicroseconds(1500 + speed); //forward
  }



}

void forwardLeftDiagonal(int speed) {
  servo_FrontRightMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
}

void forwardRightDiagonal (int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackRightMotor.writeMicroseconds(1500 + speed); //forward
}

void reverseRightDiagonal(int speed) {
  servo_FrontRightMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500 - speed); //reverse
}

void reverseLeftDiagonal(int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackRightMotor.writeMicroseconds(1500 - speed); //reverse
}

void stop_motors() {
  servo_FrontLeftMotor.writeMicroseconds(1500);
  servo_FrontRightMotor.writeMicroseconds(1500);
  servo_BackLeftMotor.writeMicroseconds(1500);
  servo_BackRightMotor.writeMicroseconds(1500);
}

//*****Pinging Functions*****

//allow the user to ping the front ultrasonic sensor
void pingFront() { //took "int delayTime" out of argument list
  digitalWrite(ci_Front_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_Front_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people

  frontDuration = pulseIn(ci_Front_Ultrasonic_Data, HIGH, 10000);

  cmFront = microsecondsToCentimeters(frontDuration);
  Serial.print("Front distance = ");
  Serial.print(cmFront);
  Serial.print("cm  ");
  //Serial.println();
}


void pingBack() {

  digitalWrite(ci_Back_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_Back_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people
  backDuration = pulseIn(ci_Back_Ultrasonic_Data, HIGH, 10000);

  cmBack = microsecondsToCentimeters(backDuration);
  Serial.print("Back distance = ");
  Serial.print(cmBack);
  Serial.print("cm  ");
  //Serial.println();

}

void pingFrontLeft() {

  digitalWrite(ci_FrontLeft_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_FrontLeft_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people

  frontleftDuration = pulseIn(ci_FrontLeft_Ultrasonic_Data, HIGH, 10000);

  cmFrontLeft = microsecondsToCentimeters(frontleftDuration);

  Serial.print("Left distance = ");
  Serial.print(cmFrontLeft);
  Serial.print("cm  ");
  //Serial.println();

}


void pingBackLeft() {

  digitalWrite(ci_BackLeft_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_BackLeft_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people

  backleftDuration = pulseIn(ci_BackLeft_Ultrasonic_Data, HIGH, 10000);

  cmBackLeft = microsecondsToCentimeters(backleftDuration);

  Serial.print("Left distance = ");
  Serial.print(cmBackLeft);
  Serial.print("cm  ");
  //Serial.println();

}




void pingFrontRight() {
  digitalWrite(ci_FrontRight_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_FrontRight_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people

  frontrightDuration = pulseIn(ci_FrontRight_Ultrasonic_Data, HIGH, 10000);

  cmFrontRight = microsecondsToCentimeters(frontrightDuration);
  Serial.print("Right distance = ");
  Serial.print(cmFrontRight);
  Serial.print("cm  ");
  Serial.println();
}



void pingBackRight() {
  digitalWrite(ci_BackRight_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_BackRight_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people

  backrightDuration = pulseIn(ci_BackRight_Ultrasonic_Data, HIGH, 10000);

  cmBackRight = microsecondsToCentimeters(backrightDuration);
  Serial.print("Right distance = ");
  Serial.print(cmBackRight);
  Serial.print("cm  ");
  Serial.println();
}



long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

//used to convert microseconds (the data inputted) into centimeters so distances can be better gauged when pinging



//////////////////////////////////////
//MARCH 20, 2016
//JULIAN ZANE
//function takes open and closed positions of grip
//closes the grip
//////////////////////////////////////
void grip_close(int open_pos, int closed_pos)
{
  for ( int n = open_pos; n < closed_pos; n++)
  {
    right_grip_servo.write(n);
    left_grip_servo.write(180 - n); //180-n makes this one count move backwards
    delay(15);
    Serial.println(right_grip_servo.read());
  }
}




//////////////////////////////////////
//MARCH 20, 2016
//JULIAN ZANE
//function takes open and closed positions of grip
//opens the grip
//////////////////////////////////////
void grip_open(int open_pos, int closed_pos)
{
  for ( int n = closed_pos; n > open_pos; n--)
  {
    right_grip_servo.write(n);
    left_grip_servo.write(180 - n); //180-n makes this one count move backwards
    delay(15);
    //Serial.println(n);
  }
}







//////////////////////////////////*
//JULIAN ZANE
//MARCH 19, 2016

//function initialzes the home position
//of the robot for future navigational use

//*/////////////////////////////////

void initPos()
{
  //ping left and read 10 times to let values stabilize
  for (int i = 0; i < 10; i++)
  {
    pingFrontLeft();
    pingFrontRight();
    home_pos[0] = cmFrontLeft; //sets x coordinate
    home_pos[1] = cmFrontRight; //sets y coordinate
  }
} //end function




////////////////////////////////////*
//JULIAN ZANE
//MARCH 19, 2016

//function takes care of driving around
//and trying to find a cube
//DOES NOT CONCERN ITSELF IF THE CUBE
//IS REAL OR NOT JSUT YET (SEPERATE FUNCTION)
//*////////////////////////////////////
void searchForCube()
{
  //while we're within our side of the course (our side of neutral zone)
  while (current_pos[0] < ((sideLength / 2) - 2))
  {
    //checks for wall in front of robot
    pingFront();
    while (cmFront > 25) //arbitrary distance
    {
      forward(100);


    } //end while

    //if here, robot needs to turn around;
    rotateClockwise(100, 180);//function should have a case for 180, where it flips
    //the directionality register in "current_pos"

    //if robot is now facing positive y-dir
    if (current_pos[2] == 0)
    {
      pingFrontLeft();
      while (cmFrontLeft < (current_pos[0]) + 10) //gets the robvot to move right one robot width
        moveRight(100);
    }
    stop_motors(); //stop robot


    if (current_pos[2] == 1)
    {
      pingFrontRight();
      while (cmFrontRight < (current_pos[0]) + 10) //gets the robvot to move right one robot width
        moveLeft(100);
    }
    stop_motors();

    /////////////////////////////////////////////
    //CRITICAL TO OPERATION, DO NOT DELETE!!!!!
    numberOfPasses = numberOfPasses + 1; //increments the number of passes variable so we keep a standard width from the wall


  }//end while

}//end function



//////////////////////////////////////////////////////////////////////////
//TO DEBUG, SERIAL PRINT THE ARRAY "current_pos" TO MAKE SURE ITS VALUES
//ARE BEING UPDATED IN REAL-TIME
//
//
//
/////////////////////////////////////////////////////////////////////////////







//Checking if the Tesseract is Real or Not using the hall effect sensor

void checkCube() {
  int raw = analogRead(0);  //**will need to change the pin number
  //Hall Effect range might be from 0-1024, will have to test this
  //and see the values that the

  // If needed to debug/see the readings uncomment this part
  //  Serial.print("Raw Flux Reading: ");
  //  Serial.println(raw);

  long magneticFlux = raw - NOFIELD;
  // Make a global variable called "NOFIELD" and make it the value that the
  // hall effect detects when there is no field

  if (magneticFlux == 0) {
    // Input code to dipose of the tesseract
    // Possibly call the pickup function and drive to home position and dispose?
    // void PickUpTesseract();

  }

  else {
    // Return to home position and callthe indexing function
    // void PickUpTesseract();
    // void goHome();
  }
}//end function

//rotate counterclock wise with the speed and angle
void rotateCounterClockwise(int speedy, int angle)
{
  //change and test numbers accordingly

  // zero the front right motor only, then count the number of ticks
  encoder_FrontRightMotor.zero();

  //find the difference of the raw position and the zero position,
  // will need to test for values and will need to change the arbitary one
  while ((encoder_FrontRightMotor.getRawPosition()) <= angle) {
    servo_FrontLeftMotor.writeMicroseconds(1500 - speedy); //reverse
    servo_FrontRightMotor.writeMicroseconds(1500 + speedy); //forward
    servo_BackLeftMotor.writeMicroseconds(1500 - speedy); //reverse
    servo_BackRightMotor.writeMicroseconds(1500 + speedy); //forward
  }

  // if the robot rotates 180 degrees, change the directionality register to
  // the opposite of whatever it currently is
  if (angle == 180) {
    if (current_pos[2] == 1) {
      current_pos[2] = 0;
    }
    else if (current_pos[2] == 0) {
      current_pos[2] = 1;
    }
  }
}
//}//ADDED THIS TO END CHECKCUBE FUNCTION





void goHome()
{
  last_known_cube_pos[0] = current_pos[0];
  last_known_cube_pos[1] = current_pos[1];
  last_known_cube_pos[2] = current_pos[2];

  // records our current position to last known so we know where to start searching


  if (current_pos[2] == 1) //facing negative y
  {

    while (current_pos[0] > home_pos[0] + 20)  //x-dir-> leaving enough room to turn to face the side wall
    {
      moveRight(100);
      pingFrontRight(); //updates our x values
    }
    stop_motors();

    //now right along the side wall
    while (current_pos[1] > home_pos[1] + 20) { //y-dir -> leaving enough room to start indexing
      forward(100);
    }
    stop_motors(); //have to stop the motors after we move, or we'll continue driving in that direction forever,
    // else it's good practise

    rotateClockwise(100, 90); //rotates 90 degrees with speedy = 100
    stop_motors();
    orientForDropOff(); //orients the robot for indexing and dropping off cubes


    if (current_pos[2] == 0) //facing positive y
    {
      rotateCounterClockwise(100, 180); //turns to face negative y direction

      while (current_pos[0] > home_pos[0])// x - dir-> leave room for rotation
      {
        moveRight(100);
        pingFrontRight(); //updates position
      }
      stop_motors();

      //now right along the side wall
      while (current_pos[1] > home_pos[1] + 20) { //y-dir -> leaving enough room to start indexing
        forward(100);
      }
      stop_motors(); //have to stop the motors after we move, or we'll continue driving in that direction forever,
      // else it's good practise

      rotateClockwise(100, 90); //rotates 90 degrees with speedy = 100
      stop_motors();
      orientForDropOff();
    }
  }
}//end go home





////////////////////////////////////////////////*
//JULIAN ZANE
//MARCH 25, 2016

//FUNCTION STARTS WHEN BOT IS FACING THE SIDE WALL AND STOPPED
//RETURNS NOTHING
//TAKES NO ARGUMENTS
//EXITS INTO INDEXING FUNCTION WHEN LINE TRACKER READS THE FIRST DARK LINE

//*////////////////////////////////////////////////

void orientForDropOff()
{
  //readLineTrackers(); //taken from Lab04 code

  //CONDITION NEEDS TO CHANGE, BUT BASICALLY THE LOOP RUNS WHILE THE GRIP LINE TRACKER IS LIGHT
  //MIGHT HAVE TO CHANGE THIS TO A "LATCHED" GLOBAL VARIABLE B/C IT MIGHT OVERSHOOT THE TARGET


  //////////////////////////////////
  //pretty much this goes in the conditional part:
  //"(gripLinetrackerdata.analogRead() < dark_cutoff_value) || (PingLeft() < 5)"

  while (cmBack == 2) //second condition prevents overshoot entirely //useless crap as placeholder in while loop condition
  {
    moveLeft(100);
    pingFrontLeft(); //updates position
    //NOTE: runing this will change our current x and y positions (they will get reversed),
    //but this shouldn't really affect anything, as we're not using the x and y variables directly here
  }
  stop_motors();


  //////NOT WRITTEN YET
  //index(); //returns into the index code
  //function still needs to be written
}





////////////////////////////////////*
//JULIAN ZANE
//INITAL CREATION: MARCH 25, 2016

//ROBOT ENTERS FUNCTION WHEN IT DROPS OFF THE CUBE AT HOME AND RETURNS ARM TO "DRIVING POSITION"
//RETURNS NOTHING
//TAKES NO ARGUMENTS
//GENERAL PLAN:
//-->ROTATE TO MATCH DIRECTION OF DIRECTIONALITY REGISTER
//-->DRIVE IN X AND Y DIRECTIONS UNTIL CURRENT UPDATES MATCH LAST KNOWN POSITION

//*////////////////////////////////////

void backToLastKnownCube()
{
  //initially facing negative x-dir
  if (last_known_cube_pos[2] == 0) //if we were facing positive direction when we picked up the cube
  {
    rotateClockwise(100, 90); //90 degrees to face positive y-dir
    current_pos[2] = 0; //manually reset directionality register var to positive y-dir
    //now facing correct direction

    //moving in positive x-dir until our current position matches the last known cube pos x-dir
    while (current_pos[0] < last_known_cube_pos[0])
    {
      moveRight(100);
      pingFrontLeft();
      cmFrontLeft = current_pos[0]; //setting x-val to current left untrasonic reading
    }
    stop_motors(); //stop moving left
    //if here, now x coordinate is correct

    //now set the y-direction back to where we were when we picked up the cube (minus some value to get some overlap in searching pattern)
    while (current_pos[1] < (last_known_cube_pos[1] - 7))
    {
      forward(100); //function takes care of driving straight, makes use of veerLeft() and veerRight()
      pingBack();
      cmBack = current_pos[1];
    }
    stop_motors();

    //now we can call search for cube function again and resume out search
    searchForCube();

  } //end if

  else if (last_known_cube_pos[2] == 1) //if we were facing negative y-dir when we picked up the last cube
  {
    rotateCounterClockwise(100, 90); //rotate to negative y- dir

    //now we're facing in the correct direction


    current_pos[2] = 1; //manually reset directionality register var to positive y-dir

    //now set the y-direction back to where we were when we picked up the cube (minus some value to get some overlap in searching pattern)
    while (current_pos[1] < (last_known_cube_pos[1] + 7))
    {
      reverse(100); //function takes care of driving straight, makes use of veerLeft() and veerRight()
      pingFront();
      cmFront = current_pos[1];
    }
    stop_motors();

    //moving in positive x-dir until our current position matches the last known cube pos x-dir
    while (current_pos[0] < last_known_cube_pos[0])
    {
      moveLeft(100);
      pingFrontRight();
      cmFrontRight = current_pos[0]; //setting x-val to current left untrasonic reading
    }
    stop_motors(); //stop moving left
    //if here, now x coordinate is correct



    //now we can call search for cube function again and resume out search
    searchForCube();



  }//end else if


}//end backtolastknowncube()



///////////////////////////////////////////////////**
//JULIAN ZANE
//INITIAL CREATION: MARCH 25, 2016

//FUNCTION RETURNS NOTHING
//FUNCTION TAKES NO ARGUMENT
//DRIVES BACKWARDS TO THE EDGE OF THE TRACK AND TURNS AROUND, DUMPING THE CUBE OUTSIDE OF THE TRACK
//THEN RETURNING TO IT'S LAST KNOWN POSITION
//--> RESUSE CODE FOR THIS PART
//ROBOT ENTERS THIS FUNCTION STOPPED AND WITH A CUBE READY TO BE PICKED UP

//**////////////////////////////////////////////////////

void disposeCube()
{
  //pickUpCube(); //FUNCTION STILL NEEDS TO BE WRITTEN TO MOVE ARM MOVEMENTS

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MIGHT HAVE TO REVERSE A LITTLE BEFORE WE TURN
  //COME BACK TO THIS AND CHECK
  if (current_pos[2] == 0) //positive y-dir
  {
    rotateCounterClockwise(100, 180);//CCW to avoid hitting other cubes
    pingFront();
    while (cmFront > 25)
    {
      forward(200);
    }
    stop_motors();
    //Get_rid_of_cube(); //NEEDS TO BE WRITTEN
    //HARD CODE IN ARM MOVEMENTS TO EXTEND AND DROP CUBE FROM HEIGHT FOR DRAMATIC EFFECT
  }//end if

  else if (current_pos[2] == 1) //negative y-dir
  {
    rotateClockwise(100, 180);//CW to avoid hitting other cubes
    pingFront();
    while (cmFront > 25)
    {
      forward(200);
    }
    stop_motors();
    //Get_rid_of_cube(); //NEEDS TO BE WRITTEN
    //HARD CODE IN ARM MOVEMENTS TO EXTEND AND DROP CUBE FROM HEIGHT FOR DRAMATIC EFFECT
  }//end else if

}//end disposeCube()






//ADD SIMILAR CODE AS IN FOWARD() TO REVERSE() TO ALLOW THE ROBOT TO MOVE BACKWARD IN A "STRAIGHT" LINE






void veerRight(int speedy, int xDistance) {
  int slower = speedy - 50;
  int leftDistance = xDistance + 5; // may need to change the +3
  // for veerLeft, the right side motors are operating faster than the left side
  // this is done until we get to our set x distance away + a small value (2-3cm)
  // this is used to drive straight and correct the drift for the omniwheels
  pingFrontLeft();
  while (current_pos[0] < leftDistance) {
    servo_FrontLeftMotor.writeMicroseconds(1500 + speedy); // the difference in the veerRight and left
    servo_FrontRightMotor.writeMicroseconds(1500 - slower); // is the speed of the wheels and the
    servo_BackLeftMotor.writeMicroseconds(1500 + speedy); // distance of the left wall to the ultrasonic
    servo_BackRightMotor.writeMicroseconds(1500 - slower);
  }
}

void veerLeft(int speedy, int xDistance) {
  int slower = speedy - 50;
  int leftDistance = xDistance + 3; // may need to change the +3
  // for veerLeft, the right side motors are operating faster than the left side
  // this is done until we get to our set x distance away + a small value (2-3cm)
  // this is used to drive straight and correct the drift for the omniwheels
  pingFrontLeft();
  while (current_pos[0] < leftDistance) {
    servo_FrontLeftMotor.writeMicroseconds(1500 + slower);
    servo_FrontRightMotor.writeMicroseconds(1500 - speedy);
    servo_BackLeftMotor.writeMicroseconds(1500 + slower);
    servo_BackRightMotor.writeMicroseconds(1500 - speedy);
  }
}







//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//JULIAN'S NOTES:

//FUNCTION NEEDS TO BE MORE GENERAL:
//-->INSTEAD OF USING VARIABLES FROM OUR LAST KNOWN POSITION ARRAY
//TRY STORING OUR POSITION WHEN WE ENTER THIS FUNCTION INTO OUR
//CURRENT POSITION ARRAY, THEN MOVE LEFT OR RIGHT A CERTAIN SPEED
//WHILE USING A SCHMIDTT TRIGGER TO KEEP THE DISTANCE FROM THE FRONT OR
//BACK WALL CONSTANT
//--> TRY USING A FUNCTION ARGUMENT TO SET WHETHER WE MONITOR THE FRONT OR BACK ULTRASONIC

//March 27 2015
//Gamaliel Obinyan
//Added moveLeft() and moveRight(), a lot of caliberation still needs to be done

/*
  void moveLeft(int slidingSpeed, int horizontalDistance)
  //lastCubePosition is where we dropped off the very last cube
  //slidingSpeed is what we caliberate as the best speed move sideways
  leftDistance = horizontalDistance;
  //hor
  {
  pingLeft();
  while(lastCubePosition - 3 < leftDistance) //"-3" indicates the the next postion for a cub; we can measure the distance; wea can also change the greater than to less than based on the orientation
  {
    servo_FrontLeftMotor.writeMicroseconds(1500 - slidingSpeed);
    servo_FrontRightMotor.writeMicroseconds(1500 + slidingSpeed);
    servo_BackLeftMotor.writeMicroseconds(1500 + slidingSpeed);
    servo_BackRightMotor.writeMicroseconds(1500 - slidingSpeed);
  }
  }
  void moveRight(int slidingSpeed, int horizontalDistance)
  //lastCubePosition is where we dropped off the very last cube
  //slidingSpeed is what we caliberate as the best speed move sideways
  leftDistance = horizontalDistance;
  //hor
  {
  pingLeft();
  while(lastCubePosition - 3 < leftDistance) //"-3" indicates the the next postion for a cub; we can measure the distance; wea can also change the greater than to less than based on the orientation
  {
    servo_FrontLeftMotor.writeMicroseconds(1500 + slidingSpeed);
    servo_FrontRightMotor.writeMicroseconds(1500 - slidingSpeed);
    servo_BackLeftMotor.writeMicroseconds(1500 - slidingSpeed);
    servo_BackRightMotor.writeMicroseconds(1500 + slidingSpeed);
  }
  }
*/


