/* 
 *  RFID Indoor Robot navigation system
 *  
 *  Copyright (C) 2017, LeRoy Miller
 *  
 *  March 1, 2017 
 */

/* 
 * RCI522 (Mifare) Code lifted from
 * Changed slightly for use here
 * Creator：Dr.Leong   ( WWW.B2CQSHOP.COM )
 * Creation date：2011.09.19
 * Modified by： Eng. Robson (robson.eletronico@gmail.com)
 * Modified date： 2013.09.10
 * Modified： Translation from Chinese to English (by google)
 * Functional Description：Mifare1 Anti-collision find cards → → → Select card reader interface
 */

#include <SPI.h>
#include "BittyBot2.h"
#include "Keypad.h"
#include <Wire.h>
#include "compass.h"

 //RFID Setup
#define  uchar unsigned char
#define uint  unsigned int

//Maximum length of the array
#define MAX_LEN 16

/////////////////////////////////////////////////////////////////////
//set the pin
/////////////////////////////////////////////////////////////////////
const int chipSelectPin = 53;
const int NRSTPD = 5;

//MF522 Command word
#define PCD_IDLE              0x00               //NO action; Cancel the current command
#define PCD_AUTHENT           0x0E               //Authentication Key
#define PCD_RECEIVE           0x08               //Receive Data
#define PCD_TRANSMIT          0x04               //Transmit data
#define PCD_TRANSCEIVE        0x0C               //Transmit and receive data,
#define PCD_RESETPHASE        0x0F               //Reset
#define PCD_CALCCRC           0x03               //CRC Calculate

// Mifare_One card command word
# define PICC_REQIDL          0x26               // find the antenna area does not enter hibernation
# define PICC_REQALL          0x52               // find all the cards antenna area
# define PICC_ANTICOLL        0x93               // anti-collision
# define PICC_SElECTTAG       0x93               // election card
# define PICC_AUTHENT1A       0x60               // authentication key A
# define PICC_AUTHENT1B       0x61               // authentication key B
# define PICC_READ            0x30               // Read Block
# define PICC_WRITE           0xA0               // write block
# define PICC_DECREMENT       0xC0               // debit
# define PICC_INCREMENT       0xC1               // recharge
# define PICC_RESTORE         0xC2               // transfer block data to the buffer
# define PICC_TRANSFER        0xB0               // save the data in the buffer
# define PICC_HALT            0x50               // Sleep


//And MF522 The error code is returned when communication
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2


//------------------MFRC522 Register---------------
//Page 0:Command and Status
#define     Reserved00            0x00    
#define     CommandReg            0x01    
#define     CommIEnReg            0x02    
#define     DivlEnReg             0x03    
#define     CommIrqReg            0x04    
#define     DivIrqReg             0x05
#define     ErrorReg              0x06    
#define     Status1Reg            0x07    
#define     Status2Reg            0x08    
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F
//Page 1:Command     
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MifareReg             0x1C
#define     Reserved13            0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F
//Page 2:CFG    
#define     Reserved20            0x20  
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg            0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
//Page 3:TestRegister     
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39  
#define     TestDAC2Reg           0x3A   
#define     TestADCReg            0x3B   
#define     Reserved31            0x3C   
#define     Reserved32            0x3D   
#define     Reserved33            0x3E   
#define     Reserved34      0x3F
//-----------------------------------------------

//4 bytes card serial number, the first 5 bytes for the checksum byte
uchar serNum[5];

uchar  writeData[16]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100};  //Initialization 100 dollars
uchar  moneyConsume = 18 ;  //Consumption of 18 yuan
uchar  moneyAdd = 10 ;  //Recharge 10 yuan
//Sector A password, 16 sectors, each sector password 6Byte
 uchar sectorKeyA[16][16] = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                             {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                             //{0x19, 0x84, 0x07, 0x15, 0x76, 0x14},
                             {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                            };
 uchar sectorNewKeyA[16][16] = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xff,0x07,0x80,0x69, 0x19,0x84,0x07,0x15,0x76,0x14},
                                 //you can set another ket , such as  " 0x19, 0x84, 0x07, 0x15, 0x76, 0x14 "
                                 //{0x19, 0x84, 0x07, 0x15, 0x76, 0x14, 0xff,0x07,0x80,0x69, 0x19,0x84,0x07,0x15,0x76,0x14},
                                 // but when loop, please set the  sectorKeyA, the same key, so that RFID module can read the card
                                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xff,0x07,0x80,0x69, 0x19,0x33,0x07,0x15,0x34,0x14},
                               };
 uchar i,tmp, checksum1;
  uchar status;
        uchar str[MAX_LEN];
        uchar RC_size;
        uchar blockAddr;  //Selection operation block address 0 to 63
        String mynum = "";

//RFID setup end

int start; //RFID starting location

/* We could use the RFID tag code to set a location as well
 * See https://github.com/automation-technology-club/NFC-indoor-location-Mockup
 * mockup_nfc_tag_indoor_robot_location for example.
 * The example finds the grid location, so if you know that you
 * can by-pass that step.
 * For this sketch target is figured from the map, the target
 * variable is below.
 */

int row;
int col;

#define ROWS 3
#define COLS 5

//Map
//My Gird is 3 x 5 (0,0 to 2,4) - I am using the "Checksum" of the RFID tags
int codes[ROWS][COLS] = {
  {67,121,127,117,107},
  {137,147,157,103,97},
  {143,133,187,177,183},
};  //Map of NFC codes 67 is location 0,0 183 is location 2,4

int target; 

/*New function to turn robot about xx degrees - it's not perfect
 * (calibration helps sometimes, If you need to calibrate it
 * the robot needs to be manually rotated 360 degrees 5 or 6 times
 * before time runs out)
 * 
 * turn(xx) where xx is a number in degrees.
 * IE: 90 will turn the robot right about 90 degrees
 * a negitive number will turn the robot left
 * (left isn't working 100%) 
 * 
 * The new code is based off the HMC5883L Compass and the
 * Original/Official Arduino Robot library.
 * 
 */

int leftspeed = 50;
int rightspeed = 50;
int forwardSpeed = 75;

BittyBot bot(44,46,36,38,40,42); //Left Enable, Right Enable, Pin1 for Left, Pin2 for Left, Pin 1 for Right, Pin 2 for Right

/*
 * Mar 23, 2017 added manual entry for orientation, self orientation is still what I would like.
 * 
 * Robot appears to work facing "UP", "LEFT" and "RIGHT" - there is a unknown issue if it's facing "DOWN"
 *  
 */

//FaceDirection is used by the robot to track where it's front is facing and how it needs to turn
int faceDirection = -999; //1= UP, 2= right, 3=down, 4=left
int moveDirection; //used for calculating move
int moveflag = 0;

//Setup 3x4 matrix keypad for target location entry.
const byte keyROWS = 4; // number of rows
const byte keyCOLS = 3; // number of columns
char keys[keyROWS][keyCOLS] = {
{'1','2','3'},
{'4','5','6'},
{'7','8','9'},
{'*','0','#'}
};

byte rowPins[keyROWS] = {49,47,45,43};
byte colPins[keyCOLS] = {41,39,37};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, keyROWS, keyCOLS);

int targetrow = -999;
int targetcol = -999;
char key;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  bot.begin();
  Wire.begin();

//Calibration settings for Franklin, Ohio USA - These may or may not work for you 
//Added if auto calibration is needed.
 compass_x_offset = 96.07;
  compass_y_offset = 483.79;
  compass_z_offset = 570.42;
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;
  
  compass_init(2);
  
   pinMode(chipSelectPin,OUTPUT);     // Set as OUTPUT to connect it to the RFID /ENABLE pin 
   digitalWrite(chipSelectPin, LOW); // Activate the RFID reader
   pinMode(NRSTPD,OUTPUT);             // Set pin Not Reset and Power-down
   digitalWrite(NRSTPD, HIGH);
 Serial.write(17); //backlight ON
  MFRC522_Init(); 

  //Serial.println(target);
  Serial.print("BittyBot  RFID");
  Serial.write(13);
  Serial.print("Indoor Location");
  delay(1000);

//Manual Calibration
Serial.write(12);
Serial.print("Calibrate? ");
Serial.write(13);
Serial.print("1=Yes");
int tempAnswer = inputTarget();
if (tempAnswer == 1) { 
  compass_debug = 1;
  compass_offset_calibration(3); }

//Enter Target Location on keypad
while (targetrow < 0 || targetrow > ROWS-1) {
  Serial.write(12);
  Serial.print("Row? ");
  targetrow = inputTarget();
  if (targetrow > ROWS) {Serial.println("Out of Bounds");}
 }
 while (targetcol < 0 || targetcol > COLS-1) {
  Serial.write(12);
  Serial.print("Col? ");
  targetcol = inputTarget();
  if (targetcol > COLS) {Serial.println("Out of Bounds");}
 }

target = codes[targetrow][targetcol];
 Serial.write(12);
  Serial.print("Target: ");
  Serial.print(targetrow);
  Serial.print(",");
  Serial.println(targetcol);

  while (faceDirection < 1 || faceDirection > 4) {
Serial.write(12);
//1= UP, 2= right, 3=down, 4=left
Serial.print("1=U,2=R,3=D,4=L");
Serial.write(13);
Serial.print("Facing? ");
faceDirection = inputTarget(); //1= UP, 2= right, 3=down, 4=left
  if (faceDirection > COLS) {Serial.write(12); Serial.println("Out of Bounds");}
  }

  Serial.write(12);
  Serial.print("Start..");
  Serial.print("5..");
  delay(1000);
  Serial.print("4..");
  delay(1000);
  Serial.print("3..");
  delay(1000);
  Serial.print("2..");
  delay(1000);
  Serial.print("1..");
  delay(1000);
  Serial.write(12);
  Serial.print("Go Robot Go!");
  
  while(checksum1<=0) {
    readRFID(); 
  start = checksum1;
  }
  Serial.write(12); //Clear Display
  Serial.print("Start: ");
  Serial.println(start);
  Serial.write(13); //Form Feed
  Serial.print("Grid: ");
  findnumbers(start);
  bot.Speed(leftspeed, rightspeed);
  selfOrientation();
}

void loop() {

moveflag = 0;

  //Now we Move!
  calculatemove();
  checksum1 = 0;
 /* if (faceDirection == moveDirection) {
    bot.forward(100);
    moveflag = 1;
  }
  */
  //Facing UP
  if (faceDirection == 1 && moveDirection == 2 && moveflag == 0) {
    //Face UP Turn Right    
    //turnright();
    turn(90); //right
    bot.stop();
    faceDirection = 2;
    moveflag = 1;
  }
  if (faceDirection == 1 && moveDirection == 4 && moveflag == 0) {
    //Face UP Turn left
    //turnleft();
    turn(-90); //left
    bot.stop();
    faceDirection = 4;
    moveflag = 1;
  }
  if (faceDirection == 1 && moveDirection == 3 && moveflag == 0) {
    //Face UP Turn 180 degree to face DOWN
    turn(-180);
    bot.stop();
        //turnleft();
    //turnleft();
    faceDirection = 3;
    moveflag = 1;
  }
  //Facing Right
if (faceDirection == 2 && moveDirection == 1 && moveflag == 0) {
    //Face Right Turn to face UP    
    turn(-90);
    bot.stop();
    //turnleft();
    faceDirection = 1;
    moveflag = 1;
  }
  if (faceDirection == 2 && moveDirection == 4 && moveflag == 0) {
    //Face Right Turn to face left
    turn(-180);
    bot.stop();
    //turnleft();
    //turnleft();
    faceDirection = 4;
    moveflag = 1;
  }
  if (faceDirection == 2 && moveDirection == 3 && moveflag == 0) {
    //Face Right Turn to face DOWN
    turn(90);
    bot.stop();
    //turnright();
    faceDirection = 3;
    moveflag = 1;
  }
  //Facing Left
  if (faceDirection == 4 && moveDirection == 1 && moveflag == 0) {
    //Face left Turn to face UP    
    turn(90);
    bot.stop();
    //turnright();
    faceDirection = 1;
    moveflag = 1;
  }
  if (faceDirection == 4 && moveDirection == 2 && moveflag == 0) {
    //Face left Turn to face right
    turn(-180);
    bot.stop();
    //turnleft();
    //turnleft();
    faceDirection = 2;
    moveflag = 1;
  }
  if (faceDirection == 4 && moveDirection == 3 && moveflag == 0) {
    //Face left Turn to face DOWN
    turn(-90);
    bot.stop();
    //turnleft();
    faceDirection = 3;
    moveflag = 1;
  }
  //Face Down
  if (faceDirection == 3 && moveDirection == 1 && moveflag == 0) {
    //Face down Turn to face UP    
    turn(-180);
    bot.stop();
    //turnleft();
    //turnleft();
    faceDirection = 1;
    moveflag = 1;
  }
  if (faceDirection == 3 && moveDirection == 4 && moveflag == 0) {
    //Face down Turn to face left
    turn(90);
    bot.stop();
    //turnright();
    faceDirection = 4;
    moveflag = 1;
  }
  if (faceDirection == 3 && moveDirection == 2 && moveflag == 0) {
    //Face down Turn to face right
    turn(-90);
    bot.stop();
    //turnleft();
    faceDirection = 2;
    moveflag = 1;
  }
  while (checksum1 == 0) {
    readRFID();
    bot.Speed(forwardSpeed, forwardSpeed);
    bot.forward(100);
    while(bot.IsRunning()) {
      bot.update();
    }
  }
  //bot.stop();
  findnumbers(checksum1);
}

int inputTarget() {

key=0;
 while(!key)  // Check for a valid key.
  {
    key = keypad.getKey();
  }
     Serial.print(" ");
     Serial.println(key);
        if (key-48>=17) {return (key-55);} else { return(key-48);}
        
    }

 /* 
   *  A negitive angle/degree should turn the robot left
   *  A positive angle/degree should turn the robot right
   *  
   *  Right appears to be a lot more consistant than turning left.
   *  
   *  180 degrees turns more than 180 degrees in either direction
   *  ??
   */

void turn(int angle){
   compass_scalled_reading();
   compass_heading();
  Serial.write(12);
  Serial.print ("Heading= ");
  Serial.print (bearing);
  Serial.println(" Degree");
  
//start turning code
int originalAngle = bearing;
int target = originalAngle+angle;
uint8_t speed=120;
target=target%360;
if(target<0) {target+=360;}  

Serial.write(12);
Serial.print("target ");
Serial.print(target);

int direction=angle;
  while(1){
    compass_scalled_reading();
    compass_heading();
    int currentAngle=bearing;
    int diff=target-currentAngle;
    direction=180 - (diff+360)%360;

    Serial.write(12);
    Serial.print("Current: ");
    Serial.print(currentAngle);
    
    if(direction>0){
     bot.Speed(speed,speed);//right
     bot.rightTight(50);
      Serial.write(12);
      Serial.print("Right");
      while (bot.IsRunning()) {
        bot.update();
      }
            
    }else{
     bot.Speed(speed,speed);//left
     bot.leftTight(50);
      Serial.write(12);
      Serial.print("Left");
      while (bot.IsRunning()) {
        bot.update();
      }
          }
    
    if(abs(diff)<9 ) {
      bot.stop();
      Serial.write(12);
      Serial.println("DONE!");
      return;
    }
    delay(66);
  }  
}

void selfOrientation() {
/*
int tempRow = row;
int tempCol = col;

if (col = 0 && row = 0) {
      //error out location 0,0 is a corner
    }
if (row = 0 && col = COLS-1) {
      //error out location is a corner
    }
if (col = COLS-1 && row = ROWS-1) {
  //error out location is a corner
}
if (col = 0 && rows = ROWS-1) {
  //error out location is a corner
}
  if (row > 0 && col != 0) {
    //not on top row or in first column
      }
  if (col > 0 && row != 0) {
    //not on 1st column
  }
  if (row < ROWS-1) {
    //not on last row
  }
  if (col < COLS-1) {
    //not on last column
  }
  */
}

void findnumbers(int findnumber) {

  /* at this point there is no error checking, if a code is not
   *  found, it will return a grid of 0,0 (this could cause problems)
   */
   
int a, b;
  for (a = 0; a < ROWS; a++) {
    for (b = 0; b < COLS; b++) {
      if (findnumber == codes[a][b]) {
        row = a;
        col = b;}
      }
  }
  /*Serial.print(row);
  Serial.print(",");
  Serial.println(col);*/
}

void calculatemove() {

/*This is the logic for the move, but currently only tells
 * a direction and how far the robot should move, it will should
 * recalculate each time it reads a new NFC tag, until target is
 * reached.
 */

moveDirection = 0;

  if (targetrow > row) {
    Serial.write(12);
    Serial.print("DOWN ");
    //Serial.print(targetrow - row);
    //Serial.println(" rows.");
    moveDirection = 3;
      }
  if (targetrow < row) {
    Serial.write(12);
    Serial.print("UP ");
    //Serial.print (row - targetrow);
    //Serial.println(" rows.");
    moveDirection = 1;
  }
  /*if (targetrow == row) {
    Serial.println("We are on the correct row.");   
  }*/
  if (targetcol > col) {
    Serial.write(12);
    Serial.print("Right ");
    //Serial.print(targetcol - col);
    //Serial.println(" columns to the right");
    moveDirection = 2;
  }
  if (targetcol < col) {
    Serial.write(12);
    Serial.print("Left ");
    //Serial.print(col - targetcol);
    //Serial.println(" columns to the left.");
    moveDirection = 4;
  }
  /*
  if (targetcol == col) {
    Serial.println("At the target column");
  }*/
  if (targetcol == col && targetrow == row) {
    Serial.write(12);
    Serial.println("We are at target.");
    while(1) {}
  }
}

void readRFID() {
  //My modified RFID code from example
  mynum = "";
  
  status = MFRC522_Request(PICC_REQIDL, str);
  
  status = MFRC522_Anticoll(str);
    memcpy(serNum, str, 5);
    if (status == MI_OK)
    {
                        checksum1 = serNum[0] ^ serNum[1] ^ serNum[2] ^ serNum[3];
                        
      //Serial.write(12);
      //Serial.write(17);
      //Serial.print(checksum1);
                             
    
  }
 MFRC522_Halt();     //Command card into hibernation
}


//Lifted RFID code
/*
 * Function Name：Write_MFRC5200
 * Function Description: To a certain MFRC522 register to write a byte of data
 * Input Parameters：addr - register address; val - the value to be written
 * Return value: None
 */
void Write_MFRC522(uchar addr, uchar val)
{
  digitalWrite(chipSelectPin, LOW);

  //Address Format：0XXXXXX0
  SPI.transfer((addr<<1)&0x7E); 
  SPI.transfer(val);
  
  digitalWrite(chipSelectPin, HIGH);
}


/*
 * Function Name：Read_MFRC522
 * Description: From a certain MFRC522 read a byte of data register
 * Input Parameters: addr - register address
 * Returns: a byte of data read from the
 */
uchar Read_MFRC522(uchar addr)
{
  uchar val;

  digitalWrite(chipSelectPin, LOW);

  //Address Format：1XXXXXX0
  SPI.transfer(((addr<<1)&0x7E) | 0x80);  
  val =SPI.transfer(0x00);
  
  digitalWrite(chipSelectPin, HIGH);
  
  return val; 
}

/*
 * Function Name：SetBitMask
 * Description: Set RC522 register bit
 * Input parameters: reg - register address; mask - set value
 * Return value: None
 */
void SetBitMask(uchar reg, uchar mask)  
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);  // set bit mask
}


/*
 * Function Name: ClearBitMask
 * Description: clear RC522 register bit
 * Input parameters: reg - register address; mask - clear bit value
 * Return value: None
*/
void ClearBitMask(uchar reg, uchar mask)  
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
} 


/*
 * Function Name：AntennaOn
 * Description: Open antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
 * Input: None
 * Return value: None
 */
void AntennaOn(void)
{
  uchar temp;

  temp = Read_MFRC522(TxControlReg);
  if (!(temp & 0x03))
  {
    SetBitMask(TxControlReg, 0x03);
  }
}


/*
  * Function Name: AntennaOff
  * Description: Close antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
  * Input: None
  * Return value: None
 */
void AntennaOff(void)
{
  ClearBitMask(TxControlReg, 0x03);
}


/*
 * Function Name: ResetMFRC522
 * Description: Reset RC522
 * Input: None
 * Return value: None
 */
void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}


/*
 * Function Name：InitMFRC522
 * Description: Initialize RC522
 * Input: None
 * Return value: None
*/
void MFRC522_Init(void)
{
  digitalWrite(NRSTPD,HIGH);

  MFRC522_Reset();
    
  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    Write_MFRC522(TModeReg, 0x8D);    //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    Write_MFRC522(TPrescalerReg, 0x3E); //TModeReg[3..0] + TPrescalerReg
    Write_MFRC522(TReloadRegL, 30);           
    Write_MFRC522(TReloadRegH, 0);
  
  Write_MFRC522(TxAutoReg, 0x40);   //100%ASK
  Write_MFRC522(ModeReg, 0x3D);   //CRC Initial value 0x6363  ???

  //ClearBitMask(Status2Reg, 0x08);   //MFCrypto1On=0
  //Write_MFRC522(RxSelReg, 0x86);    //RxWait = RxSelReg[5..0]
  //Write_MFRC522(RFCfgReg, 0x7F);      //RxGain = 48dB

  AntennaOn();    //Open the antenna
}


/*
 * Function Name：MFRC522_Request
 * Description: Find cards, read the card type number
 * Input parameters: reqMode - find cards way
 *       TagType - Return Card Type
 *        0x4400 = Mifare_UltraLight
 *        0x0400 = Mifare_One(S50)
 *        0x0200 = Mifare_One(S70)
 *        0x0800 = Mifare_Pro(X)
 *        0x4403 = Mifare_DESFire
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Request(uchar reqMode, uchar *TagType)
{
  uchar status;  
  uint backBits;      //The received data bits

  Write_MFRC522(BitFramingReg, 0x07);   //TxLastBists = BitFramingReg[2..0] ???
  
  TagType[0] = reqMode;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

  if ((status != MI_OK) || (backBits != 0x10))
  {    
    status = MI_ERR;
  }
   
  return status;
}


/*
 * Function Name: MFRC522_ToCard
 * Description: RC522 and ISO14443 card communication
 * Input Parameters: command - MF522 command word,
 *       sendData--RC522 sent to the card by the data
 *       sendLen--Length of data sent  
 *       backData--Received the card returns data,
 *       backLen--Return data bit length
 * Return value: the successful return MI_OK
 */
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen)
{
    uchar status = MI_ERR;
    uchar irqEn = 0x00;
    uchar waitIRq = 0x00;
    uchar lastBits;
    uchar n;
    uint i;

    switch (command)
    {
        case PCD_AUTHENT:   //Certification cards close
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE:  //Transmit FIFO data
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
      break;
    }
   
    Write_MFRC522(CommIEnReg, irqEn|0x80);  //Interrupt request
    ClearBitMask(CommIrqReg, 0x80);     //Clear all interrupt request bit
    SetBitMask(FIFOLevelReg, 0x80);     //FlushBuffer=1, FIFO Initialization
    
  Write_MFRC522(CommandReg, PCD_IDLE);  //NO action; Cancel the current command???

  //Writing data to the FIFO
    for (i=0; i<sendLen; i++)
    {   
    Write_MFRC522(FIFODataReg, sendData[i]);    
  }

  //Execute the command
  Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {    
    SetBitMask(BitFramingReg, 0x80);    //StartSend=1,transmission of data starts  
  }   
    
  //Waiting to receive data to complete
  i = 2000; //i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
    do 
    {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = Read_MFRC522(CommIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    ClearBitMask(BitFramingReg, 0x80);      //StartSend=0
  
    if (i != 0)
    {    
        if(!(Read_MFRC522(ErrorReg) & 0x1B))  //BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {   
        status = MI_NOTAGERR;     //??   
      }

            if (command == PCD_TRANSCEIVE)
            {
                n = Read_MFRC522(FIFOLevelReg);
                lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)
                {   
          *backLen = (n-1)*8 + lastBits;   
        }
                else
                {   
          *backLen = n*8;   
        }

                if (n == 0)
                {   
          n = 1;    
        }
                if (n > MAX_LEN)
                {   
          n = MAX_LEN;   
        }
        
        //Reading the received data in FIFO
                for (i=0; i<n; i++)
                {   
          backData[i] = Read_MFRC522(FIFODataReg);    
        }
            }
        }
        else
        {   
      status = MI_ERR;  
    }
        
    }
  
    //SetBitMask(ControlReg,0x80);           //timer stops
    //Write_MFRC522(CommandReg, PCD_IDLE); 

    return status;
}


/*
 * Function Name: MFRC522_Anticoll
 * Description: Anti-collision detection, reading selected card serial number card
 * Input parameters: serNum - returns 4 bytes card serial number, the first 5 bytes for the checksum byte
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Anticoll(uchar *serNum)
{
    uchar status;
    uchar i;
  uchar serNumCheck=0;
    uint unLen;
    

    //ClearBitMask(Status2Reg, 0x08);   //TempSensclear
    //ClearBitMask(CollReg,0x80);     //ValuesAfterColl
  Write_MFRC522(BitFramingReg, 0x00);   //TxLastBists = BitFramingReg[2..0]
 
    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
  {
    //Check card serial number
    for (i=0; i<4; i++)
    {   
      serNumCheck ^= serNum[i];
    }
    if (serNumCheck != serNum[i])
    {   
      status = MI_ERR;    
    }
    }

    //SetBitMask(CollReg, 0x80);    //ValuesAfterColl=1

    return status;
} 


/*
 * Function Name: CalulateCRC
 * Description: CRC calculation with MF522
 * Input parameters: pIndata - To read the CRC data, len - the data length, pOutData - CRC calculation results
 * Return value: None
 */
void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData)
{
    uchar i, n;

    ClearBitMask(DivIrqReg, 0x04);      //CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);     //Clear the FIFO pointer
    //Write_MFRC522(CommandReg, PCD_IDLE);

  //Writing data to the FIFO  
    for (i=0; i<len; i++)
    {   
    Write_MFRC522(FIFODataReg, *(pIndata+i));   
  }
    Write_MFRC522(CommandReg, PCD_CALCCRC);

  //Wait CRC calculation is complete
    i = 0xFF;
    do 
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));      //CRCIrq = 1

  //Read CRC calculation result
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegM);
}


/*
 * Function Name: MFRC522_SelectTag
 * Description: election card, read the card memory capacity
 * Input parameters: serNum - Incoming card serial number
 * Return value: the successful return of card capacity
 */
uchar MFRC522_SelectTag(uchar *serNum)
{
    uchar i;
  uchar status;
  uchar size;
    uint recvBits;
    uchar buffer[9]; 

  //ClearBitMask(Status2Reg, 0x08);     //MFCrypto1On=0

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
      buffer[i+2] = *(serNum+i);
    }
  CalulateCRC(buffer, 7, &buffer[7]);   //??
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
    
    if ((status == MI_OK) && (recvBits == 0x18))
    {   
    size = buffer[0]; 
  }
    else
    {   
    size = 0;    
  }

    return size;
}


/*
 * Function Name: MFRC522_Auth
 * Description: Verify card password
 * Input parameters: authMode - Password Authentication Mode
                 0x60 = A key authentication
                 0x61 = Authentication Key B
             BlockAddr--Block address
             Sectorkey--Sector password
             serNum--Card serial number, 4-byte
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey, uchar *serNum)
{
    uchar status;
    uint recvBits;
    uchar i;
  uchar buff[12]; 

  //Verify the command block address + sector + password + card serial number
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i=0; i<6; i++)
    {    
    buff[i+2] = *(Sectorkey+i);   
  }
    for (i=0; i<4; i++)
    {    
    buff[i+8] = *(serNum+i);   
  }
    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
    {   
    status = MI_ERR;   
  }
    
    return status;
}


/*
 * Function Name: MFRC522_Read
 * Description: Read block data
 * Input parameters: blockAddr - block address; recvData - read block data
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Read(uchar blockAddr, uchar *recvData)
{
    uchar status;
    uint unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }
    
    return status;
}


/*
 * Function Name: MFRC522_Write
 * Description: Write block data
 * Input parameters: blockAddr - block address; writeData - to 16-byte data block write
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Write(uchar blockAddr, uchar *writeData)
{
    uchar status;
    uint recvBits;
    uchar i;
  uchar buff[18]; 
    
    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {   
    status = MI_ERR;   
  }
        
    if (status == MI_OK)
    {
        for (i=0; i<16; i++)    //Data to the FIFO write 16Byte
        {    
          buff[i] = *(writeData+i);   
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
        
    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {   
      status = MI_ERR;   
    }
    }
    
    return status;
}


/*
 * Function Name: MFRC522_Halt
 * Description: Command card into hibernation
 * Input: None
 * Return value: None
 */
void MFRC522_Halt(void)
{
  uchar status;
    uint unLen;
    uchar buff[4]; 

    buff[0] = PICC_HALT;
    buff[1] = 0;
    CalulateCRC(buff, 2, &buff[2]);
 
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}

//end lifted code
