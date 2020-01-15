#include <Wire.h>

#define NUM_REGISTERS       40
/* Pinbelegung */

//**********************************
/* MUX TCA9548A */
//**********************************
#define TCAADDR 0x70
#define INT1 6
#define INT2 7
#define RESETZ 4
#define CLK 5 

#define CHN1 5
#define CHN2 3
//select channel to read I2C (8 channel) 


//int RESETZ = 4;
//int CLK = 5;
//int INT = 6;
//int INT2 = 7;
//const int ocr1aval = 1; //||0 = 8MHz|| 1 = 4MHz|| 3 = 2MHZ ||7 = 1MHz||
const byte interruptPin = 2;

char LED2 = 0x2A; // LED2 on AFE4404 LD3 on Heart Rate Click
char LED2A = 0x2E;// LED2 - AmbientLED2
char LED1 = 0x2C; // LED1 on AFE4404 LD2 on Heart Rate Click
char LED1A = 0x2F;// LED1 - AmbientLED1
byte ledMode1 = 1, ledMode2=1; // Two register to read ADC value Channel1 and Channel 2
//ledMode1 = 1;
//ledMode2 = 1;

struct Register {
    uint8_t addr;
    uint32_t val;
};
/* Variablendeklaration */

//float befehl = 0;
bool Ready = 1;
int AFE_ADDR = 0x58;

unsigned long counter = 0;

/* TCA9548A has default address 0x70 */
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
/* Serial interface */
unsigned long serialdata, hexAdr,hexCmd;
int inbyte;

long getSerial()
{
  serialdata = 0;
  while (inbyte != '\n')
  {
    inbyte = Serial.read();  
    if (inbyte > 0 && inbyte != '\n')
    { 
      serialdata = serialdata * 10 + inbyte - '0';
      //Serial.println(serialdata);
    }
  }
  inbyte = 0;
  return serialdata;
}
long getHEXSerial()
{
  unsigned long hexData = 0;
  while (inbyte != '\n')
  {
    inbyte = Serial.read();  
    if (inbyte > 0 && inbyte != '\n')
    { 
      switch(inbyte){
        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'E':
        case 'F':
        {
          inbyte = inbyte - 'A';
          break;  
        }
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
        {
          inbyte = inbyte - 'a';
          break;  
        }
        default:
        {
          inbyte = inbyte - '0';  
          break;  
        }
      }
      hexData = hexData * 16 + inbyte;
      
    }
  }
  inbyte = 0;
  Serial.println(hexData,HEX);
  return hexData;
}

/* state machine */
enum BoardState
{
    Idle_State,
    Config_State, //signle or dual AFE, AFE channel #
    Run_State,
    Reg_Cmd_State,
    Stop_State,
    LED_Config_State
};

BoardState afeState = Idle_State;

/* AFE440X registers */

#define AFE_CONTROL0                0x00
#define AFE_LED2STC                 0x01
#define AFE_LED2ENDC                0x02
#define AFE_LED1LEDSTC              0x03
#define AFE_LED1LEDENDC             0x04
#define AFE_ALED2STC                0x05
#define AFE_ALED2ENDC               0x06
#define AFE_LED1STC                 0x07
#define AFE_LED1ENDC                0x08
#define AFE_LED2LEDSTC              0x09
#define AFE_LED2LEDENDC             0x0a
#define AFE_ALED1STC                0x0b
#define AFE_ALED1ENDC               0x0c
#define AFE_LED2CONVST              0x0d
#define AFE_LED2CONVEND             0x0e
#define AFE_ALED2CONVST             0x0f
#define AFE_ALED2CONVEND            0x10
#define AFE_LED1CONVST              0x11
#define AFE_LED1CONVEND             0x12
#define AFE_ALED1CONVST             0x13
#define AFE_ALED1CONVEND            0x14
#define AFE_ADCRSTSTCT0             0x15
#define AFE_ADCRSTENDCT0            0x16
#define AFE_ADCRSTSTCT1             0x17
#define AFE_ADCRSTENDCT1            0x18
#define AFE_ADCRSTSTCT2             0x19
#define AFE_ADCRSTENDCT2            0x1a
#define AFE_ADCRSTSTCT3             0x1b
#define AFE_ADCRSTENDCT3            0x1c
#define AFE_PRPCOUNT                0x1d
#define AFE_CONTROL1                0x1e
#define AFE_TIA_GAIN_SEP            0x20
#define AFE_TIA_GAIN                0x21
#define AFE_LEDCNTRL                0x22
#define AFE_CONTROL2                0x23
#define AFE_ALARM                   0x29
#define AFE_LED2VAL                 0x2a
#define AFE_ALED2VAL                0x2b
#define AFE_LED1VAL                 0x2c
#define AFE_ALED1VAL                0x2d
#define AFE_LED2_ALED2VAL           0x2e
#define AFE_LED1_ALED1VAL           0x2f
#define AFE_CONTROL3                0x31
#define AFE_PDNCYCLESTC             0x32
#define AFE_PDNCYCLEENDC            0x33
#define AFE_PROG_TG_STC             0x34
#define AFE_PROG_TG_ENDC            0x35
#define AFE_LED3LEDSTC              0x36
#define AFE_LED3LEDENDC             0x37
#define AFE_CLKDIV_PRF              0x39
#define AFE_OFFDAC                  0x3a
#define AFE_DEC                     0x3d
#define AFE_AVG_LED2_ALED2VAL       0x3f
#define AFE_AVG_LED1_ALED1VAL       0x40


//char string[40];
void setup() {
  // put your setup code here, to run once:
  /* Mode: LOW | CHANGE | RISING | FALLING | HIGH */
  //pinMode(interruptPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), readADC, RISING);
  Serial.begin(115200);
  Serial.println("AFE4404 Heart Rate IC Test"); Serial.println(""); 

  
  
  pinMode(RESETZ, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(INT1, INPUT);
  pinMode(INT2, INPUT);

  afeState = Idle_State;
/******** Setup timer clock 4Mhz *******/
/*  
    
TCCR1A = ( (1 << COM1A0));
   
  TCCR1B = ((1 << WGM12) | (1 << CS10));

  TIMSK1 = 0;
   
  OCR1A = ocr1aval;
  */
  /************************************/
  AFE_RESETZ_Init();
  AFE_Trigger_HWReset();

//  //first channel of AFE4404
//  tcaselect(CHN1);
//  
//  AFE_Init();
//  delay(100);
//  //second channel of AFE4404
//  tcaselect(CHN2);
//  AFE_Init();
//  delay(100);
  
  //Serial.println("Press '1' to start and '2' to stop");
  //Serial.println("CLEARDATA");
  //Serial.println("LABEL,NptsAveraged,Time(sec),Chan1,Chan2,Chan3");
  
}

signed long readADC(byte LED)
{
    //signed long val = 0;
  //if (digitalRead(INT))
    {
     return AFE_Reg_Read(LED);
     Ready = 1;
        
        //Serial.print("Time:");Serial.println(counter); 
    }
   // }
}
long nChannel = 0;
void loop() {
  // put your main code here, to run repeatedly:
//signed long val = 0;
//if (befehl == 1)
// {
   //counter = millis();
//   while(!digitalRead(INT));
   //counter = millis() - counter;
  
//if (befehl == 2) 
//  AFE_Enable_HWPDN();

//signed long val = 0;
//counter = millis();
/* Select state to control AFE */
switch(afeState)
{
  case Idle_State:  
    {
      // Read Serial command, if 0: keep idle; 1: config AFE
      Serial.println("Idle State. Enter \"1\" to go into Config State");
      getSerial();
      switch(serialdata)
      { 
        case 0: { 
          afeState = Idle_State;
          break;}
        case 1: { 
          AFE_RESETZ_Init();
          AFE_Trigger_HWReset();
          afeState = Config_State;
          break;}   
      }
      break;
    }
  case Config_State:
  {
         //Config #AFE ICs  and #channels wishes to be run, 
         Serial.println("Config State. Enter \"1\" to Channel Mode 1: AFE1 works.");
         Serial.println("Config State. Enter \"2\" to Channel Mode 2: AFE2 works.");
         Serial.println("Config State. Enter \"3\" to Channel Mode 3: All AFEs work.");
         nChannel = getSerial();
         switch(nChannel){
          case 1: {
             //first channel of AFE4404
             tcaselect(CHN1);
             AFE_Init();
             delay(100);
             break;
          }
          case 2: {
             //second channel of AFE4404
             tcaselect(CHN2);
             AFE_Init();
             delay(100);
             break;
          }
          case 3: {
             //first channel of AFE4404
              tcaselect(CHN1);
  
              AFE_Init();
              delay(100);
//              //SELECT LED REGISTER TO OUTPUT
//              Serial.println("Select LED2[LD3]/LED2\\Amb ('1/2') LED1[LD2]/LED1\\Amb ('3/4').");
//              getSerial();
//             switch(serialdata){
//              case 1: ledReg1 = LED1; break;
//              case 2: ledReg1 = LED1A; break;
//              case 3: ledReg1 = LED2; break;
//              case 4: ledReg1 = LED2A; break;
//             }
              //second channel of AFE4404
              tcaselect(CHN2);
              AFE_Init();
              delay(100);
              //SELECT LED REGISTER TO OUTPUT
//             Serial.println("Select LED2[LD3]/LED2\\Amb ('1/2') LED1[LD2]/LED1\\Amb ('3/4').");
//             getSerial();
//             switch(serialdata){
//              case 1: ledReg2 = LED1; break;
//              case 2: ledReg2 = LED1A; break;
//              case 3: ledReg2 = LED2; break;
//              case 4: ledReg2 = LED2A; break;
//             }
            break;
          }
         }
         

         afeState = Run_State;
    break;
  }
  // State machine in RUN STATE
  case Run_State:
  {
      if(Serial.available()){
        getSerial();
        switch(serialdata){
          case 1: {
              afeState = Stop_State; break;
            }
          case 2:{
              afeState = Reg_Cmd_State; break;
          }
          case 3:{
              afeState = Idle_State;
              AFE_RESETZ_Init();
              AFE_Trigger_HWReset();
              break;
          case 4:{
              afeState = LED_Config_State;
              break;
          }
          }
        }
            
        
        
      }
      else{
          signed long val1, val2 ;
          val1 = val2 = 0;
          //depending on nChannel will read the register from the AFEs
          // "1": read AFE1
          // "2": read AFE2
          // "3": read both AFE1 and AFE2
          switch(nChannel)
          {
            case 1: {
              tcaselect(CHN1);
              while (!digitalRead(INT1));
              switch(ledMode1){
                case 1:{
                  val1 = AFE_Reg_Read(LED2);
                  Serial.println(val1);
                  break;
                }
                case 2:{
                  val1 = AFE_Reg_Read(LED2A);
                  Serial.println(val1);
                  break;
                }
                case 3:{
                  val1 = AFE_Reg_Read(LED1);
                  Serial.println(val1);
                  break;
                }
                case 4:{
                  val1 = AFE_Reg_Read(LED1A);
                  Serial.println(val1);
                  break;
                }
                case 5:{
                  val1 = AFE_Reg_Read(LED2);
                  val2 = AFE_Reg_Read(LED2A);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.println(val2);
                  break;
                }
                case 6:{
                  val1 = AFE_Reg_Read(LED2);
                  val2 = AFE_Reg_Read(LED1);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.println(val2);
                  break;
                }
                case 7:{
                  val1 = AFE_Reg_Read(LED1);
                  val2 = AFE_Reg_Read(LED1A);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.println(val2);
                  break;
                }
              }
             break;
              }
             case 2: {
              tcaselect(CHN2);
              while (!digitalRead(INT2));
              switch(ledMode2){
                case 1:{
                  val1 = AFE_Reg_Read(LED2);
                  Serial.println(val1);
                  break;
                }
                case 2:{
                  val1 = AFE_Reg_Read(LED2A);
                  Serial.println(val1);
                  break;
                }
                case 3:{
                  val1 = AFE_Reg_Read(LED1);
                  Serial.println(val1);
                  break;
                }
                case 4:{
                  val1 = AFE_Reg_Read(LED1A);
                  Serial.println(val1);
                  break;
                }
                case 5:{
                  val1 = AFE_Reg_Read(LED2);
                  val2 = AFE_Reg_Read(LED2A);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.println(val2);
                  break;
                }
                case 6:{
                  val1 = AFE_Reg_Read(LED2);
                  val2 = AFE_Reg_Read(LED1);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.println(val2);
                  break;
                }
                case 7:{
                  val1 = AFE_Reg_Read(LED1);
                  val2 = AFE_Reg_Read(LED1A);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.println(val2);
                  break;
                }
              }
              break;              
             }
             case 3:{
                tcaselect(CHN1);
                while (!digitalRead(INT1));
                switch(ledMode1){
                case 1:{
                  val1 = AFE_Reg_Read(LED2);
                  Serial.print(val1);
                  break;
                }
                case 2:{
                  val1 = AFE_Reg_Read(LED2A);
                  Serial.print(val1);
                  break;
                }
                case 3:{
                  val1 = AFE_Reg_Read(LED1);
                  Serial.print(val1);
                  break;
                }
                case 4:{
                  val1 = AFE_Reg_Read(LED1A);
                  Serial.print(val1);
                  break;
                }
                case 5:{
                  val1 = AFE_Reg_Read(LED2);
                  val2 = AFE_Reg_Read(LED2A);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.print(val2);
                  break;
                }
                case 6:{
                  val1 = AFE_Reg_Read(LED2);
                  val2 = AFE_Reg_Read(LED1);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.print(val2);
                  break;
                }
                case 7:{
                  val1 = AFE_Reg_Read(LED1);
                  val2 = AFE_Reg_Read(LED1A);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.print(val2);
                  break;
                }
              }
                Serial.print(" ");
                tcaselect(CHN2);
                while (!digitalRead(INT2));
                switch(ledMode2){
                case 1:{
                  val1 = AFE_Reg_Read(LED2);
                  Serial.println(val1);
                  break;
                }
                case 2:{
                  val1 = AFE_Reg_Read(LED2A);
                  Serial.println(val1);
                  break;
                }
                case 3:{
                  val1 = AFE_Reg_Read(LED1);
                  Serial.println(val1);
                  break;
                }
                case 4:{
                  val1 = AFE_Reg_Read(LED1A);
                  Serial.println(val1);
                  break;
                }
                case 5:{
                  val1 = AFE_Reg_Read(LED2);
                  val2 = AFE_Reg_Read(LED2A);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.println(val2);
                  break;
                }
                case 6:{
                  val1 = AFE_Reg_Read(LED2);
                  val2 = AFE_Reg_Read(LED1);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.println(val2);
                  break;
                }
                case 7:{
                  val1 = AFE_Reg_Read(LED1);
                  val2 = AFE_Reg_Read(LED1A);
                  Serial.print(val1);
                  Serial.print(" ");
                  Serial.println(val2);
                  break;
                }
              }

//                Serial.print(val1);
//                Serial.print(" ");
//                Serial.println(val2);
                break;
             }
          }
                    //val1 = 0x12345678;
          //val2 = 0xAABBCCDD;
          //    Ready = 1;
//          uint8_t msg1[sizeof(signed long)];
//          uint8_t msg2[sizeof(signed long)];
//          *(signed long*)(msg1) = val1;
//          *(unsigned long*)(msg2) = val2; 
//          char frame[] = {0xAA, 0xBB, msg1[0],msg1[1],msg1[2], msg1[3], 0xAA, 0xBB, msg2[0], msg2[1], msg2[2], msg2[3]};
          //char frame[] = {0xAA, 0xBB, msg1[0], msg1[1], msg1[2], msg1[3]};
          //Serial.println(sizeof(frame));
          //sprintf(string, "%X%X%04X%04X",0xAA,0xBB,val1,val2);
          //Serial.write(frame);
          //Serial.print("DATA",HEX);
          
//          Serial.print(val1);
//          Serial.print(" ");
//          Serial.println(val2);
//          //Serial.println(string);
          delay(10);
      }
        
        
    break;
  }
  case Stop_State:{
    Serial.println("Enter any key to continous reading PPGs!");
    getSerial();
    afeState = Run_State;
    break; 
  }
  case LED_Config_State:{
    /*
     * Mode 1: Read Register LED2
     * Mode 2: Read Register LED2 - AmbLed2
     * Mode 3: Read Register LED1
     * Mode 4: Read Register LED1 - AmbLed1
     * Mode 5: Read both Register LED2 and LED2 - AmbLed2 
     * Mode 6: Read both Register LED2 and LED1
     * Mode 7: Read both Register LED1 and LED1 - AmbLed1 
     */
    Serial.println("Config LED register to read out.");
    Serial.println("Enter \"1\" to return Run State.");
    unsigned long cmd;
    cmd = getSerial();
    if(cmd == 1)
    {
        afeState = Run_State;
     }
     else{
       Serial.println("Config AFE led mode. ");
       Serial.println("Mode 1: Read Register LED2.");
       Serial.println(" Mode 2: Read Register LED2 - AmbLed2.");
       Serial.println("Mode 3: Read Register LED1");
       Serial.println("Mode 4: Read Register LED1 - AmbLed1");
       Serial.println("Mode 5: Read both Register LED2 and LED2 - AmbLed2");
       Serial.println("Mode 6: Read both Register LED2 and LED1");
       Serial.println("Mode 7: Read both Register LED1 and LED1 - AmbLed1");
      
    switch(nChannel)
    {
      case 1: {
        Serial.println("Config AFE1. ");
        //Serial.println("Select LED2[LD3]/LED2\\Amb ('1/2') LED1[LD2]/LED1\\Amb ('3/4').");
        ledMode1 = getSerial();
//        switch(serialdata){
//          case 1: ledReg1 = LED1; break;
//          case 2: ledReg1 = LED1A; break;
//          case 3: ledReg1 = LED2; break;
//          case 4: ledReg1 = LED2A; break;
//          }
        //end config AFE1
        break;
      }
      case 2: {
        Serial.println("Config AFE2. ");
        //Serial.println("Select LED2[LD3]/LED2\\Amb ('1/2') LED1[LD2]/LED1\\Amb ('3/4').");
        ledMode2 = getSerial();
//        switch(serialdata){
//          case 1: ledReg2 = LED1; break;
//          case 2: ledReg2 = LED1A; break;
//          case 3: ledReg2 = LED2; break;
//          case 4: ledReg2 = LED2A; break;
//          }
        //end config AFE2
        break;
      }
      case 3: {
        Serial.println("Config AFE1 LED Mode. ");
        ledMode1 = getSerial();
        
        Serial.println("Config AFE2 LED Mode. ");
        ledMode2 = getSerial();
        break;
      }
            
    }
    afeState = LED_Config_State;
  }
    //end state LED_config_State
    break;
  }
  case Reg_Cmd_State:{
    Serial.println("Enter Register Address in Hex which you want to change?");
    Serial.println("Enter \"1\" to return Run State.");
    unsigned long cmd;
    cmd = getHEXSerial();
    //Example control LED1,2 adr: 0x22; turn on led2: 0x000100; led1: 0x000004; boths: 0x000104;
    if (cmd==1)
      {
            afeState = Run_State;
       }
     else 
      {
          hexAdr = cmd;
          Serial.println("Enter Command Values in HEX which you want to do?");
          hexCmd = getHEXSerial();
          Serial.println("Which the AFE you want to modified?");
          Serial.println("'1' for AFE1; '2' for AFE2 and '3' for all AFEs.");
          //TO-DO Write command to select AFE
          nChannel = getSerial();
          switch(nChannel)
          {
            case 1: {
              tcaselect(CHN1);
              AFE_Register_Change();
              hexCmd = AFE_Reg_Read(hexAdr);        
              Serial.println(hexCmd,HEX);
              break;
              }
             case 2: {
              tcaselect(CHN2);
              AFE_Register_Change();
              hexCmd = AFE_Reg_Read(hexAdr);        
              Serial.println(hexCmd,HEX);
              break;              
             }
             case 3:{
                tcaselect(CHN1);
                AFE_Register_Change();
                hexCmd = AFE_Reg_Read(hexAdr);        
                Serial.println(hexCmd,HEX);

                tcaselect(CHN2);
                AFE_Register_Change();
                hexCmd = AFE_Reg_Read(hexAdr);        
                Serial.println(hexCmd,HEX);
                break;
             }
          } 
          
          
          
          
          //AFE_Register_Change();
          //AFE_Reg_Write(hexAdr, hexCmd);
          //hexCmd = AFE_Reg_Read(hexAdr);        
          //Serial.println(hexCmd,HEX);
          afeState = Reg_Cmd_State;
      }
      
      
    break; 
  }
}

//signed long val1, val2;
//val1 = val2 = 0;
//
//tcaselect(CHN2);
//while (!digitalRead(INT2));
//
//    val2 = AFE_Reg_Read(LED2);
//    
//tcaselect(CHN1);
//while (!digitalRead(INT1));
//    val1 = AFE_Reg_Read(LED2);
//
//
//    //    Ready = 1;
//    //sprintf(string, "%d \t %d",val1,val2);
//    //Serial.print("DATA ");
//    Serial.print(val1);
//    Serial.print(" ");
//    Serial.println(val2);
    
    
        //Serial.print("Time:");Serial.println(counter); 
        //delay(10);
    
    

}

/**********************************************************************************************************/
/*                              Auslesen der seriellen Eingabeleiste                                      */
/**********************************************************************************************************/
/*
void serialEvent()
{
  befehl = Serial.parseFloat();
}
*/
/**********************************************************************************************************/
/*                              AFE4404_Initialisierung zum Start                                         */
/**********************************************************************************************************/

void AFE_Init()
{
  //AFE_RESETZ_Init();
  //AFE_Enable_HWPDN();
  //AFE_Disable_HWPDN();
  //AFE_Trigger_HWReset();
  Wire.begin();
  Wire.beginTransmission (AFE_ADDR);
  Serial.println("Initializing AFE4404...");
  if (Wire.endTransmission () == 0)
      {
        AFE_Enable_Write (); // enable to writting to registers
        AFE_Reg_Init();
       AFE_CLK_Init();
       AFE_Enable_Read ();  // enable to reading from registers
       Serial.println("Initilized Sucess!"); 
      }
  else
    Serial.println("Initilized Fail!");    

}
/**********************************************************************************************************/
/*                              AFE4404_Initialisierung zum Start                                         */
/**********************************************************************************************************/

void AFE_Register_Change()
{
  //AFE_RESETZ_Init();
  //AFE_Enable_HWPDN();
  //AFE_Disable_HWPDN();
  //AFE_Trigger_HWReset();
  Wire.begin();
  Wire.beginTransmission (AFE_ADDR);
  //Serial.println("Initializing AFE4404...");
  if (Wire.endTransmission () == 0)
      {
       AFE_Enable_Write (); // enable to writting to registers
       // AFE_Reg_Init();
       //AFE_CLK_Init();
       AFE_Reg_Write(hexAdr, hexCmd);
       AFE_Enable_Read ();  // enable to reading from registers
       Serial.println("Changing Register Sucess!"); 
      }
  else
    Serial.println("Initilized Fail!");    

}

/**********************************************************************************************************/
/*                              AFE4404_Registerinitialisierung                                           */
/**********************************************************************************************************/
void AFE_Reg_Init()
{
  //AFE_Reg_Write(34, 0x0033C3);
  AFEinitRegisters();
}


/**********************************************************************************************************/
/*                              AFE4404_Enable_Read                                                       */
/**********************************************************************************************************/
void AFE_Enable_Read ()         //Prohibit writing to registers
{
  byte configData[3];
  configData[0]=0x00;
  configData[1]=0x00;
  configData[2]=0x01;
  I2C_write (AFE_ADDR, AFE_CONTROL0, configData, 3);
}

/**********************************************************************************************************/
/*                              AFE4404_Disable_Read                                                      */
/**********************************************************************************************************/
void AFE_Enable_Write ()        //Permitt writing to registers
{
  byte configData[3];
  configData[0]=0x00;
  configData[1]=0x00;
  configData[2]=0x00;
  I2C_write (AFE_ADDR, AFE_CONTROL0, configData, 3);
}

/**********************************************************************************************************/
/*                  RESETZ des AFE4404 wird Initialisiert                                                 */
/**********************************************************************************************************/
void AFE_RESETZ_Init()
{
  digitalWrite(RESETZ, HIGH);
  delayMicroseconds(30);
}

/**********************************************************************************************************/
/*                  Reset internal registers by setting RESETZ = LOW for 25 - 50 us                       */
/**********************************************************************************************************/
void AFE_Trigger_HWReset()
{
  digitalWrite(RESETZ, LOW);              //Sets Arduino pins 22-29 LOW
  delayMicroseconds(30);
  digitalWrite(RESETZ, HIGH);              //Sets Arduino pins 22-29 HIGH
  delay(10);
}

/**********************************************************************************************************/
/*                             AFE4404 Power Down                                                         */
/**********************************************************************************************************/
void  AFE_Enable_HWPDN()
{
  digitalWrite(RESETZ, LOW);                  //Power Down by setting the RESETZ pin to LOW for more than 200 us
  delay(10);
}

/**********************************************************************************************************/
/*                                AFE4404 Power Up                                                        */
/**********************************************************************************************************/
void  AFE_Disable_HWPDN()
{
  digitalWrite(RESETZ, HIGH);                  //Power Up the AFE by setting the RESETZ pin to HIGH   
  delay(10);
}

/**********************************************************************************************************/
/*                                AFE4404 Set Clock Mode to Internal                                      */
/**********************************************************************************************************/

void AFE_CLK_Init()
{            
  AFE_Reg_Write(35, 0x104218);        //Set CLK Mode to internal clock (default Wert: 124218 mit interner CLK)
  AFE_Reg_Write(41, 0x2);             //Don´t set the internal clock to the CLK pin for outside usage
  AFE_Reg_Write(49, 0x000021);        //Division ratio for external clock mode set to fit the Arduino Mega 16MHz
  AFE_Reg_Write(57, 0);               //Set the lowes sampling rate to 61Hz (~17 ms)
}

/*********************************************************************************************************/
/*                                   AFE4404_Reg_Write                                                   */
/*********************************************************************************************************/
void AFE_Reg_Write (int reg_address, unsigned long data)
{
  byte configData[3];
  configData[0]=(byte)(data >>16);
  configData[1]=(byte)(((data & 0x00FFFF) >>8));
  configData[2]=(byte)(((data & 0x0000FF)));
  I2C_write(AFE_ADDR, reg_address, configData, 3); 
}

/*********************************************************************************************************/
/*                                   AFE4404_Reg_Read                                                    */
/*********************************************************************************************************/
signed long AFE_Reg_Read(int reg_address)
{
  byte configData[3];
  signed long retVal;
  I2C_read (AFE_ADDR, reg_address, configData, 3);
  retVal = configData[0];
  retVal = (retVal << 8) | configData[1];
  retVal = (retVal << 8) | configData[2];
  if (reg_address >= 0x2A && reg_address <= 0x2F)
  {
    if (retVal & 0x00200000)  // check if the ADC value is positive or negative
    {
      retVal &= 0x003FFFFF;   // convert it to a 22 bit value
      return (retVal^0xFFC00000);
    }
  }
  return retVal;
}

/**********************************************************************************************************/
/*                              Write to AFE on I2C                                                       */
/**********************************************************************************************************/
char I2C_write (int slave_address, int reg_address, byte configData[], int byteCount)
{
  int trans_end = 0;
  signed long retVal;
  
  Wire.beginTransmission(slave_address);
  Wire.write(reg_address);
  Serial.print(configData[0]);
  Serial.print(",");
  Serial.print(configData[1]);
  Serial.print(",");
  Serial.println(configData[2]);
  retVal = configData[0];
  retVal = (retVal << 8) | configData[1];
  retVal = (retVal << 8) | configData[2];
  Serial.println(retVal);
  Serial.println(reg_address);
  Wire.write(configData, 3);
  Serial.println("test");
  Wire.endTransmission();
  return (trans_end);
  
 /* while(1)
  {
    if(byteCount == 0)
    {
      Wire.endTransmission();
      Serial.println("test");
      return (trans_end);
    }else{
      //unsigned int reg_data = (unsigned int) *write_data++;
      Wire.write(configData, 3);
      byteCount--;
    }       
  }*/
}

/**********************************************************************************************************/
/*                              Read Data of AFE on I2C                                                   */
/**********************************************************************************************************/
char I2C_read(int slave_address, int reg_address, byte *read_Data, int byteCount)
{
  int trans_end = 0;
  
  Wire.beginTransmission(slave_address);
  Wire.write(reg_address);
  Wire.endTransmission();
  Wire.requestFrom(slave_address, 3);
  while(Wire.available() && (byteCount != 0))
  {
    *read_Data++ = Wire.read();
    byteCount--;
  }
  return (trans_end);
}

void AFEinitRegisters(void) {
    
    unsigned char i;
    struct Register reg[NUM_REGISTERS];
    reg[0].addr = 0x01; reg[0].val = 0x000050;
    reg[1].addr = 0x02; reg[1].val = 0x00018F;
    reg[2].addr = 0x03; reg[2].val = 0x000320;
    reg[3].addr = 0x04; reg[3].val = 0x0004AF;
    reg[4].addr = 0x05; reg[4].val = 0x0001E0;
    reg[5].addr = 0x06; reg[5].val = 0x00031F;
    reg[6].addr = 0x07; reg[6].val = 0x000370;
    reg[7].addr = 0x08; reg[7].val = 0x0004AF;
    reg[8].addr = 0x09; reg[8].val = 0x000000;
    reg[9].addr = 0x0A; reg[9].val = 0x00018F;
    reg[10].addr = 0x0B; reg[10].val = 0x0004FF;
    reg[11].addr = 0x0C; reg[11].val = 0x00063E;
    reg[12].addr = 0x0D; reg[12].val = 0x000198;
    reg[13].addr = 0x0E; reg[13].val = 0x0005BB;
    reg[14].addr = 0x0F; reg[14].val = 0x0005C4;
    reg[15].addr = 0x10; reg[15].val = 0x0009E7;
    reg[16].addr = 0x11; reg[16].val = 0x0009F0;
    reg[17].addr = 0x12; reg[17].val = 0x000E13;
    reg[18].addr = 0x13; reg[18].val = 0x000E1C;
    reg[19].addr = 0x14; reg[19].val = 0x00123F;
    reg[20].addr = 0x15; reg[20].val = 0x000191;
    reg[21].addr = 0x16; reg[21].val = 0x000197;
    reg[22].addr = 0x17; reg[22].val = 0x0005BD;
    reg[23].addr = 0x18; reg[23].val = 0x0005C3;
    reg[24].addr = 0x19; reg[24].val = 0x0009E9;
    reg[25].addr = 0x1A; reg[25].val = 0x0009EF;
    reg[26].addr = 0x1B; reg[26].val = 0x000E15;
    reg[27].addr = 0x1C; reg[27].val = 0x000E1B;
    reg[28].addr = 0x1D; reg[28].val = 0x009C3F;
    reg[29].addr = 0x1E; reg[29].val = 0x000103;
    reg[30].addr = 0x20; reg[30].val = 0x008025; //0x008003
    reg[31].addr = 0x21; reg[31].val = 0x000125;//0x000003 //bit8: PROG_TG_EN
    reg[32].addr = 0x22; reg[32].val = 0x000100;//ILED2 = 32
    reg[33].addr = 0x23; reg[33].val = 0x000000;
    reg[34].addr = 0x32; reg[34].val = 0x00155F;
    reg[35].addr = 0x33; reg[35].val = 0x00991F;
    reg[36].addr = 0x36; reg[36].val = 0x000190;
    reg[37].addr = 0x37; reg[37].val = 0x00031F;
    reg[38].addr = 0x34; reg[38].val = 0x001563;
    reg[39].addr = 0x35; reg[39].val = 0x00991F;

    for (i = 0; i < NUM_REGISTERS; i++) 
      AFE_Reg_Write(reg[i].addr, reg[i].val);
    
}
