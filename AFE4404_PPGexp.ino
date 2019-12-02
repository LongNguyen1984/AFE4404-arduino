#include <Wire.h>

#define NUM_REGISTERS       40
/* Pinbelegung */

int RESETZ = 4;
int CLK = 5;
int INT = 6;
//const int ocr1aval = 1; //||0 = 8MHz|| 1 = 4MHz|| 3 = 2MHZ ||7 = 1MHz||
const byte interruptPin = 2;

char LED2 = 0x2A; // LED2 on AFE4404

struct Register {
    uint8_t addr;
    uint32_t val;
};
/* Variablendeklaration */

//float befehl = 0;
bool Ready = 1;
int AFE_ADDR = 0x58;

unsigned long counter = 0;

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



void setup() {
  // put your setup code here, to run once:
  /* Mode: LOW | CHANGE | RISING | FALLING | HIGH */
  //pinMode(interruptPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), readADC, RISING);
  Serial.begin(115200);
   
  pinMode(RESETZ, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(INT, INPUT);
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
  
  AFE_Init();
  Serial.println("Press '1' to start and '2' to stop");
  
}
signed long val = 0;
void readADC()
{
  
  //if (digitalRead(INT))
    {
        val = AFE_Reg_Read(LED2);
        Ready = 1;
        
        //Serial.print("Time:");Serial.println(counter); 
    
    }
     
   // }
}
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
/*
if (Ready){
  Serial.println(val);
  Ready = 0;
}
  */
  
//signed long val = 0;
//counter = millis();
if (digitalRead(INT))
    {
        val = AFE_Reg_Read(LED2);
    //    Ready = 1;
        Serial.println(val);
        //Serial.print("Time:");Serial.println(counter); 
        delay(10);
    
    }

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
