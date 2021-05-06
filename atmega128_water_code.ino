//#include <atmega-timers.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
//#include "timer.h"
//#include <TimerOne.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include<avr/interrupt.h>
#define sensorPin1 4//2
#define sensorPin2 5//3
#define sensorPin3 6
#define sensorPin4 7
void(* resetFunc) (void) = 0;
/*
  #define down      9//67   //8
  #define enter    11 // 69   //11  #define up       8 //66   //9

  #define back     12 // 65   //12
  #define forward  13 // 68   //13

  #define T_SW_1  33 //37  //33
  #define T_SW_2  36  //36
  #define T_SW_3  35  //34
  #define T_SW_4  34  //35

  #define BO_SW   37//33
  #define Buzzer  A8//34//
*/

#define down      40  /*8*/
#define up        41  /*9*/
#define enter     38  /*11*/
#define back      42  /*12*/
#define forward   39  /*13*/

#define T_SW_1  37  /*33*/
#define T_SW_2  36  /*36*/
#define T_SW_3  35  /*35*/
#define T_SW_4  34  /*35*/

#define BO_SW   33
#define Buzzer  32
//*/
#define GSM_Active 0 //Gsm_activate ---> non-zero   or  Gsm_deactivate --> zero
#define Recharge_Active 0//Recharge_activate ---> non-zero   or  Recharge_deactivate --> zero

#define Address  0x68
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// LiquidCrystal lcd(28, 27, 26, 25, 24, 23); //old board version
LiquidCrystal lcd(23, 24, 25, 26, 27, 28);//new borad version
//pin 40 & 41 for relays
// pin 8 -> down ,9 -> up ,11 -> enter , 12 -> back, 13 -> forward
/*           MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
              Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
  Signal      Pin          Pin           Pin       Pin        Pin              Pin
  -----------------------------------------------------------------------------------------
** RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
   SPI SS      SDA(SS)      10            53        D10        10               10
   SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
   SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
   SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
*/
byte lcd_check1 = 0, lcd_sec = 0;
unsigned int serial_no = 0, id_no = 0, value_no = 0;
byte x, x1, month_t, date, year_t, wt_flag = 0;

byte buffer3[9] = {0, 0, 0, 0, 0, 0, 0, 44, 88};
byte buffer2[9] = {0, 0, 0, 0, 0, 0, 0, 22, 91};
byte buffer4[9] = {0, 0, 0, 0, 0, 0, 0, 11, 11};

unsigned int serial_num = 0;
const unsigned long interval = 50000;
extern volatile unsigned long timer0_millis;
unsigned prev = 0;
unsigned long current = 0;
byte  menu_on_off_1 = 0, menu_on_off_2 = 0, test_dd_prev = 0, TS_No = 0, check_rfid_status = 0;
unsigned long int L1 = ((EEPROM.read(34 + 0)) * 256) + EEPROM.read(35 + 0);
unsigned long int L2 = ((EEPROM.read(34 + 7)) * 256) + EEPROM.read(35 + 7);
byte flag_int2, flag_int1, serial_menu_flag = 0;
unsigned long int pulseCount1, pulseCount2, total_min2, total_min1, total_max2 = 0, total_max1 = 0, x_countable;;
int fc, count, count1, user, addr = 0, time_check = 0;
int counter = 0, currentState = 0, previousState = 0, currentState1 = 0, previousState1 = 0;
int coun = 0, val1, flag_1 = 1, flag_2 = 1, flag_3 = 1, ff, menu_flag = 0, v1, val = 0, flag_4 = 0;;
byte admin_no[10], user_no[10], paytm_no[10], temp_no[10];
byte b_previousState = 0, b_currentState = 0, f_previousState = 0, f_currentState = 0, e_previousState = 0, e_currentState = 0;
unsigned int f1 = 0, f2 = 0, x_csq;
byte k = 0, rs = 0, RF_status, no_of_tap = EEPROM.read(70);
unsigned int amt_o = 0, amt_n = 0, B_amt, A_amt;
byte key_r = 0, key_p = 0, free_key = 0, menu_no = 0, f12_key = 0, status_check = 0;
byte test_dd, test_mm, test_yy, status_write_flag = 0;
unsigned long int amt_rg, amt_swp, amt_ltr;
constexpr uint8_t RST_PIN = 12;     // Configurable, see typical pin layout above
constexpr uint8_t SS_PIN = 8;     // Configurable, see typical pin layout above
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522

byte buffer1[18]; //Rfid card data store
unsigned int seriall[5] = {0, 0, 0, 0, 0};
void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}
long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

byte RFID_ULTD_card()
{
  byte check = 0;
  byte block = 4;
  byte len = 18;
  byte buffer_ultd[18] = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  byte buffer[34] = {0};

  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  if ( ! mfrc522.PICC_IsNewCardPresent())
    return 0;

  if ( ! mfrc522.PICC_ReadCardSerial())
    return 0;
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  MFRC522::StatusCode status;
  Serial.setTimeout(20000L) ;

  for (int i = 0; i < 5; i++)
  {
    buffer_ultd[i] = EEPROM.read(500 + i);
  }
  for (byte i = 0; i < 5; i++)
    if (i < 18)
      buffer[i] = buffer_ultd[i];    // pad with spaces
    else
      buffer[i] = 0;   // pad with
  buffer[7] = 1;
  /* for (byte i = 0; i < 8; i++)
     Serial.print(buffer[i]);
    Serial.println();*/

  block = 4;
  len = 18;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_ultd;
  }
  status = mfrc522.MIFARE_Read(block, buffer1, &len);
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_ultd;
  }
  // for (int i = 0; i < 19; i++) Serial.print(buffer1[i]);

  if ((buffer1[7] == buffer3[7]) || ( buffer1[8] == buffer3[8]) || (buffer1[7] == buffer2[7]) || (buffer1[8] == buffer2[8]) || (buffer1[7] == buffer4[7]) || (buffer1[8] == buffer4[8]))
  {
    lcd.clear();
    lcd.setCursor(3, 1);
    lcd.print("Master card");
    delay(2000);
    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.print("Place an ULTD card ");
    goto end_ultd;
  }
  //buffer_ultd[7] = 1;
  block = 4; len = 18;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file

  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_ultd;
  }

  status = mfrc522.MIFARE_Write(block, buffer, 16);
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    //Serial.println("Status failed_write");
    goto end_ultd;
  }
  block = 5; len = 18;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 5, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file

  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_ultd;
  }

  status = mfrc522.MIFARE_Write(block, buffer_ultd, 16);
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    //Serial.println("Status failed_write");
    goto end_ultd;
  }
  else check = 1;
end_ultd:
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  time_check = 0;
  Serial.println(check);
  return check;
}

byte RFID_new_card()   //----------------------------------------------------------------------------------RFID_card Reader----------------------------------------//
{
  // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
  unsigned int check_ncard = 0;
  byte check = 1, b_s;
  unsigned char buffer_newid[18] = {1, 18, 89, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  unsigned char buffer_newid_status[18] = {1, 18, 89, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  unsigned char buffer_status[18] = {0};
  byte buffer[34] = {0};
  byte block;
  byte len;

  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  if ( ! mfrc522.PICC_IsNewCardPresent())
    return 0;

  if ( ! mfrc522.PICC_ReadCardSerial())
    return 0;
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  MFRC522::StatusCode status;
  Serial.setTimeout(20000L) ;

  for (int i = 0; i < 5; i++)
  {
    buffer2[i] = EEPROM.read(500 + i);
  }
  for (byte x = 0; x < 5; x++)
  {
    buffer_newid[x] = buffer2[x];
    //buffer_newid_status[x]=buffer2[x];
  }
  for (byte i = 0; i < 5; i++)
  {
    if (i < 18)
      buffer[i] = buffer2[i];    // pad with spaces
    else
      buffer[i] = 0;   // pad with
  }
  block = 4;
  len = 18;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid_new;
  }
  status = mfrc522.MIFARE_Read(block, buffer1, &len);
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid_new;
  }
  // for (int i = 0; i < 19; i++) Serial.print(buffer1[i]);
  if ((buffer1[7] == buffer3[7]) || ( buffer1[8] == buffer3[8]) || (buffer1[7] == buffer2[7]) || (buffer1[8] == buffer2[8]) || (buffer1[7] == buffer4[7]) || (buffer1[8] == buffer4[8]))
  {
    lcd.clear();
    lcd.setCursor(3, 1);
    lcd.print("Master card");
    delay(2000);
    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.print("Place a new card ");
    goto end_rfid_new;
  }

  block = 1;
  len = 18;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 1, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file

  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto  end_rfid_new;
  }

  status = mfrc522.MIFARE_Read(block, buffer_status, &len);

  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid_new;
  }

  for (b_s = 0; b_s <= 15; b_s++)
  {
    if (buffer_status[b_s] == buffer_newid_status[b_s])
      continue;
    else break;
  }
  if (b_s == 16)
  {
    check = 2; goto  end_rfid_new;
  }

  block = 4;
  len = 18;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file

  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid_new;
  }

  status = mfrc522.MIFARE_Write(block, buffer, 16);

  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid_new;
  }

  block = 1;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 1, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file

  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid_new;
  }

  status = mfrc522.MIFARE_Write(block, buffer_newid_status, 16);
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid_new;
  }

  if (check == 1)
  {
    if ((EEPROM.read(200) == 255) && (EEPROM.read(201) == 255))
    {
      EEPROM.write(200, 0);
      EEPROM.write(201, 0);
    }

    check_ncard = EEPROM.read(200) * 255 + EEPROM.read(201);
    check_ncard = check_ncard + 1;
    EEPROM.write(200, ((check_ncard >> 8) & 0xFF));
    EEPROM.write(201, check_ncard & 0xFF);
  }

end_rfid_new :
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  time_check = 0;
  return check;
}

byte my_RFID_card()
{
  byte check = 1, limit_check, check_inc = 0, check_dd = 0, check_mm = 0, check_yy = 0, owner_card = 0;
  byte buffer_w[16], buffer_d[16];
  byte block, len = 18;

  unsigned int ser_no = 0;
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  if ( ! mfrc522.PICC_IsNewCardPresent())
    return 0;

  if ( ! mfrc522.PICC_ReadCardSerial())
    return 0;

  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);

  MFRC522::StatusCode status;
  Serial.setTimeout(20000L) ;     // wait until 20 seconds for input from serial

  block = 4;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK)
  {
    check = 5;
    goto end_rfid;
  }

  status = mfrc522.MIFARE_Read(block, buffer1, &len);
  if (status != MFRC522::STATUS_OK)
  {
    check = 5;
    goto end_rfid;
  }

  if ((buffer1[7] == buffer4[7]) && (buffer1[8] == buffer4[8]))
  {
    check = 9;
    goto end_rfid;
  }

  block = 4;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK)
  {
    check = 5;
    goto end_rfid;
  }
  status = mfrc522.MIFARE_Read(block, buffer1, &len);
  if (status != MFRC522::STATUS_OK)
  {
    check = 5;
    goto end_rfid;
  }
  for (byte i = 0; i < 5; i++)
    ser_no = (ser_no * 10) + buffer1[i];

  if (ser_no != serial_num)
  {
    lcd.clear();
    lcd.setCursor(1, 0); lcd.print("Sorry .....");
    lcd.setCursor(1, 1); lcd.print("Invalid card ....");
    lcd.setCursor(1, 2); lcd.print("AVL BAL:");
    amt_o = ((buffer1[5]) << 8 & 0xff00) + buffer1[6];
    check = 5;
    lcd.setCursor(10, 2); lcd.print(amt_o);
    delay(500);
    goto end_rfid;
  }
  if (buffer1[7] == buffer2[7]) menu_no++;
  if (buffer1[8] == buffer2[8]) menu_no++;
  if (buffer1[7] == buffer3[7]) TS_No++;
  if (buffer1[8] == buffer3[8]) TS_No++;
  if (menu_no == 2)
  {
    // menu_no=10;
    check = 1; goto end_rfid;
  }
  if (TS_No == 2)
  {
    // TS_No=10;
    check = 1; goto end_rfid;
  }

  if ( (flag_int1 == 0) && (no_of_tap >= 1))
    key_r = 1;
  else if (( flag_int2 == 0) && (no_of_tap >= 2))
    key_r = 2;
  else
  {
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.write(" Taps Are Busy");
    delay(2000);
    check = 1; goto end_rfid;
  }
  block = 5;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 5, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
  }
  status = mfrc522.MIFARE_Read(block, buffer_d, &len);
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
  }

  check_inc = buffer_d[0];
  check_dd = buffer_d[1];
  check_mm = buffer_d[2];
  check_yy = buffer_d[3];
  if (buffer_d[7] == 1) check_inc = 0;

  amt_o = ((buffer1[5]) << 8 & 0xff00) + buffer1[6];

  {
    limit_check = EEPROM.read(71);
    test_dd = EEPROM.read(113);
    test_mm = EEPROM.read(114);
    if ((check_dd != test_dd) || (check_mm != test_mm))
    {
      check_inc = 0; check_dd = test_dd;    check_mm = test_mm;
    }
    if ((check_inc < limit_check) && (check_dd == test_dd) && (check_mm == test_mm))
    {
      lcd.clear();
      //           reg_amt=0;

      B_amt = amt_o;
      unsigned long int  amt_swp1 = 0, amt_ltr1 = 0;
      amt_swp = 0; amt_ltr = 0;
      amt_swp = EEPROMReadlong(125);
      amt_ltr = EEPROMReadlong(129);

      amt_swp1 = EEPROMReadlong(105);
      amt_ltr1 = EEPROMReadlong(109);

      amt_o = amt_o - EEPROM.read(36 + ((key_r - 1) * 7));
      amt_swp = amt_swp + EEPROM.read(36 + ((key_r - 1) * 7));
      amt_ltr = amt_ltr + EEPROM.read(30 + ((key_r - 1) * 7));

      amt_swp1 = amt_swp1 + EEPROM.read(36 + ((key_r - 1) * 7));
      amt_ltr1 = amt_ltr1 + EEPROM.read(30 + ((key_r - 1) * 7));

      if (amt_o > 255) buffer1[5] = amt_o / 256;
      else  buffer1[5] = 0;
      buffer1[6] = amt_o % 256;
      unsigned int amt_c = 0, amt_c1 = 1000;
      if ((amt_o <= amt_c) || (amt_o > amt_c1))
      {
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Sorry .....");
        lcd.setCursor(1, 2);
        lcd.print("Insufficent balance");
        delay(1000);
        lcd.clear();
        key_p = 0;
        check = 5; goto end_rfid;
      }

      check_inc++;
      if (buffer_d[7] == 1) check_inc = 0;
      buffer_d[0] = check_inc;
      buffer_d[1] = check_dd;
      buffer_d[2] = check_mm;

      block = 5;
      status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 5, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
      if (status != MFRC522::STATUS_OK)
      {
        check = 5;
      }
      status = mfrc522.MIFARE_Write(block, buffer_d, 16);
      if (status != MFRC522::STATUS_OK)
      {
        check = 5;
      }

      EEPROMWritelong(125, amt_swp);
      EEPROMWritelong(129, amt_ltr);

      EEPROMWritelong(105, amt_swp1);
      EEPROMWritelong(109, amt_ltr1);
     
      Serial.println(amt_swp);
      Serial.println(amt_ltr);
      Serial.println(amt_swp1);
      Serial.println(amt_ltr1);
      A_amt = amt_o;
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.write("Your Limit Over ");
      delay(500);
      lcd.clear();
      check = 4; goto end_rfid;
    }
  }
  for (byte i = 0; i < 16; i++)   buffer_w[i] = '\0';    // pad with spaces

  for (byte i = 0; i < 16; i++)  buffer_w[i] = buffer1[i];    // pad with spaces

  block = 4;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.println("Authenticate failed");
    check = 0;
  }
  status = mfrc522.MIFARE_Write(block, buffer_w, 16);
  if (status != MFRC522::STATUS_OK) {
    check = 0;
    Serial.println("Write failed");
    check_rfid_status = 1;
    status_write_flag = 1;
  }
  else check_rfid_status = 0;
  Serial.println(check);
  check = 2;
end_rfid :
  mfrc522.PICC_HaltA(); // Halt PICC
  mfrc522.PCD_StopCrypto1();  // Stop encryption on PCD
  Serial.println(check);
  return check;
}

void rfid_status_write()
{
  unsigned long int  amt_swp1 = 0, amt_ltr1 = 0;
  amt_swp = 0; amt_ltr = 0;
  amt_swp = EEPROMReadlong(125);
  amt_ltr = EEPROMReadlong(129);

  amt_swp1 = EEPROMReadlong(105);
  amt_ltr1 = EEPROMReadlong(109);

  amt_swp = amt_swp + EEPROM.read(36 + ((key_r - 1) * 7));
  amt_ltr = amt_ltr + EEPROM.read(30 + ((key_r - 1) * 7));

  amt_swp1 = amt_swp1 + EEPROM.read(36 + ((key_r - 1) * 7));
  amt_ltr1 = amt_ltr1 + EEPROM.read(30 + ((key_r - 1) * 7));

  EEPROMWritelong(125, amt_swp);
  EEPROMWritelong(129, amt_ltr);

  EEPROMWritelong(105, amt_swp1);
  EEPROMWritelong(109, amt_ltr1);
}

byte RFID_card_write(byte reg_amt)   /////---------------------------------------------RFID_card_write_dis() ----------------------------//
{
  byte check = 1, limit_check, check_inc = 0, check_dd = 0, check_mm = 0, check_yy = 0, owner_card = 0;
  byte buffer_w[16], buffer_d[16];
  byte block, len = 18;

  unsigned int ser_no = 0;
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  if ( ! mfrc522.PICC_IsNewCardPresent())
    return 0;

  if ( ! mfrc522.PICC_ReadCardSerial())
    return 0;

  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);

  MFRC522::StatusCode status;
  Serial.setTimeout(20000L) ;     // wait until 20 seconds for input from serial

  block = 4;
  len = 18;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid;
  }
  status = mfrc522.MIFARE_Read(block, buffer1, &len);
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid;
  }
  if ((buffer1[7] == buffer3[7]) || ( buffer1[8] == buffer3[8]) || (buffer1[7] == buffer2[7]) || (buffer1[8] == buffer2[8]) || (buffer1[7] == buffer4[7]) || (buffer1[8] == buffer4[8]))
  {
    lcd.clear();
    lcd.setCursor(3, 1);
    lcd.print("Master card");
    delay(2000);
    //check = 1;
    lcd.clear();
    goto end_rfid;
  }

  block = 4;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid;
  }
  status = mfrc522.MIFARE_Read(block, buffer1, &len);
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid;
  }

  block = 5;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 5, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid;
  }
  status = mfrc522.MIFARE_Read(block, buffer_d, &len);
  if (status != MFRC522::STATUS_OK)
  {
    check = 0;
    goto end_rfid;
  }

  check_inc = buffer_d[0];
  check_dd = buffer_d[1];
  check_mm = buffer_d[2];
  check_yy = buffer_d[3];

  for (byte i = 0; i < 5; i++)
    ser_no = (ser_no * 10) + buffer1[i];
  owner_card = buffer1[12];
  if (ser_no != serial_num)
  {
    lcd.clear();
    lcd.setCursor(1, 1); lcd.print("Sorry .....");
    lcd.setCursor(1, 2); lcd.print("Invalid card ....");
    check = 2; goto end_rfid;
  }
  amt_o = (buffer1[5] * 256) + buffer1[6];
  if (reg_amt == 1)
  {
    reg_amt = 0;
    if (amt_o > 1000)
    {

      lcd.clear();
      lcd.setCursor(1, 1); lcd.print("Sorry .....");
      lcd.setCursor(1, 2); lcd.print("Max Recharge over..");
      check = 2; goto end_rfid;
    }
    unsigned long int amt_rg1 = 0;
    amt_rg = 0;
    lcd.setCursor(14, 0);     lcd.print(amt_o); lcd.print(" ");
    amt_rg = EEPROMReadlong(121);
    amt_rg1 = EEPROMReadlong(101);
    if (key_r == 1)  {
      amt_o = amt_o + 5;
      amt_rg = amt_rg + 5;
      amt_rg1 = amt_rg1 + 5;
    }
    else if (key_r == 2)  {
      amt_o = amt_o + 20;
      amt_rg = amt_rg + 20;
      amt_rg1 = amt_rg1 + 20;
    }
    else if (key_r == 3)  {
      amt_o = amt_o + 50;
      amt_rg = amt_rg + 50;
      amt_rg1 = amt_rg1 + 50;
    }
    else if (key_r == 4)
    {
      amt_o = amt_o + 1;
      amt_rg = amt_rg + 1;
      amt_rg1 = amt_rg1 + 1;
    }
    buffer1[5] = amt_o / 256;
    buffer1[6] = amt_o % 256;
    lcd.setCursor(14, 1);
    lcd.print(amt_o); lcd.print(" ");

    EEPROMWritelong(121, amt_rg);
    EEPROMWritelong(101, amt_rg1);

  }
  if (reg_amt == 2)
  {
    limit_check = EEPROM.read(71);
    test_dd = EEPROM.read(113);
    test_mm = EEPROM.read(114);
    if ((check_dd != test_dd) || (check_mm != test_mm))
    {
      check_inc = 0; check_dd = test_dd;    check_mm = test_mm;
    }
    if ((check_inc < limit_check) && (check_dd == test_dd) && (check_mm == test_mm))
    {
      lcd.clear();
      reg_amt = 0;
      lcd.setCursor(14, 0);
      lcd.print(amt_o);
      lcd.print(" ");
      unsigned long int  amt_swp1 = 0, amt_ltr1 = 0;
      amt_swp = 0; amt_ltr = 0;
      amt_swp = EEPROMReadlong(125);
      amt_ltr = EEPROMReadlong(129);

      amt_swp1 = EEPROMReadlong(105);
      amt_ltr1 = EEPROMReadlong(109);

      amt_o = amt_o - EEPROM.read(36 + ((key_r - 1) * 7));
      amt_swp = amt_swp + EEPROM.read(36 + ((key_r - 1) * 7));
      amt_ltr = amt_ltr + EEPROM.read(30 + ((key_r - 1) * 7));

      amt_swp1 = amt_swp1 + EEPROM.read(36 + ((key_r - 1) * 7));
      amt_ltr1 = amt_ltr1 + EEPROM.read(30 + ((key_r - 1) * 7));


      if (amt_o > 255) buffer1[5] = amt_o / 256;
      else  buffer1[5] = 0;
      buffer1[6] = amt_o % 256;

      unsigned int amt_c = 1000;
      if (amt_o > amt_c)
      {
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Sorry .....");
        lcd.setCursor(1, 2);
        lcd.print("Insufficent balance");
        delay(1000);
        lcd.clear();
        key_p = 0;
        check = 2; goto end_rfid;
      }
      check_inc++;
      buffer_d[0] = check_inc;
      buffer_d[1] = check_dd;
      buffer_d[2] = check_mm;

      block = 5;
      status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 5, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
      if (status != MFRC522::STATUS_OK)
      {
        check = 2;
      }
      status = mfrc522.MIFARE_Write(block, buffer_d, 16);
      if (status != MFRC522::STATUS_OK)
      {
        check = 2;
      }

      EEPROMWritelong(125, amt_swp);
      EEPROMWritelong(129, amt_ltr);

      EEPROMWritelong(105, amt_swp1);
      EEPROMWritelong(109, amt_ltr1);

      lcd.setCursor(14, 1);
      lcd.print(amt_o);
      lcd.write(" ");
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.write("Your Limit Over ");
      delay(500);
      lcd.clear();
      check = 2; goto end_rfid;
    }
  }
  for (byte i = 0; i < 16; i++)   buffer_w[i] = '\0';    // pad with spaces

  for (byte i = 0; i < 16; i++)  buffer_w[i] = buffer1[i];    // pad with spaces

  block = 4;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    check = 2;
    goto end_rfid;
  }
  status = mfrc522.MIFARE_Write(block, buffer_w, 16);
  if (status != MFRC522::STATUS_OK) {
    check = 2;
    goto end_rfid;
  }

end_rfid :
  mfrc522.PICC_HaltA(); // Halt PICC
  mfrc522.PCD_StopCrypto1();  // Stop encryption on PCD
  return check;
}


byte RFID_card_copy_past(byte key_cp)   /////---------------------------------------------RFID_card_write_copy_past() ----------------------------//
{
  byte check = 1;
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  if ( ! mfrc522.PICC_IsNewCardPresent())
    return 0;

  if ( ! mfrc522.PICC_ReadCardSerial())
    return 0;

  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  byte buffer_[5][18] = {0};
  byte block, len, arr = 0;
  MFRC522::StatusCode status;

  Serial.setTimeout(20000L) ;     // wait until 20 seconds for input from serial

  if (key_cp == 1) //--------------------------------------------------------------------------------------pasting function over--------------------------------------------------//
  {
    key_cp = 0;
    len = 18;
    for ( block = 5; block < 11; block++)
    {
      if (block == 7)
        continue;
      status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
      if (status != MFRC522::STATUS_OK)
      {
        check = 0;
        goto end_rfid;
      }
      status = mfrc522.MIFARE_Read(block, buffer_[arr], &len);
      if (status != MFRC522::STATUS_OK)
      {
        check = 0;
        goto end_rfid;
      }
      arr++;
    }
    for (byte i = 0; i < 80; i++)
    {
      if (i < 16)
        EEPROM.write(i, buffer_[0][i % 16]);
      else if ((i > 15) && (i < 32))
        EEPROM.write(i, buffer_[1][i % 16]);
      else if ((i > 31) && (i < 48))
        EEPROM.write(i, buffer_[2][i % 16]);
      else if ((i > 47) && (i < 64))
        EEPROM.write(i, buffer_[3][i % 16]);
      else if ((i > 63) && (i < 80))
        EEPROM.write(i, buffer_[4][i % 16]);
    }
  }
  if (key_cp == 2) //--------------------------------------------------------------------------------------copying function--------------------------------------------------//
  {
    key_cp = 0;
    //   Serial1.println(" copying...........");
    for (byte i = 0; i < 80; i++)
    {
      if (i < 16)
        buffer_[0][i % 16] = EEPROM.read(i);
      else if ((i > 15) && (i < 32))
        buffer_[1][i % 16] = EEPROM.read(i);
      else if ((i > 31) && (i < 48))
        buffer_[2][i % 16] = EEPROM.read(i);
      else if ((i > 47) && (i < 64))
        buffer_[3][i % 16] = EEPROM.read(i);
      else if ((i > 63) && (i < 80))
        buffer_[4][i % 16] = EEPROM.read(i);
    }

    for ( block = 5; block < 11; block++)
    {
      if (block == 7)
        continue;
      status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
      if (status != MFRC522::STATUS_OK)
      {
        check = 0;
        goto end_rfid;
      }
      status = mfrc522.MIFARE_Write(block, buffer_[arr], 16);
      if (status != MFRC522::STATUS_OK)
      {
        check = 0;
        goto end_rfid;
      }
      arr++;
    }
  }
end_rfid :
  mfrc522.PICC_HaltA(); // Halt PICC
  mfrc522.PCD_StopCrypto1();  // Stop encryption on PCD
  return check;
}

//----------------------------------------------------------------------------------Phone Number change-------------------------------//
void Phone_no_edit(byte addr, byte pos)
{
  int currentStat = 0, previousStat = 0, counte = 2, v, counterx1;
  byte cur = 1;
  while (1)
  {
    if (cur == 1)
    {
      lcd.setCursor(1, pos);
      lcd.write("> ");
      lcd.setCursor(3, pos);
      cur = 0;
    }
    for (v = 2; v < 12; v++)
    {
      time_check = 0;
      counterx1 = temp_no[v - 2];
      while (1)
      {
        counterx1 = up_down(counterx1);

        if (counterx1 >= 9)  counterx1 = 9;
        if (counterx1 < 0)  counterx1 = 0;

        lcd.setCursor(v + 1, pos);
        lcd.print(counterx1);
        lcd.setCursor(v + 1, pos);
        temp_no[v - 2] = counterx1;
        delay(10);

        if (front_key())      break;
        if ( enter_key())    goto end_phoneedit;
        if (time_check == 5) goto end_phoneedit;
      }
    }
  }
end_phoneedit :
  time_check = 0;
  for (int i = 0; i < 10; i++)
    EEPROM.write(addr + i, temp_no[i]);
  delay(10);
  ff = 0;
}
//---------------------------------------------------------------------------------------------GSM SETTINGs ------------------------------------------------------------//
void Gsm_Setting()
{
  int counte = 0, currentStat = 0, previousStat = 0;
  coun = 2;
  lcd.clear();
  lcd.setCursor(3, 0);
  for (int i = 0; i < 10; i++)
  {
    admin_no[i] = EEPROM.read(i);
    lcd.print(admin_no[i]);
    delay(10);
  }
  lcd.print(" Admin");
  lcd.setCursor(3, 1);
  for (int i = 10; i < 20; i++)
  {
    user_no[i - 10] = EEPROM.read(i);
    lcd.print(user_no[i - 10]);
    delay(10);
  }
  lcd.print(" USER");
  lcd.setCursor(3, 2);
  for (int i = 20; i < 30; i++)
  {
    paytm_no[i - 20] = EEPROM.read(i);
    lcd.print(paytm_no[i - 20]);
    delay(10);
  }
  lcd.print(" Paytm");

  ff = 0;
  while (1)  //---------------------------------------main while for up & down in gsm------------------------
  {
    counte = 1; fc = 0;

    if (ff == 0)
    {
      while (1)
      {
        coun = up_down(coun);
        if (coun < 0)  coun = 2;
        if (coun > 2)  coun = 0;

        if (coun == 2)
        {
          lcd.setCursor(0, 2);
          lcd.write("  ");
          lcd.setCursor(0, 0);
          lcd.write("> ");
          lcd.setCursor(0, 1);
          lcd.write("  ");
          fc = 1;
          if (back_key())
            goto endq;
          if (time_check == 5)
            goto endq;
        }

        if (coun == 1)
        {
          lcd.setCursor(0, 0);
          lcd.write("  ");
          lcd.setCursor(0, 1);
          lcd.write("> ");
          lcd.setCursor(0, 2);
          lcd.write("  ");
          fc = 2;
          if (back_key())
            goto endq;
          if (time_check == 5)
            goto endq;
        }
        if (coun == 0)
        {
          lcd.setCursor(0, 1);
          lcd.write("  ");
          lcd.setCursor(0, 2);
          lcd.write("> ");
          lcd.setCursor(0, 0);
          lcd.write("  ");
          fc = 3;
          if (back_key())
            goto endq;
          if (time_check == 5)
            goto endq;
        }

        lcd.noCursor();
        if (enter_key())
        {
          ff = 1;
          delay(1000);
          lcd.cursor();
          break;

        }
      }
    }

    if (fc == 1)        //##-------------------------------------------------------------while ADMIN-----------------------------------
    {
      for (int i = 0; i < 10; i++)
        temp_no[i] = admin_no[i];
      Phone_no_edit(0, 0);
    }
    if (fc == 2) //###--------------------------------------------------------while USER-------------------------------------//
    {
      for (int i = 0; i < 10; i++)
        temp_no[i] = user_no[i];
      Phone_no_edit(10, 1);
    }
    if (fc == 3)   //###--------------------------------------------------------paytm-------------------------------------//
    {
      for (int i = 0; i < 10; i++)
        temp_no[i] = paytm_no[i];
      Phone_no_edit(20, 2);
    }
    lcd.noCursor();
  }                             //-------------------------main while up & down end------------------------------------------
endq :
  time_check = 0;
  ff = 1;
  lcd.clear();
  flag_2 = 1;
}

void Default_setting() //------------------------------------------------------------Default_setting-------------------------------------------------------------//
{
  byte x = 0;
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.write(" long press enter ");
  lcd.setCursor(1, 2);
  lcd.write(" or Press back  ");
  while (x < 100)
  {
    if (!digitalRead(enter))
      x++;
    else if (back_key())
      goto end_def;
    else     x = 0;
    delay(20);
    if (time_check == 5)
      goto end_def;
  }
  for (int i = 0; i < 100; i++)
    EEPROM.write(i, 0);

  lcd.setCursor(1, 2);
  EEPROM.write(71, 1);
  EEPROM.write(70, 1);
  //  EEPROM.write(150,0);
  lcd.write("    Ok Done   ");
  delay(2000);
end_def:
  time_check = 0;
  flag_2 = 1;
}

void Recharge_mode() //------------------------------------------------------------Recharge_mode-------------------------------------------------------------//
{
  byte Reg_staus;
first_r :
  time_check = 0;
  key_r = 0;
  amt_o = 0;
  digitalWrite(Buzzer, HIGH);
  delay(250);
  digitalWrite(Buzzer, LOW);
  lcd.clear();
  lcd.print(" recharge sucess");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write("Old Balance : ");
  lcd.setCursor(0, 1);
  lcd.write("New Balance : ");
  lcd.setCursor(0, 2);
  lcd.write("Add Amount  : ");
  lcd.setCursor(0, 3);
  lcd.write("Recharge Mode Active");
  delay(1500);



  while (1)
  {
    if (!digitalRead(up))
    {
      key_r = 1;
      lcd.setCursor(14, 2);
      lcd.print(5);
      lcd.print("  ");
    }
    if (front_key())
    {
      key_r = 2;
      lcd.setCursor(14, 2);
      lcd.print(20);
      lcd.print("  ");
    }
    if (!digitalRead(down))
    {
      key_r = 3;
      lcd.setCursor(14, 2);
      lcd.print(50);
      lcd.print(" ");
    }
    if (enter_key())
    {
      key_r = 4;
      lcd.setCursor(14, 2);
      lcd.print(1);
    }
    delay(10);
    Reg_staus = RFID_card_write(1);
    if (Reg_staus == 1)
    {
      delay(1500);
      goto first_r;

    }
    if (Reg_staus == 2)
    { lcd.print("recharge failed");
      delay(1000);
      goto first_r;
    }
    if (back_key() || (time_check == 5))  break;
  }
  delay(1000);
  time_check = 0;
  lcd.clear();
  flag_2 = 1;
}
byte RTC_Edit(int rtc_value, int counter) //--------------------------------------------------------------------------RTC Value Edit-----------------------------------------------//
{
  while (1)
  {

    counter = up_down(counter);

    if (rtc_value == 2)
    {
      lcd.setCursor(0, 3);
      if (counter > 23 || counter < 0)
        counter = 0;
    }
    else   if (rtc_value == 1)
    {
      lcd.setCursor(3, 3);
      if (counter > 59 || counter < 0)
        counter = 0;
    }
    else   if (rtc_value == 4)
    {
      lcd.setCursor(6, 3);
      if (counter > 31 || counter < 1)
        counter = 1;
    }
    else   if (rtc_value == 5)
    {
      lcd.setCursor(9, 3);
      if (counter > 12 || counter < 1)
        counter = 1;
    }
    else   if (rtc_value == 6)
    {
      lcd.setCursor(14, 3);
      if (counter > 100 || counter < 0)   counter = 0;
    }

    lcd.print(counter / 10);
    lcd.print(counter % 10);
    if ( front_key())
    {
      time_check = 0;
      RTC_Write(rtc_value, ((counter / 10) << 4) + (counter % 10));
      return 0;
    }
    if ( back_key() || (time_check == 5))
    {
      time_check = 0;
      RTC_Write(rtc_value, ((counter / 10) << 4) + (counter % 10));
      return 1;
    }
  }
}

void TIME_DATE() //---------------------------------------------------------------------------------------------Time & Date Edit --------------------------------------------------------
{
  byte hr, m, dd, mm, yy, x, x1, x2, x3, x4, x5, i = 0;
first_rtc :
  lcd.clear();
  lcd.setCursor(0, 3);
  hr = RTC_Read(2);
  x1 = (hr >> 4) * 10;
  x = hr & 0x0f;
  x1 = x1 + x;
  lcd.print(x1 / 10);
  lcd.print(x1 % 10);
  lcd.print(":");

  m = RTC_Read(1);
  x2 = (m >> 4) * 10;
  x = m & 0x0f;
  x2 = x2 + x;
  lcd.print(x2 / 10);
  lcd.print(x2 % 10);
  lcd.print(" ");

  dd = RTC_Read(4);
  x3 = (dd >> 4) * 10;
  x = dd & 0x0f;
  x3 = x3 + x;
  lcd.print(x3 / 10);
  lcd.print(x3 % 10);
  lcd.print("/");

  mm = RTC_Read(5);
  x4 = (mm >> 4) * 10;
  x = mm & 0x0f;
  x4 = x4 + x;
  lcd.print(x4 / 10);
  lcd.print(x4 % 10);
  lcd.print("/");

  yy = RTC_Read(6);
  x5 = (yy >> 4) * 10;
  x = yy & 0x0f;
  x5 = x5 + x;
  lcd.print(20);
  lcd.print(x5 / 10);
  lcd.print(x5 % 10);
  delay(10);
  while (1)
  {
    lcd.setCursor(0, 2);
    lcd.write("Hr                 ");
    lcd.setCursor(0, 3);
    i = RTC_Edit(2, x1);                        //---------------------------------------------------Hours Edit-------------------------------------------//
    if (i == 1)
      goto end_rtc;

    lcd.setCursor(0, 2);
    lcd.write("                     ");
    lcd.setCursor(3, 2);
    lcd.write("mn");
    lcd.setCursor(3, 3);
    i = RTC_Edit(1, x2);                         //---------------------------------------------------Minutes Edit-------------------------------------------//
    if (i == 1)
      goto end_rtc;

    lcd.setCursor(0, 2);
    lcd.write("                    ");
    lcd.setCursor(6, 2);
    lcd.write("dd");
    lcd.setCursor(6, 3);
    i = RTC_Edit(4, x3);                         //---------------------------------------------------Date Edit-------------------------------------------//
    if (i == 1)
      goto end_rtc;

    lcd.setCursor(0, 2);
    lcd.write("                    ");
    lcd.setCursor(9, 2);
    lcd.write("mm");
    lcd.setCursor(9, 3);
    i = RTC_Edit(5, x4);                         //---------------------------------------------------Month Edit-------------------------------------------//
    if (i == 1)
      goto end_rtc;

    lcd.setCursor(0, 2);   lcd.write("                    ");
    lcd.setCursor(12, 2);  lcd.write("year");
    lcd.setCursor(14, 3);  i = RTC_Edit(6, x5);                         //---------------------------------------------------year Edit-------------------------------------------//
    if (i == 1)  goto end_rtc;
    if (i == 0)  goto first_rtc;
  }
end_rtc :
  time_check = 0;
  flag_1 = 1;
  lcd.clear();
}

int up_down(int count_up_down)  //------------------------------------------------------------------------- UP(9)  & Down(8) ----------------------------------------------------//
{

  if (!digitalRead(up))   currentState = 1;
  else currentState = 0;

  if (currentState != previousState) {
    if (currentState == 1) {
      time_check = 0;
      count_up_down = count_up_down + 1;
    }
  }
  previousState = currentState;
  if (!digitalRead(down)) currentState1 = 1;
  else currentState1 = 0;

  if (currentState1 != previousState1) {
    if (currentState1 == 1) {
      time_check = 0;
      count_up_down = count_up_down - 1;
    }
  }
  previousState1 = currentState1;
  return count_up_down;
}

void EEPROM_read(byte i)    //------------------------------------------------------------------------- EEPROM Read----------------------------------------------------//
{
  byte k1, k2, f11, f12, f21, f22;
  k = EEPROM.read(30 + i);

  f11 = EEPROM.read(32 + i);
  f12 = EEPROM.read(33 + i);
  f21 = EEPROM.read(34 + i);
  f22 = EEPROM.read(35 + i);
  rs = EEPROM.read(36 + i);

  f1 = f11 * 256 + f12;
  f2 = f21 * 256 + f22;
}
void EEPROM_write(byte i)    //------------------------------------------------------------------------- EEPROM Write ----------------------------------------------------//
{
  EEPROM.write(30 + i, k);
  EEPROM.write(32 + i, (f1 >> 8 ) & 0xFF);
  // EEPROM.write(32+i,0);
  EEPROM.write(33 + i, f1 & 0xff);
  EEPROM.write(34 + i, (f2 >> 8) & 0xFF);
  //   EEPROM.write(34+i,0);
  EEPROM.write(35 + i, f2 & 0xFF);
  EEPROM.write(36 + i, rs);
}
byte enter_key()    //------------------------------------------------------------------------- Enter key (11)----------------------------------------------------//
{
  if (!digitalRead(enter))   e_currentState = 1;
  else e_currentState = 0;

  if (e_currentState != e_previousState) {
    if (e_currentState == 1) {
      e_previousState = e_currentState;
      time_check = 0;
      return 1;
    }
  }
  e_previousState = e_currentState;
  return 0;
}
byte back_key()       //------------------------------------------------------------------------- Back key (12)----------------------------------------------------//
{

  if (!digitalRead(back))   b_currentState = 1;
  else b_currentState = 0;

  if (b_currentState != b_previousState) {
    if (b_currentState == 1) {
      b_previousState = b_currentState;
      time_check = 0;
      return 1;
    }
  }
  b_previousState = b_currentState;
  return 0;
}
byte front_key()         //------------------------------------------------------------------------- fron_key (13)----------------------------------------------------//
{
  if (!digitalRead(forward))   f_currentState = 1;
  else f_currentState = 0;

  if (f_currentState != f_previousState) {
    if (f_currentState == 1) {
      f_previousState = f_currentState;
      time_check = 0;
      return 1;
    }
  }
  f_previousState = f_currentState;
  return 0;
}

void No_of_cards()
{
  unsigned int total_cards;
  if ((EEPROM.read(200) >= 255) && (EEPROM.read(201) >= 255))
  {
    EEPROM.write(200, 0); EEPROM.write(201, 0);
  }
  total_cards = ((EEPROM.read(200) << 8) & 0xFF00) + ((EEPROM.read(201)) & 0xFF);
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("No.of cards using");
  while (1)
  {
    lcd.setCursor(9, 2);
    lcd.print(total_cards);
    delay(100);

    if (back_key())    goto end_ncard;
    if (time_check == 4) goto end_ncard;
  }
end_ncard :
  time_check = 0;
  flag_1 = 1;
  lcd.clear();
}
void New_Rfid_card()
{
  byte id_new = 0;
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Place a new card ");
  while (1)
  {
    id_new = RFID_new_card();
    if (id_new == 1)
    {
      lcd.setCursor(1, 2);
      lcd.print("     OK Done     ");
      delay(1000);
    }
    else     if (id_new == 2)
    {
      lcd.setCursor(1, 2);
      lcd.print(" Already Activated ");
      delay(1000);
    }
    else
    {
      lcd.setCursor(1, 2); lcd.print("                  ");
    }
    if (enter_key())   break;
    if (back_key())    goto end_dis;
    if (time_check == 4) goto end_dis;
  }
end_dis :
  time_check = 0;
  flag_4 = 1;
  lcd.clear();
}

void clear_status() //------------------------------------------------------------Default_setting-------------------------------------------------------------//
{
  byte x = 0;
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.write(" long press enter ");
  lcd.setCursor(1, 2);
  lcd.write(" for clear status  ");
  while (x < 100)
  {
    if (!digitalRead(enter))     x++;
    else if (back_key()) goto end_def;
    else     x = 0;
    delay(20);
    if (time_check == 4) goto end_def;
  }
  for (int i = 100; i < 140; i++)
    EEPROM.write(i, 0);

  lcd.setCursor(1, 2);
  test_dd_prev = EEPROM.read(113);
  EEPROM.write(150, 1);
  lcd.write("    Ok Done      ");
  delay(2000);
end_def:
  time_check = 0;

  flag_4 = 1;
  lcd.clear();
  delay(100);
}

void key_edit(byte key) // ------------------------------------------------------Key_Edit---------------------------------------------//
{
  int count = 3;
  if (key == 1)
    EEPROM_read(0);
  else if (key == 2)
    EEPROM_read(7);
  else if (key == 3)
    EEPROM_read(14);
  else if (key == 4)
    EEPROM_read(21);
  else if (key == 5)
    EEPROM_read(28);

  lcd.setCursor(3, 0);
  lcd.write("key");
  lcd.print(key);
  lcd.write("  ");
  lcd.print(k);
  lcd.print(" Liters");
  lcd.write("    ");

  lcd.setCursor(3, 1);
  lcd.write("F1    ");
  lcd.print(f1);
  lcd.write("    ");

  lcd.setCursor(3, 2);
  lcd.write("F2    ");
  lcd.print(f2);
  lcd.write("    ");

  lcd.setCursor(3, 3);
  lcd.write("Price ");
  lcd.print(rs);
  lcd.write("    ");

  if (enter_key())
  {
    time_check = 0;
    while (1)
    {
      count = up_down(count);

      if (count > 3)
        count = 0;
      if (count < 0)
        count = 3;

      if (count == 3)
      {
        lcd.setCursor(0, 3);
        lcd.write(" ");
        lcd.setCursor(0, 0);
        lcd.write("> ");
        lcd.setCursor(0, 1);
        lcd.write(" ");
      }
      else  if (count == 2)
      {
        lcd.setCursor(0, 0);
        lcd.write(" ");
        lcd.setCursor(0, 1);
        lcd.write("> ");
        lcd.setCursor(0, 2);
        lcd.write(" ");
      }
      else  if (count == 1)
      {
        lcd.setCursor(0, 1);
        lcd.write(" ");
        lcd.setCursor(0, 2);
        lcd.write("> ");
        lcd.setCursor(0, 3);
        lcd.write(" ");
      }
      else  if (count == 0)
      {
        lcd.setCursor(0, 2);
        lcd.write(" ");
        lcd.setCursor(0, 3);
        lcd.write("> ");
        lcd.setCursor(0, 0);
        lcd.write(" ");
      }

      if ((count == 3) && (enter_key()))
        while (1)
        {
          lcd.setCursor(1, 0);
          lcd.write(">");
          lcd.setCursor(9, 0);
          if (!digitalRead(up))
          {
            k++;  time_check = 0;
            delay(200);
          }
          if (!digitalRead(down))
          {
            k--;  time_check = 0;
            delay(200);
          }
          if (k > 200)  k = 0;
          if (k < 0)    k = 200;
          lcd.print(k);
          lcd.print(" Liters");
          lcd.print("     ");
          if ( enter_key() || (time_check == 5))
            break;
        }

      if ((count == 2) && (enter_key()))
        while (1)
        {
          lcd.setCursor(1, 1);
          lcd.write(">");
          lcd.setCursor(9, 1);
          if (!digitalRead(up))
          {
            f1++;  time_check = 0;
            delay(40);
          }
          if (!digitalRead(down))
          {
            f1--;  time_check = 0;
            delay(40);
          }
          if (f1 > 65535)
            f1 = 0;
          if (f1 < 0)
            f1 = 65535;
          lcd.print(f1);
          lcd.write("    ");
          if (enter_key() || (time_check == 5))
            break;
        }

      if ((count == 1) && (enter_key()))
        while (1)
        {
          lcd.setCursor(1, 2);
          lcd.write(">");
          lcd.setCursor(9, 2);
          if (!digitalRead(up))
          {
            f2++; time_check = 0;
            delay(40);
          }
          if (!digitalRead(down))
          {
            f2--;  time_check = 0;
            delay(40);
          }
          if (f2 > 65535)   f2 = 0;
          if (f2 < 0)       f2 = 65535;
          lcd.print(f2);
          lcd.write("    ");
          if (enter_key() || (time_check == 5))
            break;
        }
      if ((count == 0) && (enter_key()))
        while (1)
        {
          lcd.setCursor(1, 3);
          lcd.write(">");
          lcd.setCursor(9, 3);
          if (!digitalRead(up))
          {
            rs++; time_check = 0;
            delay(150);
          }
          if (!digitalRead(down))
          {
            rs--;
            time_check = 0;
            delay(150);
          }
          if (rs > 100) rs = 0;
          if (rs < 0)  rs = 100;
          lcd.print(rs);
          lcd.write("    ");

          if (enter_key() || (time_check == 5) )    break;

        }
      if (back_key() || (time_check == 5))
      {
        lcd.clear();   break;
      }
    }
  }
  if (key == 1)
    EEPROM_write(0);
  else if (key == 2)
    EEPROM_write(7);
  else if (key == 3)
    EEPROM_write(14);
  else if (key == 4)
    EEPROM_write(21);
  else if (key == 5)
    EEPROM_write(28);

}

void confg_volume()   //--------------------------------------------------- Edit-------------------------------------------//
{
  lcd.clear();
  byte key = 1;
  while (1)
  {
    if (key == 1)
      key_edit(1);
    else if (key == 2)
      key_edit(2);
    else if (key == 3)
      key_edit(3);
    else if (key == 4)
      key_edit(4);
    else if (key == 5)
      key_edit(5);

    if (back_key())
      break;
    if (time_check == 5)
      break;
    if (front_key())  key++;

    if (key > 5)  key = 1;
  }
  time_check = 0;
  flag_1 = 1;
  lcd.clear();

}
void setup()
{
  pinMode(17, OUTPUT);
  // wdt_enable(WDTO_8S);
  digitalWrite(17, HIGH);
  //byte flag_eeprom = EEPROM.read(242);
  Wire.begin();
  delay(100);
  Serial.begin(9600);    // Setting the baud rate of Serial Monitor (Arduino)
  Serial1.begin(9600);
  RTC_Write(0, 30);
  for (int i = 0; i <= 4; i++)
  {
    seriall[i] = EEPROM.read(500 + i);
  }
  for (int i = 0; i <= 4; i++)
  {
    serial_num = (serial_num * 10) + seriall[i];
  }
  //Serial.println(serial_num);
  if (serial_num > 0)status_check = 1;
  /*  if (flag_eeprom != 0)
    {
      for (byte e = 0; e < 255; e++)
        EEPROM.write(e, 0);
      EEPROM.write(150, 1);
      RTC_Write(0, 30);
      //Serial.println("ok");
    }*/
  //Timer1.initialize(100000 * 50 * 2);
  //timer1(TIMER1_PRESCALER_1024, 7812U, toggle);
  //sei();


  SPI.begin();                                                  // Init SPI bus
  mfrc522.PCD_Init();

  pinMode(down, INPUT_PULLUP); //down
  pinMode(up, INPUT_PULLUP); // up
  pinMode(enter, INPUT_PULLUP);//enter
  pinMode(back, INPUT_PULLUP);//back
  pinMode(forward, INPUT_PULLUP);//forward


  pinMode(T_SW_1, OUTPUT);
  digitalWrite(T_SW_1, HIGH);
  pinMode(T_SW_2, OUTPUT);
  digitalWrite(T_SW_2, HIGH);
  pinMode(BO_SW, OUTPUT);
  digitalWrite(BO_SW, LOW);
  pulseCount1 = 0;
  pulseCount2 = 0;
  pinMode(sensorPin1, INPUT_PULLUP);
  pinMode(sensorPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin1), pulseCounter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(sensorPin2), pulseCounter2, FALLING);
  home_screen();

  if (GSM_Active)
  {
    lcd.setCursor(0, 3);
    lcd.write("Network Searching...");
  }
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, HIGH);
  delay(800);
  lcd.clear();
  x_csq = 0;

  test_dd_prev = EEPROM.read(113);
  //test_dd_prev=test_dd;
  digitalWrite(Buzzer, LOW);
  /// wdt_enable(WDTO_8S);

}

void loop()//---------------------------------------------------------------------------------------------Loop Page------------------------------------------------------------//
{
  unsigned int lcd_check = 100, total_mins = 0;
  byte lcd_no = 0, hr_check = 0, min_check = 0, status_send = 0, sec_check = 0, server_send_status = 0;
  while (1)
  {
    serial_no = 0, id_no = 0, value_no = 0;
    if (menu_flag == 0)
    {
      //Serial.write(' ');

      if ((menu_on_off_1 == 0) && (menu_on_off_2 == 0))
      {
        if (wt_flag == 1)
        {
          // wdt_enable(WDTO_8S);
          mfrc522.PCD_Init();
          wt_flag = 0;
        }
        wdt_reset();

        if (lcd_check1 == 0)
        {
          lcd.begin(20, 4);
          lcd_check1 = 1;
        }
        if ((lcd_sec % 19) == 1)
        {
          lcd_no = 0;
        }
        if (lcd_no == 0)
          if ((lcd_sec % 19) == 0)
          {
            lcd.begin(20, 4);
            lcd_no = 1;
          }

        digitalWrite(BO_SW, LOW);
        lcd.setCursor(0, 0);
        lcd.write("     Touch CARD  ");
        if (GSM_Active)
        {
          lcd.setCursor(17, 0);
          lcd.write('G');
          if ((x_csq >= 0) && (x_csq <= 100))
            lcd.print(x_csq);
        }
        lcd.setCursor(0, 1);
        lcd.write("   GTR Solutions  ");
        lcd.setCursor(0, 2);
        lcd.print("Device ID:");
        lcd.print(serial_num);
        lcd.setCursor(0, 3);

        x = RTC_Read(2);                     //-----------------------------hr
        x1 = (x >> 4);
        lcd.print(x1);
        x = x & 0x0f;
        lcd.print(x);
        lcd.write(":");
        hr_check = x1 * 10 + x;

        x = RTC_Read(1);                   //---------------------------------min
        x1 = (x >> 4);
        lcd.print(x1);
        x = x & 0x0f;
        lcd.print(x);
        lcd.write(":");
        min_check = x1 * 10 + x;
        total_mins = hr_check * 60 + min_check;

        x = RTC_Read(0);                  //----------------------------------sec
        x1 = (x >> 4);
        lcd.print(x1);
        x = x & 0x0f;
        lcd.print(x);
        lcd.write(" ");
        sec_check = (x1 * 10 + x);

        lcd_sec = x1 * 10 + x;
        x = RTC_Read(4);                   //---------------------------------date
        x1 = (x >> 4);
        lcd.print(x1);
        x = x & 0x0f;
        lcd.print(x);
        lcd.write("/");
        date = x1 * 10 + x;

        x = RTC_Read(5);                   //----------------------------------month
        x1 = (x >> 4);
        lcd.print(x1);
        x = x & 0x0f;
        lcd.print(x);
        lcd.write("/");
        month_t = x1 * 10 + x;

        x = RTC_Read(6);                  //-----------------------------------year
        lcd.print(20);
        x1 = (x >> 4);
        lcd.print(x1);
        x = x & 0x0f;
        lcd.print(x);
        lcd.print(' ');
        year_t = x1 * 10 + x;

        if (GSM_Active)
        {
          if (status_send == 0)
          {
            //  if((total_mins>=540)&&(total_mins<541))
            if (total_mins % 60 == 0)
            {
              server_send_status = 1;
              Serial1.write('*');
              delay(400);
              String phone = "", pstr = "";
              byte i;
              for (i = 0; i < 10; i++)
                phone = phone + EEPROM.read(i);
              pstr += phone;
              pstr += '#';
              pstr += "\nSerial number:" + (String)serial_num + "\n";
              for (byte ts = 0; ts < 21; ts = ts + 20)
              {
                amt_rg = 0;    amt_swp = 0;    amt_ltr = 0;
                amt_rg = EEPROMReadlong(101 + ts);
                amt_swp = EEPROMReadlong(105 + ts);
                amt_ltr = EEPROMReadlong(109 + ts);
                if (ts == 0) pstr += "   Today Status \n";
                if (ts == 20) pstr += "   Total Status \n";
                pstr += ("Amt_R : " + (String)amt_rg + "\n" + "Amt_S : " + (String)amt_swp + "\n" + "Amt_L : " + (String)amt_ltr + "\n");
              }
              pstr += '#';
              delay(1000);
              Serial1.println(pstr);
              //Serial.println(pstr);
              //delay(2000);
              status_send = 1;
            }
          }
          //if((total_mins>=541)&&(total_mins<542))
          if (total_mins % 60 == 1)
          {
            status_send = 0;
          }
          //Serial.println(min_check);
          if (server_send_status == 0)
          {
            //if (sec_check == 20)
            if (min_check % 10 == 0)
            {
              Serial1.write('@');
              delay(400);
              String phone = "", pstr = "";
              byte i;
              /*for (i = 0; i < 10; i++)
                phone = phone + EEPROM.read(i);
                pstr += phone;
                pstr += '#';*/
              pstr += "/" + (String)serial_num;
              for (byte ts = 0; ts < 21; ts = ts + 20)
              {
                amt_rg = 0;    amt_swp = 0;    amt_ltr = 0;
                amt_rg = EEPROMReadlong(101 + ts);
                amt_swp = EEPROMReadlong(105 + ts);
                amt_ltr = EEPROMReadlong(109 + ts);
                if (ts == 0) pstr += ("/" + (String)amt_rg + "/" + (String)amt_swp + "/" + (String)amt_ltr );
                if (ts == 20) pstr += ("/" + (String)amt_rg + "/" + (String)amt_swp + "/" + (String)amt_ltr + "\n");
              }
              //pstr += '#';

              delay(1000);
              Serial1.print(pstr);
              server_send_status = 1;
            }
          }
          if (min_check % 10 != 0)server_send_status = 0;
          //if (sec_check < 5) server_send_status = 0;
        }
      }

      if ((menu_on_off_1 == 1) || (menu_on_off_2 == 1))
      {
        if (((pulseCount1 % lcd_check) == 1) || ((pulseCount2 % lcd_check) == 1))
        {
          lcd.begin(20, 4);
          lcd_check1 = 0;
        }
        //digitalWrite(BO_SW, HIGH);

        lcd.setCursor(0, 0);
        lcd.write(" TAP-1      TAP-2");
        lcd.setCursor(1, 1);
        lcd.print(L1);
        lcd.setCursor(12, 1);
        lcd.print(L2);
        lcd.setCursor(1, 2);
        lcd.print(pulseCount1); lcd.write("    ");
        lcd.setCursor(12, 2);
        lcd.print(pulseCount2); lcd.write("    ");
        delay(15);
        wdt_reset();
      }

      if ((test_dd_prev != date))
      {
        EEPROM.write(113, date);
        EEPROM.write(114, month_t);
        EEPROM.write(115, year_t);
        test_mm = month_t;
        test_yy = year_t;
        test_dd_prev = EEPROM.read(113);
        for (byte z = 101; z < 113; z++)
          EEPROM.write(z, 0);
        // Serial.println(x_countable);
        //  x_countable=1000;
      }
      if (status_write_flag)
      {
        rfid_status_write();
        status_write_flag = 0;
      }
      RF_status = 0;
      RF_status = my_RFID_card();
      if (RF_status == 9)
      {
        serial_menu_flag = 1;
        wdt_disable();
        time_check = 0;
        //Serial.println("Senior Master card");
        //counter = 0;
        lcd.clear();
        digitalWrite(Buzzer, HIGH);
        delay(300);
        digitalWrite(Buzzer, LOW);
        //delay(1000);
        current = millis();
        if (current >= interval)
        {

          /* lcd.clear();
            lcd.setCursor(0, 0);
            lcd.write("     Touch card");
            lcd.begin(20, 4);
            lcd.setCursor(0, 1);
            lcd.write("     Welcome To");
            lcd.setCursor(0, 2);
            lcd.write("   GTR Solutions  ");
            lcd.setCursor(0, 3);
            lcd.print("Device ID:");
            lcd.print(serial_num);
            lcd.setCursor(0, 6);
            lcd.setCursor(0, 0);
            lcd.noCursor();
            lcd.clear();*/
          //menu();

          resetFunc();
          timer0_millis = 0;
        }
        //Timer1.attachInterrupt(timer_check);
        //timer1(TIMER1_PRESCALER_1024, 7812U, toggle);
        //sei();

      }
      if (serial_menu_flag == 1)device_menu();

      if ((3 <= RF_status) && (RF_status <= 5))
      {
        digitalWrite(Buzzer, HIGH);
        delay(300);
        digitalWrite(Buzzer, LOW);
        delay(1000);
      }
      else  if ((0 < RF_status) && (RF_status <= 2))
      {
        digitalWrite(Buzzer, HIGH);
        delay(300);
        digitalWrite(Buzzer, LOW);

        if (GSM_Active)
          status_check = EEPROM.read(150);
        else
          status_check = 1;

        for (byte i = 0; i < 5; i++)
          serial_no = (serial_no * 10) + buffer1[i];
        if (status_check)
        {
          for (byte i = 3; i < 5; i++)
            id_no = id_no * 100 + buffer1[i];
          for (byte i = 5; i < 7; i++)
            value_no = value_no * 256 + buffer1[i];

          if ((menu_no != 2) && (TS_No != 2))
          {
            if (serial_no == serial_num)
            {
              wdt_disable();
              time_check = 0;
              lcd.clear();

              if (!Recharge_Active)
              {
                lcd.setCursor(1, 1);
                lcd.write("Your Amount : ");
                lcd.print(value_no);
                if (check_rfid_status == 0)
                {
                  for (byte i = 0; i < 2000; i++)
                  {
                    if (scan_card())
                      break;
                    //    delay(20);
                  }
                }
              }

              else
              {
                lcd.setCursor(1, 0);
                lcd.write("User Card ...      ");
                lcd.setCursor(1, 1);
                lcd.write("                   ");      // lcd.print(id_no);
                lcd.setCursor(1, 2);
                lcd.write("Serial No : ");
                lcd.print(serial_no);
                lcd.setCursor(1, 3);
                lcd.write("Your Amount : ");
                lcd.print(value_no);
                for (byte i = 0; i < 200; i++)
                {
                  if (back_key())
                    break;
                  delay(10);
                }
              }
              lcd.clear();
              //  wdt_enable(WDTO_8S);
            }
          }
        }

        if (serial_no == serial_num)
          if (TS_No == 2)
          {

            menu_no = 0;
            flag_4 = 1;
            counter = 0;
            for (byte i = 0; i < 9; i++)
              buffer1[i] = 0;
            wdt_disable();
            current = millis();
            if (current >= interval)
            {

              /* lcd.clear();
                lcd.setCursor(0, 0);
                lcd.write("     Touch Card");
                lcd.begin(20, 4);
                lcd.setCursor(0, 1);
                lcd.write("     Welcome To");
                lcd.setCursor(0, 2);
                lcd.write("   GTR Solutions  ");
                lcd.setCursor(0, 3);
                lcd.print("Device ID:");
                lcd.print(serial_num);
                lcd.setCursor(0, 6);
                lcd.setCursor(0, 0);
                lcd.noCursor();*/

              resetFunc();
              timer0_millis = 0;
            }

            // Timer1.attachInterrupt(timer_check);
            //  timer1(TIMER1_PRESCALER_1024, 7812U, toggle);
            //sei();
            //Serial.write('X');
            while (TS_No)
              menu_1();
          }
      }
      if (menu_no == 2)
      {
        menu_no = 0;
        menu_flag = 1;
        flag_1 = 1;
        counter = 11;
        for (byte i = 0; i < 9; i++)
          buffer1[i] = 0;
        lcd.clear();
        lcd.setCursor(2, 0);
        lcd.write("Master card.... ");
        lcd.setCursor(1, 2);
        lcd.write("Serial No : ");
        lcd.print(serial_no);
        delay(1000);
      }
      if (GSM_Active)
      {
        serialEvent3_int();
      }
    }
    if (menu_flag == 1)
    {
      wdt_disable();
      current = millis();
      if (current >= interval)
      {
        /* lcd.clear();
          lcd.setCursor(0, 0);
          lcd.write("     Touch Card");
          lcd.begin(20, 4);
          lcd.setCursor(0, 1);
          lcd.write("     Welcome To");
          lcd.setCursor(0, 2);
          lcd.write("   GTR Solutions  ");
          lcd.setCursor(0, 3);
          lcd.print("Device ID:");
          lcd.print(serial_num);
          lcd.setCursor(0, 6);
          lcd.setCursor(0, 0);
          lcd.noCursor();*/
        resetFunc();
        timer0_millis = 0;
      }
      //   Timer1.attachInterrupt(timer_check);
      // timer1(TIMER1_PRESCALER_1024, 7812U, toggle);
      //sei();
      Serial1.write('X');
      menu();
    }
  }
}


void device_menu() //-----------------------------------------------DEVICE ID CREATION MENU--------------------------------//
{
  byte enter_key_count = 0, front_key_count = 2, cnt = 0, addr_count = 0;
  unsigned int eeprom_addr = 500, kk = 0; unsigned int serial_number = 0;
  lcd.setCursor(1, 0);
  lcd.print(' ');
  lcd.setCursor(2, 0);
  for (int i = 0; i <= 4; i++)
  {
    lcd.print(EEPROM.read(eeprom_addr + i));
  }
  lcd.print("  DeviceID");
  while (serial_menu_flag)
  {
    if (back_key() || (time_check == 5))
    {
      time_check = 0;
      lcd.clear();
      lcd_check1 = 0;
      serial_menu_flag = 0;
      // wdt_enable(WDTO_8S);
      //  Timer1.detachInterrupt();

      goto last;
    }
    if (enter_key())
    {
      enter_key_count++;
    }
    if (enter_key_count > 2)
    {
      enter_key_count = 0;
      lcd.setCursor(0, 0);
      lcd.print(' ');
    }
    if (enter_key_count == 0)
    {
      lcd.setCursor(0, 0);
      lcd.print(' ');
      lcd.setCursor(front_key_count, 1);
      lcd.print(' ');
    }
    if (enter_key_count == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print('>');

      if (front_key())
      {
        kk++;
        if (kk > 5) kk = 0;
        cnt = EEPROM.read(500 + kk);
        front_key_count++;
      }
      if (front_key_count < 7)
      {
        if (front_key_count == 2)
        {
          eeprom_addr = 500;
          lcd.setCursor(2, 1);
          lcd.print('^');
          lcd.setCursor(3, 1);
          lcd.print(' ');
          lcd.setCursor(4, 1);
          lcd.print(' ');
          lcd.setCursor(5, 1);
          lcd.print(' ');
          lcd.setCursor(6, 1);
          lcd.print(' ');
        }
        if (front_key_count == 3)
        {
          eeprom_addr = 501;
          lcd.setCursor(2, 1);
          lcd.print(' ');
          lcd.setCursor(3, 1);
          lcd.print('^');
          lcd.setCursor(4, 1);
          lcd.print(' ');
          lcd.setCursor(5, 1);
          lcd.print(' ');
          lcd.setCursor(6, 1);
          lcd.print(' ');
        }
        if (front_key_count == 4)
        {
          eeprom_addr = 502;
          lcd.setCursor(2, 1);
          lcd.print(' ');
          lcd.setCursor(3, 1);
          lcd.print(' ');
          lcd.setCursor(4, 1);
          lcd.print('^');
          lcd.setCursor(5, 1);
          lcd.print(' ');
          lcd.setCursor(6, 1);
          lcd.print(' ');
        }
        if (front_key_count == 5)
        {
          eeprom_addr = 503;
          lcd.setCursor(2, 1);
          lcd.print(' ');
          lcd.setCursor(3, 1);
          lcd.print(' ');
          lcd.setCursor(4, 1);
          lcd.print(' ');
          lcd.setCursor(5, 1);
          lcd.print('^');
          lcd.setCursor(6, 1);
          lcd.print(' ');
        }
        if (front_key_count == 6)
        {
          eeprom_addr = 504;
          lcd.setCursor(2, 1);
          lcd.print(' ');
          lcd.setCursor(3, 1);
          lcd.print(' ');
          lcd.setCursor(4, 1);
          lcd.print(' ');
          lcd.setCursor(5, 1);
          lcd.print(' ');
          lcd.setCursor(6, 1);
          lcd.print('^');
        }

        if (!digitalRead(up))
        {
          cnt++;
          delay(400);
        }
        if (!digitalRead(down))
        {
          cnt--;
          delay(400);
        }
        if (cnt > 9)cnt = 0;
        lcd.setCursor(front_key_count, 0);
        lcd.print(cnt);
        EEPROM.write(eeprom_addr, cnt);
      }
    }
    //serial_n[kk] = cnt;
  }
last:
  {
  }
}
void menu_1() //----------------------------------------------------------------------------Menu Page up and down----------------------------------------------------------------//
{
  counter = up_down(counter);
  if (back_key() || (time_check == 5))
  {
    TS_No = 0;
    time_check = 0;
    menu_flag = 0;
    flag_4 = 0;
    lcd.clear();
    lcd_check1 = 0;
    Serial.write(' ');
    //wdt_enable(WDTO_8S);
    //Timer1.detachInterrupt();
    goto menu_end;
  }
  if (counter > 1) counter = 0;
  if (counter < 0)  counter = 1;
  //Serial.println(counter);
  if (counter <= 2)
  {
    if (flag_4 == 1)
      menu_page4();   flag_4 = 0;
  }
  if (counter >= 0)
  {
    if ((counter == 0))
    {

      lcd.setCursor(0, 0);
      lcd.write("> ");
      lcd.setCursor(0, 1);
      lcd.write(" ");
    }
    if ((counter == 1))
    {
      lcd.setCursor(0, 0);
      lcd.write(" ");
      lcd.setCursor(0, 1);
      lcd.write("> ");
    }
    /*if ( (counter == 2))
      {
      lcd.setCursor(0, 1);
      lcd.write(" ");
      lcd.setCursor(0, 2);
      lcd.write(">");
      lcd.setCursor(0, 0);
      lcd.write(" ");
      }
      if ((counter == 3))
      {
      lcd.setCursor(0, 2);
      lcd.write(" ");
      lcd.setCursor(0, 1);
      lcd.write(" ");
      lcd.setCursor(0, 0);
      lcd.write(">");
      }*/
  }
  if (counter <= 2)
  {
    /* if ((counter == 0) && enter_key())
       clear_status();//Serial1.println(" Function 3");*/

    if ((counter == 0) && enter_key())
      New_Rfid_card();//Serial1.println(" Function 2");

    if ((counter == 1) && enter_key())unlimted_card();
  }
menu_end :
  { }
}

void unlimted_card()
{
  byte id_new = 0;
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Place an ULTD card ");
  while (1)
  {
    id_new = RFID_ULTD_card();
    if (id_new == 1)
    {
      lcd.setCursor(1, 2);
      lcd.print("     OK Done     ");
      delay(1000);
    }
    else
    {
      lcd.setCursor(1, 2); lcd.print("                  ");
    }
    if (enter_key())   break;
    if (back_key())    goto end_dis;
    if (time_check == 4) goto end_dis;
  }
end_dis :
  time_check = 0;
  flag_4 = 1;
  lcd.clear();
}

void menu() //----------------------------------------------------------------------------Menu Page up and down----------------------------------------------------------------//
{
  counter = up_down(counter);
  if (back_key() || (time_check == 5))
  {
    time_check = 0;
    menu_flag = 0;
    flag_1 = 0;   flag_2 = 0;    flag_3 = 0;
    lcd.clear();
    lcd_check1 = 0;
    Serial1.write(' ');
    //wdt_enable(WDTO_8S);
    L1 = ((EEPROM.read(34 + 0)) * 256) + EEPROM.read(35 + 0);
    L2 = ((EEPROM.read(34 + 7)) * 256) + EEPROM.read(35 + 7);
    goto menu_end;
  }
  if (counter > 11) counter = 0;
  if (counter < 0)  counter = 11;

  if (counter<12 & counter>7)
  {
    if (flag_1 == 1)
      menu_page1();
    flag_1 = 0;   flag_2 = 1; flag_3 = 1;
  }
  else if (counter < 8 && counter > 3)
  {
    if (flag_2 == 1)
      menu_page2();   flag_2 = 0;   flag_3 = 1; flag_1 = 1;
  }
  if (counter <= 3)
  {
    if (flag_3 == 1)
      menu_page3();   flag_3 = 0;   flag_1 = 1;  flag_2 = 1;
  }
  if (counter >= 0)
  {
    if ((counter == 11) || (counter == 7) || (counter == 3))
    {
      lcd.setCursor(0, 3);
      lcd.write(" ");
      lcd.setCursor(0, 0);
      lcd.write("> ");
      lcd.setCursor(0, 1);
      lcd.write(" ");
    }
    if ((counter == 10) || (counter == 6) || (counter == 2))
    {
      lcd.setCursor(0, 0);
      lcd.write(" ");
      lcd.setCursor(0, 1);
      lcd.write("> ");
      lcd.setCursor(0, 2);
      lcd.write(" ");
    }
    if ((counter == 9) || (counter == 5) || (counter == 1))
    {
      lcd.setCursor(0, 1);
      lcd.write(" ");
      lcd.setCursor(0, 2);
      lcd.write("> ");
      lcd.setCursor(0, 3);
      lcd.write(" ");
    }
    if ((counter == 8) || (counter == 4) || (counter == 0))
    {
      lcd.setCursor(0, 2);
      lcd.write(" ");
      lcd.setCursor(0, 3);
      lcd.write("> ");
      lcd.setCursor(0, 0);
      lcd.write(" ");
    }
  }
  if (counter<12 & counter>7)
  {
    if ((counter == 11) && enter_key())
      TIME_DATE();

    if ((counter == 10) && enter_key())
      No_of_cards();//Serial1.println("New_Rfid_card()");// New_Rfid_card();

    if ((counter == 9) && enter_key())
      No_of_Taps();

    if ((counter == 8) && enter_key())
      confg_volume();

  }
  else if (counter < 8 && counter > 3)
  {
    if ((counter == 7) && enter_key())
      Gsm_Setting();               //Gsm setting

    if ((counter == 6) && enter_key())
      Default_setting();

    if ((counter == 5) && enter_key())
      Recharge_mode();

    if ((counter == 4) && enter_key())
      User_limit();                             // Serial1.println(" User limit()");    //Gsm Screen view();

  }
  else if (counter <= 3)
  {
    if ((counter == 3) && enter_key())
      copy_past(2);                           //Serial1.println(" Copy Your Data");

    if ((counter == 2) && enter_key())
      copy_past(1);                           //Serial1.println(" Past Your Data");

    if ((counter == 1) && enter_key())
      status_TT(0);    //Serial1.println(" Today Status");

    if ((counter == 0) && enter_key())
      status_TT(20);   //Serial1.println(" Total Status");
  }
menu_end :
  {}
  //Timer1.detachInterrupt();
}
//---------------------------------------------------------------------------------------------Menu page ---------------------------------------------------//
void menu_page1()
{
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Time & Date ");
  lcd.setCursor(3, 1);
  lcd.print("No.of RFID Cards  ");
  lcd.setCursor(3, 2);
  lcd.print("Select No.of Taps");
  lcd.setCursor(3, 3);
  lcd.print("Confg Volume ");
}
//---------------------------------------------------------------------------------------------Menu page ---------------------------------------------------//
void menu_page2()
{
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("GSM Settings ");
  lcd.setCursor(3, 1);
  lcd.print("Default Settings   ");
  lcd.setCursor(3, 2);
  lcd.print("Recharge Mode  ");
  lcd.setCursor(3, 3);
  lcd.print("User Limits");
}
void menu_page3()
{
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Copy  Settings ");
  lcd.setCursor(3, 1);
  lcd.print("Paste Settings ");
  lcd.setCursor(3, 2);
  lcd.print("Today Status ");
  lcd.setCursor(3, 3);
  lcd.print("Total Status ");
}
void menu_page4()
{
  lcd.clear();
  //lcd.setCursor(3, 0);
  //lcd.print("Status Clear ");
  lcd.setCursor(3, 0);
  lcd.print("New RFID Card  ");
  lcd.setCursor(3, 1);
  lcd.print("Unlimted card");
  lcd.setCursor(3, 2);
  lcd.print("               ");
}

byte RTC_Read(byte highaddr)     //-------------------------------------------------------------------------------EEPROM read------------------------------------------------------------//
{
  byte test_read_rtc = 100;

  Wire.beginTransmission(Address);
  Wire.write(highaddr);
  Wire.endTransmission();
  Wire.requestFrom(Address, 1);
  while (test_read_rtc)
    if (Wire.available())  break;
    else test_read_rtc--;
  if (test_read_rtc <= 0)
    return 0;
  else
    return Wire.read();
}
void RTC_Write(byte highaddr, byte data)      //----------------------------------------------------------------------------EEPROM write------------------------------------------------------------//
{
  Wire.beginTransmission(Address);
  Wire.write(highaddr);
  Wire.write(data);
  Wire.endTransmission();
}

void No_of_Taps() // -------------------------------------- No of cards --------------------------------------------------------
{
  byte total_taps = EEPROM.read(70);
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("No.of Taps");
  while (1)
  {

    lcd.setCursor(9, 2);
    lcd.print(total_taps);
    delay(500);
    if (!digitalRead(up))
    {
      total_taps++ ;
      time_check = 0;
    }
    if (!digitalRead(down))
    {
      total_taps--;
      time_check = 0;
    }
    if (total_taps >= 5) total_taps = 1;
    if (back_key())
      goto end_ncard;
    if (time_check == 4)
      goto end_ncard;
  }

end_ncard :
  EEPROM.write(70, total_taps);
  no_of_tap = EEPROM.read(70);
  time_check = 0;
  flag_1 = 1;
  lcd.clear();
}

void serialEvent3_int()
{
  if (!(Serial1.available() > 0))
    return;
  String phone = "", s;
  byte i;
  for (i = 0; i < 10; i++)
    phone = phone + EEPROM.read(i);

  s = Serial1.readString();
  s.toLowerCase();
  Serial.println(s);

  String ss1;
  ss1 = phone;
  ss1 += '#';
  if (s.indexOf(phone) > 0)
  {
    if (s.indexOf("menu open") > 0)
    { delay(1000);
      Serial1.println(ss1);
      Serial.println(ss1);
      delay(2000);
      menu_flag = 1;
      flag_1 = 1;
      counter = 11;
      lcd.clear();
    }
    else if (s.indexOf("status ?") > 0)
    {
      ss1 += "\nSerial number:" + (String)serial_num + "\n";
      for (byte ts = 0; ts < 21; ts = ts + 20)
      {
        amt_rg = 0;    amt_swp = 0;    amt_ltr = 0;
        amt_rg = EEPROMReadlong(101 + ts);
        amt_swp = EEPROMReadlong(105 + ts);
        amt_ltr = EEPROMReadlong(109 + ts);
        if (ts == 0) ss1 += "   Today Status \n";
        if (ts == 20) ss1 += "   Total Status \n";
        ss1 += ("Amt_R : " + (String)amt_rg + "\n" + "Amt_S : " + (String)amt_swp + "\n" + "Amt_L : " + (String)amt_ltr + "\n");
      }
      ss1 += '#';
      delay(1000);
      Serial1.println(ss1);
      Serial.println(ss1);
      delay(2000);
    }
    else if (s.indexOf("server ?") > 0)
    {
      ss1 += "\nSerial number:" + (String)serial_num + "\n";
      for (byte ts = 0; ts < 21; ts = ts + 20)
      {
        amt_rg = 0;    amt_swp = 0;    amt_ltr = 0;
        amt_rg = EEPROMReadlong(101 + ts);
        amt_swp = EEPROMReadlong(105 + ts);
        amt_ltr = EEPROMReadlong(109 + ts);
        if (ts == 0) ss1 += "   Today Status \n";
        if (ts == 20) ss1 += "   Total Status \n";
        ss1 += ("Amt_R : " + (String)amt_rg + "\n" + "Amt_S : " + (String)amt_swp + "\n" + "Amt_L : " + (String)amt_ltr + "\n");
      }
      ss1 += '#';
      delay(1000);
      Serial1.println(ss1);
      Serial.println(ss1);
      delay(2000);
    }
    else if (s.indexOf("device activate") > 0)
    {
      Serial1.println(ss1);
      EEPROM.write(150, 0);
    }
    else if (s.indexOf("device deactivate") > 0)
    {
      Serial1.println(ss1);
      Serial.println(ss1);
      EEPROM.write(150, 1);
    }
    else
    {
      ss1 += "wrong";
      Serial1.println(ss1);
    }
  }
  else if (s.indexOf("csq") > 0)
  {
    byte buff[2];
    s.remove(0, s.indexOf("+csq:") + 6);
    s.remove(2, 8);
    //  s.toCharArray(buff, s.length() + 1);
    x_csq = (((buff[0] - 48) * 10 + (buff[1] - 48) + 1) * 100) / 31;
  }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;

    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
byte scan_card()            //---------------------------------------------------------- Card Scan() --------------------------------------------------//
{
  byte i = 0, tap = 0, exit_rfid = 0, tap1 = 0;
  unsigned int total_min = 0, total_max = 0, PR_check = 0;
  //amt_o=0;
  //key_r=0;
  key_p = 0;

  byte key_tap = 0;
  //no_of_tap=EEPROM.read(70);
  tap = 1;
  tap1 = tap;

  while (1)
  {
    key_tap = key_r;
    if (key_r != 0 && key_p != key_r)
    { delay(10);
      lcd.setCursor(0, 0);
      lcd.write("  Place a Card");

      lcd.setCursor(0, 2);
      lcd.write("Amount      : ");
      lcd.setCursor(14, 2);
      lcd.print(EEPROM.read(36 + ((key_r - 1) * 7)));  lcd.print("  ");
      lcd.setCursor(0, 3);
      lcd.write("Water in  L : ");
      lcd.print(EEPROM.read(30 + ((key_r - 1) * 7)));

      key_p = key_r;
    }
    PR_check = EEPROM.read(30 + ((key_r - 1) * 7));
    if (PR_check != 0)
      exit_rfid = RF_status;
    if (exit_rfid == 2)
    {
      if (key_p == 0)
        goto first_d;
      lcd.setCursor(0, 0);
      lcd.write("Old Balance : ");
      lcd.setCursor(14, 0);
      lcd.print(B_amt);
      lcd.write(" ");
      lcd.setCursor(0, 1);
      lcd.write("New Balance : ");
      lcd.setCursor(14, 1);
      lcd.print(A_amt);
      lcd.write(" ");
      lcd.setCursor(0, 2);
      lcd.write("Amount      : ");
      lcd.setCursor(14, 2);
      lcd.print(EEPROM.read(36 + ((key_r - 1) * 7)));
      lcd.print("  ");
      lcd.setCursor(0, 3);
      lcd.write("Water in  L : ");
      lcd.print(EEPROM.read(30 + ((key_r - 1) * 7)));
      delay(1500);
      key_r = 0;

      delay(10);
      goto first_d;
    }
    else if ((exit_rfid == 2) || (back_key()) || (time_check == 2))
    {
      delay(500);  goto first_d1;
    }
    if (i > 100)         goto first_d1;
    delay(20);
    i++;
  }
first_d :
  lcd.clear();

  if (flag_int1 == 0)
  {
    total_min1 = 0;
    total_max1 = 0;
    key_tap = 1;
    total_min1 = ((EEPROM.read(32 + ((key_tap - 1) * 7)) * 256) + EEPROM.read(33 + ((key_tap - 1) * 7)));
    total_max1 = ((EEPROM.read(34 + ((key_tap - 1) * 7)) * 256) + EEPROM.read(35 + ((key_tap - 1) * 7)));
    L1 = total_max1;
    L2 = ((EEPROM.read(34 + ((key_tap) * 7)) * 256) + EEPROM.read(35 + ((key_tap) * 7)));
    flag_int1 = 1;

    if ((total_max1 != 0) && (total_min1 <= total_max1))
    {

      menu_on_off_1 = 1; lcd.clear();
      digitalWrite(T_SW_1, LOW);
      delay(150);
      lcd.begin(20, 4);
      lcd.clear();
      digitalWrite(Buzzer, HIGH);
      delay(1000);
      digitalWrite(Buzzer, LOW);
    }

    pulseCount1 = 0;
  }
  else if (flag_int2 == 0)
  {
    key_tap = 2;
    total_min2 = 0;
    total_max2 = 0;
    total_min2 = ((EEPROM.read(32 + ((key_tap - 1) * 7)) * 256) + EEPROM.read(33 + ((key_tap - 1) * 7)));
    L1 = ((EEPROM.read(34 + ((key_tap - 2) * 7)) * 256) + EEPROM.read(35 + ((key_tap - 2) * 7)));
    total_max2 = ((EEPROM.read(34 + ((key_tap - 1) * 7)) * 256) + EEPROM.read(35 + ((key_tap - 1) * 7)));
    L2 = total_max2;
    flag_int2 = 1;
    if ((total_max2 != 0) && (total_min2 <= total_max2))
    {

      menu_on_off_2 = 1;
      digitalWrite(T_SW_2, LOW);
      delay(150);
      lcd.begin(20, 4);
      lcd.clear();
      digitalWrite(Buzzer, HIGH);
      delay(1000);
      digitalWrite(Buzzer, LOW);
    }

    pulseCount2 = 0;
  }
first_d1 :

  delay(10);
  return 1;
}

void copy_past(byte cp)  ///--------------------------------------------------  copy_past  --------------------------------------------------------//
{
  time_check = 0;
  lcd.clear();
  while (!RFID_card_copy_past(cp))
  {
    lcd.setCursor(1, 0);
    lcd.write(" place a card ");
    lcd.setCursor(1, 1);
    lcd.write("      or ");
    lcd.setCursor(1, 2);
    lcd.write(" press back key ");
    delay(100);
    if (back_key() || (time_check == 5))  break;
  }
  time_check = 0;
  flag_3 = 1;
  lcd.clear();
}

void status_TT(byte ts) //-----------------------------------------------------------Total and Today status ------------------------------------//
{
  time_check = 0;
  lcd.clear();
  amt_rg = 0;    amt_swp = 0;    amt_ltr = 0;
  amt_rg = EEPROMReadlong(101 + ts);
  amt_swp = EEPROMReadlong(105 + ts);
  amt_ltr = EEPROMReadlong(109 + ts);

  lcd.setCursor(3, 0);
  if (ts == 0)     lcd.write(" Today Status ");
  else      lcd.write(" Total Status ");
  lcd.setCursor(1, 1);
  lcd.write("AMT_R : ");
  lcd.print(amt_rg);
  lcd.setCursor(1, 2);
  lcd.write("AMT_S : ");
  lcd.print(amt_swp);
  lcd.setCursor(1, 3);
  lcd.write("AMT_L : ");
  lcd.print(amt_ltr);
  while (!back_key())
  {
    if (time_check == 5) break;
  }
  time_check = 0;
  flag_3 = 1;
  lcd.clear();
}

void User_limit()  //-----------------------------------------------------------  user limit ------------------------------------------//
{
  time_check = 0;
  lcd.clear();
  byte limit = EEPROM.read(71);
  while (1)
  {
    lcd.setCursor(1, 1);
    lcd.write("Limit :");
    lcd.print(limit);
    lcd.write("    ");
    delay(250);
    if (!digitalRead(up))
    {
      limit++ ;
      time_check = 0;
    }
    if (!digitalRead(down))
    {
      limit--;
      time_check = 0;
    }

    if (limit > 100) limit = 0;
    if (back_key() || (time_check == 5)) break;
  }
  time_check = 0;
  EEPROM.write(71, limit);
  flag_2 = 1;
  lcd.clear();
}
void timer_check()      //-----------------------------------------------------------  timer_check ------------------------------------------//
{
  unsigned int i, j;
  time_check++;
  if (time_check == 5)
  {
    digitalWrite(Buzzer, HIGH);
    for (i = 0; i < 300; i++)
      delay(110);
    digitalWrite(Buzzer, LOW);
  }
}
void pulseCounter1()
{
  if ((total_max1 != 0) && (total_min1 <= total_max1))
  {
    menu_on_off_1 = 1;
    pulseCount1++;
    if (pulseCount1 <= total_max1)
    {
      if ((pulseCount1 >= total_min1) && (pulseCount1 <= total_max1)) // Only process counters once per second
      {
        digitalWrite(T_SW_1, HIGH);
        delay(2000);
        flag_int1 = 0;
        pulseCount1 = 0;
        menu_on_off_1 = 0;
        total_max1 = 0;
        wt_flag = 1;
      }
    }
  }
}

void pulseCounter2()
{
  if ((total_max2 != 0) && (total_min2 <= total_max2))
  {
    menu_on_off_2 = 1;
    pulseCount2++;
    if (pulseCount2 <= total_max2)
    {
      if ((pulseCount2 >= total_min2) && (pulseCount2 <= total_max2)) // Only process counters once per second
      {
        digitalWrite(T_SW_2, HIGH);
        flag_int2 = 0;
        pulseCount2 = 0;
        menu_on_off_2 = 0;
        total_max2 = 0;

        wt_flag = 1;
        delay(3000);
      }
    }
  }
}
void toggle() {
  static uint8_t output = 0xff;
  PORTB = output;
  output = !output;
}

ISR (TIMER1_OVF_vect)    // Timer1 ISR
{
  PORTD ^= (1 << LED);
  TCNT1 = 63974;   // for 1 sec at 16 MHz
}
void home_screen() {
  lcd.begin(20, 4);
  lcd.setCursor(0, 0);
  lcd.write("     Welcome To");
  lcd.setCursor(0, 1);
  lcd.write("   GTR Solutions  ");
  lcd.setCursor(0, 2);
  lcd.print("Device ID:");
  lcd.print(serial_num);
  lcd.setCursor(0, 3);

}
