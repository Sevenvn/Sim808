/* SIM808
    D3      < - >     TX
    D2      < - >     RX
    D13     < - >     PWK
    A1      < - >     ST

 Press Left/ Right to swich LCD display
 Press Select to send sms of current desktop   
 */

#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

// Buttons
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5


// RMC
#define GPS_RSTS                0
#define GPS_FSTS                1
#define GPS_TIME                2
#define GPS_LAT                 3
#define GPS_LON                 4
#define GPS_MSL                 5
#define GPS_SPEED               6
#define GPS_ORG                 7

typedef union
{
  struct
  {
    unsigned int year;
    unsigned char month;
    unsigned char date;
    unsigned char hour;
    unsigned char minute;
    unsigned char sec;
  }time;
}SysTime;

typedef enum
{
  DISPLAY_GPS = 0,
  DISPLAY_RTC,
  DISPLAY_SPEED_ORG,
  DISPLAY_MAX
}DISPLAY_E;

typedef struct{       
  bool State_gps;     /*!< trang thai gps 1 = co tin hieu, 0 = mat tin hieu */
  SysTime time_gps;   /*!< time gps  */
  float Lat;     /*!< vi do */
  float Lng;     /*!< kinh do */
  float SpeedGPS;     /*!< Toc do GPS */
  float OrGPS;      /*!< Huong GPS 360 do */
}GPS_T;     /*!< cau truc du lieu hanh trinh */


int phone_number_to_send = "0982694660";
int PWK_PIN  =  13;      // the number of the PWK pin
int STATUS_PIN = A1;    // select the input pin for detect module power-on


SoftwareSerial modemSerial(2, 3); // RX, TX
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

GPS_T g_gps_data = {false, 0, 0, 0, 0};

int read_LCD_buttons(){                   // read the buttons
    int adc_key_in = analogRead(0);       // read the value from the sensor 
 
    // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
    // we add approx 50 to those values and check to see if we are close
    // We make this the 1st option for speed reasons since it will be the most likely result
 
    if (adc_key_in > 1000) return btnNONE; 
 
    // For V1.1 us this threshold
    if (adc_key_in < 50)   return btnRIGHT;  
    if (adc_key_in < 250)  return btnUP; 
    if (adc_key_in < 450)  return btnDOWN; 
    if (adc_key_in < 650)  return btnLEFT; 
    if (adc_key_in < 850)  return btnSELECT;  
    return btnNONE;                // when all others fail, return this.
}

int button_pressed, tmp_button, table_display = 0;
void setup() {
  modemSerial.begin(9600);
   // initialize serial communications
  Serial.begin(115200);

    // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  Serial.write("SIM808 DEMO");
  lcd.print("SIM808 DEMO");
}

char respond[300];
  
void loop() {
  static unsigned long previousMillis; 
  unsigned long currentMillis;
  
  ModemInit();

// Update GPS data
  currentMillis  = millis();
  if (currentMillis - previousMillis >= 2000) {
    modemSerial.write("AT+CGNSINF\r");
    if(readSerialFrame(respond) > 0)
    {
      if(strncmp(respond, "\r\n+CGNSINF: ", 12) == 0)
      {
        NMEA_GPRMC_Decoder(&g_gps_data, respond + 12);
      }
    }
    previousMillis = currentMillis;
  }
// End Update GPS data


// Keypad sevice
  tmp_button = read_LCD_buttons();
  if(button_pressed != tmp_button)
  {
    Serial.println("Key pressed");
    button_pressed = tmp_button;
    if(button_pressed == btnRIGHT)
    {
      lcd.clear();
      table_display++;
      if(table_display >= DISPLAY_MAX)
      {
        table_display = 0;
      }
    }
    if(button_pressed == btnLEFT)
    {
      if(table_display == 0)
      {
        table_display = DISPLAY_MAX - 1;
      }
      else
      {
        lcd.clear();
        table_display--;  
      }
    }
    if(button_pressed == btnSELECT)
    {
      char data_to_send_sms[160];
      char tmp_str1[20], tmp_str2[20];
      if(table_display == DISPLAY_GPS)
      {
        
        dtostrf(g_gps_data.Lat, 10, 6, tmp_str1);
        dtostrf(g_gps_data.Lng, 10, 6, tmp_str2);
        sprintf(data_to_send_sms, "https://maps.google.com/maps?q=%s,%s", tmp_str1, tmp_str2);
        SendSMS(phone_number_to_send, data_to_send_sms);
       
      }
      if(table_display == DISPLAY_RTC)
      {
        sprintf(data_to_send_sms, "TIME %04d/%02d/%02d  %02d:%02d:%02d"  , g_gps_data.time_gps.time.year  \
                                          , g_gps_data.time_gps.time.month  \
                                          , g_gps_data.time_gps.time.date \
                                          , g_gps_data.time_gps.time.hour   \
                                          , g_gps_data.time_gps.time.minute   \
                                          , g_gps_data.time_gps.time.sec);
        SendSMS(phone_number_to_send, data_to_send_sms);
      }
      if(table_display == DISPLAY_SPEED_ORG)
      {
        dtostrf(g_gps_data.SpeedGPS, 5, 2, tmp_str1);
        dtostrf(g_gps_data.OrGPS, 5, 2, tmp_str2);
        
        sprintf(data_to_send_sms, "SPEED = %s (km/h)   ORG = %s",tmp_str1, tmp_str2);
        SendSMS(phone_number_to_send, data_to_send_sms);
      }
    }
  }
// End Keypad sevice


// Display LCD
  switch(table_display)
  {
    char print_lcd[16];
    char float_str[10];
    case DISPLAY_GPS:
    {
      if(g_gps_data.State_gps == true)
      {
        lcd.setCursor(0, 0);
        dtostrf(g_gps_data.Lat, 10, 6, float_str);
        sprintf(print_lcd, "lat: %s", float_str);
        lcd.print(print_lcd);
        lcd.setCursor(0, 1);
        dtostrf(g_gps_data.Lng, 10, 6, float_str);
        sprintf(print_lcd, "lng: %s", float_str);
        lcd.print(print_lcd);
        
      }
      else
      {
        lcd.setCursor(0, 0);
        lcd.print("GPS not fix");
      }
      break;
    }
    case DISPLAY_RTC:
    {
      lcd.setCursor(0, 0);
      sprintf(print_lcd, "%04d/%02d/%02d"  , g_gps_data.time_gps.time.year  \
                                          , g_gps_data.time_gps.time.month  \
                                          , g_gps_data.time_gps.time.date);
      lcd.print(print_lcd);
      lcd.setCursor(0, 1);
      sprintf(print_lcd, "%02d:%02d:%02d", g_gps_data.time_gps.time.hour   \
                                          , g_gps_data.time_gps.time.minute   \
                                          , g_gps_data.time_gps.time.sec);
      lcd.print(print_lcd);
      break;
    }
    case DISPLAY_SPEED_ORG:
    {
      lcd.setCursor(0, 0);
      dtostrf(g_gps_data.SpeedGPS, 5, 2, float_str);
      sprintf(print_lcd, "SPEED : %s", float_str);
      lcd.print(print_lcd);
      lcd.setCursor(0, 1);
      dtostrf(g_gps_data.OrGPS, 5, 2, float_str);
      sprintf(print_lcd, "ORG : %s", float_str);
      lcd.print(print_lcd);
      break;
    }
    default:
    break;
  }
//End Display LCD
}


/*
  Read input serial
 */
int readSerial(char result[], int time_out) {
  int i = 0;
  unsigned long currentMillis, previousMillis;
  previousMillis = millis();
  currentMillis = previousMillis;
  while (1) {
    if (currentMillis - previousMillis >= time_out) 
    {
      return 0;
    }
    else
    {
      while (modemSerial.available() > 0) {
        char inChar = modemSerial.read();
        if (inChar == '\r') {
          result[i] = '\0';
          modemSerial.flush();
          return i;
        }
        if (inChar != '*') {
          result[i] = inChar;
          i++;
        }
      }
    }
    currentMillis = millis();
  }
}

bool SendSMS(char *phone_num, char *data)
{
  unsigned long currentMillis, previousMillis;
  char sms_cmd[50];
  char respond[30];
  char command_en[2] = {0x1A, 0};
   
  lcd.clear();
  lcd.print("Sending SMS");
       
  sprintf(sms_cmd,"AT+CMGS=\"%s\"\r",phone_num);
  modemSerial.write(sms_cmd);
  if(readSerialFrame(respond) > 0)
  {
    Serial.println(respond);
    if(strcmp(respond, "\r\n>") != 0)
    {
      modemSerial.write(data);
    }
  }
  modemSerial.write(command_en);
  previousMillis = millis();
  currentMillis = previousMillis;
  while(currentMillis - previousMillis < 5000)      // timeout 5s
  {
    currentMillis = millis();
    if(readSerialFrame(respond) > 0)
    {
      Serial.println(respond);
      if(strcmp(respond, "\r\nOK\r\n") != 0)
      {
         lcd.clear();
          lcd.print("Send done");
     }
    }
  }
  lcd.clear();
  lcd.print("Send false");
  return false;
}

void ModemInit(void)
{
  static bool is_modem_init_ok = false;
  int status_pin_level = 0;  // to detect modem has been turn-on
 // Turnon modem:
  try_on:
  status_pin_level = analogRead(STATUS_PIN);
  if(status_pin_level < 500)    // Check module on ? if not turn-off
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    is_modem_init_ok = false;
    lcd.print("No Sim808");
    Serial.write("No Sim808");
    digitalWrite(PWK_PIN, HIGH);
    delay(2000);
    digitalWrite(PWK_PIN, LOW);
    delay(2000);
    goto try_on;                 // We try turn-on until module SIM ready too use
  }

  if(is_modem_init_ok == false)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initting...");
    modemSerial.write("ATE0\r");
    if(readSerialFrame(respond) > 0)
    {
      if(strcmp(respond, "\r\nOK\r\n") != 0)
      {
        goto try_on;
      }
    }
    else
    {
      goto try_on;
    }

    modemSerial.write("AT+CMGF=1\r");
    if(readSerialFrame(respond) > 0)
    {
      if(strcmp(respond, "\r\nOK\r\n") != 0)
      {
        goto try_on;
      }
    }
    else
    {
      goto try_on;
    }

    modemSerial.write("AT+CGNSPWR=1\r");
    if(readSerialFrame(respond) > 0)
    {
      if(strcmp(respond, "\r\nOK\r\n") != 0)
      {
        goto try_on;
      }
    }
    else
    {
      goto try_on;
    }

    modemSerial.write("AT+CGNSSEQ=\"RMC\"\r");
    if(readSerialFrame(respond) > 0)
    {
      if(strcmp(respond, "\r\nOK\r\n") != 0)
      {
        goto try_on;
      }
    }
    else
    {
      goto try_on;
    }
    
    modemSerial.write("AT+CMGF=1\r");
    if(readSerialFrame(respond) > 0)
    {
      if(strcmp(respond, "\r\nOK\r\n") != 0)
      {
        goto try_on;
      }
    }
    else
    {
      goto try_on;
    }
    
    modemSerial.write("AT+CNMI=2,2,0,0,0\r");
    if(readSerialFrame(respond) > 0)
    {
      if(strcmp(respond, "\r\nOK\r\n") != 0)
      {
        goto try_on;
      }
    }
    else
    {
      goto try_on;
    }
    is_modem_init_ok = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initted");
    delay(2000);
  }
}

int readSerialFrame(char result[]) {
  int i = 0;
  unsigned long currentMillis, previousMillis;
  previousMillis = millis();
  currentMillis = previousMillis;
  while (1) {
  if (currentMillis - previousMillis >= 100) 
  {
    return i;
  }
  else
  {
    if (modemSerial.available() > 0) {
      previousMillis = currentMillis;
      char inChar = modemSerial.read();
      result[i] = inChar;
      i++;
      result[i] = 0;
    }
  }
    currentMillis = millis();
  }
}


static int GetMessageFeilds(unsigned char **message_feildls, unsigned char *msg, unsigned char separate_char, int max_feild_get)
{
  int count_feild = 0;
  *message_feildls = msg;
  message_feildls++;
  count_feild++;
  while(*msg !='\0')
  {
    if(*msg == separate_char)
    {
      *msg = '\0';
      *message_feildls = msg + 1;
      message_feildls++;
      count_feild++;
      if(count_feild >= max_feild_get)
        return count_feild;
    }
    msg++;
  }
  return count_feild;
}



static void NMEA_GPRMC_Decoder(GPS_T *gps_data, char *data_bytes)
{
  //Get data
  char *message_field[13];
  if(GetMessageFeilds(message_field, data_bytes, ',', 13) == 13)
  {
      Serial.write(message_field[GPS_FSTS]);
      Serial.write(message_field[GPS_SPEED]);
      Serial.write(message_field[GPS_ORG]);
      Serial.write(message_field[GPS_LON]);
      Serial.write(message_field[GPS_TIME]);
      
      if(strcmp(message_field[GPS_FSTS], "1") == 0)
      {
        gps_data->State_gps = true;
        gps_data->State_gps = true;
        
        gps_data->SpeedGPS = atof(message_field[GPS_SPEED]);
        gps_data->OrGPS = atof(message_field[GPS_ORG]);
        gps_data->Lat = atof(message_field[GPS_LAT]);
        gps_data->Lng = atof(message_field[GPS_LON]); 
      }
      else
      {
        gps_data->State_gps = false;
        
        gps_data->Lat = 0;
        gps_data->Lng = 0;
        gps_data->SpeedGPS = 0;
      }
    
      gps_data->time_gps.time.sec = message_field[GPS_TIME][13] - '0'  + 10*(message_field[GPS_TIME][12] - '0');
      gps_data->time_gps.time.minute = message_field[GPS_TIME][11] - '0'  + 10*(message_field[GPS_TIME][10] - '0');
      gps_data->time_gps.time.hour = message_field[GPS_TIME][9] - '0'  + 10*(message_field[GPS_TIME][8] - '0');
    
      gps_data->time_gps.time.date = message_field[GPS_TIME][7] - '0'  + 10*(message_field[GPS_TIME][6] - '0');
      gps_data->time_gps.time.month = message_field[GPS_TIME][5] - '0'  + 10*(message_field[GPS_TIME][4] - '0');
      gps_data->time_gps.time.year = message_field[GPS_TIME][3] - '0'  + 10*(message_field[GPS_TIME][2] - '0') + 100*(message_field[GPS_TIME][1] - '0') + 1000*(message_field[GPS_TIME][0] - '0');
    
  }
}