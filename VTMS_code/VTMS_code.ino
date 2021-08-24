#include <TinyGPS++.h>
#include <OneButton.h>
#include <EEPROMex.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

#define PswdRst A1
#define pinLen 4
#define gsmSerial Serial2
#define gpsSerial Serial1

OneButton passwordResetButton(PswdRst, true);
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x3f, 16, 2);


const unsigned long locationInterval = 1000;
unsigned long previousTime = 0;

String message;
String mobile;
int addressPin;
String pin;
bool resetPassword = false;
bool validLocation = false;
bool temp = false;

void setup()
{
  pinMode(13, OUTPUT);
  Serial .begin(9600);
  gpsSerial.begin(9600);
  digitalWrite(13, LOW);
  initLCD();
  initEEPROM();
  delay(200);
  initGSM();
  delay(100);
  gsmSerial.begin(9600);

  passwordResetButton.attachLongPressStop(onPswdRstLongClick);
  temp = false;
}

void loop()
{

  passwordResetButton.tick();


  updateSerial();
  while (temp)
  {
    Serial.println("Fetching Location");
    while (gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read());
      if (gps.location.isUpdated())
      {
        temp = false;
        digitalWrite(13, HIGH);
        tracking();
      }
      if (!temp)
        break;
    }
  }
  digitalWrite(13, LOW);
}


void tracking() {
  float _lat = gps.location.lat();
  float _lng = gps.location.lng();
  String msg = "Location is: \nlat:" + String (_lat) + "\nlng: " + String (_lng);
  if (mobile != NULL) {
    sendMsg(mobile, msg);
  }
}

void initLCD() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LCD init");
  delay(500);
}

void printLCD(String msg) {
  Serial.println(msg);
}

void initEEPROM() {
  addressPin = EEPROM.getAddress(sizeof(char) * pinLen);
  pin = read_PinEEPROM(addressPin);
  if (pin == "") {
    Serial.println("empty");
  }
  printLCD("EEPROM init");
  Serial.println(pin);
}


void initGSM() {
  gsmSerial.begin(9600);
  delay(200);
  int at_flag = 1;
  while (at_flag) {
    printLCD("Checking GSM");
    gsmSerial.println("AT");
    delay(500);
    while (gsmSerial.available())
    {
      if (gsmSerial.readString().indexOf("OK") != -1) {
        at_flag = 0;
        printLCD("GSM Found");
      }
    }
  }

  int signal_flag = 1;
  while (signal_flag) {
    printLCD("Finding Network...");
    gsmSerial.println("AT+CPIN?"); //turn off echo
    delay(500);
    while (gsmSerial.available())
    {
      if (gsmSerial.readString().indexOf("READY") != -1) {
        signal_flag = 0;
        printLCD("Network Found");
      }
    }
  }

  int echo_flag = 1;
  while (echo_flag) {
    printLCD("Disabling Echo...");
    gsmSerial.println("ATE0"); //turn off echo
    delay(500);
    while (gsmSerial.available())
    {
      if (gsmSerial.readString().indexOf("OK") != -1) {
        echo_flag = 0;
        printLCD("Echo OFF");
      }
    }
  }


  int settingSave_flag = 1;
  while (settingSave_flag) {
    gsmSerial.println("AT&W");
    delay(500);
    while (gsmSerial.available())
    {
      if (gsmSerial.readString().indexOf("OK") != -1) {
        settingSave_flag = 0;
        printLCD("Setting Saved");
      }
    }
  }

  receiveMsg();
}



void onPswdRstLongClick() {
  digitalWrite(13, HIGH);
  resetPassword = true;
  Serial.println("Password reset Mode on");
}

void updateSerial()
{
  Serial.println("Update Serial working");
  delay(500);


  while (gsmSerial.available())
  {
    message = gsmSerial.readString();
    processMessage();
  }
}



void processMessage() {
  int i = message.indexOf("+CMT");
  mobile = message.substring(9, 22);
  message.toLowerCase();
  int h = message.indexOf("help");
  printLCD(mobile);
  if (h != -1) {
    printLCD("Help");
    sendMsg(mobile, "Help Message");
  }
  else if (!resetPassword) {
    pin = read_PinEEPROM(addressPin);
    delay(500);
    if (i != -1) {
      if (pin != "") {
        message.toLowerCase();
        int j = message.indexOf("track " + pin);
        int pn = message.indexOf("track" + pin);
        int t = message.indexOf("track");
        if (j != -1 || pn != -1) {
          printLCD("Track Pin received");
          temp = true;
        }
        else if (t != -1) {
          printLCD("Incorrect Pin");
          sendMsg(mobile, "Incorrect Pin entered\nSend 'Help' to get more info.");
          receiveMsg();
        }
        else if (t == -1) {
          printLCD("Message not recognised");
          sendMsg(mobile, "Your message was not recognised, please send 'Help' for help");
          receiveMsg();
        }
      }
      else {

        message.toLowerCase();
        int p = message.indexOf("pin");
        if (p != -1) {
          String newPin = message.substring(p + 4, p + 8);
          Serial.print("********MSG: "); Serial.println(message);
          Serial.print("$$$$$$ Pin: "); Serial.println(newPin);
          write_PinEEPROM(addressPin, message.substring(p + 4, p + 8)); // newPin = message.substring(p + 4, p + 8)
          printLCD("new pin " + message.substring(p + 4, p + 8));
          delay(500);

          sendMsg(mobile, "Your pin has been set to " + message.substring(p + 4, p + 8));
          receiveMsg();

        }
        else {
          printLCD("Pin not found");
          sendMsg(mobile, "Please set 4 digit pin first. Send the following command to device to set the pin by replacing 'X' with the digits:\nPin XXXX");
        }
      }
    }
  }
  else {
    message.toLowerCase();
    int newPinIndex = message.indexOf("new pin");
    if (newPinIndex != -1) {
      clearEEPROMPin(addressPin);
      write_PinEEPROM(addressPin, message.substring(newPinIndex + 8, newPinIndex + 12));//String new_Pin = message.substring(newPinIndex + 8, newPinIndex + 12);
      printLCD("new pin " + message.substring(newPinIndex + 8, newPinIndex + 12));
      String pinChangesMsg = "New Pin Changed to " + message.substring(newPinIndex + 8, newPinIndex + 12);
      sendMsg(mobile, pinChangesMsg );
      resetPassword = false;
      Serial.println("Password reset Mode off");
      digitalWrite(13, LOW);
    }
    else {
      resetPassword = false;
      digitalWrite(13, LOW);
      printLCD("Unable to set new pin.");
      sendMsg(mobile, "Invalid format.Use format:\nNew Pin XXXX");
    }
  }
}

void sendMsg(String mobile, String msg) {
  gsmSerial.println("AT+CMGS=\"" + mobile + "\"\r"); //Mobile phone number to send message
  delay(1000);
  gsmSerial.println(msg);                        // This is the message to be sent.
  delay(1000);
  gsmSerial.println((char)26);                       // ASCII code of CTRL+Z to finalized the sending of sms
  delay(1000);

}

void receiveMsg() {
  int receive_flag = 1;
  while (receive_flag) {
    delay(100);
    gsmSerial.println("AT+CMGF=1"); // Configuring TEXT mode
    gsmSerial.println("AT+CNMI=1,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
    while (gsmSerial.available()) {
      if (gsmSerial.readString().indexOf("OK") != -1) {
        receive_flag = 0;
        printLCD("Ready");
      }
    }
  }
}

bool write_PinEEPROM(int Addr, String input) {
  char charbuf[pinLen + 1];
  input.toCharArray(charbuf, pinLen + 1);
  return EEPROM.writeBlock<char>(Addr, charbuf, pinLen + 1);
}


String read_PinEEPROM(int Addr) {
  String outputEEPROM;
  char output[] = "    ";
  byte outputByte;
  outputByte = EEPROM.read(Addr);
  Serial.print("Output byte: "); Serial.println(outputByte);
  if (outputByte == 255 || outputByte == 0) {
    Serial.println("empty one");
    outputEEPROM = "";
  }
  else {

    Serial.println("Non empty");
    EEPROM.readBlock<char>(Addr, output, pinLen);
    outputEEPROM = String (output);
    Serial.print("Output Pin: "); Serial.println(outputEEPROM);
  }

  return outputEEPROM;
}





void clearEEPROMPin(int address) {
  for (int i = address; i < address + pinLen + 1; i++) {
    EEPROM.write(i, 0);
  }
}
