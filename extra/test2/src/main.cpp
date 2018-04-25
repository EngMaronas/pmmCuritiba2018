#include <Arduino.h>
#include <pmmGeneralFunctions.h>

const int sentenceSize = 80;

char sentence[sentenceSize];
void getField(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < sentenceSize)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
}

void displayGPS()
{
  char field[20];
  getField(field, 0);
  if (strcmp(field, "$GPRMC") == 0)
  {
    Serial.print("Lat: ");
    getField(field, 3);  // number
    Serial.print(field);
    getField(field, 4); // N/S
    Serial.print(field);

    Serial.print(" Long: ");
    getField(field, 5);  // number
    Serial.print(field);
    getField(field, 6);  // E/W
    Serial.println(field);
  }
}

SdManager sdManager;

char dummyString[] = {"01234567890123456789012345678901234567890123456789\n"};
char newString[100];
void setup()
{
  delay(1000);
  Serial.begin(250000);
  Serial4.begin(9600);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only
  Serial.println("vrau");
  if(!sdManager.init())
    Serial.println("SD INIT Success!");
  else
    Serial.println("SD INIT fail :(");

  char filename[] = {"gpsTest3.txt"};
  sdManager.setFilename(filename);
}

int counter = 0;
void loop()
{
  static int i = 0;
  if (Serial4.available())
  {
    char ch = Serial4.read();
    if (ch != '\n' && i < sentenceSize)
    {
      sentence[i] = ch;
      i++;
    }
    else
    {
     sentence[i] = '\0';
     i = 0;
     displayGPS();
    }

  }
  sprintf(newString, "%i: %s\n", counter++, dummyString);
  if(sdManager.writeToFile(newString, strlen(newString)))
  {
      Serial.println("SD ERROR!");
      delay(10000000);
  }
  Serial.print(newString);
  delay(1);
}
