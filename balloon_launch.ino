#include <math.h>
#include <SoftwareSerial.h>
#include <SD.h>

//setting up the SD File
File dataFile;

//Analog Ports
const int PRESSURE = A0; //pressure sensor
const int INTERNAL_TEMP = A1; //internal temp

const int MG_PIN = A2; //declan ind exper
const int SOLAR1 = A3; //sophie ind exper
const int SOLAR2 = A4; //sophie ind exper
const int EXTERNAL_TEMP = A5; //external temp
const int METHANE = A6; //angel ind exper

const int MIC = A15; //ned ind exper


//Digital Ports
const int CUT = 12; //cutdown (Ned)
const int HEATING_PAD = 7; //heating pad (angel)
const int trigPin = 9; //diya ind exper
const int echoPin = 8; //diya ind exper
const int RX = 10; //GPS RX
const int TX = 11; //GPS RX

const int CAMERA = 20; //camera (Avery)
const int SOUND = 21; //buzzer (Ned/Avery)


//Setting up Software Serial for GPS
SoftwareSerial ss(RX, TX); // RX, TX

//Important Contants and Global Variables

//Constants to calculate internal temp
const int resistor = 9900;
const double A = 0.00100269685334;
const double B = 0.000217231869284;
const double C = 0.00000022318675389;

//Global Variables for GPS
const int sentenceSize = 80;
char sentence[sentenceSize];

//Global Variables for Ned's individual experiment
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
bool buzzerOn = false;

//Variables for Diya's individual experiment
float d = .074; 
int t;
float vs;

//Varialbles for timing
int timing = 0;
bool hasCutDown = false;
int counter = 0;

//Variables for Declan's individual experiment
float DC_GAIN = 8.5;
int READ_SAMPLE_INTERVAL = 50;
int READ_SAMPLE_TIMES = 5;
float ZERO_POINT_VOLTAGE = 3.469;
float REACTION_VOLTAGE = 0.030;
float CO2Curve[3]  =  {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTAGE / (2.602 - 3))};

//the SD file
SdFile myFile;

//global variables to have throughout
float internalTemp;
float externalTemp;
float globalPressure;
float alt;
unsigned long timeStart;

void setup() {
  //analog pins
  pinMode(INTERNAL_TEMP, INPUT);
  pinMode(PRESSURE, INPUT);
  pinMode(SOLAR1, INPUT);
  pinMode(SOLAR2, INPUT);
  pinMode(MIC, INPUT);
  pinMode(METHANE, INPUT);
  pinMode(EXTERNAL_TEMP, INPUT);


  //digital pins
  pinMode(CUT, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(HEATING_PAD, OUTPUT);
  pinMode(CAMERA, OUTPUT);
  pinMode(SOUND, OUTPUT);

  //Setting up serial
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  ss.begin(9600);

  //Printing the column headers for the CSV file
  Serial.print("Millis, Internal Temperature Voltage, Internal Temperature in Celsius (C), External Temperature Voltage, External Temperature in Celsius (C), Pressure Voltage, Pressure (kPa),");
  Serial.print("Altitude (from pressure), Voltage across solar panel 1 (Sophie), Voltage across solar panel 2 (Sophie), Voltage Difference (Sophie),");
  Serial.print("Voltage for Ned, Time taken for Pulse to Travel (Diya), Speed of Sound (Diya),");
  Serial.print("Voltage across sensor (Declan), Co2 concentration (Declan), cutdown,");
  Serial.print("Angel Methane Data, Time, Lat, Long, Altitude");
  Serial.println();
  delay(4000);

  //Turn on camera
  digitalWrite(CAMERA, HIGH);

  //Get the time start
  timeStart = millis();
  
}


void loop() {
  Serial.print(millis());
  Serial.print(",");
  getTemperature();
  getPressure();
  sophieIndExper();
  nedIndExper();
  diyaIndExper();
  declanIndExper();
  cutdown();
  heatingPad();
  angelIndExper();
  if (hasCutDown) {
    //we only want the buzzer to start after we cut down
    soundControl();
  }

  //Getting the GPS data
  static int i = 0;
  while (true) {
    //Read characters until it is the end of a line
    if (ss.available())
    {
      char ch = ss.read();
      if (ch != '\n' && i < sentenceSize) {
        sentence[i] = ch;
        i++;
      }
    }
    else {
      delay(700);
      sentence[i] = '\0';
      i = 0;
      displayGPS();
      break;
    }
  }
  Serial.println();
}


/**
 * Angel's Individual Experiment
 * Reads in the analog voltage of the Methane sensor
 */
void angelIndExper() {
  Serial.print(analogRead(METHANE));
  Serial.print(",");
}

/**
 * Heating Pad Control
 * If the internal temperature is less than 21 degrees, 
 * turns on the heating pad
 */
void heatingPad() {
  if (internalTemp < 21) {
    digitalWrite(HEATING_PAD, HIGH);
  } else {
    digitalWrite(HEATING_PAD, LOW);
  }
}

/**
 * Buzzer Control
 * Every 15 runs of the code, turns the buzzer on or off 
 * so that it goes of intermittently
 */
void soundControl() {
  timing += 1;
  if (timing == 15) {
    //turn buzzer on / off
    if (!buzzerOn) {
      digitalWrite(SOUND, HIGH);
      buzzerOn = true;
    }
    else {
      digitalWrite(SOUND, LOW);
      buzzerOn = false;
    }
    timing = 0;
  }
}

/**
 * Cutdown Trigger
 * Runs the cutdown if we have reached the specified pressure
 * or time
 */
void cutdown() {
  digitalWrite(CUT, LOW);
  double pressure_voltage = analogRead(PRESSURE);
  if ((pressure_voltage <= 195 && (millis () - timeStart) >= 7200000) || ((millis () - timeStart) >= 10800000))
  {
    //we are cutting down if we are high enough and it has been
    //as least two hours, or if it has been three hours
    hasCutDown = true;
    digitalWrite(CUT, HIGH); //Turn on wire
    delay(30000); //Wait for magic
    digitalWrite(CUT, LOW); //Turn off wire
    digitalWrite(SOUND, LOW);
  }
  Serial.print(hasCutDown);
  Serial.print(",");
}

/**
 * Declan's Individual Experiment
 * Gets the reading of the CO2 sensors
 */
void declanIndExper() {
  int percentage;
  float volts;

  volts = MGRead(MG_PIN);
  Serial.print( "SEN0159:" );
  Serial.print(volts);
  Serial.print("V,");

  percentage = MGGetPercentage(volts, CO2Curve);
  Serial.print("CO2:");
  if (percentage == -1) {
    Serial.print( "<400" );
  } else {
    Serial.print(percentage);
  }

  Serial.print( "ppm" );
  Serial.print(",");
}

/**
 * Sophie's Individual Experiment
 * Gets the reading across two solar panels, one with a UV filter
 * and one without it. Converts the analog reading into volts and 
 * finds the difference
 */
void sophieIndExper() {
  int reading1 = analogRead(SOLAR1);
  float voltage1 = reading1 * (5.0 / 1023.0);
  int reading2 = analogRead(SOLAR2);
  float voltage2 = reading2 * (5.0 / 1023.0);
  Serial.print(voltage1);
  Serial.print(",");
  Serial.print(voltage2);
  Serial.print(",");
  float difference = (voltage2 - voltage1);
  Serial.print(difference);
  Serial.print(",");
}

/**
 * Ned's Individual Experiment
 * If the buzzer is on, record the peak amplitude of the sound
 */
void nedIndExper() {
  if (buzzerOn) {
    unsigned long startMillis = millis(); // Start of sample window
    unsigned int peakToPeak = 0;   // peak-to-peak level

    unsigned int signalMax = 0;
    unsigned int signalMin = 1024;
    // collect data for 50 mS
    while (millis() - startMillis < sampleWindow)
    {
      int sample = analogRead(MIC);
      if (sample < 1024)  // toss out spurious readings
      {
        if (sample > signalMax)
        {
          signalMax = sample;  // save just the max levels
        }
        else if (sample < signalMin)
        {
          signalMin = sample;  // save just the min levels
        }
      }
    }
    peakToPeak = signalMax - signalMin;
    double volts = (peakToPeak * 5.0) / 1024;// max - min = peak-peak amplitude
    Serial.print(volts);
    Serial.print(",");
  } else {
    Serial.print("NA"); //if buzzer is not on, still print NA to keep spacing
    Serial.print(",");
  }
}

/**
 * Diya's Individual Experiment
 * Record the time taken for the sound to travel, and the 
 * speed of sound based on the distance
 */
void diyaIndExper() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2000);
  digitalWrite(trigPin, HIGH); //trig pin
  delayMicroseconds(10);  //pulse width 10 microseconds
  digitalWrite(trigPin, LOW); //trig pin off
  t = pulseIn(echoPin, HIGH);  //pulseIn(), function return time in microseconds
  vs = (d / (float(t) * .000001));

  //print time on the serial monitor
  Serial.print(t);
  Serial.print(" microseconds");
  Serial.print(",");
  Serial.print(vs);
  Serial.print(",");
}

/**
 * GPS
 * Putting the GPS data into a string, and seeing if it is the 
 * format we need
 */
void displayGPS() {
  String testString = "";
  for (int i = 0; i < sentenceSize; i++) {
    testString += sentence[i];
  }
  if (testString.substring(0, 6) == "$GPGGA") { //we need to parse the string
    parseString(testString);
  }
}

/**
 * GPS
 * Uses the GPGGA format to parse the information we need
 * from the GPS
 */
void parseString(String s) {
  String temp = s;
  String pieces[16];
  int i = 0;

  //put the string into an array broken at the commas
  while (temp.indexOf(",") != -1) {
    int index = temp.indexOf(",");
    pieces[i] = temp.substring(0, index);
    temp = temp.substring(index + 1);
    i++;
  }

  //uses the GPGGA format and the array to get the pieces of data 
  //that we want
  String timestring = pieces[1];
  Serial.print(timestring.substring(0, 2) + ":" + timestring.substring(2, 4) + ":" + timestring.substring(4) + " UTC ,");
  String latstring = pieces[2];
  Serial.print(latstring.substring(0, 2) + "° " + latstring.substring(2) + "' " + pieces[3] + ",");
  String longstring = pieces[4];
  Serial.print(longstring.substring(0, 3) + "° " + longstring.substring(3) + "' " + pieces[5] + ",");
  Serial.print(pieces[9] + "M ,");
  alt = pieces[9].toInt();
}

/**
 * Pressure
 * Gets the reading of the pressure, and converts the analog
 * read into pressure in kPa and altititude in meters
 */
void getPressure() {
  int pressureVoltage = analogRead(PRESSURE);
  Serial.print(String(pressureVoltage));
  Serial.print(",");
  double pressure = (0.168 * double(pressureVoltage)) - 25.8;
  globalPressure = pressure;
  Serial.print(String(pressure) + "kPa,");
  double altitude = log((101.325 / pressure));
  Serial.print(String(altitude) + "m ,");

}

/**
 * Temperature
 * Gets the analog reading for both the internal and external
 * temperature, and uses the equation to turn that into the 
 * temperature in degrees celsius
 */
void getTemperature() {
  Serial.print(analogRead(INTERNAL_TEMP));
  Serial.print(",");
  double temp_resistance = resistor * ((1023.0 / float(analogRead(INTERNAL_TEMP))) - 1);
  double logR = log(temp_resistance);
  double one_over_t = A + B * logR + C * logR * logR * logR;
  double T_Kelvin = 1 / one_over_t;
  double T_Celsius = T_Kelvin - 273.15;
  Serial.print(String(T_Celsius) + "°C ,");
  internalTemp = T_Celsius;

  Serial.print(analogRead(EXTERNAL_TEMP));
  Serial.print(",");
  double ext_resistance = 150000.0 * ((1023.0 / float(analogRead(EXTERNAL_TEMP))) - 1);
  double log2 = log(ext_resistance);
  double one_over_T_ext = -0.00711919 + 0.000365161 * log2 - 0.00000036626023513 * log2 * log2 * log2;
  double ext_temp = (1 / one_over_T_ext) + 273.15;
  Serial.print(String(ext_temp) + "°C");
  externalTemp = ext_temp;
  Serial.print(",");

}

/**
 * MGRead (Helper Function for Declan's Individual Experiment)
 * Reads the output of the CO2 Sensor
 */
float MGRead(int mg_pin)
{
  int i;
  float v = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    v += analogRead(mg_pin);
    delay(READ_SAMPLE_INTERVAL);
  }
  v = (v / READ_SAMPLE_TIMES) * 5 / 1024 ;
  return v;
}

/**
 * MQ Get Percentage (Helper Function for Declan's Individual Experiment)
 * Input:   volts   - SEN-000007 output measured in volts
         pcurve  - pointer to the curve of the target gas
  Output:  ppm of the target gas
  Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(MG-811 output) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
 */
int  MGGetPercentage(float volts, float *pcurve)
{
  if ((volts / DC_GAIN ) >= ZERO_POINT_VOLTAGE) {
    return -1;
  } else {
    return pow(10, ((volts / DC_GAIN) - pcurve[1]) / pcurve[2] + pcurve[0]);
  }
}
