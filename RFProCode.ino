// ITC Cell Signal Measuring Tool

//#include <QMC5883L.h> //old e-compass
//QMC5883L compass; //old e-compass

#include <HMC5883L.h>
HMC5883L compass;

#include <SPI.h>
#include <GD2.h>
#include <RFExplorer_3GP_IoT.h>
#include <Wire.h>

RFExplorer_3GP_IoT g_objRF;      //Global 3G+ object for access to all RF Explorer IoT functionality

int o = 150; //needs to be changed for each e-compass 290

int offset = 10; //arbitrary offset for vertical separation of displayed entries on LCD
int Yaw = 0; //compass rotational angle
unsigned long int nFreqPeakKHZBest2 = 0;
int16_t nPeakDBMBest2 = -120;
int YawSingleScan2 = 0;
int BestArrayIndex2 = 0;
unsigned long int nFreqPeakKHZ = 0;
int16_t nPeakDBM = 0;
unsigned long int nFreqPeakKHZBest = 0;
int16_t nPeakDBMBest = -120;
int BestArrayIndex = 0;
int BestPosition = 0;
int DoneStatus = 0;
int counter = 0;
int YawAvg = 60;
int Position = 0;
int MeasurementsTaken = 1;
unsigned short int nProcessResult = _RFE_IGNORE;

unsigned long int nStartFreqKHZ[7] = {729000, 734000, 746000, 869000, 1930000, 1930000, 2110000};
unsigned long int nEndFreqKHZ[7] = {746000, 746000, 756000, 894000, 1990000, 1995000, 2155000};

int i = 0;
int q = 0;
int j = 0;
int YawSingleScan = 0;
int set[7] = {1, 1, 1, 1, 1, 1, 1};
int done = 0;
int results = 0;
int currentDB[7] = {0, 0, 0, 0, 0, 0, 0};
int currentAngle;
int bestContDB = -120;
int bestContAngle;
int bestDB[7] = { -120, -120, -120, -120, -120, -120, -120};
int bestAngle[7] = {0, 0, 0, 0, 0, 0, 0};
int bestAngleOverall = 0;

void setup()
{
  //RFexplorer initializing
  digitalWrite(_RFE_GPIO2, LOW);      //Set _RFE_GPIO2 as output, LOW -> Device mode to 2400bps
  g_objRF.resetHardware();            //Reset 3G+ board

  delay(5000);                        //Wait for 3G+ to complete initialization routines
  g_objRF.init();                     //Initialize 3G+ library - Monitor SerialDebugger set 57600bps

  g_objRF.changeBaudrate(115200);     //Change baudrate to 115Kbps, max reliable in Arduino DUE.
  delay(1000);                        //Wait 1sec to stablish communication
  digitalWrite(_RFE_GPIO2, HIGH);
  pinMode(_RFE_GPIO2, INPUT_PULLUP);  //Set _RFE_GPIO2 as a general port, no longer needed after start completed

  //g_objRF.SetInputStage(LNA_25dB);  ///Uncomment this line to configure input stage. Use Direct, Attenuator_30dB or LNA_25dB. Device always boots in Direct mode.

  g_objRF.sendNewConfig(nStartFreqKHZ[i], nEndFreqKHZ[i]);
  Wire.begin();

  Serial.begin(9600);
  Serial.println("Initialize HMC5883L");
  if (compass.begin()) {
    // Set measurement range
    compass.setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    compass.setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    compass.setDataRate(HMC5883L_DATARATE_30HZ);

    // Set number of samples averaged
    compass.setSamples(HMC5883L_SAMPLES_8);

    // Set calibration offset. See HMC5883L_calibration.ino
    compass.setOffset(-20, -106); //needs to changed for each e-compass.

    GD.begin(); //Gameduino intialize

    freqSelection();
    while (1) {
      bestDB[0] = -120;
      bestDB[1] = -120;
      bestDB[2] = -120;
      bestDB[3] = -120;
      bestDB[4] = -120;
      bestDB[5] = -120;
      bestDB[6] = -120;

      currentDB[0] = -120;
      currentDB[1] = -120;
      currentDB[2] = -120;
      currentDB[3] = -120;
      currentDB[4] = -120;
      currentDB[5] = -120;
      currentDB[6] = -120;

      bestAngle[0] = 0;
      bestAngle[1] = 0;
      bestAngle[2] = 0;
      bestAngle[3] = 0;
      bestAngle[4] = 0;
      bestAngle[5] = 0;
      bestAngle[6] = 0;

      currentAngle = 0;
      bestContDB = -120;
      bestContAngle = 0;

      nFreqPeakKHZBest2 = 0;
      nPeakDBMBest2 = -120;
      YawSingleScan2 = 0;
      BestArrayIndex2 = 0;

      nFreqPeakKHZBest = 0;
      nPeakDBMBest = -120;
      YawSingleScan = 0;
      BestArrayIndex = 0;

      GD.ClearColorRGB(0xFF0000);
      GD.Clear();
      GD.get_inputs();

      GD.Tag(1);
      GD.cmd_button(20, 60, 140, 120, 18, 0, "360 Scan");
      GD.Tag(2);
      GD.cmd_button(170, 60, 140, 120, 18, 0, "Single Scan");
      GD.Tag(3);
      GD.cmd_button(320, 60, 140, 120, 18, 0, "Continuous Scan");
      GD.Tag(4);
      GD.cmd_button(20, 200, 140, 50, 18, 0, "Back");
      GD.Tag(255);

      Vector norm = compass.readNormalize();

      float heading = atan2(norm.YAxis, norm.XAxis);
      heading = -heading;

      // Formula: (deg + (min / 60.0)) / (180 / M_PI);
      float declinationAngle = (47 + (9 / 60.0)) / (180 / M_PI);
      heading += declinationAngle;

      // Correct for heading < 0deg and heading > 360deg
      if (heading < 0)
      {
        heading += 2 * PI;
      }

      if (heading > 2 * PI)
      {
        heading -= 2 * PI;
      }

      // Convert to degrees
      float headingDegrees = -heading * (180 / M_PI);
      headingDegrees += 360;
      if (headingDegrees < o) {
        headingDegrees = headingDegrees - o + 360;
      } else {
        headingDegrees = headingDegrees - o;
      }
      YawAvg = headingDegrees;
      delay(100);

      GD.cmd_text(20, 20, 18, 0, "The current Yaw angle is: ");
      GD.cmd_number(225, 20, 18, OPT_SIGNED, YawAvg);

      GD.get_inputs();
      if (GD.inputs.tag == 1) { //360 scan
        // tag 1 was pressed
        DoneStatus = 0;
        j = 1;

        results = 1;
        loop();

        if (results == 1) { //display results screen
          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          GD.Tag(7);
          GD.cmd_button(330, 210, 140, 50, 18, 0, "Back");

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  |Yaw Angle|");
          if (BestArrayIndex2 == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |         |");
          } else if (BestArrayIndex2 == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |         |");
          } else if (BestArrayIndex2 == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |         |");
          } else if (BestArrayIndex2 == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |         |");
          } else if (BestArrayIndex2 == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |         |");
          } else if (BestArrayIndex2 == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |         |");
          } else if (BestArrayIndex2 == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |         |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest2); //DB
          GD.cmd_number(275, 30, 16, OPT_SIGNED, YawSingleScan2); //Yaw angle

          GD.cmd_text(10, 70, 16, 0, "Best From Each Band");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |         |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, bestDB[0]); //DB
            GD.cmd_number(275, 90, 16, OPT_SIGNED, bestAngle[0]); //Yaw
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |         |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, bestDB[1]); //DB
            GD.cmd_number(275, 110, 16, OPT_SIGNED, bestAngle[1]); //Yaw
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |         |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, bestDB[2]); //DB
            GD.cmd_number(275, 130, 16, OPT_SIGNED, bestAngle[2]); //Yaw
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |         |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, bestDB[3]); //DB
            GD.cmd_number(275, 150, 16, OPT_SIGNED, bestAngle[3]); //Yaw
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |         |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, bestDB[4]); //DB
            GD.cmd_number(275, 170, 16, OPT_SIGNED, bestAngle[4]); //Yaw
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |         |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, bestDB[5]); //DB
            GD.cmd_number(275, 190, 16, OPT_SIGNED, bestAngle[5]); //Yaw
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |         |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, bestDB[6]); //DB
            GD.cmd_number(275, 210, 16, OPT_SIGNED, bestAngle[6]); //Yaw
          }

          GD.swap();

          while (results == 1) {
            GD.get_inputs();
            if (GD.inputs.tag == 7) {
              results = 0;
            }
          }
        }

      } else if (GD.inputs.tag == 2) { //single scan
        // tag 2 was pressed
        nPeakDBMBest = -120;
        nFreqPeakKHZBest = 0;
        Vector norm = compass.readNormalize();
        float heading = atan2(norm.YAxis, norm.XAxis);
        heading = -heading;

        // Formula: (deg + (min / 60.0)) / (180 / M_PI);
        float declinationAngle = (47 + (9 / 60.0)) / (180 / M_PI);
        heading += declinationAngle;

        // Correct for heading < 0 deg and heading > 360 deg
        if (heading < 0)
        {
          heading += 2 * PI;
        }

        if (heading > 2 * PI)
        {
          heading -= 2 * PI;
        }

        // Convert to degrees
        float headingDegrees = -heading * (180 / M_PI);
        headingDegrees += 360;
        if (headingDegrees < o) {
         headingDegrees = headingDegrees - o + 360;
        } else {
         headingDegrees = headingDegrees - o;
        }
        YawSingleScan = headingDegrees;

        delay(100);

        i = 0;
        Position = 0;
        MeasurementsTaken = 0;
        g_objRF.setRun();
        j = 2;
        measuring();

        results = 1;

        if (results == 1) { //display results screen
          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          GD.Tag(7);
          GD.cmd_button(330, 210, 140, 50, 18, 0, "Back");

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  |Yaw Angle|");
          if (BestArrayIndex == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |         |");
          } else if (BestArrayIndex == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |         |");
          } else if (BestArrayIndex == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |         |");
          } else if (BestArrayIndex == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |         |");
          } else if (BestArrayIndex == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |         |");
          } else if (BestArrayIndex == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |         |");
          } else if (BestArrayIndex == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |         |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest); //DB
          GD.cmd_number(275, 30, 16, OPT_SIGNED, YawSingleScan); //Yaw angle

          GD.cmd_text(10, 70, 16, 0, "From Each Band");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |         |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, bestDB[0]); //DB
            GD.cmd_number(275, 90, 16, OPT_SIGNED, bestAngle[0]); //Yaw
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |         |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, bestDB[1]); //DB
            GD.cmd_number(275, 110, 16, OPT_SIGNED, bestAngle[1]); //Yaw
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |         |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, bestDB[2]); //DB
            GD.cmd_number(275, 130, 16, OPT_SIGNED, bestAngle[2]); //Yaw
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |         |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, bestDB[3]); //DB
            GD.cmd_number(275, 150, 16, OPT_SIGNED, bestAngle[3]); //Yaw
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |         |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, bestDB[4]); //DB
            GD.cmd_number(275, 170, 16, OPT_SIGNED, bestAngle[4]); //Yaw
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |         |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, bestDB[5]); //DB
            GD.cmd_number(275, 190, 16, OPT_SIGNED, bestAngle[5]); //Yaw
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |         |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, bestDB[6]); //DB
            GD.cmd_number(275, 210, 16, OPT_SIGNED, bestAngle[6]); //Yaw
          }

          GD.swap();

          while (results == 1) {
            GD.get_inputs();
            if (GD.inputs.tag == 7) {
              results = 0;
            }
          }
        }
      } else if (GD.inputs.tag == 3) { //continuous scan
        j = 3;
        GD.ClearColorRGB(0xFF0000);
        GD.Clear();

        GD.cmd_text(50, 50 + 7 * offset, 16, 0, "Measurements are currently running. Please wait.");
        GD.swap();
        while (q == 0) {
          nPeakDBMBest = -120;
          nFreqPeakKHZBest = 0;
          Vector norm = compass.readNormalize();
          float heading = atan2(norm.YAxis, norm.XAxis);
          heading = -heading;

          // Formula: (deg + (min / 60.0)) / (180 / M_PI);
          float declinationAngle = (47 + (9 / 60.0)) / (180 / M_PI);
          heading += declinationAngle;

          // Correct for heading < 0 deg and heading > 360 deg
          if (heading < 0)
          {
            heading += 2 * PI;
          }

          if (heading > 2 * PI)
          {
            heading -= 2 * PI;
          }

          // Convert to degrees
          float headingDegrees = -heading * (180 / M_PI);
          headingDegrees += 360;
          if (headingDegrees < o) {
           headingDegrees = headingDegrees - o + 360;
          } else {
           headingDegrees = headingDegrees - o;
          }
          YawSingleScan = headingDegrees;

          delay(100);
          i = 0;
          Position = 0;
          MeasurementsTaken = 0;
          g_objRF.setRun();

          measuring();
          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          if (nPeakDBMBest > nPeakDBMBest2) {
            nPeakDBMBest2 = nPeakDBMBest;
            nFreqPeakKHZBest2 = nFreqPeakKHZBest;
            BestArrayIndex2 = BestArrayIndex;
            YawSingleScan2 = YawSingleScan;
          }

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  |Yaw Angle|");
          if (BestArrayIndex2 == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |         |");
          } else if (BestArrayIndex2 == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |         |");
          } else if (BestArrayIndex2 == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |         |");
          } else if (BestArrayIndex2 == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |         |");
          } else if (BestArrayIndex2 == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |         |");
          } else if (BestArrayIndex2 == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |         |");
          } else if (BestArrayIndex2 == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |         |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest2); //DB
          GD.cmd_number(275, 30, 16, OPT_SIGNED, YawSingleScan2); //Yaw angle

          GD.cmd_text(10, 70, 16, 0, "Current");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |         |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, currentDB[0]); //DB
            GD.cmd_number(275, 90, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |         |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, currentDB[1]); //DB
            GD.cmd_number(275, 110, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |         |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, currentDB[2]); //DB
            GD.cmd_number(275, 130, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |         |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, currentDB[3]); //DB
            GD.cmd_number(275, 150, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |         |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, currentDB[4]); //DB
            GD.cmd_number(275, 170, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |         |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, currentDB[5]); //DB
            GD.cmd_number(275, 190, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |         |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, currentDB[6]); //DB
            GD.cmd_number(275, 210, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }

          GD.Tag(5);
          GD.cmd_button(330, 10, 140, 50, 18, 0, "View Results");

          GD.swap();
          delay(1000); //delay 1 sec
          GD.get_inputs();
          if (GD.inputs.tag == 5) {
            q = 1;
          }

          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  |Yaw Angle|");
          if (BestArrayIndex2 == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |         |");
          } else if (BestArrayIndex2 == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |         |");
          } else if (BestArrayIndex2 == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |         |");
          } else if (BestArrayIndex2 == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |         |");
          } else if (BestArrayIndex2 == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |         |");
          } else if (BestArrayIndex2 == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |         |");
          } else if (BestArrayIndex2 == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |         |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest2); //DB
          GD.cmd_number(275, 30, 16, OPT_SIGNED, YawSingleScan2); //Yaw angle

          GD.cmd_text(10, 70, 16, 0, "Current");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |         |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, currentDB[0]); //DB
            GD.cmd_number(275, 90, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |         |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, currentDB[1]); //DB
            GD.cmd_number(275, 110, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |         |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, currentDB[2]); //DB
            GD.cmd_number(275, 130, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |         |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, currentDB[3]); //DB
            GD.cmd_number(275, 150, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |         |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, currentDB[4]); //DB
            GD.cmd_number(275, 170, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |         |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, currentDB[5]); //DB
            GD.cmd_number(275, 190, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |         |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, currentDB[6]); //DB
            GD.cmd_number(275, 210, 16, OPT_SIGNED, YawSingleScan); //Yaw
          }

          GD.swap();
        }

        q = 0;
        results = 1;

        if (results == 1) { //display results screen
          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          GD.Tag(7);
          GD.cmd_button(330, 210, 140, 50, 18, 0, "Back");

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  |Yaw Angle|");
          if (BestArrayIndex2 == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |         |");
          } else if (BestArrayIndex2 == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |         |");
          } else if (BestArrayIndex2 == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |         |");
          } else if (BestArrayIndex2 == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |         |");
          } else if (BestArrayIndex2 == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |         |");
          } else if (BestArrayIndex2 == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |         |");
          } else if (BestArrayIndex2 == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |         |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest2); //DB
          GD.cmd_number(275, 30, 16, OPT_SIGNED, YawSingleScan2); //Yaw angle

          GD.cmd_text(10, 70, 16, 0, "Best From Each Band");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |         |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, bestDB[0]); //DB
            GD.cmd_number(275, 90, 16, OPT_SIGNED, bestAngle[0]); //Yaw
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |         |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, bestDB[1]); //DB
            GD.cmd_number(275, 110, 16, OPT_SIGNED, bestAngle[1]); //Yaw
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |         |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, bestDB[2]); //DB
            GD.cmd_number(275, 130, 16, OPT_SIGNED, bestAngle[2]); //Yaw
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |         |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, bestDB[3]); //DB
            GD.cmd_number(275, 150, 16, OPT_SIGNED, bestAngle[3]); //Yaw
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |         |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, bestDB[4]); //DB
            GD.cmd_number(275, 170, 16, OPT_SIGNED, bestAngle[4]); //Yaw
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |         |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, bestDB[5]); //DB
            GD.cmd_number(275, 190, 16, OPT_SIGNED, bestAngle[5]); //Yaw
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |         |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, bestDB[6]); //DB
            GD.cmd_number(275, 210, 16, OPT_SIGNED, bestAngle[6]); //Yaw
          }

          GD.swap();

          while (results == 1) {
            GD.get_inputs();
            if (GD.inputs.tag == 7) {
              results = 0;
            }
          }
        }

      } else if (GD.inputs.tag == 4) { //back button
        freqSelection();
      }

      GD.swap(); //redraw the screen
    }
  } else {
    GD.begin(); //Gameduino intialize

    freqSelection();
    while (1) {
      bestDB[0] = -120;
      bestDB[1] = -120;
      bestDB[2] = -120;
      bestDB[3] = -120;
      bestDB[4] = -120;
      bestDB[5] = -120;
      bestDB[6] = -120;

      currentDB[0] = -120;
      currentDB[1] = -120;
      currentDB[2] = -120;
      currentDB[3] = -120;
      currentDB[4] = -120;
      currentDB[5] = -120;
      currentDB[6] = -120;

      bestAngle[0] = 0;
      bestAngle[1] = 0;
      bestAngle[2] = 0;
      bestAngle[3] = 0;
      bestAngle[4] = 0;
      bestAngle[5] = 0;
      bestAngle[6] = 0;

      bestAngleOverall = 0;

      bestContDB = -120;
      nFreqPeakKHZBest2 = 0;
      nPeakDBMBest2 = -120;
      BestArrayIndex2 = 0;

      nFreqPeakKHZBest = 0;
      nPeakDBMBest = -120;
      BestArrayIndex = 0;

      GD.ClearColorRGB(0xFF0000);
      GD.Clear();
      GD.get_inputs();

      GD.Tag(1);
      GD.cmd_button(20, 60, 140, 120, 18, 0, "360 Scan");
      GD.Tag(2);
      GD.cmd_button(170, 60, 140, 120, 18, 0, "Single Scan");
      GD.Tag(3);
      GD.cmd_button(320, 60, 140, 120, 18, 0, "Continuous Scan");
      GD.Tag(4);
      GD.cmd_button(20, 200, 140, 50, 18, 0, "Back");
      GD.Tag(255);

      GD.get_inputs();
      if (GD.inputs.tag == 1) { //360 scan
        // tag 1 was pressed
        GD.ClearColorRGB(0xFF0000);
        GD.Clear();
        DoneStatus = 0;
        j = 1;

        results = 1;
        loop2();

        if (results == 1) { //display results screen
          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          GD.Tag(7);
          GD.cmd_button(330, 210, 140, 50, 18, 0, "Back");

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  | Degree |");
          if (BestArrayIndex2 == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |        |");
          } else if (BestArrayIndex2 == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |        |");
          } else if (BestArrayIndex2 == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |        |");
          } else if (BestArrayIndex2 == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |        |");
          } else if (BestArrayIndex2 == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |        |");
          } else if (BestArrayIndex2 == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |        |");
          } else if (BestArrayIndex2 == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |        |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest2); //DB
          GD.cmd_number(275, 30, 16, OPT_SIGNED, bestAngle[BestArrayIndex2]); //angle

          GD.cmd_text(10, 70, 16, 0, "Best From Each Band");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |         |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, bestDB[0]); //DB
            GD.cmd_number(275, 90, 16, OPT_SIGNED, bestAngle[0]); //angle
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |         |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, bestDB[1]); //DB
            GD.cmd_number(275, 110, 16, OPT_SIGNED, bestAngle[1]); //angle
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |         |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, bestDB[2]); //DB
            GD.cmd_number(275, 130, 16, OPT_SIGNED, bestAngle[2]); //angle
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |         |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, bestDB[3]); //DB
            GD.cmd_number(275, 150, 16, OPT_SIGNED, bestAngle[3]); //angle
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |         |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, bestDB[4]); //DB
            GD.cmd_number(275, 170, 16, OPT_SIGNED, bestAngle[4]); //angle
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |         |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, bestDB[5]); //DB
            GD.cmd_number(275, 190, 16, OPT_SIGNED, bestAngle[5]); //angle
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |         |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, bestDB[6]); //DB
            GD.cmd_number(275, 210, 16, OPT_SIGNED, bestAngle[6]); //angle
          }

          GD.swap();

          while (results == 1) {
            GD.get_inputs();
            if (GD.inputs.tag == 7) {
              results = 0;
            }
          }
        }

      } else if (GD.inputs.tag == 2) { //single scan
        // tag 2 was pressed
        nPeakDBMBest = -120;
        nFreqPeakKHZBest = 0;
        i = 0;
        Position = 0;
        MeasurementsTaken = 0;
        g_objRF.setRun();
        j = 2;
        measuring();

        results = 1;

        if (results == 1) { //display results screen
          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          GD.Tag(7);
          GD.cmd_button(330, 210, 140, 50, 18, 0, "Back");

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  |");
          if (BestArrayIndex == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |");
          } else if (BestArrayIndex == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |");
          } else if (BestArrayIndex == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |");
          } else if (BestArrayIndex == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |");
          } else if (BestArrayIndex == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |");
          } else if (BestArrayIndex == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |");
          } else if (BestArrayIndex == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest); //DB

          GD.cmd_text(10, 70, 16, 0, "From Each Band");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, bestDB[0]); //DB
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, bestDB[1]); //DB
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, bestDB[2]); //DB
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, bestDB[3]); //DB
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, bestDB[4]); //DB
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, bestDB[5]); //DB
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, bestDB[6]); //DB
          }

          GD.swap();

          while (results == 1) {
            GD.get_inputs();
            if (GD.inputs.tag == 7) {
              results = 0;
            }
          }
        }
      } else if (GD.inputs.tag == 3) { //continuous scan
        j = 3;
        GD.ClearColorRGB(0xFF0000);
        GD.Clear();

        GD.cmd_text(50, 50 + 7 * offset, 16, 0, "Measurements are currently running. Please wait.");
        GD.swap();
        while (q == 0) {
          nPeakDBMBest = -120;
          nFreqPeakKHZBest = 0;
          i = 0;
          Position = 0;
          MeasurementsTaken = 0;
          g_objRF.setRun();

          measuring();
          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          if (nPeakDBMBest > nPeakDBMBest2) {
            nPeakDBMBest2 = nPeakDBMBest;
            nFreqPeakKHZBest2 = nFreqPeakKHZBest;
            BestArrayIndex2 = BestArrayIndex;
          }

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  |");
          if (BestArrayIndex2 == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |");
          } else if (BestArrayIndex2 == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |");
          } else if (BestArrayIndex2 == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |");
          } else if (BestArrayIndex2 == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |");
          } else if (BestArrayIndex2 == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |");
          } else if (BestArrayIndex2 == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |");
          } else if (BestArrayIndex2 == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest2); //DB

          GD.cmd_text(10, 70, 16, 0, "Current");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, currentDB[0]); //DB
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, currentDB[1]); //DB
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, currentDB[2]); //DB
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, currentDB[3]); //DB
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, currentDB[4]); //DB
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, currentDB[5]); //DB
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, currentDB[6]); //DB
          }

          GD.Tag(5);
          GD.cmd_button(330, 10, 140, 50, 18, 0, "View Results");

          GD.swap();
          delay(1000); //delay 1 sec
          GD.get_inputs();
          if (GD.inputs.tag == 5) {
            q = 1;
          }

          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  |");
          if (BestArrayIndex2 == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |");
          } else if (BestArrayIndex2 == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |");
          } else if (BestArrayIndex2 == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |");
          } else if (BestArrayIndex2 == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |");
          } else if (BestArrayIndex2 == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |");
          } else if (BestArrayIndex2 == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |");
          } else if (BestArrayIndex2 == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest2); //DB

          GD.cmd_text(10, 70, 16, 0, "Current");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, currentDB[0]); //DB
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, currentDB[1]); //DB
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, currentDB[2]); //DB
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, currentDB[3]); //DB
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, currentDB[4]); //DB
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, currentDB[5]); //DB
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, currentDB[6]); //DB
          }

          GD.swap();
        }

        q = 0;
        results = 1;

        if (results == 1) { //display results screen
          GD.ClearColorRGB(0xFF0000);
          GD.Clear();

          GD.Tag(7);
          GD.cmd_button(330, 210, 140, 50, 18, 0, "Back");

          GD.cmd_text(10, 10, 16, 0, "Best Overall         |  dBM  |");
          if (BestArrayIndex2 == 0) {
            GD.cmd_text(10, 30, 16, 0, "12. 729-746 MHz      |       |");
          } else if (BestArrayIndex2 == 1) {
            GD.cmd_text(10, 30, 16, 0, "17. 734-746 MHz      |       |");
          } else if (BestArrayIndex2 == 2) {
            GD.cmd_text(10, 30, 16, 0, "13. 746-756 MHz      |       |");
          } else if (BestArrayIndex2 == 3) {
            GD.cmd_text(10, 30, 16, 0, "5.  869-894 MHz      |       |");
          } else if (BestArrayIndex2 == 4) {
            GD.cmd_text(10, 30, 16, 0, "2.  1930-1990 MHz    |       |");
          } else if (BestArrayIndex2 == 5) {
            GD.cmd_text(10, 30, 16, 0, "25. 1930-1995 MHz    |       |");
          } else if (BestArrayIndex2 == 6) {
            GD.cmd_text(10, 30, 16, 0, "4.  2110-2155 MHz    |       |");
          }

          GD.cmd_number(200, 30, 16, OPT_SIGNED, nPeakDBMBest2); //DB

          GD.cmd_text(10, 70, 16, 0, "Best From Each Band");
          if (set[0] == 1) {
            GD.cmd_text(10, 90, 16, 0, "12. 729-746 MHz      |       |");
            GD.cmd_number(200, 90, 16, OPT_SIGNED, bestDB[0]); //DB
          }
          if (set[1] == 1) {
            GD.cmd_text(10, 110, 16, 0, "17. 734-746 MHz      |       |");
            GD.cmd_number(200, 110, 16, OPT_SIGNED, bestDB[1]); //DB
          }
          if (set[2] == 1) {
            GD.cmd_text(10, 130, 16, 0, "13. 746-756 MHz      |       |");
            GD.cmd_number(200, 130, 16, OPT_SIGNED, bestDB[2]); //DB
          }
          if (set[3] == 1) {
            GD.cmd_text(10, 150, 16, 0, "5.  869-894 MHz      |       |");
            GD.cmd_number(200, 150, 16, OPT_SIGNED, bestDB[3]); //DB
          }
          if (set[4] == 1) {
            GD.cmd_text(10, 170, 16, 0, "2.  1930-1990 MHz    |       |");
            GD.cmd_number(200, 170, 16, OPT_SIGNED, bestDB[4]); //DB
          }
          if (set[5] == 1) {
            GD.cmd_text(10, 190, 16, 0, "25. 1930-1995 MHz    |       |");
            GD.cmd_number(200, 190, 16, OPT_SIGNED, bestDB[5]); //DB
          }
          if (set[6] == 1) {
            GD.cmd_text(10, 210, 16, 0, "4.  2110-2155 MHz    |       |");
            GD.cmd_number(200, 210, 16, OPT_SIGNED, bestDB[6]); //DB
          }

          GD.swap();

          while (results == 1) {
            GD.get_inputs();
            if (GD.inputs.tag == 7) {
              results = 0;
            }
          }
        }

      } else if (GD.inputs.tag == 4) { //back button
        freqSelection();
      }

      GD.swap(); //redraw the screen
    }
  }
}


void loop() {
  int YawSum = 0;
  Position = 0;


  while (DoneStatus == 0) {
    g_objRF.setHold();

    Vector norm = compass.readNormalize();
    float heading = atan2(norm.YAxis, norm.XAxis);
    heading = -heading;

    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    float declinationAngle = (47 + (9 / 60.0)) / (180 / M_PI);
    heading += declinationAngle;

    // Correct for heading < 0 deg and heading > 360 deg
    if (heading < 0)
    {
      heading += 2 * PI;
    }

    if (heading > 2 * PI)
    {
      heading -= 2 * PI;
    }

    // Convert to degrees
    float headingDegrees = -heading * (180 / M_PI);
    headingDegrees += 360;
    if (headingDegrees < o) {
     headingDegrees = headingDegrees - o + 360;
    } else {
     headingDegrees = headingDegrees - o;
    }
    YawAvg = headingDegrees;

    delay(100);

    GD.ClearColorRGB(0xFF0000);
    GD.Clear();

    GD.Tag(6);
    GD.cmd_button(320, 200, 140, 50, 18, 0, "Cancel"); //cancel 360

    if (MeasurementsTaken == 1) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 0 degrees.");
    } else if (MeasurementsTaken == 2) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 20 degrees");
    } else if (MeasurementsTaken == 3) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 40 degrees");
    } else if (MeasurementsTaken == 4) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 60 degrees");
    } else if (MeasurementsTaken == 5) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 80 degrees");
    } else if (MeasurementsTaken == 6) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 100 degrees");
    } else if (MeasurementsTaken == 7) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 120 degrees");
    } else if (MeasurementsTaken == 8) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 140 degrees");
    } else if (MeasurementsTaken == 9) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 160 degrees");
    } else if (MeasurementsTaken == 10) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 180 degrees");
    } else if (MeasurementsTaken == 11) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 200 degrees");
    } else if (MeasurementsTaken == 12) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 220 degrees");
    } else if (MeasurementsTaken == 13) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 240 degrees");
    } else if (MeasurementsTaken == 14) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 260 degrees");
    } else if (MeasurementsTaken == 15) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 280 degrees");
    } else if (MeasurementsTaken == 16) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 300 degrees");
    } else if (MeasurementsTaken == 17) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 320 degrees");
    } else if (MeasurementsTaken == 18) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 340 degrees");
    }

    GD.cmd_text(70, 60 + 6 * offset, 16, 0, "The current Yaw angle is: "); GD.cmd_number(275, 60 + 6 * offset, 16, OPT_SIGNED, YawAvg);

    GD.swap();

    //0
    if (YawAvg > 358 || (YawAvg < 2 && YawAvg > 0)) {
      Position = 1;
    }
    //20
    if (YawAvg > 18 && YawAvg < 22) {
      Position = 2;
    }
    //40
    if (YawAvg > 38 && YawAvg < 42) {
      Position = 3;
    }
    if (YawAvg > 58 && YawAvg < 62) {
      Position = 4;
    }
    if (YawAvg > 78 && YawAvg < 82) {
      Position = 5;
    }
    if (YawAvg > 98 && YawAvg < 102) {
      Position = 6;
    }
    if (YawAvg > 118 && YawAvg < 122) {
      Position = 7;
    }
    if (YawAvg > 138 && YawAvg < 142) {
      Position = 8;
    }
    if (YawAvg > 158 && YawAvg < 162) {
      Position = 9;
    }
    if (YawAvg > 178 && YawAvg < 182) {
      Position = 10;
    }
    if (YawAvg > 198 && YawAvg < 202) {
      Position = 11;
    }
    if (YawAvg > 218 && YawAvg < 222) {
      Position = 12;
    }
    if (YawAvg > 238 && YawAvg < 242) {
      Position = 13;
    }
    if (YawAvg > 258 && YawAvg < 262) {
      Position = 14;
    }
    if (YawAvg > 278 && YawAvg < 282) {
      Position = 15;
    }
    if (YawAvg > 298 && YawAvg < 302) {
      Position = 16;
    }
    if (YawAvg > 318 && YawAvg < 322) {
      Position = 17;
    }
    if (YawAvg > 338 && YawAvg < 342) {
      Position = 18;
    }


    if (Position == MeasurementsTaken) {

      Serial.print("Measurements are being run. Please wait.");
      Serial.print("The current measurement is: "); Serial.println(MeasurementsTaken);
      g_objRF.setRun();
      i = 0;
      measuring();

      if (nPeakDBMBest > nPeakDBMBest2) {
        nPeakDBMBest2 = nPeakDBMBest;
        nFreqPeakKHZBest2 = nFreqPeakKHZBest;
        BestArrayIndex2 = BestArrayIndex;
        YawSingleScan2 = YawAvg;
      }
    }


    GD.get_inputs();
    if (GD.inputs.tag == 6) {
      DoneStatus = 1;
      Position = 0;
      MeasurementsTaken = 1;
      results = 0;
    }
  }
}

void measuring() {
  //Call to these two functions to refresh library received data.
  //If this function is not being called regularly, data received from 3G+ may be lost
  //and keep unprocessed
  while (i < 7 ) {
    if (set[i] == 1) {
      do
      {
        g_objRF.updateBuffer();
        nProcessResult = g_objRF.processReceivedString();
      }
      while (!((nProcessResult == _RFE_SUCCESS) && (g_objRF.getLastMessage() == _CONFIG_MESSAGE)));
      g_objRF.sendNewConfig(nStartFreqKHZ[i], nEndFreqKHZ[i]);
      GD.ClearColorRGB(0xFF0000);
      GD.Clear();
      if (j == 1) { //360 scan
        GD.cmd_text(50, 50 + 7 * offset, 16, 0, "Measurements are currently running. Please wait.");
        YawSingleScan = YawAvg;
        GD.swap();
      } else if (j == 2) { //single scan
        GD.cmd_text(50, 50 + 7 * offset, 16, 0, "Measurements are currently running. Please wait.");
        GD.swap();
      }

      //Message received is a new configuration from 3G+
      //We show new Start/Stop KHZ range here from the new configuration

      g_objRF.getMonitorSerial().println("Change Config-------------");
      g_objRF.getMonitorSerial().print("Position:");
      g_objRF.getMonitorSerial().println(Position);
      g_objRF.getMonitorSerial().print("StartKHz: ");
      g_objRF.getMonitorSerial().println(nStartFreqKHZ[i]);
      g_objRF.getMonitorSerial().print("StopKHz:  ");
      g_objRF.getMonitorSerial().println(nEndFreqKHZ[i]);

      do
      {
        g_objRF.updateBuffer();
        nProcessResult = g_objRF.processReceivedString();
      }
      while (!((nProcessResult == _RFE_SUCCESS) && (g_objRF.getLastMessage() == _SWEEP_MESSAGE) && g_objRF.isValid()));

      //Message received was actual sweep data, we can now use internal functions
      //to get sweep data parameters
      g_objRF.getPeak(&nFreqPeakKHZ, &nPeakDBM);
      //Display frequency and amplitude of the signal peak

      g_objRF.getMonitorSerial().print(nFreqPeakKHZ);
      g_objRF.getMonitorSerial().print(" KHz = ");
      g_objRF.getMonitorSerial().print(nPeakDBM);
      g_objRF.getMonitorSerial().println(" dBm");
      g_objRF.getMonitorSerial().print("REALStartKHz: ");
      g_objRF.getMonitorSerial().println(g_objRF.getConfiguration()->getStartKHZ());
      g_objRF.getMonitorSerial().print("REALStopKHz:  ");
      g_objRF.getMonitorSerial().println(g_objRF.getConfiguration()->getEndKHZ());

      if (nPeakDBM > nPeakDBMBest) {
        nPeakDBMBest = nPeakDBM;
        nFreqPeakKHZBest = nFreqPeakKHZ;
        BestArrayIndex = i;
        BestPosition = Position;
      }

      currentDB[i] = nPeakDBM;

      if (currentDB[i] > bestDB[i]) {
        bestDB[i] = currentDB[i];
        bestAngle[i] = YawSingleScan;
      }
    }

    //Move to the next band in array, and send the new configuration to the RF explorer
    i++;

    if (i >= 7) {
      //All bands scanned, increase measurements taken, hold data acquisition
      MeasurementsTaken++;

      //If this is last Measurement needed for 360 degrees, done scanning
      if (MeasurementsTaken == 19) {
        DoneStatus = 1;
        MeasurementsTaken = 1;
        g_objRF.getMonitorSerial().print("DONESTATUS:");
      }

      //Send Command to change RF module configuration to first band
      do
      {
        g_objRF.updateBuffer();
        nProcessResult = g_objRF.processReceivedString();
      }
      while (!((nProcessResult == _RFE_SUCCESS) && (g_objRF.getLastMessage() == _CONFIG_MESSAGE)));
      g_objRF.sendNewConfig(nStartFreqKHZ[0], nEndFreqKHZ[0]);
      g_objRF.setHold();

    }
    else {
      //Wait for message received is New Config
      do
      {
        g_objRF.updateBuffer();
        nProcessResult = g_objRF.processReceivedString();
      }
      while (!((nProcessResult == _RFE_SUCCESS) && (g_objRF.getLastMessage() == _CONFIG_MESSAGE)));

      g_objRF.sendNewConfig(nStartFreqKHZ[i], nEndFreqKHZ[i]);

    }
  }

}


void freqSelection() {
  done = 0;
  while (done == 0) {
    GD.swap();
    GD.ClearColorRGB(0xFF0000);
    GD.Clear();
    GD.get_inputs();

    if (set[0] == 1) {
      GD.Tag(10);
      GD.cmd_button(20, 20, 140, 50, 18, 0, "729-746 MHz"); //729000-746000 kHz
    }

    if (set[1] == 1) {
      GD.Tag(11);
      GD.cmd_button(170, 20, 140, 50, 18, 0, "734-746 MHz"); //734000-746000 kHz
    }

    if (set[2] == 1) {
      GD.Tag(12);
      GD.cmd_button(320, 20, 140, 50, 18, 0, "746-756 MHz"); //746000-756000 kHz
    }

    if (set[3] == 1) {
      GD.Tag(13);
      GD.cmd_button(20, 80, 140, 50, 18, 0, "869-894 MHZ"); //869000-894000 kHZ
    }

    if (set[4] == 1) {
      GD.Tag(14);
      GD.cmd_button(170, 80, 140, 50, 18, 0, "1930-1990 MHZ"); //1990000-1930000 kHZ
    }

    if (set[5] == 1) {
      GD.Tag(15);
      GD.cmd_button(320, 80, 140, 50, 18, 0, "1930-1995 MHz"); //1930000-1995000 kHz
    }

    if (set[6] == 1) {
      GD.Tag(16);
      GD.cmd_button(20, 140, 140, 50, 18, 0, "2110-2155 MHZ"); //2110000-2155000 kHZ
    }

    GD.Tag(19);
    GD.cmd_button(320, 200, 140, 50, 18, 0, "Next");

    GD.Tag(20);
    GD.cmd_button(170, 200, 140, 50, 18, 0, "Clear");

    GD.get_inputs();

    if (GD.inputs.tag == 10) {
      set[0] = 0;
    } else if (GD.inputs.tag == 11) {
      set[1] = 0;
    } else if (GD.inputs.tag == 12) {
      set[2] = 0;
    } else if (GD.inputs.tag == 13) {
      set[3] = 0;
    } else if (GD.inputs.tag == 14) {
      set[4] = 0;
    } else if (GD.inputs.tag == 15) {
      set[5] = 0;
    } else if (GD.inputs.tag == 16) {
      set[6] = 0;
    } else if (GD.inputs.tag == 19) {
      //next
      done = 1;
    } else if (GD.inputs.tag == 20) {
      set[0] = 1;
      set[1] = 1;
      set[2] = 1;
      set[3] = 1;
      set[4] = 1;
      set[5] = 1;
      set[6] = 1;
    }
  }
}

void loop2() {
  Position = 0;


  while (DoneStatus == 0) {
    g_objRF.setHold();

    GD.ClearColorRGB(0xFF0000);
    GD.Clear();

    GD.Tag(6);
    GD.cmd_button(320, 200, 140, 50, 18, 0, "Cancel"); //cancel 360

    if (MeasurementsTaken == 1) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 0 degrees.");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 2) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 20 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 3) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 40 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 4) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 60 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 5) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 80 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 6) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 100 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 7) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 120 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 8) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 140 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 9) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 160 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 10) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 180 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 11) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 200 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 12) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 220 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 13) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 240 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 14) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 260 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 15) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 280 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 16) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 300 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 17) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 320 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    } else if (MeasurementsTaken == 18) {
      GD.cmd_text(70, 50 + 5 * offset, 16, 0, "Please rotate the antenna to 340 degrees");
      GD.cmd_text(70, 70 + 5 * offset, 16, 0, "Then begin scan.");
    }

    GD.Tag(30);
    GD.cmd_button(170, 200, 140, 50, 18, 0, "Scan");

    GD.swap();
    GD.get_inputs();
    if (GD.inputs.tag == 6) {
      DoneStatus = 1;
      Position = 0;
      MeasurementsTaken = 1;
      results = 0;
    }

    if (GD.inputs.tag == 30) {
      g_objRF.setRun();
      i = 0;
      measuring2();

      if (nPeakDBMBest > nPeakDBMBest2) {
        nPeakDBMBest2 = nPeakDBMBest;
        nFreqPeakKHZBest2 = nFreqPeakKHZBest;
        BestArrayIndex2 = BestArrayIndex;
      }

    }
  }
}

void measuring2() {
  //Call to these two functions to refresh library received data.
  //If this function is not being called regularly, data received from 3G+ may be lost
  //and keep unprocessed
  while (i < 7 ) {
    if (set[i] == 1) {
      do
      {
        g_objRF.updateBuffer();
        nProcessResult = g_objRF.processReceivedString();
      }
      while ((!((nProcessResult == _RFE_SUCCESS) && (g_objRF.getLastMessage() == _CONFIG_MESSAGE))));
      g_objRF.sendNewConfig(nStartFreqKHZ[i], nEndFreqKHZ[i]);
      GD.ClearColorRGB(0xFF0000);
      GD.Clear();
      if (j == 1) { //360 scan
        GD.cmd_text(50, 50 + 7 * offset, 16, 0, "Measurements are currently running. Please wait.");
        GD.swap();
      } else if (j == 2) { //single scan
        GD.cmd_text(50, 50 + 7 * offset, 16, 0, "Measurements are currently running. Please wait.");
        GD.swap();
      }

      do
      {
        g_objRF.updateBuffer();
        nProcessResult = g_objRF.processReceivedString();
      }
      while ((!((nProcessResult == _RFE_SUCCESS) && (g_objRF.getLastMessage() == _SWEEP_MESSAGE) && g_objRF.isValid())));

      //Message received was actual sweep data, we can now use internal functions
      //to get sweep data parameters
      g_objRF.getPeak(&nFreqPeakKHZ, &nPeakDBM);

      if (nPeakDBM > nPeakDBMBest) {
        nPeakDBMBest = nPeakDBM;
        nFreqPeakKHZBest = nFreqPeakKHZ;
        BestArrayIndex = i;
        BestPosition = Position;
      }

      currentDB[i] = nPeakDBM;

      if (currentDB[i] > bestDB[i]) {
        bestDB[i] = currentDB[i];

        if (j == 1) {
          if (MeasurementsTaken == 1) {
            bestAngle[i] = 0;
          } else if (MeasurementsTaken == 2) {
            bestAngle[i] = 20;
          } else if (MeasurementsTaken == 3) {
            bestAngle[i] = 40;
          } else if (MeasurementsTaken == 4) {
            bestAngle[i] = 60;
          } else if (MeasurementsTaken == 5) {
            bestAngle[i] = 80;
          } else if (MeasurementsTaken == 6) {
            bestAngle[i] = 100;
          } else if (MeasurementsTaken == 7) {
            bestAngle[i] = 120;
          } else if (MeasurementsTaken == 8) {
            bestAngle[i] = 140;
          } else if (MeasurementsTaken == 9) {
            bestAngle[i] = 160;
          } else if (MeasurementsTaken == 10) {
            bestAngle[i] = 180;
          } else if (MeasurementsTaken == 11) {
            bestAngle[i] = 200;
          } else if (MeasurementsTaken == 12) {
            bestAngle[i] = 220;
          } else if (MeasurementsTaken == 13) {
            bestAngle[i] = 240;
          } else if (MeasurementsTaken == 14) {
            bestAngle[i] = 260;
          } else if (MeasurementsTaken == 15) {
            bestAngle[i] = 280;
          } else if (MeasurementsTaken == 16) {
            bestAngle[i] = 300;
          } else if (MeasurementsTaken == 17) {
            bestAngle[i] = 320;
          } else if (MeasurementsTaken == 18) {
            bestAngle[i] = 340;
          }
        }
      }
    }


    //Move to the next band in array, and send the new configuration to the RF explorer
    i++;

    if (i >= 7) {
      //All bands scanned, increase measurements taken, hold data acquisition
      MeasurementsTaken++;

      //If this is last Measurement needed for 360 degrees, done scanning
      if (MeasurementsTaken == 19) {
        DoneStatus = 1;
        MeasurementsTaken = 1;
      }

      //Send Command to change RF module configuration to first band
      do
      {
        g_objRF.updateBuffer();
        nProcessResult = g_objRF.processReceivedString();
      }
      while ((!((nProcessResult == _RFE_SUCCESS) && (g_objRF.getLastMessage() == _CONFIG_MESSAGE))));
      g_objRF.sendNewConfig(nStartFreqKHZ[0], nEndFreqKHZ[0]);
      g_objRF.setHold();

    }
    else {
      //Wait for message received is New Config
      do
      {
        g_objRF.updateBuffer();
        nProcessResult = g_objRF.processReceivedString();
      }
      while ((!((nProcessResult == _RFE_SUCCESS) && (g_objRF.getLastMessage() == _CONFIG_MESSAGE))));

      g_objRF.sendNewConfig(nStartFreqKHZ[i], nEndFreqKHZ[i]);

    }
  }
}
