#include "HapticMaster.h"

const int iNrSegments = 10;
const double AxisLengt = 0.25; //meter

double LeftSideArc = -0.5; //radians
double RightSideArc = 0.5; //radians

double MinRadius = 0.28; //meter
double MaxRadius = 0.64; //meter

double CenterOffset = -0.45; //meter

double TotalHeight = 0.4; //meter
double PosHeight = 0.2; //meter
double NegHeight = -0.2; //meter

//---------------------------------------------------------------------
//               U N D E R S C O R E   2   S P A C E
//
// Replace all '_' charachters with a ' ' charachter.
//---------------------------------------------------------------------
char* Underscore2Space(char* inputStream) {
   for (unsigned int i = 0; i < strlen(inputStream); i++) {
      if (inputStream[i] == '_') {
         inputStream[i] = ' ';
      }
   }

   return inputStream;
}

//---------------------------------------------------------------------
//          H A   S E N D   C O M M A N D   -   n o   p a r a m s
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect no extra parameters for the command.
//---------------------------------------------------------------------
int  haSendCommand ( long inDev, 
                     const char* inCommand, 
                     char* outCommand ) {

   if ( haDeviceSendString(inDev, inCommand, outCommand) ) {
      printf ( "--- ERROR: Command could not be sent to the Real-time\n" );
      return -1;
   }
   else {
      if ( strstr(outCommand, "--- ERROR") ) {
         return -1;
      }
      else {
         return 0;
      }
   }
}

//---------------------------------------------------------------------
//          H A   S E N D   C O M M A N D   -   1   d o u b l e
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect one double as a parameter for the command.
//---------------------------------------------------------------------
int  haSendCommand ( long inDev, 
                     const char* inCommand, 
                     double inDouble1,
                     char* outCommand ) {
   char tempString[100];
   sprintf(tempString, "%s %g", inCommand, inDouble1);
   if ( haDeviceSendString(inDev, tempString, outCommand) ) {
      printf ( "--- ERROR: Command could not be sent to the Real-time\n" );
      return -1;
   }
   else {
      if ( strstr(outCommand, "--- ERROR") ) {
         return -1;
      }
      else {
         return 0;
      }
   }
}

//---------------------------------------------------------------------
//         H A   S E N D   C O M M A N D   -   3   d o u b l e s
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect 3 doubles as parameters for the command.
//---------------------------------------------------------------------
int  haSendCommand ( long inDev, 
                     const char* inCommand, 
                     double inDouble1,
                     double inDouble2,
                     double inDouble3,
                     char* outCommand ) {
   char tempString[200];
   sprintf(tempString, "%s [%g,%g,%g]", inCommand, inDouble1, inDouble2, inDouble3);
   if ( haDeviceSendString(inDev, tempString, outCommand) ) {
      printf ( "--- ERROR: Command could not be sent to the Real-time\n" );
      return -1;
   }
   else {
      if ( strstr(outCommand, "--- ERROR") ) {
         return -1;
      }
      else {
         return 0;
      }
   }
}

//---------------------------------------------------------------------
//         H A   S E N D   C O M M A N D   -   4   d o u b l e s
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect 4 doubles as parameters for the command.
//---------------------------------------------------------------------
int  haSendCommand ( long inDev, 
                     const char* inCommand, 
                     double inDouble1,
                     double inDouble2,
                     double inDouble3,
                     double inDouble4,
                     char* outCommand ) {
   char tempString[200];
   sprintf(tempString, "%s [%g,%g,%g,%g]", inCommand, inDouble1, inDouble2, inDouble3, inDouble4);
   if ( haDeviceSendString(inDev, tempString, outCommand) ) {
      printf ( "--- ERROR: Command could not be sent to the Real-time\n" );
      return -1;
   }
   else {
      if ( strstr(outCommand, "--- ERROR") ) {
         return -1;
      }
      else {
         return 0;
      }
   }
}

//---------------------------------------------------------------------
//         H A   S E N D   C O M M A N D   -   V E C T O R 3 D
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect 1 Vector3d object as a parameter for the command.
//---------------------------------------------------------------------
int  haSendCommand ( long inDev, 
                     const char* inCommand, 
                     Vector3d inVector,
                     char* outCommand ) {
   char tempString[200];
   sprintf(tempString, "%s [%g,%g,%g]", inCommand, inVector.x, inVector.y, inVector.z);
   if ( haDeviceSendString(inDev, tempString, outCommand) ) {
      printf ( "--- ERROR: Command could not be sent to the Real-time\n" );
      return -1;
   }
   else {
      if ( strstr(outCommand, "--- ERROR") ) {
         return -1;
      }
      else {
         return 0;
      }
   }
}

//---------------------------------------------------------------------
//               I N I T I A L I Z E   D E V I C E
//
// This function initializes the HapticMASTER device.
// It first searches for the end positions. When all ends are found,
// the HapticMASTER is set to the force_sensitive state.
//---------------------------------------------------------------------
void InitializeDevice( long dev ) 
{
    char outputString[200] = "";

    haSendCommand( dev, "remove all", outputString);
    printf("remove all ==> %s\n", outputString);

    haDeviceSendString ( dev, "get os", outputString);
    printf("get os ==> %s\n", outputString);
    if ( strcmp(outputString, "Linux;") == 0) {
        haDeviceSendString ( dev, "get emergencybuttonpushed", outputString);
        printf("get emergencybuttonpushed ==> %s\n", outputString);
        if ( strcmp(outputString, "true;") == 0 ) {
            printf( "Emergency button is down, please release button!\n" );
        }

        haDeviceSendString ( dev, "get emergencyrelay", outputString);
        printf("get emergencyrelay ==> %s\n", outputString);
        if(strcmp(outputString, "false;") == 0) {
            printf( "HapticMaster not started, push start button!\n" );
        }        
    }
    
    char isCalibratedStr[10] = "";

    haSendCommand (dev, "get position_calibrated", outputString);
    printf("get position_calibrated ==> %s\n", outputString);
    strcpy(isCalibratedStr, outputString);

    if ( !strcmp(isCalibratedStr, "false;") ) {
        haDeviceSendString ( dev, "set state init", outputString);
        printf("set state init ==> %s\n", outputString);
        if (!strstr(outputString, "--- ERROR:") ) {
            printf( "Initializing the HapticMASTER. Please wait...\n" );
        }

        haDeviceSendString ( dev, "get state", outputString );

        while( strcmp(outputString, "stop;") ) {
            haDeviceSendString ( dev, "get state", outputString );
        }
    }
    printf("Setting to state Force\n");

    haDeviceSendString ( dev, "set state force", outputString);
    printf("set state force ==> %s\n", outputString);
}

//---------------------------------------------------------------------
//                 P A R S E   F L O A T   V E C
//
// Break a string vector into its components.
// Returns 0 if was succesful.
//---------------------------------------------------------------------
int ParseFloatVec( const char* inputString, double& xValue, double& yValue, double& zValue ) {
	char xValueStr[100] = "";
	char yValueStr[100] = "";
	char zValueStr[100] = "";

	char c;
	unsigned int srcIndex = 1;

	unsigned int dstIndex = 0;
	c = inputString[srcIndex];
	while (c != ',' && srcIndex < strlen(inputString) ) {
		xValueStr[dstIndex] = inputString[srcIndex];
		srcIndex++;
		dstIndex++;
		c = inputString[srcIndex];
	}
	xValueStr[dstIndex] = '\0';
	xValue = atof(xValueStr);

	srcIndex++;
	dstIndex = 0;

	c = inputString[srcIndex];
	while (c != ',' && srcIndex < strlen(inputString)) {
		yValueStr[dstIndex] = inputString[srcIndex];
		srcIndex++;
		dstIndex++;
		c = inputString[srcIndex];
	}
	yValueStr[dstIndex] = '\0';
	yValue = atof(yValueStr);

	srcIndex++;
	dstIndex = 0;

	c = inputString[srcIndex];
	while (c != ']' && srcIndex < strlen(inputString)) {
		zValueStr[dstIndex] = inputString[srcIndex];
		srcIndex++;
		dstIndex++;
		c = inputString[srcIndex];
	}
	zValueStr[dstIndex] = '\0';
	zValue = atof(zValueStr);

	return 0;	
}

//---------------------------------------------------------------------
//                   B R E A K   R E S P O N S E
//---------------------------------------------------------------------
int BreakResponse( char* outputString, const char* inputString, unsigned int wantedStringNumber ) {
   unsigned int beginIdx = 0;
   unsigned int endIdx = 0;
   unsigned int currIdx = 0;
   char c;
   bool seperatorFound = false;

   for (unsigned int i = 0; i < wantedStringNumber; i++) {
      seperatorFound = false;
      beginIdx = currIdx;
      while (!seperatorFound) {
         c = inputString[currIdx];

         if ( c == ';' || c == '\0' ) {
            seperatorFound = true;
         }
         else {
            currIdx++;
         }
      } // while
      endIdx = currIdx;
      currIdx++;
   } // for

   for (unsigned int i = beginIdx; i < endIdx; i++) {
      outputString[i-beginIdx] = inputString[i];
   }
   outputString[endIdx-beginIdx] = '\0';

   return 0;
} // BreakResponse














