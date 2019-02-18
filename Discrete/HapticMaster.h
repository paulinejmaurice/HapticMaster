//---------------------------------------------------------------------
//                      H A P T I C   M A S T E R
//
// Included functions.
// 
//---------------------------------------------------------------------

#ifndef _HAPTIC_MASTER_H_
#define _HAPTIC_MASTER_H_

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <math.h>
#ifdef WIN32
	#include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include "HapticAPI.h"
#include "Vector3d.h"

#ifndef Pi
#define Pi 3.14159265358979
#endif

#ifndef Pi2
#define Pi2 6.28318530717958
#endif

#ifndef DegPerRad
#define DegPerRad 180/Pi
#endif

#ifndef RadPerDeg
#define RadPerDeg Pi/180
#endif

/*
const int iNrSegments = 10;
const double AxisLengt = 0.25; //meter

double LeftSideArc; //radians
double RightSideArc; //radians

double MinRadius; //meter
double MaxRadius; //meter

double CenterOffset; //meter

double TotalHeight; //meter
double PosHeigh; //meter
double NegHeight; //meter
*/

typedef enum FcsAxes { X = 0, Y, Z };



//---------------------------------------------------------------------
//               U N D E R S C O R E   2   S P A C E
//
// Replace all '_' charachters with a ' ' charachter.
//---------------------------------------------------------------------
char* Underscore2Space(char* inputStream);

//---------------------------------------------------------------------
//          H A   S E N D   C O M M A N D   -   n o   p a r a m s
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect no extra parameters for the command.
//---------------------------------------------------------------------
int  haSendCommand(long inDev,
	const char* inCommand,
	char* outCommand); 

//---------------------------------------------------------------------
//          H A   S E N D   C O M M A N D   -   1   d o u b l e
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect one double as a parameter for the command.
//---------------------------------------------------------------------
int  haSendCommand(long inDev,
	const char* inCommand,
	double inDouble1,
	char* outCommand); 

//---------------------------------------------------------------------
//         H A   S E N D   C O M M A N D   -   3   d o u b l e s
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect 3 doubles as parameters for the command.
//---------------------------------------------------------------------
int  haSendCommand(long inDev,
	const char* inCommand,
	double inDouble1,
	double inDouble2,
	double inDouble3,
	char* outCommand); 

//---------------------------------------------------------------------
//         H A   S E N D   C O M M A N D   -   4   d o u b l e s
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect 4 doubles as parameters for the command.
//---------------------------------------------------------------------
int  haSendCommand(long inDev,
	const char* inCommand,
	double inDouble1,
	double inDouble2,
	double inDouble3,
	double inDouble4,
	char* outCommand); 

//---------------------------------------------------------------------
//         H A   S E N D   C O M M A N D   -   V E C T O R 3 D
//
// Overloaded function to send a command to the HapticMASTER.
// This version excpect 1 Vector3d object as a parameter for the command.
//---------------------------------------------------------------------
int  haSendCommand(long inDev,
	const char* inCommand,
	Vector3d inVector,
	char* outCommand);

//---------------------------------------------------------------------
//               I N I T I A L I Z E   D E V I C E
//
// This function initializes the HapticMASTER device.
// It first searches for the end positions. When all ends are found,
// the HapticMASTER is set to the force_sensitive state.
//---------------------------------------------------------------------
void InitializeDevice(long dev);

//---------------------------------------------------------------------
//                 P A R S E   F L O A T   V E C
//
// Break a string vector into its components.
// Returns 0 if was succesful.
//---------------------------------------------------------------------
int ParseFloatVec(const char* inputString, double& xValue, double& yValue, double& zValue); 

//---------------------------------------------------------------------
//                   B R E A K   R E S P O N S E
//---------------------------------------------------------------------
int BreakResponse(char* outputString, const char* inputString, unsigned int wantedStringNumber); 


#endif














