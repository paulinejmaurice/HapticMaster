#ifndef HAPTIC_H_INCLUDED
#define HAPTIC_H_INCLUDED

//#include "HapticAPI.h"
#include "HapticMaster.h"
#include "math.h"

#define IPADDRESS "10.30.203.37"
#define  ERROR_MSG "--- ERROR:" // TODO espace ou pas avant ERROR

class Haptic
{
	// Methods
public:
	// Constructor
	Haptic(double inertiaHM, double floorHeight = 0., int pX = 0, int pY = 1, int pZ = 2);
	// Destructor 
	~Haptic();
	// Get function
	double* GetCurrentPosition();
	double* GetCurrentVelocity();
	double* GetCurrentAcceleration();
	double* GetCurrentForce();
	// Functions used by glut display
	void UpdateForcePositionVelocityAcceleration();
	void Terminate();
	// This Function Connects To The HapticMASTER And Initializes haptic objects
	int InitHapticMaster();
	// Enable/Disable a spring to bring back end-effector to start position
	void EnableStartPositionSpring();
	void DisableStartPositionSpring();
	void UpdateStartPositionSpring(double referencePosition[3]);
	void EnableRestrict1DMotion();
	void DisableRestrict1DMotion();
	void EnableDamper();
	void DisableDamper();
	void UpdateBallForce(double force[3]);
	void EnableBallForce();
	void DisableBallForce();
	
private:
	// Attributes
	long hapticMaster;	
	double inertia;
	double currentPosition[3];
	double currentVelocity[3];
	double currentAcceleration[3];
	double currentForce[3];
	int waitStateChange;

	// define axes orientation
	int posX, posY, posZ;

	// Spring to constrain the motion in 1D
	double springStiffness_smooth;
	double springDamping_smooth;
	double springStiffness_stiff;
	double springDamping_stiff;
	double springDeadband;
	double springPosition[3];
	double springDirection_X[3];
	double springDirection_Y[3];
	double springDirection_Z[3];
	double maxAllowedForceInSpring; // Maximum force that can be exerted by the Y spring to prevent too fast motions

	// Damper to make end of motion smoother
	double dampingCoef[3];

	// Force from ball on cup (haptic feedback)
	double maxAllowedFeedbackForce; // Prevent HM from applying too high force

};

#endif // HAPTIC_H_INCLUDED
