#include "haptic.h"
#include <map>

Haptic::Haptic(double inertiaHM, double floorHeight, int pX, int pY, int pZ) // all the haptic objects must be created afterwards because hapticmaster is not created yet
{
	hapticMaster = HARET_ERROR;
	inertia = inertiaHM;

	posX = pX;
	posY = pY;
	posZ = pZ;

	// Will be automatically updated when glut Loop begins
	for (int i = 0; i<3; i++)
	{
		currentPosition[i] = 0.0;
		currentVelocity[i] = 0.0;
		currentAcceleration[i] = 0.0;
		currentForce[i] = 0.0;
	}

	waitStateChange = 100;

	springStiffness_stiff = 5000.;
	springDamping_stiff = 10; //2*sqrt(inertia * springStiffness); //10. // Critical damping
	springStiffness_smooth = 100.;
	springDamping_smooth = 0.7; 
	springDeadband = 0.;
	for (int i = 0; i<3; i++)
	{
		springPosition[i] = 0.;
		springDirection_X[i] = 0.;
		springDirection_Y[i] = 0.;
		springDirection_Z[i] = 0.;
	}
	springPosition[posZ] = floorHeight;
	springDirection_X[posX] = 1.0;
	springDirection_Y[posY] = 1.0;
	springDirection_Z[posZ] = 1.0;

	dampingCoef[0] = 0.;
	dampingCoef[1] = 30.; // TODO may need adjustments (should it be given as a parameter?) 
	dampingCoef[2] = 0.;

	maxAllowedForceInSpring = 400.0; // Otherwise very slow (it seems hig as a value but the motion of the HM is still quite slow)
	maxAllowedFeedbackForce = 100.0;

}

Haptic::~Haptic()
{	
	int returnValue = haDeviceClose (hapticMaster);
	if (returnValue == HARET_ERROR)
		std::cout << "ERROR unable to close Haptic Master" << std::endl;
}

double* Haptic::GetCurrentPosition()
{
	return currentPosition;
}

double* Haptic::GetCurrentVelocity()
{
	return currentVelocity;
}

double* Haptic::GetCurrentAcceleration()
{
	return currentAcceleration;
}

double* Haptic::GetCurrentForce()
{
	return currentForce;
}

void Haptic::UpdateForcePositionVelocityAcceleration() //TODO check error in returned string
{
	char res[400]; // TODO verifier que c'est assez long pour la reponse concatenee
	char str_pos[100], str_vel[100], str_acc[100], str_force[100];
	
	if(haSendCommand(hapticMaster, "get modelpos; get modelvel; get modelacc; get measforce", res) || strstr(res, ERROR_MSG))
		std::cout << "Error on reading HM position/velocity/acceleration/force" << std::endl;
	else	
	{
		BreakResponse(str_pos, res, 1); // TODO verifier les indices si commence a 0 ou 1
		ParseFloatVec(str_pos, currentPosition[posX], currentPosition[posY], currentPosition[posZ]);

		BreakResponse(str_vel, res, 2);
		ParseFloatVec(str_vel, currentVelocity[posX], currentVelocity[posY], currentVelocity[posZ]);
		
		BreakResponse(str_acc, res, 3);
		ParseFloatVec(str_acc, currentAcceleration[posX], currentAcceleration[posY], currentAcceleration[posZ]);
		
		BreakResponse(str_force, res, 4);
		ParseFloatVec(str_force, currentForce[posX], currentForce[posY], currentForce[posZ]);
	}	
}

void Haptic::Terminate()
{
	char res[100];
	// Clean Up All The Haptic Object On The HapticMASTER Side
	if (haSendCommand(hapticMaster, "remove all", res) || strstr(res, ERROR_MSG) || haSendCommand(hapticMaster, "set state stop", res) || strstr(res, ERROR_MSG))
		std::cout << "ERROR on cleaning HM" << std::endl;
}

int Haptic::InitHapticMaster()
{
	char res[100];
	
	hapticMaster = haDeviceOpen(IPADDRESS);

   	if(hapticMaster == HARET_ERROR)
   	{
      std::cout << "ERROR unable to connect to Haptic Master" << std::endl;
      return HARET_ERROR;
   	}
	
	else
	{
		InitializeDevice(hapticMaster);
		if (haSendCommand(hapticMaster, "remove all", res) || strstr(res, ERROR_MSG))
		{
			std::cout << "ERROR unable to remove objects on initializing Haptic Master" << std::endl;	
			return HARET_ERROR;
		}	
		
		if (haSendCommand(hapticMaster, "set inertia", inertia, res) || strstr(res, ERROR_MSG))
		{
			std::cout << "ERROR can't set Haptic Master inertia" << std::endl;
			return HARET_ERROR;
		}
	}

	// Initialize all haptic objects	
 	if (haSendCommand(hapticMaster, "create spring spring_X", res) || strstr(res, ERROR_MSG))
 	{
 		std::cout << "ERROR on X spring creation" << std::endl;
 		return HARET_ERROR;
 	}
 	else
 	{
		if(	haSendCommand(hapticMaster, "set spring_X stiffness", springStiffness_smooth, res) || strstr(res, ERROR_MSG)||
			haSendCommand(hapticMaster, "set spring_X dampfactor", springDamping_smooth, res)	|| strstr(res, ERROR_MSG)||
			haSendCommand(hapticMaster, "set spring_X deadband", springDeadband, res)	|| strstr(res, ERROR_MSG)||
			haSendCommand(hapticMaster, "set spring_X direction", springDirection_X[posX], springDirection_X[posY], springDirection_X[posZ], res) || strstr(res, ERROR_MSG)||
			haSendCommand(hapticMaster, "set spring_X pos", springPosition[posX], springPosition[posY], springPosition[posZ], res) || strstr(res, ERROR_MSG)||
			haSendCommand(hapticMaster, "set spring_X enable", res) || strstr(res, ERROR_MSG))	// X and Z springs used to constrain motion in 1D so always enabled	
		{
	 		std::cout << "ERROR on spring_X initialization" << std::endl;
 			return HARET_ERROR;
		}
	}
	
 	if (haSendCommand(hapticMaster, "create spring spring_Y", res) || strstr(res, ERROR_MSG))
 	{
 		std::cout << "ERROR on Y spring creation" << std::endl;
 		return HARET_ERROR;
 	}
 	else
 	{
		if(	haSendCommand(hapticMaster, "set spring_Y stiffness", springStiffness_smooth, res) || strstr(res, ERROR_MSG)||
			haSendCommand(hapticMaster, "set spring_Y dampfactor", springDamping_smooth, res)  || strstr(res, ERROR_MSG)||
			haSendCommand(hapticMaster, "set spring_Y deadband", springDeadband, res)	|| strstr(res, ERROR_MSG)||
			haSendCommand(hapticMaster, "set spring_Y direction", springDirection_Y[posX], springDirection_Y[posY], springDirection_Y[posZ], res) || strstr(res, ERROR_MSG)||
			haSendCommand(hapticMaster, "set spring_Y pos", springPosition[posX], springPosition[posY], springPosition[posZ], res) || strstr(res, ERROR_MSG) ||
			haSendCommand(hapticMaster, "set spring_Y maxforce", maxAllowedForceInSpring, res) || strstr(res, ERROR_MSG) ||
			haSendCommand(hapticMaster, "set spring_Y disable", res) || strstr(res, ERROR_MSG))	// Y Spring is used only to move HM back to start position so not enabled at first
		{
	 		std::cout << "ERROR on spring_Y initialization" << std::endl;
 			return HARET_ERROR;
		}
	}
	
 	if (haSendCommand(hapticMaster, "create spring spring_Z", res) || strstr(res, ERROR_MSG)) 
 	{
 		std::cout << "ERROR on Z spring creation" << std::endl;
 		return HARET_ERROR;
 	}
 	else
 	{
		if(	haSendCommand(hapticMaster, "set spring_Z stiffness", springStiffness_smooth, res) || strstr(res, ERROR_MSG) ||
			haSendCommand(hapticMaster, "set spring_Z dampfactor", springDamping_smooth, res)	|| strstr(res, ERROR_MSG) ||
			haSendCommand(hapticMaster, "set spring_Z deadband", springDeadband, res)	|| strstr(res, ERROR_MSG) ||
			haSendCommand(hapticMaster, "set spring_Z direction", springDirection_Z[posX], springDirection_Z[posY], springDirection_Z[posZ], res) || strstr(res, ERROR_MSG) ||
			haSendCommand(hapticMaster, "set spring_Z pos", springPosition[posX], springPosition[posY], springPosition[posZ], res) || strstr(res, ERROR_MSG) ||
			haSendCommand(hapticMaster, "set spring_Z enable", res) || strstr(res, ERROR_MSG))	// X and Z springs used to constrain motion in 1D so always enabled
		{
	 		std::cout << "ERROR on spring_Z initialization" << std::endl;
 			return HARET_ERROR;
		}	
	}
	
 	if (haSendCommand(hapticMaster, "create damper damper_Y", res) || strstr(res, ERROR_MSG)) 
 	{
 		std::cout << "ERROR on damper creation" << std::endl;
 		return HARET_ERROR;
 	}
	
 	else
 	{
		if(haSendCommand(hapticMaster, "set damper_Y dampcoef", dampingCoef[posX], dampingCoef[posY], dampingCoef[posZ], res) || strstr(res, ERROR_MSG) || haSendCommand(hapticMaster, "set damper_Y disable", res) || strstr(res, ERROR_MSG))
		{
	 		std::cout << "ERROR on damper_Y initialization" << std::endl;
 			return HARET_ERROR;
		}	
	}

	// Initialize forces at zero
	double force[3];
	for (int i=0; i<3; i++)
	{
		force[i] = 0.;
	}
	
 	if (haSendCommand(hapticMaster, "create biasforce ballForce", res) || strstr(res, ERROR_MSG)) 
 	{
 		std::cout << "ERROR on ballForce creation" << std::endl;
 		return HARET_ERROR;
 	}
 	else
 	{
		if(haSendCommand(hapticMaster, "set ballForce force", force[posX], force[posY], force[posZ], res) || strstr(res, ERROR_MSG) || haSendCommand(hapticMaster, "set ballForce disable", res) || strstr(res, ERROR_MSG))
		{
	 		std::cout << "ERROR on ballForce initialization" << std::endl;
 			return HARET_ERROR;
		}
	}

	return 0;
	
}


void Haptic::EnableDamper()
{
	char res[100];
	if (haSendCommand(hapticMaster, "set damper enable", res) || strstr(res, ERROR_MSG))
		std::cout << "ERROR on damper enabling" << std::endl;
}

void Haptic::DisableDamper()
{
	char res[100];
	if (haSendCommand(hapticMaster, "set damper disable", res) || strstr(res, ERROR_MSG))
		std::cout << "ERROR on damper disabling" << std::endl;
}

void Haptic::EnableStartPositionSpring()
{
	char res[100];
	if (haSendCommand(hapticMaster, "set spring_Y enable", res) || strstr(res, ERROR_MSG))
		std::cout << "ERROR on spring enabling" << std::endl;
}

void Haptic::DisableStartPositionSpring()
{
	char res[100];
	if (haSendCommand(hapticMaster, "set spring_Y disable", res) || strstr(res, ERROR_MSG))
		std::cout << "ERROR on spring disabling" << std::endl;
}


void Haptic::UpdateStartPositionSpring(double referencePosition[3]) 
{
	char res[100];
	if(haSendCommand(hapticMaster, "set spring_Y pos", referencePosition[posX], referencePosition[posY], referencePosition[posZ], res) || strstr(res, ERROR_MSG))
		std::cout << "ERROR on spring_Y position updating" << std::endl;
}

void Haptic::EnableRestrict1DMotion()
{
	char res[100];
	if (haSendCommand(hapticMaster, "set spring_X stiffness", springStiffness_stiff, res) || strstr(res, ERROR_MSG) ||
		haSendCommand(hapticMaster, "set spring_X dampfactor", springDamping_stiff, res)	|| strstr(res, ERROR_MSG))
		std::cout << "ERROR on spring_X stiffening" << std::endl;

	if (haSendCommand(hapticMaster, "set spring_Z stiffness", springStiffness_stiff, res) || strstr(res, ERROR_MSG) ||
		haSendCommand(hapticMaster, "set spring_Z dampfactor", springDamping_stiff, res)	|| strstr(res, ERROR_MSG))
		std::cout << "ERROR on spring_Z stiffening" << std::endl;
}

void Haptic::DisableRestrict1DMotion()
{
	char res[100];
	if (haSendCommand(hapticMaster, "set spring_X stiffness", springStiffness_smooth, res) || strstr(res, ERROR_MSG) ||
		haSendCommand(hapticMaster, "set spring_X dampfactor", springDamping_smooth, res)	|| strstr(res, ERROR_MSG))
		std::cout << "ERROR on spring_X smoothing" << std::endl;

	if (haSendCommand(hapticMaster, "set spring_Z stiffness", springStiffness_smooth, res) || strstr(res, ERROR_MSG) ||
		haSendCommand(hapticMaster, "set spring_Z dampfactor", springDamping_smooth, res)	|| strstr(res, ERROR_MSG))
		std::cout << "ERROR on spring_Z smoothing" << std::endl;
}

void Haptic::UpdateBallForce(double force[3])
{
	char res[100];
	// Check whether the force to apply is not to big, otherwise scale it
	double forceMagnitude  = sqrt(pow(force[0],2) + pow(force[1],2) + pow(force[2],2));
	if ( forceMagnitude > maxAllowedFeedbackForce)
	{
		for(int i=0; i<3; i++)
			force[i] = force[i] / forceMagnitude * maxAllowedFeedbackForce;
	}
	if(haSendCommand(hapticMaster, "set ballForce force", force[posX], force[posY], force[posZ], res) || strstr(res, ERROR_MSG))  // no time limit, the current force is valid until a new one is calculated
		std::cout << "ERROR on ballForce updating" << std::endl;
}


void Haptic::EnableBallForce()
{
	char res[100];
	// Before activating force, clear the old value force that could remain from a previous trial
	double force[3];
	for (int i=0; i<3; i++)
		force[i] = 0;
	if(haSendCommand(hapticMaster, "set ballForce force", force[posX], force[posY], force[posZ], res)  || strstr(res, ERROR_MSG) || haSendCommand(hapticMaster, "set ballForce enable", res) || strstr(res, ERROR_MSG))
		std::cout << "ERROR on ballForce enabling" << std::endl;
}

void Haptic::DisableBallForce()
{
	char res[100];
	if(haSendCommand(hapticMaster, "set ballForce disable", res) || strstr(res, ERROR_MSG))
		std::cout << "ERROR on ballForce disabling" << std::endl;
}


