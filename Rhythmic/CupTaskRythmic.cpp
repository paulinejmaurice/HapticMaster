#include "parseParamFile.h"
#include "display.h"

Display* pDisplay; // needs to be global because of GLUT functions


int main(int argc, char** argv)
{
	int mainLoopPeriod = 10; // (ms) The real timestep cannot go below 16ms (and the real timestep is used for model integration), so this parameter does not really matter (except if you set a very long timeStep, you will see it graphically)
	int mainLoopTimerID = 1;
	std::string output_filename = "Default"; // Name of file where results are written. Name is modified when reading parameters file
	
	// Containers for the parameters to read in a file
	const std::string param_filename = "param.txt";
	std::vector<std::pair<std::string, std::string>> param_name_type;
	std::map<std::string, int> param_map_int;
	std::map<std::string, bool> param_map_bool;
	std::map<std::string, double> param_map_double;
	
	// Fill in vectors with the parameters that should be read in the param file
	param_name_type.push_back(std::pair<std::string, std::string>("nbTrials", TYPE_INT));					// number of trials in one block	
	param_name_type.push_back(std::pair<std::string, std::string>("projector", TYPE_BOOL));					// choose between local display or projector screen
	param_name_type.push_back(std::pair<std::string, std::string>("sound", TYPE_BOOL));						// ahether a sound is played to indicate the start and end of each  trial
	param_name_type.push_back(std::pair<std::string, std::string>("autoStart", TYPE_BOOL));					// whether each trial starts automatically or starts when a motion of the HM end-effector is detected
	param_name_type.push_back(std::pair<std::string, std::string>("ballEscape", TYPE_BOOL));					// whether the ball can escape the cup or not	
	param_name_type.push_back(std::pair<std::string, std::string>("selfPaced", TYPE_BOOL));					// whether user chooses the frequency or imposed by metronome
	param_name_type.push_back(std::pair<std::string, std::string>("speedHint", TYPE_BOOL));					// whether a message telling you to go faster/slower or OK is displayed on the screen (can only be used when metronome paced)
	param_name_type.push_back(std::pair<std::string, std::string>("visualScalingFactor", TYPE_DOUBLE));		// ratio between the distances on screen and the physical distances for everything (cup, blocks, start to target distance)
	param_name_type.push_back(std::pair<std::string, std::string>("cupAdditionalVisualScalingFactor", TYPE_DOUBLE));		// additional scaling for the cup and block size. The full scaling is visualScalingFactor * cupAdditionalVisualScalingFactor
	param_name_type.push_back(std::pair<std::string, std::string>("durationOfOneTrial", TYPE_DOUBLE));		// (s)	
	param_name_type.push_back(std::pair<std::string, std::string>("floorHeight", TYPE_DOUBLE));				// (meters) height of the HM end-effector constrained by springs (can be positive or negative, reference is the HM frame)
	param_name_type.push_back(std::pair<std::string, std::string>("goalFrequencyOfOscillations", TYPE_DOUBLE));  	// (m)
	param_name_type.push_back(std::pair<std::string, std::string>("startToTargetDistance", TYPE_DOUBLE));  	// (m)
	param_name_type.push_back(std::pair<std::string, std::string>("accuracyFactor", TYPE_DOUBLE));			// (%) ratio between the size of the target and the size of the cup (must be >1). Target is reached when the cup stops entirely inside target block
	param_name_type.push_back(std::pair<std::string, std::string>("accelerationAmplification", TYPE_DOUBLE)); // amplify the acceleration of the cart in the model (compared to the ral acceleration of the HM)
	param_name_type.push_back(std::pair<std::string, std::string>("inertiaHM", TYPE_DOUBLE));				// of the HM (related to cart + pendulum mass)
	param_name_type.push_back(std::pair<std::string, std::string>("arcCup", TYPE_DOUBLE));					// (degres for simplicity) length of the arc representing the cup. The cup is symetric wrt verical axis. The shape of the cup is defined both by arcCup (portion of full circle) and by pendulumLength (curvature)
	param_name_type.push_back(std::pair<std::string, std::string>("pendulumMass", TYPE_DOUBLE));				// (kg)
	param_name_type.push_back(std::pair<std::string, std::string>("pendulumLength", TYPE_DOUBLE));			// (m)
	param_name_type.push_back(std::pair<std::string, std::string>("pendulumDamping", TYPE_DOUBLE));			// (N.m.s)
	param_name_type.push_back(std::pair<std::string, std::string>("pendulumInitialAngle", TYPE_DOUBLE));		// (degree for simplicity)
	param_name_type.push_back(std::pair<std::string, std::string>("pendulumInitialVelocity", TYPE_DOUBLE));	// (degree/s)
	
	// Actually read in file (and check whether all the parameters are given a value otherwise abort)
	if(parseParamFile(param_filename, output_filename, param_name_type, param_map_int, param_map_bool, param_map_double) == -1)
		return -1;

	// Convert degrees to rad (easier for all trigonometry operations)
	param_map_double["arcCup"] *= M_PI / 180.; 
	param_map_double["pendulumInitialAngle"] *= M_PI / 180.;
	param_map_double["pendulumInitialVelocity"] *= M_PI / 180.;

	// Create pointer (hence "new" mandatory) to object which takes care of basically everything
	pDisplay = new Display(mainLoopPeriod, mainLoopTimerID, output_filename, 
							param_map_int["nbTrials"], 
							param_map_double["durationOfOneTrial"], 
							param_map_double["goalFrequencyOfOscillations"], 
							param_map_double["floorHeight"], 
							param_map_double["startToTargetDistance"],
							param_map_double["arcCup"], 
							param_map_double["pendulumLength"], 
							param_map_double["pendulumMass"], 
							param_map_double["pendulumDamping"], 
							param_map_double["pendulumInitialAngle"], 
							param_map_double["pendulumInitialVelocity"], 
							param_map_double["inertiaHM"], 
							param_map_double["accuracyFactor"], 
							param_map_double["accelerationAmplification"], 
							param_map_bool["selfPaced"], 
							param_map_bool["ballEscape"], 
							param_map_bool["autoStart"],  
							param_map_double["visualScalingFactor"], 
							param_map_double["cupAdditionalVisualScalingFactor"], 
							param_map_bool["projector"],
							param_map_bool["sound"],
							param_map_bool["speedHint"]);

	// Initialize HM and visual 	
	if (pDisplay->Initialize(argc, argv) != 0) // if HM initialization fails
	{
		// Clean before leaving
		delete pDisplay;
		return -1;
	}

	// Launch main loop
	pDisplay->LaunchLoop();

	// Ending
	delete pDisplay;
	return 0;
	
}
