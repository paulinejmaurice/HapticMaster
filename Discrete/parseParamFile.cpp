#include "parseParamFile.h"

int parseParamFile(const std::string filename_input, std::string &filename_output, std::vector<std::pair<std::string, std::string> > &param_name_type, std::map<std::string, int> &param_map_int, std::map<std::string, bool> &param_map_bool, std::map<std::string, double> &param_map_double)
{

	const char *delimiter_comment = "%";
	const char *delimiter_equal = "=";
	std::string line, param_str, name, value, type;
	unsigned int index;
  	time_t rawtime = time(NULL);
	char time_str[80];
	
	std::ifstream file(filename_input.c_str());
  	if (file)
  	{
    	while (getline(file,line)) // read line by line
    	{
    		if (line.size()>0 && line[0] != *delimiter_comment) // check whether this is only a comment line
    		{
    			param_str = line.substr(0, line.find(delimiter_comment)); // if there is a comment afterwards, remove it
    			param_str.erase(remove(param_str.begin(), param_str.end(), ' '), param_str.end()); // remove the potential spaces before/after parameter name and value   			
    			name = param_str.substr(0, param_str.find(delimiter_equal));  // separate the name (before "=" sign) from the value (after "=" sign)
    			value = param_str.substr(param_str.find(delimiter_equal)+1,-1); 
    			
    			if (name == FILENAME_OUTPUT)
    			{
  					strftime(time_str,80,"%d%b%Y_%H-%M-%S", localtime(&rawtime)); // Add date and time when starting block to filename for output data, so that each file has a unique ID (even if the output_filnename parameter os not changed in the parameters file). All the trials in the same block have the same date and time, but they differ by the trial number which is added in the filename
					filename_output = value + "_" + time_str;
				}
    			else
    			{					
					index = 0;
					while(param_name_type[index].first != name && index < param_name_type.size())
						index ++;
					if (index != param_name_type.size()) // parameter was found in the list
					{	
						type = param_name_type[index].second;
						// Add param in the right map
						if (type == TYPE_INT)
							param_map_int.insert( std::pair<std::string, int>(name, std::atoi(value.c_str())));
						else if (type == TYPE_BOOL)
							param_map_bool.insert( std::pair<std::string, bool>(name, value=="1"));
						else if (type == TYPE_DOUBLE)	
							param_map_double.insert( std::pair<std::string, double>(name, std::atof(value.c_str())));
						// remove the paramater from the vector of parameter that should be read in the file
						param_name_type.erase(param_name_type.begin() + index);
					}
				}
			}

    	}
	 	file.close();
  	}
  	else
  	{
  		std::cout << "Problem opening parameters file" << std::endl;
  		return -1;
  	}
  	// Check whether all the parameters are given a value  	
  	if (!param_name_type.empty()) // at least one parameter is missing
  	{
  		while (!param_name_type.empty())
  		{
  			std::cout << "Parameter missing: " << param_name_type[param_name_type.size()-1].first << std::endl;
  			param_name_type.pop_back();
  		} 
  		return -1;
	}
	return 0;
}


