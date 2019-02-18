#ifndef PARSEPARAMFILE_H_INCLUDED
#define PARSEPARAMFILE_H_INCLUDED

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <map>
#include <vector>
#include <ctime>

#define TYPE_INT "int"
#define TYPE_BOOL "bool"
#define TYPE_DOUBLE "double"
#define FILENAME_OUTPUT "outputFilename"

int parseParamFile(const std::string filename_input, std::string &filename_output, std::vector<std::pair<std::string, std::string> > &param_name_type, std::map<std::string, int> &param_map_int, std::map<std::string, bool> &param_map_bool, std::map<std::string, double> &param_map_double);


#endif // PARSEPARAMFILE_H_INCLUDED
