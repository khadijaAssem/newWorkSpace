#pragma once

#include <regex>
#include <math.h>
#include <map> 

using namespace std;

class Auxillary {

public:
	Auxillary();
    int checkRegex(string input) const;
    std::string getPath(int gateBox) const;

};

