

#include "Auxillary.h"

map<int, std::string> toGate;


Auxillary::Auxillary(){
    for(int box=1;box<10;box++){
        int j = 2 - ceil(box/3.0);
        int i = 2 - box - (j-1)*3;
        std::string directions = "";

        (i<0)? directions.insert(0, abs(i), 'L') : directions.insert(0, abs(i), 'R');

        (j>0)? directions.insert(0, abs(j), 'D') : directions.insert(0, abs(j), 'U');

        toGate.insert( { box, directions} );
    }
}

int Auxillary::checkRegex(string input) const{
    std::regex rx("The gate is in square number ([1-9])"); // Declare the regex with a raw string literal
    std::smatch m;
    int out = 0;
    while (regex_search(input, m, rx)) {
        out = stoi(m[1]);
        //std::cout << "Number found: " << out << std::endl; // Get Captured Group 1 text
        input = m.suffix().str(); // Proceed to the next match
    } 
    return out;      
}

std::string Auxillary::getPath(int gateBox) const{
    //cout << toGate.find(gateBox)->second << endl;
    return toGate.find(gateBox)->second;
}

