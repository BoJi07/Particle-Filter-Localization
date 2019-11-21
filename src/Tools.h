#ifndef TOOLS_H
#define TOOLS_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>


using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::stringstream;
using std::string;
using std::getline;
using std::vector;



class Tools{

public:
    //constructor
    Tools(){}
    //destructor
    virtual ~Tools(){}
    // retrieve the data in 1 dimension
    vector<float> read1D(string path);
    // retrieve the data in 2 dimensions
    vector<vector<float>> read2D(string path);

    void print1D(const vector<float> &input);
    void print2D(const vector<vector<float>> &input);

};


#endif