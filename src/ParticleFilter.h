#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>
#include <iostream>
#include <math.h>
#include <random>
#include <stdlib.h>
#include <numeric>
#include <algorithm>
#include <chrono>

using std::max;
using std::accumulate;
using std::vector;
using std::cout;
using std::endl;

class ParticleFilter{

public:
    ParticleFilter();

    ParticleFilter(vector<float> &init,vector<float> &uncertainty, int N);

    void setReference(const vector<vector<float>> &ref);

    void prediction(float dt, vector<float> &input);

    void update(vector<float> &input, vector<float> &ref, float d);

    void resampling();

    vector<float> computeMean();
    vector<float> computeVariance(vector<float> &mean);

    ~ParticleFilter(){}
private:
    int num; //number of particle
    vector<float> state; //state variable state[0]->posx state[1]->posy state[2]->bearing
    vector<vector<float>> particles; // all the particles
    vector<vector<float>> landMark;
    vector<float> weights;
    //unvertainty
    float var_a;
    float var_b;
    float var_range;
    float var_bearing;

    const float initPosGuess = 5;
    const float initYawGuess = 1;

    void generateParticle();
    void predictParticle(float dt, vector<float> &input, vector<float> &s);
    float wrapToPi(float x);
    float normalDistribution(float mean, float variance);
    float gaussFunction(float x,float mean, float varinace);
    float generateRandom(); //generate random value form 0-1
    

};



#endif