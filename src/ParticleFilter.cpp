#include "ParticleFilter.h"
//defaul paramter for the particle filter
ParticleFilter::ParticleFilter(){
    num = 100;
    state = {0.0,0.0,0.0};
    var_a = 0.1;
    var_b = 0.1;
    var_range = 0.1;
    var_bearing = 0.1;
    generateParticle();
    weights = vector<float>(num,1);
}
// user input paramters for the particle filter
ParticleFilter::ParticleFilter(vector<float> &init, vector<float> &uncertainty, int N){
    num = N;
    state = init;
    var_a = uncertainty[0];
    var_b = uncertainty[1];
    var_range = uncertainty[2];
    var_bearing = uncertainty[3];
    generateParticle();
    weights = vector<float>(num,1);
}
// set the landmark on the map
void ParticleFilter::setReference(const vector<vector<float>> &ref){
    landMark = ref;
}
// state transisition of each particle
void ParticleFilter::predictParticle(float dt, vector<float> &input, vector<float> &s){
    float posx = s[0];
    float posy = s[1];
    float yaw = s[2];

    //motion update
    posx = posx + dt * cos(yaw) * (input[0]+normalDistribution(0.0,var_a));
    posy = posy + dt * sin(yaw) * (input[0]+normalDistribution(0.0,var_a));
    yaw = yaw + dt * (input[1]+normalDistribution(0.0,var_b));
    //keep heading between -pi to pi
    yaw = wrapToPi(yaw);

    s[0] = posx;
    s[1] = posy;
    s[2] = yaw;
}

void ParticleFilter::prediction(float dt, vector<float> &input){
    //make prediction of the robot state
    predictParticle(dt,input,this->state);
    for(auto &particle: particles){
        //make prediction of each particle
        predictParticle(dt,input,particle);
    }

}

//for each particle compute the likehood as the weights based on the measuremnt and measument model
void ParticleFilter::update(vector<float>& input, vector<float> &ref, float d){
    for(int i = 0; i<this->particles.size(); i++){
        float posx = this->particles[i][0];
        float posy = this->particles[i][1];
        float yaw =  this->particles[i][2];
        //the measurement model
        float dx = ref[0] - posx - d*cos(yaw);
        float dy = ref[1] - posy - d*sin(yaw);
        float range = sqrt(dx*dx+dy*dy) + normalDistribution(0.0,var_range);
        float bearing = atan2(dy,dx)-yaw + normalDistribution(0.0,var_bearing);
        //the likehood computations
        bearing = wrapToPi(bearing);
        weights[i] *= gaussFunction(range,input[0],var_range);
        weights[i] *= gaussFunction(bearing,input[1],var_bearing);
    
    }

    //normalized weights
    float total = accumulate(weights.begin(),weights.end(),0.0);
    for(int i = 0; i<weights.size(); i++){
        weights[i]/=total;
    }
}

// systermatically resample the particles 
void ParticleFilter::resampling(){
    vector<vector<float>> new_particles;
    float beta = 0.0;
    int idx = rand()%num;
    //find the max weights first
    float maxWeight = weights[0];
    for(int i = 1; i<weights.size(); i++){
        maxWeight = max(maxWeight,weights[i]);
    }
    //resample wheel
    for(int i = 0; i<weights.size(); i++){
        beta += generateRandom()*2*maxWeight;
        while(beta>weights[idx]){
            beta -= weights[idx];
            idx = (idx+1)%num;
        }
        //save the selected particles
        new_particles.push_back(particles[idx]);
        
    }
    particles = new_particles;
    //set equal weight for each new particles
    weights = vector<float>(num,1.0);
}

//generate the initilized particles around the current locations
void ParticleFilter::generateParticle(){
    for(int i = 0; i<this->num; i++){
        vector<float> temp;
        for(int j = 0; j<this->state.size(); j++){
            if(j<=1){
                float var = normalDistribution(0.0,this->var_a);
                temp.push_back(var+this->state[j]);
            }
            else{
                float var = normalDistribution(0.0,this->var_b);
                temp.push_back(var+this->state[j]);
            }
        }
        particles.push_back(temp);
    } 
}

// gaussian normal distribution function
float ParticleFilter::normalDistribution(float mean, float variance){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<float> distribution(mean,variance);
    return distribution(generator);
}

//guassion function used to compute the likehoods
float ParticleFilter::gaussFunction(float x, float mean, float variance){
    float p = 1.0/sqrt(2 * variance * variance * M_PI)*exp(-(x-mean)*(x-mean)/(2*variance*variance));
    return p;
}
//keep the heading angle to pi and -pi
float ParticleFilter::wrapToPi(float x){
    while(x>M_PI){
        x -= 2*M_PI;
    }
    while(x<-M_PI){
        x += 2*M_PI;
    }
    return x;
}
//uniform distribution function 
float ParticleFilter::generateRandom(){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    return distribution(generator);
}

vector<float> ParticleFilter::computeMean(){
    vector<float> mean = {0.0,0.0,0.0};
    for(int i = 0; i<particles.size();i++){

        mean[0] += particles[i][0];
        mean[1] += particles[i][1];
        mean[2] += particles[i][2];
    }

    mean[0]/=(float)num;
    mean[1]/=(float)num;
    mean[2]/=(float)num;
    return mean;
}

vector<float> ParticleFilter::computeVariance(vector<float> &mean){
    vector<float> variance = {0.0,0.0,0.0};
    for(int i = 0; i<particles.size(); i++){
        variance[0] += (particles[i][0]-mean[0])*(particles[i][0]-mean[0]);
        variance[1] += (particles[i][1]-mean[1])*(particles[i][1]-mean[1]);
        variance[2] += (particles[i][2]-mean[2])*(particles[i][2]-mean[2]);
    }
    variance[0]/=(float)num;
    variance[1]/=(float)num;
    variance[2]/=(float)num;
    return variance;
}

