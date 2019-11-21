#include "Tools.h"
#include "ParticleFilter.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main(){
    //path information
    string path_time = "../data/time_stamp.txt";
    string path_init = "../data/init_pose.txt";
    string path_range = "../data/range.txt";
    string path_bearing = "../data/bearing.txt";
    string path_vel = "../data/velocity.txt";
    string path_rot = "../data/rotation_speed.txt";
    string path_landmark = "../data/landmark.txt";
    string path_dist = "../data/dist.txt"; 
    Tools tool;

    vector<float> time = tool.read1D(path_time);
    vector<float> initPos = tool.read1D(path_init);
    vector<vector<float>> range = tool.read2D(path_range);
    vector<vector<float>> bearing = tool.read2D(path_bearing);
    vector<float> velocity = tool.read1D(path_vel);
    vector<float> rot = tool.read1D(path_rot);
    vector<vector<float>> ref = tool.read2D(path_landmark);
    vector<float> d = tool.read1D(path_dist);

    float var_a = 0.1;
    float var_w = 0.1;
    float var_bearing = 1;
    float var_range = 0.1;
    vector<float> uncertainty = {var_a,var_w,var_range,var_bearing};
    int num = 3000; //number of particle
    ParticleFilter solver(initPos,uncertainty,num);
    solver.setReference(ref);

    vector<float> estPosx;
    vector<float> estPosy;
    vector<float> estYaw;

    vector<vector<float>> estState(3,vector<float>(time.size()));
    estState[0][0] = initPos[0];
    estState[1][0] = initPos[1];
    estState[2][0] = initPos[2];
    vector<vector<float>> estCov(3,vector<float>(time.size()));

    for(int i = 1 ; i<time.size(); i++){
        float dt = time[i]-time[i-1];
        vector<float> input = {velocity[i-1],rot[i-1]};
        solver.prediction(dt,input);
        for(int j = 0; j<ref.size(); j++){
            vector<float> mea = {range[i][j],bearing[i][j]};
            solver.update(mea,ref[j],d[0]);
        }
        solver.resampling();

        vector<float> mean = solver.computeMean();
        estState[0][i] = mean[0];
        estState[1][i] = mean[1];
        estState[2][i] = mean[2];
        vector<float> var = solver.computeVariance(mean);

    }
    
    // plot the x vs y
    plt::figure_size(1200, 780);
    plt::plot(estState[0], estState[1]);
    plt::show();
    // plot the yaw angle
    plt::figure_size(1200, 780);
    plt::plot(estState[2]);
    plt::show();


}