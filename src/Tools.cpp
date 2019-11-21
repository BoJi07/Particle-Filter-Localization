#include "Tools.h"


vector<float> Tools::read1D(string path){
    ifstream input;
    float val;
    vector<float> data;
    // check the file avaiability
    input.open(path);
    if(!input){
        cerr<<"can't open the file !";
        exit(1);
    }

    while(input>>val){
        data.push_back(val);
    }   
    input.close();

    return data;
}


vector<vector<float>> Tools::read2D(string path){
    ifstream input;
    float val;
    string line;
    vector<vector<float>> data;
    // check the file avaiability
    input.open(path);
    if(!input){
        cerr<<"can't open the file!";
        exit(1);
    }

    while(getline(input,line)){
        stringstream ss(line);
        vector<float> temp;
        while(ss>>val){
            temp.push_back(val);
        }
        data.push_back(temp);
    }
    input.close();
    return data;
}



void Tools::print1D(const vector<float> &input){
    for(const float &data : input){
        cout<<data<<endl;
    }
}

void Tools::print2D(const vector<vector<float>> &input){
    for(auto &row : input){
        for(const float data: row){
           cout<<data<<" ";
        }
    }
    cout<<endl;
}

