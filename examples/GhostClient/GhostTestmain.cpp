#include "client.h"
#include <iostream>
using namespace std;
int main(int argc, char *argv[]){
    int counter =0;
    const char* robo_path = "/home/mamad/hand_RL_ws/bullet3/examples/GhostClient/kuka_box_model/model.sdf";
    connect();
    loadSDF(robo_path);
    while(1){
       stepSimulation();
       cout<<"counter::"<<counter<<endl;

    }

    return 0;
}
