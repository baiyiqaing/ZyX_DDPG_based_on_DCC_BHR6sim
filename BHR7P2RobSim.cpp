#include "SimulationLib/CRemotelib/remote_api.h"
#include "SimulationLib/Mujocolib/MujocoBHR_api.h"

int main() {

printf("Select simulation platform: (\"m\" for Mujoco, \"v\" for coppelias)\n");
char cPlatformFlag = getchar();
int nPlatformSelect = 0; // default

if(cPlatformFlag == 'M' || cPlatformFlag == 'm') nPlatformSelect = 0;
else if(cPlatformFlag == 'V' || cPlatformFlag == 'v') nPlatformSelect = 1;

if(nPlatformSelect == 0) {
    // choose your model
    fnvMujocoSimuInit(1, "../SimulationLib/Mujocolib/models/bhr7p2.xml"); 

    // start simulation in a background thread
    std::thread MujocoSimThread(fnvMujocoSimuLoop); 

    // render loop in the current thread
    while (!glfwWindowShouldClose(MJwindow) && !settings.exitrequest) fnvMujocoRenderLoop();

    // end simulation
    fnvMujocoSimuEnd();
    MujocoSimThread.join();
}
else if(nPlatformSelect == 1) {
    nVrepSimUpdate();
}
else printf("Wrong platform!!\n");
    return 0;
}

