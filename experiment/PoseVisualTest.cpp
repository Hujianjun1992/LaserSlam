#include "../include/CommonHeaders.h"
#include "../include/PoseVisual.h"

using namespace LaserSlam;

int main(int argc, char *argv[])
{

    ParameterReader para( "./parameter/PoseVisualTest.txt" );
    PoseVisual poseVisual( para );
    poseVisual.readPoseFile();
    //poseVisual.printPoseInfo();
    poseVisual.drawPath();
    return 0;
}
