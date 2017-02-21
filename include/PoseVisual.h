#ifndef POSEVISUAL_H_KDQ3XUDY
#define POSEVISUAL_H_KDQ3XUDY

#include "CommonHeaders.h"
#include "ParameterReader.h"

namespace LaserSlam {

    struct RobotPose {
        double x;
        double y;
        double threa;
    };
    class PoseVisual
    {
    public:
        PoseVisual ( ParameterReader& para );
        virtual ~PoseVisual ();
        void drawPath();
        void printPoseInfo();
        void readPoseFile();

    private:
        float xPose, yPose, threaPose;
        int xStart, yStart;
        int xEnd, yEnd;
        int xLoc, yLocl;
        int nextPoseData;
        string poseFileDir;
        std::vector<RobotPose> robotPoseV;

    };


}

#endif /* end of include guard: POSEVISUAL_H_KDQ3XUDY */
