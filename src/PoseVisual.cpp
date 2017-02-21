#include "../include/CommonHeaders.h"
#include "../include/PoseVisual.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace LaserSlam;

PoseVisual::PoseVisual( ParameterReader& para ) :
    xStart( 0 ), yStart( 0 ), xEnd( 0 ), yEnd( 0 ),
    xPose( 0 ), yPose( 0 ), threaPose( 0 ), nextPoseData( 0 ) {

        poseFileDir = para.GetData<string>( "poseFileDir" );
}

PoseVisual::~PoseVisual() {

}

void PoseVisual::drawPath() {

    int currentIdx = 0;

    cv::Mat pathPicture;
    pathPicture.create(800, 800, CV_8UC3);

    //cout << "nextPoseData : " <<  nextPoseData << endl;
    while (currentIdx < nextPoseData - 2 ) {
        xStart = pathPicture.cols/2 + robotPoseV[ currentIdx ].x * 5;
        yStart = pathPicture.rows/2 + robotPoseV[ currentIdx ].y * 5;

        xEnd = pathPicture.cols/2 + robotPoseV[ currentIdx + 1 ].x * 5;
        yEnd = pathPicture.rows/2 + robotPoseV[ currentIdx + 1 ].y * 5;

        if ( xStart > 0 && yStart > 0 && xEnd < pathPicture.cols && yEnd < pathPicture.cols ) {
            cv::line(pathPicture, cv::Point(xStart,yStart), cv::Point(xEnd,yEnd), CV_RGB(0, 255, 0), 1, 8, 0);
        }
        cv::imshow("RobotPose", pathPicture);
        if ( cv::waitKey(10) == 'q' )
        {
            break;
        }
        cout << currentIdx << endl;
        currentIdx ++;
    }

    cv::imwrite( "./picture/RobotPose.png", pathPicture );


}

void PoseVisual::readPoseFile() {

    string dataType;
    ifstream poseFile( poseFileDir.c_str() );

    while ( !poseFile.eof() ) {

        robotPoseV.push_back( RobotPose() );
        //cout << nextPoseData << endl;
        poseFile >> dataType >> robotPoseV[nextPoseData].x >> robotPoseV[nextPoseData].y >> robotPoseV[nextPoseData].threa;
        nextPoseData ++;
    }

    std::cout << "一共有 " << nextPoseData << " pose"<< std::endl;
    poseFile.close();

}

void PoseVisual::printPoseInfo() {
    int i = 0;
    for ( auto iter = robotPoseV.begin(); iter != robotPoseV.end(); iter ++ ) {
        cout << i ++ << " : " << iter->x << ", " << iter->y << ", " << iter->threa << "." << endl;
    }
}


