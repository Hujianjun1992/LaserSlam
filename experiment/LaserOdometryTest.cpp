#include "../include/LaserOdometry.h"
#include "../include/CommonHeaders.h"

using namespace LaserSlam;


int main(int argc, char *argv[])
{
    ParameterReader para( "./parameter/ParamLaserOdometryTest.txt" );
    LaserOdometry laserodometryTest( para );

    laserodometryTest.init();

    laserodometryTest.run();



    laserodometryTest.stop();

    return 0;
}
