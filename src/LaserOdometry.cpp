#include "../include/LaserOdometry.h"
#include "../include/CommonHeaders.h"
#include "../include/Conversions.h"

using namespace LaserSlam;


LaserOdometry::LaserOdometry( ParameterReader& para ) :
    LastScan( nullptr ), icp( nullptr ), counter( 0 ), currentPose(),
    currentCovariance( Eigen::Matrix3d::Zero() ),parameterReader( para ),
    frameReader( para )  {
    }
LaserOdometry::~LaserOdometry() {

    if ( icp ) {
        delete icp;
    }

}


void LaserOdometry::init() {

   loadConfig( parameterReader );
   setupICP();
   printConfing();
   //std::cout << "i am here" << std::endl;
   //if ( LastScan == nullptr ) {
       //std::cout << "I am here! " << std::endl;
   //}
}


void LaserOdometry::run(void) {

    //LastScan = LaserToScan( &frameReader.laserscan[ counter ++ ]);
    while ( counter < frameReader.CurrentIndex ) {

        if ( LastScan == nullptr ) {
            //std::cout << "counter---->" << counter << std::endl;
            LastScan = LaserSlam::LaserToScan( frameReader.laserscan[ counter ++ ] );
            //frameReader.printData( counter - 1 );
            //std::cout << "counter---->" << counter << std::endl;
        } else {
            Eigen::Vector3d displacement;
            Eigen::Matrix3d covariance;
            Scan *newScan = LaserSlam::LaserToScan( frameReader.laserscan[ counter ++ ] );
            //frameReader.printData( counter - 1 );
            //std::cout << "counter---->" << counter << std::endl;
            //std::cout << *LastScan << std::endl;
            //std::cout << *newScan << std::endl;
            icp->scanMatch( *LastScan, *newScan, displacement, covariance );
            //std::cout << "displacement" << std::endl;
            //std::cout << displacement << std::endl;
            //std::cout << "covariance" << std::endl;
            //std::cout << covariance << std::endl;
            //std::cout << "counter---->" << counter << std::endl;
#ifdef LaserOdometry_Debug
            static std::ofstream *f = new std::ofstream( "./icp.txt", std::ios::out );
            *f << "last = " << *LastScan << ";" << endl;
            *f << "new = " << *newScan << ";" << endl;
            *f << "rotated = " << Rototranslation2D( displacement ) * ( *newScan ) << ";" << endl;
            *f << "roto = {" << displacement[ 0 ] << "," << displacement[ 1 ] << "," << displacement[ 2 ] << "};" << endl;
#endif
            Eigen::Vector3d newPose = LaserSlam::updatePose( currentPose, displacement );
#ifdef LaserOdometry_Debug
            static std::ofstream *ff = new std::ofstream( "./pose.txt", std::ios::out );
            *ff << "newPose"  << endl;
            *ff << newPose << endl;
#endif

            currentCovariance = LaserSlam::updatePoseCovariance( currentPose, currentCovariance, displacement, covariance );
            currentPose = newPose;

            delete LastScan;
            LastScan = newScan;
        }



    }


}


void LaserOdometry::stop() {

}

void LaserOdometry::loadConfig( const ParameterReader& para ) {

    //laserOdometryConfig.NnEngine = para.GetData<NNEngineType>( "NNEngineType" );
    //laserOdometryConfig.IcpAlgorithm = para.GetData<ICPAlgroithmType> ( "ICPAlgroithmType" );
    string engine, icpalgorithm;
    engine = para.GetData<string>( "NNEngineType" );
    icpalgorithm = para.GetData<string> ( "ICPAlgroithmType" );

    laserOdometryConfig.NnEngine = ANN;
    laserOdometryConfig.IcpAlgorithm = PointToLine;

    laserOdometryConfig.IcpMetricL = para.GetData<double>( "IcpMetricL" );
    laserOdometryConfig.IcpTrimRatio = para.GetData<double>( "IcpTrimRatio" );
    laserOdometryConfig.IcpConvError = para.GetData<double>( "IcpConvError" );
    laserOdometryConfig.IcpMaxIter = para.GetData<int>( "IcpMaxIter" );

}

void LaserOdometry::printConfing() {

        cout << "NNEngineType is " << laserOdometryConfig.NnEngine << endl;
        cout << "ICPAlgroithmType is " << laserOdometryConfig.IcpAlgorithm << endl;
        cout << "IcpMetricL is " << laserOdometryConfig.IcpMetricL << endl;
        cout << "IcpTrimRatio is " << laserOdometryConfig.IcpTrimRatio << endl;
        cout << "IcpConvError is " << laserOdometryConfig.IcpConvError << endl;
        cout << "IcpMaxIter is " << laserOdometryConfig.IcpMaxIter << endl;
}

void LaserOdometry::setupICP() {

    NearestNeighbours *nn = nullptr;

    switch ( laserOdometryConfig.NnEngine ) {
        case ON2:
            nn = new NearestNeighboursON2();
            break;
        case ANN:
            nn = new NearestNeighboursANN();
            std::cout << "NearestNeighboursANN()" << std::endl;
            break;
        default:
            break;
    }

    switch ( laserOdometryConfig.IcpAlgorithm ) {
        case Classic:
            icp = new ClassicICP(nn);
            break;
        case Metric:
            icp = new MetricICP(laserOdometryConfig.IcpMetricL);
            break;
        case PointToLine:
            icp = new PointToLineICP(nn);
            std::cout << "PointToLineICP()" << std::endl;
            break;
        default:
            break;
    }



}
