#ifndef LASERODEMETRYNODE_H_OXVC3YOI
#define LASERODEMETRYNODE_H_OXVC3YOI

#include "CommonHeaders.h"
#include "Conversions.h"
#include "ParameterReader.h"
#include "Equations.h"

#define LaserOdometry_Debug

namespace LaserSlam {

    enum NNEngineType { ON2, ANN };
    enum ICPAlgroithmType { Classic, Metric, PointToLine };

    struct LaserOdometryConfig  {
        double IcpMetricL;
        double IcpTrimRatio;
        double IcpConvError;
        int IcpMaxIter;
        NNEngineType NnEngine;
        ICPAlgroithmType IcpAlgorithm;
    };

    class LaserOdometry {

    private:
        Scan *LastScan;
        ICP *icp;
        long counter;
        Eigen::Vector3d currentPose;
        Eigen::Matrix3d currentCovariance;
        LaserOdometryConfig laserOdometryConfig;

    public:
        LaserOdometry( ParameterReader& para );
        virtual ~LaserOdometry();
        void init();
        void loadConfig( const ParameterReader& para );
        void printConfing();
        void run();
        void stop();
        FrameReader frameReader;

    private:
        void setupICP();
        int dataCnt;

    protected :
        const ParameterReader parameterReader;

    };


};

#endif /* end of include guard: LASERODEMETRYNODE_H_OXVC3YOI */
