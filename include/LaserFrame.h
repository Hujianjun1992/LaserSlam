#ifndef LASERFRAME_H_VNZZ1TKK
#define LASERFRAME_H_VNZZ1TKK

#include "CommonHeaders.h"
#include "ParameterReader.h"
namespace LaserSlam {

    struct LaserConfig {
        double min_angle;
        double max_angle;
        double ang_increment;
        double time_increment;
        double scan_time;
        double min_range;
        double max_range;
        double range_res;
    };

    class LaserScan {
        public:
            std::vector<double> ranges;
            std::vector<float> intensities;
            long long self_time_stamp;
            long long system_time_stamp;
            LaserConfig config;

            typedef shared_ptr<LaserScan> ptr;
    };

    class LaserFrame {

        public:
            typedef shared_ptr<LaserFrame> Ptr;

    };

    class FrameReader {

        public:
            FrameReader ( ParameterReader& para );
            ~FrameReader();
            void printData(int idx);

            LaserFrame::Ptr next();

            void reset();

        public:
            ifstream LaserDataFile;
            vector<LaserScan> laserscan;
            int CurrentIndex = 0;

        protected:
            void InitData( ParameterReader& para );
            int StartIndex = 0;
            string  LaserDataDir;
            //const ParameterReader& parameterReader;

    };
};

#endif /* end of include guard: LASERFRAME_H_VNZZ1TKK */
