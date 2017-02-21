#ifndef CONFIG_H_PSRIXS1C
#define CONFIG_H_PSRIXS1C

#include <string>

namespace Config {

    enum NNEngineType {
        ON2, ANN
    };

    enum ICPAlgroithmType {
        Classic, Metric, PointToLine
    };

    extern double IcpMetricL;
    extern double IcpTrimRatio;
    extern double IcpConvError;
    extern int IcpMaxIter;

    extern NNEngineType NnEngine;
    extern ICPAlgroithmType IcpAlgorithm;

};

#endif /* end of include guard: CONFIG_H_PSRIXS1C */
