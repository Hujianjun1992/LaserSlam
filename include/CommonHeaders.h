#ifndef COMMONHEADERS_H_4YH62MVR
#define COMMONHEADERS_H_4YH62MVR

#include <iostream>
#include <string>
#include <map>
#include <string>
#include <vector>
#include <fstream>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/timer.hpp>
#include <boost/lexical_cast.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../Thirdparty/scanmatching/src/ClassicICP.h"
#include "../Thirdparty/scanmatching/src/ICP.h"
#include "../Thirdparty/scanmatching/src/MetricICP.h"
#include "../Thirdparty/scanmatching/src/MetricICP.h"
#include "../Thirdparty/scanmatching/src/MobileAverage.h"
#include "../Thirdparty/scanmatching/src/NearestNeighbours.h"
#include "../Thirdparty/scanmatching/src/NearestNeighboursANN.h"
#include "../Thirdparty/scanmatching/src/NearestNeighboursON2.h"
#include "../Thirdparty/scanmatching/src/Point.h"
#include "../Thirdparty/scanmatching/src/PointToLineICP.h"
#include "../Thirdparty/scanmatching/src/Rototranslation2D.h"
#include "../Thirdparty/scanmatching/src/Scan.h"
#include "../Thirdparty/scanmatching/src/ScanMatcher.h"
#include "../Thirdparty/scanmatching/src/TopValues.h"

#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

#endif /* end of include guard: COMMONHEADERS_H_4YH62MVR */
