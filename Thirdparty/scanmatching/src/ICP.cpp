/*
 * ICP.cpp
 *
 *  Created on: 12/giu/2010
 *      Author: Mladen Mazuran
 */

#include "ICP.h"
#include "../Config.h"
#include <cmath>

ICP::ICP(NearestNeighbours *nearestNeighbours):
        nearestNeighbours(nearestNeighbours) {
}

ICP::~ICP() {
    if(nearestNeighbours)
        delete nearestNeighbours;
}

void ICP::scanMatch(
        const Scan &refScan,
        const Scan &queryScan,
        Eigen::Vector3d &rototranslation,
        Eigen::Matrix3d &covariance,
        const Eigen::Vector3d &rototranslationGuess) {

    Scan rotatedScan;
    Rototranslation2D RTcompose(rototranslationGuess);

    unsigned int queryScanSize = queryScan.size();

    unsigned int queryToReferenceMapping[queryScanSize][2];
    double nearestDistances[queryScanSize];

    const int maxIterations = config::icp_max_iter;
    const double eps = config::icp_trim_ratio;
    const double convError = config::icp_conv_error;

    nearestNeighbours->initialize(refScan);

    //std::cout << "ListAnimate[{";

    for (int iteration = 0; iteration < maxIterations; iteration++) {
        Rototranslation2D RT;
        double distanceToTrim;
        int trimSize;
        Eigen::Vector3d q;

        /* 1 - Rototranslate the second scan using the current RT estimate              */
        rotatedScan = RTcompose * queryScan;

        /* 2 - Find for each (xxTi,yyTi) the corresponding segment                      */
        nearestNeighbours->getNearestNeighbours(
                rotatedScan, queryToReferenceMapping, nearestDistances);

        /* 3 - Use a trimming procedure (The Trimmed Iterative Closest Point Algorithm) */
        /* 3.1 - Order the distance in ascending order and select only eps*N pairs      */
        std::vector<double> distSorted(nearestDistances, nearestDistances + queryScanSize);
        std::sort (distSorted.begin(), distSorted.end());
        trimSize = (unsigned int) std::floor(eps * queryScanSize);
        distanceToTrim = distSorted[trimSize - 1];

        //std::cout << "Show[{Graphics[{Green";

        /* 3.2 - Assign the right pairs in order to do the correct minimization         */
        unsigned int queryIndices[trimSize];
        for (unsigned int j = 0, k = 0; k < queryScanSize; k++) {
            if (nearestDistances[k] <= distanceToTrim) {
                // TODO: WTF?
                // Remove the bias given by matching always the first or last point
                // if ((distNearest[k] < distanceToTrim) && (idxRef[k][0] != 0)
                //        && (idxRef[k][0] != static_cast<unsigned int>(sizeR-1))) {
                queryIndices[j++] = k;
                //std::cout << ",Line[{" << rotatedScan[k] << "," << refScan[queryToReferenceMapping[k][0]] << "}]";
            }
        }
        //std::cout << "}]";


        //std::cout << ",\nListPlot[{" << refScan << "," << rotatedScan <<
        //        "},AspectRatio->1,PlotStyle->{Red,Blue}]},PlotRange->{{-4,4},{-2,3}}]," << std::endl;


        /* 4 - Optimize, find the optimal transformation                                */
        q = this->getBestRototranslationVector(
                refScan, rotatedScan, queryToReferenceMapping, queryIndices, trimSize);

        /* 5 - Use compound to get the new RTCompose                                    */
        RT = Rototranslation2D(q).inverse();
        RTcompose = RT * RTcompose;

        //std::cout << "max(" << q[0] << "," << q[1] << "," << q[2] << ") < " << convError
        //        << "? " << (std::max(std::max(q[0], q[1]), q[2]) < convError) << std::endl;

        /* 6 - Termination condition                                                    */
        if(std::max(std::max(q[0], q[1]), q[2]) < convError) {
            break;
        }
    }

    //std::cout << "ListPlot[{" << refScan << "," << RTcompose * queryScan <<
    //        "},AspectRatio->1,PlotStyle->{Red,Blue},PlotRange->{{-4,4},{-2,3}}]}]" << std::endl;

    nearestNeighbours->deinitialize();
    rototranslation = RTcompose.getVectorForm();

    covariance = this->computeCovariance(
            refScan, queryScan, RTcompose, queryToReferenceMapping);
}
