/*
 * NearestNeighboursANN.h
 *
 *  Created on: 18/giu/2010
 *      Author: Mladen Mazuran
 */

#ifndef NEARESTNEIGHBOURSANN_H_
#define NEARESTNEIGHBOURSANN_H_

#include "NearestNeighbours.h"
#include "ANN/include/ANN/ANN.h"
#include <iostream>

//! Nearest neighbours solver that uses the ANN library
class NearestNeighboursANN: public NearestNeighbours {
private:
    static const int DIMENSION = 2;
    ANNpointArray      dataPoints;    // data points
    ANNpoint           queryPoint;    // query point
    ANNidxArray        indexes;       // near neighbor indices
    ANNdistArray       distances;     // near neighbor distances
    ANNkd_tree*        kdTree;
    int                nPoints;

public:
    //! Create a new nearest neighbours solver
    NearestNeighboursANN();
    virtual ~NearestNeighboursANN();

    virtual void initialize(const Scan &refScan);
    virtual void deinitialize();

    virtual void getNearestNeighbours(
            const Scan &queryScan,
            unsigned int queryToReferenceMapping[][2],
            double nearestDistances[]);
};

#endif /* NEARESTNEIGHBOURSANN_H_ */
