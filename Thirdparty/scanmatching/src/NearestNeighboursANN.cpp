/*
 * NearestNeighboursANN.cpp
 *
 *  Created on: 18/giu/2010
 *      Author: Mladen Mazuran
 */

#include "NearestNeighboursANN.h"

NearestNeighboursANN::NearestNeighboursANN():
        dataPoints(NULL), queryPoint(NULL), indexes(NULL), distances(NULL), kdTree(NULL), nPoints(0) {

    queryPoint = annAllocPt(DIMENSION);
}

NearestNeighboursANN::~NearestNeighboursANN() {
    this->deinitialize();
    annDeallocPt(queryPoint);
    annClose();
}

void NearestNeighboursANN::initialize(const Scan &refScan) {
    //NearestNeighbours::initialize(refScan);
    static const int k = 2;                                 // number of nearest neighbors

    nPoints     = static_cast<int>(refScan.size());
    dataPoints  = annAllocPts(nPoints, DIMENSION);          // allocate data points
    indexes     = new ANNidx[k];                            // allocate near neigh indices
    distances   = new ANNdist[k];                           // allocate near neighbor dists

    for (int i = 0; i < nPoints; i++) {
        dataPoints[i][0] = refScan[i].getX();
        dataPoints[i][1] = refScan[i].getY();
    }

    kdTree = new ANNkd_tree(                                // build search structure
            dataPoints,                                     // the data points
            nPoints,                                        // number of points
            DIMENSION);
}

void NearestNeighboursANN::deinitialize() {
    if(dataPoints) {
        annDeallocPts(dataPoints);
    }

    if(indexes) {
        delete[] indexes;
        indexes = NULL;
    }

    if(distances) {
        delete[] distances;
        distances = NULL;
    }

    if(kdTree) {
        delete kdTree;
        kdTree = NULL;
    }
}

void NearestNeighboursANN::getNearestNeighbours(
        const Scan &queryScan,
        unsigned int queryToReferenceMapping[][2],
        double nearestDistances[]) {

    static const int k = 2;                                 // number of nearest neighbors
    static const double eps = 0;                            // error bound

    for (unsigned int i = 0; i < queryScan.size(); i++) {   // read query points
        queryPoint[0] = queryScan[i].getX();
        queryPoint[1] = queryScan[i].getY();

        kdTree->annkSearch(                                 // search
                queryPoint,                                 // query point
                k,                                          // number of near neighbors
                indexes,                                    // nearest neighbors (returned)
                distances,                                  // distance (returned)
                eps);                                       // error bound

        for (int j = 0; j < k; j++) {
            queryToReferenceMapping[i][j] = indexes[j];
        }

        nearestDistances[i] = distances[0];                 // take the first distance
    }

}
