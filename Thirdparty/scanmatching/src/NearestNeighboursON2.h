/*
 * NearestNeighboursON2.h
 *
 *  Created on: 17/giu/2010
 *      Author: Mladen Mazuran
 */

#ifndef NEARESTNEIGHBOURSON2_H_
#define NEARESTNEIGHBOURSON2_H_

#include "NearestNeighbours.h"

//! Nearest neighbours solver that uses a basic O(N^2) implementation
class NearestNeighboursON2: public NearestNeighbours {

public:
    //! Create a new nearest neighbours solver
    NearestNeighboursON2();
    virtual ~NearestNeighboursON2();

    virtual void initialize(const Scan &refScan);
    virtual void deinitialize();

    virtual void getNearestNeighbours(
            const Scan &queryScan,
            unsigned int queryToReferenceMapping[][2],
            double nearestDistances[]);
};

#endif /* NEARESTNEIGHBOURSON2_H_ */
