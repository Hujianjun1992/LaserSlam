/*
 * NearestNeighbours.h
 *
 *  Created on: 17/giu/2010
 *      Author: Mladen Mazuran
 */

#ifndef NEARESTNEIGHBOURS_H_
#define NEARESTNEIGHBOURS_H_

#include "Scan.h"

//! Interface for solvers of the 2d "nearest neighbors" problem
class NearestNeighbours {
protected:
    //! Reference scan
    const Scan *refScan;

public:
    //! Initializes the internal data structures (if any) for use with the reference scan
    /*!
     * \param refScan reference scan
     */
    virtual void initialize(const Scan &refScan) { this->refScan = &refScan; }

    //! Calculates the nearest neighbours associations from the query to the reference scan
    /*!
     * This method returns for each point in the query scan the two nearest points of the
     * reference scan according to the internal distance function (which is usually euclidean,
     * with the exception of the metric case).
     * \param queryScan query scan
     * \param queryToReferenceMapping nearest associations
     * \param nearestDistances array of nearest distances
     */
    virtual void getNearestNeighbours(
            const Scan &queryScan,
            unsigned int queryToReferenceMapping[][2],
            double nearestDistances[]) = 0;

    //! Deinitializes the internal data structures (if any)
    virtual void deinitialize() = 0;
};

#endif /* NEARESTNEIGHBOURS_H_ */
