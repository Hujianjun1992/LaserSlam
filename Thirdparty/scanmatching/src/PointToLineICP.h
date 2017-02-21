/*
 * PointToLineICP.h
 *
 *  Created on: 12/giu/2010
 *      Author: Mladen Mazuran
 */

#ifndef POINTTOLINEICP_H_
#define POINTTOLINEICP_H_

#include "ICP.h"
#include "NearestNeighboursANN.h"

//! %ICP that implements Censi's point to line metric
class PointToLineICP: public ICP {
protected:
    Eigen::Vector3d getBestRototranslationVector(
            const Scan &refScan,
            const Scan &rotatedScan,
            const unsigned int queryToReferenceMapping[][2],
            const unsigned int queryIndices[],
            unsigned int queryIndexCount);
    Eigen::Matrix3d computeCovariance(
            const Scan &refScan,
            const Scan &queryScan,
            const Rototranslation2D &rototranslation,
            const unsigned int queryToReferenceMapping[][2]);

public:
    //! Create a new %PointToLineICP object
    /*!
     * \param sigma2 the multiplicative coefficient in the Censi polar covariance
     * \param nearestNeighbours nearest neighbours solver
     */
    PointToLineICP(NearestNeighbours *nearestNeighbours = new NearestNeighboursANN());
    virtual ~PointToLineICP();

    static Eigen::Matrix3d computeCensiCovariance(
            const Scan &refScan,
            const Scan &queryScan,
            const Rototranslation2D &rototranslation,
            const unsigned int queryToReferenceMapping[][2]);

};

#endif /* POINTTOLINEICP_H_ */
