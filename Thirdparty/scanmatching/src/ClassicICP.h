/*
 * ClassicICP.h
 *
 *  Created on: 18/giu/2010
 *      Author: Mladen Mazuran
 */

#ifndef CLASSICICP_H_
#define CLASSICICP_H_

#include "ICP.h"
#include "NearestNeighboursANN.h"

//! Classic/Vanilla %ICP implementation
class ClassicICP: public ICP {
private:
    void doCentering(
            const Scan &sourceScan,
            const unsigned int indexes[],
            unsigned int indexCount,
            Eigen::Vector2d centered[],
            Eigen::Vector2d &centroid) const;

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
    //! Create a new classic ICP object
    /*!
     * \param nearestNeighbours nearest neighbours algorithm to use
     */
    ClassicICP(NearestNeighbours *nearestNeighbours = new NearestNeighboursANN());
    virtual ~ClassicICP();
};

#endif /* CLASSICICP_H_ */
