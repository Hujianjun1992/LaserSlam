/*
 * ICP.h
 *
 *  Created on: 12/giu/2010
 *      Author: Mladen Mazuran
 */

#ifndef ICP_H_
#define ICP_H_

#include "ScanMatcher.h"
#include "Rototranslation2D.h"
#include "NearestNeighbours.h"

//! %ICP Common implementation
/*!
 * This abstract class implements the main loop and the common aspects of the
 * Iterative Closest Point algorithm. For a full working ICP all the five
 * protected pure virtual methods must be implemented by subclassing this class.
 */
class ICP: public ScanMatcher {
private:
    NearestNeighbours *nearestNeighbours;

protected:
    //! Calculates the rototranslation that minimizes the distance function with the given associations
    /*!
     * The distance function is related to the actual implementation of the %ICP
     * \param refScan reference scan
     * \param rotatedScan query scan (rotated by the icp main loop)
     * \param queryToReferenceMapping point associations returned by the nearest neighbours algorithm
     * \param queryIndexes indexes of the not trimmed points
     * \param queryIntexCount length of the queryIndexes array
     * \return the best rototranslation in vector form
     */
    virtual Eigen::Vector3d getBestRototranslationVector(
            const Scan &refScan,
            const Scan &rotatedScan,
            const unsigned int queryToReferenceMapping[][2],
            const unsigned int queryIndexes[],
            unsigned int queryIndexCount) = 0;

    //! Computes the covariance with the given associations
    /*!
     * \param refScan reference scan
     * \param queryScan query scan
     * \param rototranslation rototranslation calculated by the main loop
     * \param queryToReferenceMapping point associations returned by the nearest neighbours algorithm
     */
    virtual Eigen::Matrix3d computeCovariance(
            const Scan &refScan,
            const Scan &queryScan,
            const Rototranslation2D &rototranslation,
            const unsigned int queryToReferenceMapping[][2]) = 0;

public:
    //! Create a new %ICP object
    /*!
     * \param nearestNeighbours nearest neighbours algorithm to use
     */
    ICP(NearestNeighbours *nearestNeighbours);
    virtual ~ICP();

    virtual void scanMatch(
            const Scan &refScan,
            const Scan &queryScan,
            Eigen::Vector3d &rototranslation,
            Eigen::Matrix3d &covariance,
            const Eigen::Vector3d &rototranslationEstimate = Eigen::Vector3d(0, 0, 0));
};

#endif /* ICP_H_ */
