/*
 * ScanMatcher.h
 *
 *  Created on: 12/giu/2010
 *      Author: Mladen Mazuran
 */

#ifndef SCANMATCHER_H_
#define SCANMATCHER_H_

#include <Eigen/Core>
#include "Scan.h"

class Scan;

//! Interface that defines a scan matcher
class ScanMatcher {
public:
    //! Matches two scans
    /*!
     * Calculates the optimal rototranslation (x, y, theta) from queryScan to refScan
     * and its estimated error covariance using an optional initial guess.
     *
     * \param refScan reference scan
     * \param queryScan query scan
     * \param rotoTranslation vector where the rototranslation is returned
     * \param covariance matrix where the error covariance is returned
     * \param rotoTranslationEstimate initial guess
     */
    virtual void scanMatch(
            const Scan &refScan,
            const Scan &queryScan,
            Eigen::Vector3d &rotoTranslation,
            Eigen::Matrix3d &covariance,
            const Eigen::Vector3d &rotoTranslationEstimate = Eigen::Vector3d(0, 0, 0)) = 0;

};

#endif /* SCANMATCHER_H_ */
