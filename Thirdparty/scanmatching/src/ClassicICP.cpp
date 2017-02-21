/*
 * ClassicICP.cpp
 *
 *  Created on: 18/giu/2010
 *      Author: Mladen Mazuran
 */

#include "ClassicICP.h"
#include <Eigen/LU>
#include <Eigen/SVD>
#include "PointToLineICP.h"
#include "../Config.h"

ClassicICP::ClassicICP(NearestNeighbours *nearestNeighbours):
        ICP(nearestNeighbours) {
}

ClassicICP::~ClassicICP() {
    // TODO Auto-generated destructor stub
}

Eigen::Vector3d ClassicICP::getBestRototranslationVector(
        const Scan &refScan,
        const Scan &rotatedScan,
        const unsigned int queryToReferenceMapping[][2],
        const unsigned int queryIndices[],
        unsigned int queryIndexCount) {

    Eigen::Vector2d refCenter, queryCenter;
    Eigen::Vector2d refCentered  [queryIndexCount];
    Eigen::Vector2d queryCentered[queryIndexCount];
    unsigned int refIndices[queryIndexCount];

    for (unsigned int i = 0; i < queryIndexCount; i++) {
        refIndices[i] = queryToReferenceMapping[queryIndices[i]][0];
    }

    doCentering(refScan, refIndices, queryIndexCount, refCentered, refCenter);
    doCentering(rotatedScan, queryIndices, queryIndexCount, queryCentered, queryCenter);


    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();

    for (unsigned int i = 0; i < queryIndexCount; i++) {
        H += refCentered[i] * queryCentered[i].transpose();
    }

    // Now use H to find R and t
    Eigen::JacobiSVD<Eigen::Matrix2d> svdH(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svdH.matrixU();
    Eigen::Matrix2d V = svdH.matrixV();

    Eigen::Matrix2d R = V * U.transpose();


    // If det = -1 then change V matrix
    if(R.determinant() < 0) {
        V.block<2,1>(0,1) *= -1;
        R = V * U.transpose();
    }

    Eigen::Vector2d t = queryCenter - R * refCenter;

    return Eigen::Vector3d(t[0], t[1], std::atan2(R(1, 0), R(0, 0)));
}

void ClassicICP::doCentering(
        const Scan &sourceScan,
        const unsigned int indices[],
        unsigned int indexCount,
        Eigen::Vector2d centered[],
        Eigen::Vector2d &centroid) const {

    //Find Average
    double sumX = 0;
    double sumY = 0;

    for (unsigned int i = 0; i < indexCount; i++) {
        unsigned int k = indices[i];
        sumX += sourceScan[k].getX();
        sumY += sourceScan[k].getY();
    }

    // Compute the centroid
    centroid[0] = sumX / indexCount;
    centroid[1] = sumY / indexCount;

    // Compute centered scan
    for (unsigned int i = 0; i < indexCount; i++) {
        unsigned int k = indices[i];
        centered[i][0] = sourceScan[k].getX() - centroid[0];
        centered[i][1] = sourceScan[k].getY() - centroid[1];
    }
}

Eigen::Matrix3d ClassicICP::computeCovariance(
        const Scan &refScan,
        const Scan &queryScan,
        const Rototranslation2D &rototranslation,
        const unsigned int queryToReferenceMapping[][2]) {

    // Define this if you want to use the actual classic ICP cost function instead of the point to
    // line cost function. The point to line one works much better.
#ifdef STRICT_CENSI_COVARIANCE
    // This routine assumes that only the range of the laser are subject to
    // error and not the angle

    const unsigned int refScanSize = refScan.size();
    const unsigned int queryScanSize = queryScan.size();
    const Eigen::Vector3d v = rototranslation.getVectorForm();
    const double tx = v[0], ty = v[1], a = v[2];

    // Hessian of point to line cost function wrt rototranslation
    Eigen::Matrix3d H_xx = Eigen::Matrix3d::Zero();

    // Hessian of point to line cost function wrt rototranslation and the norm
    // (polar form) of the points from both scans
    Eigen::MatrixXd H_xp = Eigen::MatrixXd::Zero(3, refScanSize + queryScanSize);

    for(unsigned int i = 0; i < queryScanSize; i++) {
        unsigned int j = queryToReferenceMapping[i][0];

        // Load the two points, one from the reference scan, one from the query
        const Point &ref0  = refScan  [j];
        const Point &query = queryScan[i];

        j += queryScanSize;

        // Put the points in polar form
        const double rho0 = ref0.getRho(), rhoq = query.getRho();
        const double ct0  = ref0.getX()  / rho0, st0 = ref0.getY()  / rho0;
        const double ctq  = query.getX() / rhoq, stq = query.getY() / rhoq;

        // Automatically optimised code for the computation of the Hessian wrt rototranslation
        const double tmp_xx_0 = std::cos(a);
        const double tmp_xx_1 = std::sin(a);
        const double tmp_xx_2 = -2*rhoq*(stq*tmp_xx_0 + ctq*tmp_xx_1);
        const double tmp_xx_3 = 2*rhoq*(ctq*tmp_xx_0 - stq*tmp_xx_1);

        // Component of Hessian wrt rototranslation
        H_xx(0,0) += 2;
        H_xx(0,1) += 0;
        H_xx(0,2) += tmp_xx_2;
        H_xx(1,0) += 0;
        H_xx(1,1) += 2;
        H_xx(1,2) += tmp_xx_3;
        H_xx(2,0) += tmp_xx_2;
        H_xx(2,1) += tmp_xx_3;
        H_xx(2,2) += 2*rhoq*(tmp_xx_1*(ctq*rho0*st0 - ct0*rho0*stq + stq*tx - ctq*ty) +
                tmp_xx_0*(ct0*ctq*rho0 + rho0*st0*stq - ctq*tx - stq*ty));


        // Component of Hessian wrt rototranslation and query point norm
        H_xp(0,i) += 2*ctq*tmp_xx_0 - 2*stq*tmp_xx_1;
        H_xp(1,i) += 2*(stq*tmp_xx_0 + ctq*tmp_xx_1);
        H_xp(2,i) += -2*(tmp_xx_0*(ctq*rho0*st0 - ct0*rho0*stq + stq*tx - ctq*ty) +
                tmp_xx_1*(-(ct0*ctq*rho0) - rho0*st0*stq + ctq*tx + stq*ty));

        // Component of Hessian wrt rototranslation and reference point norm
        H_xp(0,j) += -2*ct0;
        H_xp(1,j) += -2*st0;
        H_xp(2,j) += 2*rhoq*((-(ctq*st0) + ct0*stq)*tmp_xx_0 + (ct0*ctq + st0*stq)*tmp_xx_1);

    }

    // Jacobian of minimisation algorithm wrt the data (the norm of the scan points)
    Eigen::MatrixXd J = H_xx.inverse() * H_xp;

    // Covariance is J * R * J.transpose(), where R is the covariance of the input data; if we
    // assume all point norms to be uncorrelated and with variance sigma2 it becomes:
    return config::sigma2 * J * J.transpose();
#else
    return PointToLineICP::computeCensiCovariance(
            refScan, queryScan, rototranslation, queryToReferenceMapping);
#endif
}
