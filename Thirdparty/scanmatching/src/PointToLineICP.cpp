/*
 * PointToLineICP.cpp
 *
 *  Created on: 12/giu/2010
 *      Author: Mladen Mazuran
 */

#include "PointToLineICP.h"
#include <cmath>
#include <Eigen/LU>
#include "../Config.h"
extern "C" {
    #include "gpc/gpc.h"
}

PointToLineICP::PointToLineICP(NearestNeighbours *nearestNeighbours):
        ICP(nearestNeighbours) {
}

PointToLineICP::~PointToLineICP() {
    // TODO Auto-generated destructor stub
}

Eigen::Vector3d PointToLineICP::getBestRototranslationVector(
        const Scan &refScan,
        const Scan &rotatedScan,
        const unsigned int queryToReferenceMapping[][2],
        const unsigned int queryIndices[],
        unsigned int queryIndexCount) {

    Eigen::Vector3d q;
    struct gpc_corr c[queryIndexCount];
    double x[3] = {0, 0, 0}; // give the solution
    double n[2]; // normal to segment
    double normN;

    for (unsigned int k = 0; k < queryIndexCount; k++){
        unsigned int h = queryIndices[k];
        double rx0 = refScan[queryToReferenceMapping[h][0]].getX();
        double ry0 = refScan[queryToReferenceMapping[h][0]].getY();
        double rx1 = refScan[queryToReferenceMapping[h][1]].getX();
        double ry1 = refScan[queryToReferenceMapping[h][1]].getY();

        // first scan [use the nearest point]
        c[k].p[0] = rx0;
        c[k].p[1] = ry0;

        // second scan
        c[k].q[0] = rotatedScan[h].getX();
        c[k].q[1] = rotatedScan[h].getY();

        // point to line metric is wi*ni*ni' where ni is the normal to the segment
        //TODO check segno (MODIFICATO SEGNO DI n[0])
        n[0] = -(ry0 - ry1);
        //n[0] = (yyR[idxRef[h][0]] - yyR[idxRef[h][1]]);
        n[1] = rx0 - rx1;

        // You need to normalize the vector?
        normN = std::sqrt(n[0] * n[0] + n[1] * n[1]);
        n[0] = n[0] / normN;
        n[1] = n[1] / normN;

        //n[0]*=-1;
        //n[1]*=-1;
        c[k].C[0][0] = n[0] * n[0];
        c[k].C[0][1] = c[k].C[1][0] = n[0] * n[1];
        c[k].C[1][1] = n[1] * n[1];
    }

    gpc_solve(queryIndexCount, c, x);

    q << x[0], x[1], x[2];

    return q;
}

Eigen::Matrix3d PointToLineICP::computeCovariance(
        const Scan &refScan,
        const Scan &queryScan,
        const Rototranslation2D &rototranslation,
        const unsigned int queryToReferenceMapping[][2]) {
    return computeCensiCovariance(refScan, queryScan, rototranslation, queryToReferenceMapping);
}

Eigen::Matrix3d PointToLineICP::computeCensiCovariance(
        const Scan &refScan,
        const Scan &queryScan,
        const Rototranslation2D &rototranslation,
        const unsigned int queryToReferenceMapping[][2]) {

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
        unsigned int j0 = queryToReferenceMapping[i][0];
        unsigned int j1 = queryToReferenceMapping[i][1];

        // Load the three points, two from the reference scan, one from the query
        const Point &ref0  = refScan  [j0];
        const Point &ref1  = refScan  [j1];
        const Point &query = queryScan[i];

        j0 += queryScanSize;
        j1 += queryScanSize;

        // Put the points in polar form
        const double rho0 = ref0.getRho(), rho1 = ref1.getRho(), rhoq = query.getRho();
        const double ct0  = ref0.getX()  / rho0, st0 = ref0.getY()  / rho0;
        const double ct1  = ref1.getX()  / rho1, st1 = ref1.getY()  / rho1;
        const double ctq  = query.getX() / rhoq, stq = query.getY() / rhoq;

        // Automatically optimised code for the computation of the Hessian wrt rototranslation
        const double tmp_xx_0 = rho0*st0;
        const double tmp_xx_1 = -(rho1*st1);
        const double tmp_xx_2 = tmp_xx_0 + tmp_xx_1;
        const double tmp_xx_3 = (tmp_xx_2*tmp_xx_2);
        const double tmp_xx_4 = (rho0*rho0);
        const double tmp_xx_5 = 1/((rho1*rho1) - 2*rho0*rho1*(ct0*ct1 + st0*st1) + tmp_xx_4);
        const double tmp_xx_6 = ct0*rho0 - ct1*rho1;
        const double tmp_xx_7 = -2*tmp_xx_2*tmp_xx_5*tmp_xx_6;
        const double tmp_xx_8 = std::cos(a);
        const double tmp_xx_9 = std::sin(a);
        const double tmp_xx_10 = (ct0*ctq*rho0 + rho0*st0*stq - rho1*(ct1*ctq + st1*stq))*
                tmp_xx_8 + (ctq*rho0*st0 - ctq*rho1*st1 - ct0*rho0*stq + ct1*rho1*stq)*tmp_xx_9;
        const double tmp_xx_11 = -2*rhoq*tmp_xx_10*tmp_xx_2*tmp_xx_5;
        const double tmp_xx_12 = 2*rhoq*tmp_xx_10*tmp_xx_5*tmp_xx_6;
        const double tmp_xx_13 = stq*tmp_xx_8 + ctq*tmp_xx_9;
        const double tmp_xx_14 = ctq*tmp_xx_8 - stq*tmp_xx_9;
        const double tmp_xx_15 = -(rho0*st0);

        // Component of Hessian wrt rototranslation
        H_xx(0,0) += 2*tmp_xx_3*tmp_xx_5;
        H_xx(0,1) += tmp_xx_7;
        H_xx(0,2) += tmp_xx_11;
        H_xx(1,0) += tmp_xx_7;
        H_xx(1,1) += 2*tmp_xx_5*(tmp_xx_6*tmp_xx_6);
        H_xx(1,2) += tmp_xx_12;
        H_xx(2,0) += tmp_xx_11;
        H_xx(2,1) += tmp_xx_12;
        H_xx(2,2) += 2*tmp_xx_5*(std::pow(rhoq*tmp_xx_13*tmp_xx_2 + rhoq*tmp_xx_14*tmp_xx_6,2) +
                (rhoq*tmp_xx_14*tmp_xx_2 - rhoq*tmp_xx_13*tmp_xx_6)* ((rho1*st1 + tmp_xx_15)*
                        (-(ct0*rho0) + ctq*rhoq*tmp_xx_8 - rhoq*stq*tmp_xx_9 + tx) + tmp_xx_6*
                        (tmp_xx_15 + rhoq*stq*tmp_xx_8 + ctq*rhoq*tmp_xx_9 + ty)));

        // Automatically optimised code for the computation of the Hessian wrt rototranslation and
        // points
        const double tmp_xp_0 = -(rho0*st0);
        const double tmp_xp_1 = rho1*st1;
        const double tmp_xp_2 = tmp_xp_0 + tmp_xp_1;
        const double tmp_xp_3 = (rho0*rho0);
        const double tmp_xp_4 = (rho1*rho1);
        const double tmp_xp_5 = ct0*ct1;
        const double tmp_xp_6 = st0*st1 + tmp_xp_5;
        const double tmp_xp_7 = -2*rho0*rho1*tmp_xp_6;
        const double tmp_xp_8 = tmp_xp_3 + tmp_xp_4 + tmp_xp_7;
        const double tmp_xp_9 = 1/tmp_xp_8;
        const double tmp_xp_10 = ct0*rho0;
        const double tmp_xp_11 = -(ct1*rho1) + tmp_xp_10;
        const double tmp_xp_12 = std::cos(a);
        const double tmp_xp_13 = std::sin(a);
        const double tmp_xp_14 = stq*tmp_xp_12 + ctq*tmp_xp_13;
        const double tmp_xp_15 = ctq*tmp_xp_12 - stq*tmp_xp_13;
        const double tmp_xp_16 = tmp_xp_11*tmp_xp_14 + tmp_xp_15*tmp_xp_2;
        const double tmp_xp_17 = std::pow(tmp_xp_8,-2);
        const double tmp_xp_18 = ct0*ctq + st0*stq;
        const double tmp_xp_19 = ct1*rho1*st0 - ct0*rho1*st1 + rhoq*(-(ctq*st0) + ct0*stq)*
                tmp_xp_12 + rhoq*tmp_xp_13*tmp_xp_18 - st0*tx + ct0*ty;
        const double tmp_xp_20 = -rho0 + ct0*ct1*rho1 + rho1*st0*st1;
        const double tmp_xp_21 = tmp_xp_0 + rhoq*stq*tmp_xp_12 + ctq*rhoq*tmp_xp_13 + ty;
        const double tmp_xp_22 = -(ct0*rho0) + ctq*rhoq*tmp_xp_12 - rhoq*stq*tmp_xp_13 + tx;
        const double tmp_xp_23 = tmp_xp_11*tmp_xp_21 + tmp_xp_2*tmp_xp_22;
        const double tmp_xp_24 = -(ct1*tmp_xp_21) + st1*tmp_xp_22;
        const double tmp_xp_25 = ct0*ct1*rho0 - rho1 + rho0*st0*st1;
        const double tmp_xp_26 = ct1*ctq + st1*stq;
        const double tmp_xp_27 = ct0*ctq*rho0 + rho0*st0*stq - rho1*tmp_xp_26;
        const double tmp_xp_28 = rho0*st0;
        const double tmp_xp_29 = -(rho1*st1);
        const double tmp_xp_30 = rhoq*tmp_xp_11*tmp_xp_15 + rhoq*tmp_xp_14*(tmp_xp_28 + tmp_xp_29);

        // Component of Hessian wrt rototranslation and query point norm
        H_xp(0,i) += 2*tmp_xp_16*tmp_xp_2*tmp_xp_9;
        H_xp(1,i) += 2*tmp_xp_11*tmp_xp_16*tmp_xp_9;
        H_xp(2,i) += 2*((ctq*rho0*st0 - ctq*rho1*st1 - ct0*rho0*stq + ct1*rho1*stq)*tmp_xp_13 +
                tmp_xp_12*tmp_xp_27)*tmp_xp_9*(2*rhoq* ((-(ctq*rho0*st0) + ctq*rho1*st1 +
                        ct0*rho0*stq - ct1*rho1*stq)* tmp_xp_12 + tmp_xp_13*tmp_xp_27) - rho0*st0*tx
                        + rho1*st1*tx + ct1*rho1*(tmp_xp_28 - ty) + ct0*rho0*(tmp_xp_29 + ty));

        // Component of Hessian wrt rototranslation and reference point 1 norm
        H_xp(0,j0) += 2*tmp_xp_17*(2*tmp_xp_2*tmp_xp_20*tmp_xp_23 + tmp_xp_19*tmp_xp_2*tmp_xp_8 -
                st0*tmp_xp_23*tmp_xp_8);
        H_xp(1,j0) += 2*tmp_xp_17*(2*tmp_xp_11*tmp_xp_20*tmp_xp_23 + tmp_xp_11*tmp_xp_19*tmp_xp_8 +
                ct0*tmp_xp_23*tmp_xp_8);
        H_xp(2,j0) += 2*tmp_xp_17*(2*tmp_xp_20*tmp_xp_23*tmp_xp_30 + rhoq*((ctq*st0 - ct0*stq)*
                tmp_xp_13 + tmp_xp_12*tmp_xp_18)*tmp_xp_23*tmp_xp_8 + tmp_xp_19*tmp_xp_30*tmp_xp_8);

        // Component of Hessian wrt rototranslation and reference point 2 norm
        H_xp(0,j1) += 2*tmp_xp_17*(2*tmp_xp_2*tmp_xp_23*tmp_xp_25 + st1*tmp_xp_23*tmp_xp_8 +
                tmp_xp_2*tmp_xp_24*tmp_xp_8);
        H_xp(1,j1) += 2*tmp_xp_17*(2*tmp_xp_11*tmp_xp_23*tmp_xp_25 - ct1*tmp_xp_23*tmp_xp_8 +
                tmp_xp_11*tmp_xp_24*tmp_xp_8);
        H_xp(2,j1) += 2*tmp_xp_17*(2*tmp_xp_23*tmp_xp_25*tmp_xp_30 - rhoq*tmp_xp_23*((ctq*st1 -
                ct1*stq)*tmp_xp_13 + tmp_xp_12*tmp_xp_26)*tmp_xp_8 + tmp_xp_24*tmp_xp_30*tmp_xp_8);

    }

    // Jacobian of minimisation algorithm wrt the data (the norm of the scan points)
    Eigen::MatrixXd J = H_xx.inverse() * H_xp;

    // Covariance is J * R * J.transpose(), where R is the covariance of the input data; if we
    // assume all point norms to be uncorrelated and with variance sigma2 it becomes:
    return config::sigma2 * J * J.transpose();
}
