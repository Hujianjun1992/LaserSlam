/*
 * MetricICP.cpp
 *
 *  Created on: 24/giu/2010
 *      Author: Mladen Mazuran
 */

#include "MetricICP.h"
#include <Eigen/LU>
#include "PointToLineICP.h"
#include "../Config.h"

MetricICP::MetricICP(double l):
        ICP(new MetricICP::NearestNeighboursMetricON2(l)), l(l)  {
    // TODO Auto-generated constructor stub
}

MetricICP::~MetricICP() {
    // TODO Auto-generated destructor stub
}

Eigen::Vector3d MetricICP::getBestRototranslationVector(
        const Scan &refScan,
        const Scan &rotatedScan,
        const unsigned int queryToReferenceMapping[][2],
        const unsigned int queryIndexes[],
        unsigned int queryIndexCount) {
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    double pix, piy, cix, ciy, ki, cxpxPcypy, cxpyMcypx;

    for(unsigned int k = 0; k < queryIndexCount; k++){
        unsigned int h = queryIndexes[k];
        cix = refScan[queryToReferenceMapping[h][0]].getX();
        ciy = refScan[queryToReferenceMapping[h][0]].getY();
        pix = rotatedScan[h].getX();
        piy = rotatedScan[h].getY();

        ki = pix * pix + piy * piy + l * l;

        cxpxPcypy = cix * pix + ciy * piy;
        cxpyMcypx = cix * piy - ciy * pix;

        A(0,0) = A(0,0) + 1.0 - piy * piy / ki;
        A(0,1) = A(0,1) + pix * piy / ki;
        A(0,2) = A(0,2) - ciy + piy / ki * cxpxPcypy;
        A(1,1) = A(1,1) + 1.0 - pix * pix / ki;
        A(1,2) = A(1,2) + cix - pix / ki * cxpxPcypy;
        A(2,2) = A(2,2) + cix*cix + ciy*ciy - cxpxPcypy*cxpxPcypy / ki;

        //std::cout << std::endl << A << std::endl;


        b[0] = b[0] + cix - pix - piy / ki * cxpyMcypx;
        b[1] = b[1] + ciy - piy + pix / ki * cxpyMcypx;
        b[2] = b[2] + (cxpxPcypy / ki - 1.0) * cxpyMcypx;
    }

    // Complete the A-matrix by assigning the symmetric portions of it
    A(1,0) = A(0,1);
    A(2,0) = A(0,2);
    A(2,1) = A(1,2);

    return - A.inverse() * b;
}

Eigen::Matrix3d MetricICP::computeCovariance(
        const Scan &refScan,
        const Scan &queryScan,
        const Rototranslation2D &rototranslation,
        const unsigned int queryToReferenceMapping[][2]) {

    // Define this if you want to use the actual metric ICP cost function instead of the point to
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
        const double tmp_xx_0 = (l*l);
        const double tmp_xx_1 = (rhoq*rhoq);
        const double tmp_xx_2 = 1/(tmp_xx_0 + tmp_xx_1);
        const double tmp_xx_3 = 2*ctq*stq*tmp_xx_1*tmp_xx_2;
        const double tmp_xx_4 = -2*rho0*(st0*tmp_xx_0 + ctq*(ctq*st0 - ct0*stq)*tmp_xx_1)*tmp_xx_2;
        const double tmp_xx_5 = (stq*stq);
        const double tmp_xx_6 = tmp_xx_0 + tmp_xx_1*tmp_xx_5;
        const double tmp_xx_7 = 2*rho0*tmp_xx_2*(-(ctq*st0*stq*tmp_xx_1) + ct0*tmp_xx_6);

        // Component of Hessian wrt rototranslation
        H_xx(0,0) += 2*(tmp_xx_0 + (ctq*ctq)*tmp_xx_1)*tmp_xx_2;
        H_xx(0,1) += tmp_xx_3;
        H_xx(0,2) += tmp_xx_4;
        H_xx(1,0) += tmp_xx_3;
        H_xx(1,1) += 2*tmp_xx_2*tmp_xx_6;
        H_xx(1,2) += tmp_xx_7;
        H_xx(2,0) += tmp_xx_4;
        H_xx(2,1) += tmp_xx_7;
        H_xx(2,2) += 2*(rho0*rho0)*tmp_xx_2*(tmp_xx_0 + tmp_xx_1*(-2*ct0*ctq*st0*stq + (st0*st0)*
                (1 - 2*tmp_xx_5) + tmp_xx_5));

        // Automatically optimised code for the computation of the Hessian wrt rototranslation and
        // points
        const double tmp_xp_0 = (l*l);
        const double tmp_xp_1 = (rhoq*rhoq);
        const double tmp_xp_2 = tmp_xp_0 + tmp_xp_1;
        const double tmp_xp_3 = (tmp_xp_2*tmp_xp_2);
        const double tmp_xp_4 = 1/tmp_xp_3;
        const double tmp_xp_5 = (stq*stq);
        const double tmp_xp_6 = (tmp_xp_0*tmp_xp_0);
        const double tmp_xp_7 = (tmp_xp_1*tmp_xp_1);
        const double tmp_xp_8 = 1/tmp_xp_2;
        const double tmp_xp_9 = a*ct0;
        const double tmp_xp_10 = (st0*st0);
        const double tmp_xp_11 = rhoq*st0;
        const double tmp_xp_12 = ct0*tx;

        // Component of Hessian wrt rototranslation and query point norm
        H_xp(0,i) += tmp_xp_4*(-4*rhoq*tmp_xp_0*tmp_xp_5*(ct0*rho0 - a*rho0*st0 + tx) - 2*ctq*
                (tmp_xp_6 + tmp_xp_7 + 2*rhoq*tmp_xp_0* (rhoq - stq*(a*ct0*rho0 + rho0*st0 + ty))));
        H_xp(1,i) += tmp_xp_4*(-4*a*rho0*rhoq*tmp_xp_0*(ct0 + ctq*st0*stq - ct0*tmp_xp_5) -
                2*stq*tmp_xp_6 - 2*stq*tmp_xp_7 + 4*rhoq*tmp_xp_0*(rho0*(ct0*ctq*stq + st0*
                        (-1 + tmp_xp_5)) - ty + stq*(-rhoq + ctq*tx + stq*ty)));
        H_xp(2,i) += -2*rho0*tmp_xp_4*(2*ct0*rho0*rhoq*st0*tmp_xp_0 + ct0*stq*tmp_xp_3 - 4*ct0*
                rho0*rhoq*st0*tmp_xp_0*tmp_xp_5 + 2*a*rho0*rhoq*tmp_xp_0*(1 + 2*ct0*ctq*st0*stq -
                        tmp_xp_5 + tmp_xp_10*(-1 + 2*tmp_xp_5)) - 2*rhoq*st0*tmp_xp_0*tmp_xp_5*tx +
                        2*ct0*rhoq*tmp_xp_0*ty - 2*ct0*rhoq*tmp_xp_0*tmp_xp_5*ty - ctq*(st0*
                                tmp_xp_6 + st0*tmp_xp_7 + 2*rhoq*tmp_xp_0*(tmp_xp_11 + stq*(rho0 -
                                        2*rho0*tmp_xp_10 + tmp_xp_12 - st0*ty))));

        // Component of Hessian wrt rototranslation and reference point norm
        H_xp(0,j) += tmp_xp_8*(2*(ct0 - a*st0)*(tmp_xp_0 + (ctq*ctq)*tmp_xp_1) + 2*ctq*stq*
                tmp_xp_1*(st0 + tmp_xp_9));
        H_xp(1,j) += 2*(st0 - ctq*(a*ct0*ctq + ctq*st0 - ct0*stq + a*st0*stq)*tmp_xp_1*tmp_xp_8 +
                tmp_xp_9);
        H_xp(2,j) += 2*tmp_xp_8*(-(ct0*rhoq*stq*tmp_xp_0) - 2*ct0*rho0*st0*tmp_xp_1 - ct0*rhoq*stq*
                tmp_xp_1 + 4*ct0*rho0*st0*tmp_xp_1*tmp_xp_5 + 2*a*rho0*(tmp_xp_0 + tmp_xp_1*(-2*
                        ct0*ctq*st0*stq + tmp_xp_10 + tmp_xp_5 - 2*tmp_xp_10*tmp_xp_5)) - st0*
                        tmp_xp_0*tx - st0*tmp_xp_1*tx + st0*tmp_xp_1*tmp_xp_5*tx + ct0*(tmp_xp_0 +
                                tmp_xp_1*tmp_xp_5)*ty + ctq*rhoq*(st0*tmp_xp_0 + rhoq* (2*rho0*stq +
                                        tmp_xp_11 + stq*(tmp_xp_12 - st0*(4*rho0*st0 + ty)))));
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

static inline double distance(double p2x, double p2y, double p1x, double p1y, double l) {
    double dx = p2x - p1x;
    double dy = p2y - p1y;
    double tmp = dx * p1y - dy * p1x;

    return dx * dx + dy * dy - (tmp * tmp / (p1y * p1y + p1x * p1x + l * l));
}

MetricICP::NearestNeighboursMetricON2::NearestNeighboursMetricON2(double l): l(l) {

}

MetricICP::NearestNeighboursMetricON2::~NearestNeighboursMetricON2() {

}

void MetricICP::MetricICP::NearestNeighboursMetricON2::initialize(const Scan &refScan) {
    NearestNeighbours::initialize(refScan);
}

void MetricICP::NearestNeighboursMetricON2::deinitialize() {

}

void MetricICP::NearestNeighboursMetricON2::getNearestNeighbours(
        const Scan &queryScan,
        unsigned int queryToReferenceMapping[][2],
        double nearestDistances[]) {

    static const int DISTSIZE = 2;
    const Scan &refScan = *(this->refScan);
    const unsigned int queryScanSize = queryScan.size(), refScanSize = refScan.size();

    double distTemp[DISTSIZE]; // save the first two nearest DISTANCES
    double idxRefTemp[DISTSIZE]; // save the first two nearest indexes

    for (unsigned int i = 0; i < queryScanSize; i++) {
        // initialize the distance indexes
        for (int d = 0; d < DISTSIZE; d++) {
            distTemp[d] = INFINITY;
        }

        for (unsigned int j = 0; j < refScanSize; j++) {
            // Compute the distance
            double dist = distance(refScan[j].getX(), refScan[j].getY(), queryScan[i].getX(), queryScan[i].getY(), l);

            for (int h = 0; h < DISTSIZE; h++) {
                if (dist <= distTemp[h]) {
                    // Update the distTemp
                    for (int d = DISTSIZE - 2; d >= h; d--){
                        distTemp[d + 1] = distTemp[d];
                        idxRefTemp[d + 1] = idxRefTemp[d];
                    }
                    distTemp[h] = dist;
                    idxRefTemp[h] = j;
                    break;
                }
            }
        }

        queryToReferenceMapping[i][0] = idxRefTemp[0];
        queryToReferenceMapping[i][1] = idxRefTemp[1];
        nearestDistances[i] = distTemp[0];
    }

}
