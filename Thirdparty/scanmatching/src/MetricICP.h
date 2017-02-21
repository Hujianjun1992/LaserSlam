/*
 * MetricICP.h
 *
 *  Created on: 24/giu/2010
 *      Author: Mladen Mazuran
 */

#ifndef METRICICP_H_
#define METRICICP_H_

#include "ICP.h"
#include "NearestNeighbours.h"

//! Metric %ICP described by Minguez et al
/*!
 * Uses an internal implementation of the nearest neighbours
 * algorithm that uses a non euclidean metric.
 */
class MetricICP: public ICP {
private:
    const double l;

    void doCentering(
            const Scan &sourceScan,
            const unsigned int indexes[],
            unsigned int indexCount,
            double centeredX[],
            double centeredY[],
            double &xCentroid,
            double &yCentroid) const;

    class NearestNeighboursMetricON2: public NearestNeighbours {
    private:
        const double l;

    public:
        NearestNeighboursMetricON2(double l = 3.0);
        virtual ~NearestNeighboursMetricON2();

        virtual void initialize(const Scan &refScan);
        virtual void deinitialize();

        virtual void getNearestNeighbours(
                const Scan &queryScan,
                unsigned int queryToReferenceMapping[][2],
                double nearestDistances[]);
    };

protected:
    Eigen::Vector3d getBestRototranslationVector(
            const Scan &refScan,
            const Scan &rotatedScan,
            const unsigned int queryToReferenceMapping[][2],
            const unsigned int queryIndexes[],
            unsigned int queryIndexCount);

    Eigen::Matrix3d computeCovariance(
                const Scan &refScan,
                const Scan &queryScan,
                const Rototranslation2D &rototranslation,
                const unsigned int queryToReferenceMapping[][2]);

public:
    //! Create a new metric %ICP object
    /*!
     * \param l angular coefficient used in the non euclidean metric
     */
    MetricICP(double l = 3.0);
    virtual ~MetricICP();
};

#endif /* METRICICP_H_ */
