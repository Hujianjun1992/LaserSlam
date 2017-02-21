/*
 * Scan.cpp
 *
 *  Created on: 12/lug/2010
 *      Author: Mladen Mazuran
 */

#include "Scan.h"
#include "MedianFilter.h"
#include "MobileAverage.h"

void Scan::medianFilter(unsigned int windowSize) {

    unsigned int size = content.size();
    double rhos[size], rhosmedian[size];

    for(unsigned int i = 0; i < size; i++)
        rhos[i] = content[i].getRho();

    doMedianFilter(rhos, rhosmedian, size, windowSize);

    for(unsigned int i = 0; i < size; i++)
        content[i].setRho(rhosmedian[i]);
}

void Scan::mobileAverage(unsigned int windowSize) {

    unsigned int size = content.size();
    double source[size], targetX[size], targetY[size];

    for(unsigned int i = 0; i < size; i++)
        source[i] = content[i].getX();

    doMobileAverage(source, targetX, size, windowSize);

    for(unsigned int i = 0; i < size; i++)
        source[i] = content[i].getY();

    doMobileAverage(source, targetY, size, windowSize);

    for(unsigned int i = 0; i < size; i++)
        content[i] = Point::Cartesian(targetX[i], targetY[i]);
}

void Scan::mergeTo(Scan &c) {
    unsigned int size = c.content.size();
    for(unsigned int i = 0; i < size; i++) {
        content.push_back(c.content[i]);
    }
}

Scan operator*(const Rototranslation2D &rt, const Scan &s) {
    Scan ret(s.content.size());
    std::vector<Point>::const_iterator it;
    int i = 0;
    for (it = s.content.begin(); it < s.content.end(); it++, i++) {
        double xp, yp;
        rt.doRotoTranslation((*it).getX(), (*it).getY(), xp, yp);
        ret.content[i] = Point::Cartesian(xp, yp);
    }
    return ret;
}

std::ostream &operator<<(std::ostream &stream, const Scan &s) {
    std::vector<Point>::const_iterator it;
    unsigned int i = s.size();
    stream << "{";
    for (it = s.content.begin(); it < s.content.end(); it++) {
        stream << (*it);
        if(--i > 0)
            stream << ", ";
    }
    return stream << "}";
}
