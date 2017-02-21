/*
 * Scan.h
 *
 *  Created on: 08/giu/2010
 *      Author: Mladen Mazuran & Matteo Luperto
 */

#ifndef SCAN_H_
#define SCAN_H_

#include <vector>
#include "Point.h"
#include "Rototranslation2D.h"

//! Collection of point representing a scan
class Scan {

private:
    std::vector<Point> content;

public:

    //! Copy constructor
    inline Scan() {}

    //! Initializes a Scan copying from another Scan
    inline Scan(const Scan &c) { content = c.content; }

    //! Initializes a Scan with 'size' points, all with value zero
    /*!
     * \param size the number of points to be initialized
     */
    inline Scan(int size): content(size, Point()) {}

    inline Point &operator[](int index)                     { return content[index]; }
    inline const Point &operator[](int index) const         { return content[index]; }

    //! Returns the number of points in the Scan
    inline unsigned int size() const                        { return content.size(); }

    //! Add a point to the collection of poins of the Scan
    inline void push_back(Point p)                          { content.push_back(p);  }

    inline std::vector<Point>::const_iterator begin()       { return content.begin(); }
    inline std::vector<Point>::const_iterator end()         { return content.end();   }

    inline Scan &operator=(const Scan &c) { content = c.content; return *this; }

    //! Apply a median filter with a window of size windowSize
    /*!
     * \param windowSize the size of the window of the filter
     */
    void medianFilter(unsigned int windowSize = 5);

    //! Makes the mobile average of the points in a window of size windowSize
    /*!
     * \param windowSize the size of the window of the filter
     */
    void mobileAverage(unsigned int windowSize = 5);

    //! Takes the points from the input Scan 'c' and adds them to the Scan
    /*!
     * \param c the Scan that has to be merged with this object
     */
    void mergeTo(Scan &c);

    friend Scan operator*(const Rototranslation2D &rt, const Scan &s);
    friend std::ostream &operator<<(std::ostream &stream, const Scan &s);
};

#endif /* SCAN_H_ */
