/*
 * Point.h
 *
 *  Created on: 08/giu/2010
 *      Author: Mladen Mazuran & Matteo Luperto
 */

#ifndef POINT_H_
#define POINT_H_

#include <cmath>
#include <iostream>

//! This class defines a couple of values x,y representing a Point in a 2 dimensional space.
class Point {
private:
    double x, y;

    inline Point(double _x, double _y): x(_x), y(_y) {}

public:
    inline Point() {}

    //! Returns the cartesian value X, given the polar coordinates r and t
    /*!
     * \param r the polar coordinate rho
     * \param t the polar coordinate theta
     */
    static inline double polarToX        (double r, double t) { return r * std::cos(t); }

    //! Returns the cartesian value Y, given the polar coordinates r and t
    /*!
     * \param r the polar coordinate rho
     * \param t the polar coordinate theta
     */
    static inline double polarToY        (double r, double t) { return r * std::sin(t); }

    //! Returns the polar value rho, given the cartesian couple x and y
    /*!
     * \param x the cartesian value x
     * \param y the cartesian value y
     */
    static inline double cartesianToRho  (double x, double y) { return std::sqrt(x * x + y * y); }

    //! Returns the polar value theta, given the cartesian couple x and y
    /*!
     * \param x the cartesian value x
     * \param y the cartesian value y
     */
    static inline double cartesianToTheta(double x, double y) { return std::atan2(y, x); }

    //! Creates a point with coordinates x and y
    /*!
     * \param x the x value
     * \param y the y value
     */
    static inline Point Cartesian(double x, double y) {
        return Point(x, y);
    }

    //! Creates a point with polar coordinates rho and theta
    /*!
     * \param r the rho value
     * \param t the theta value
     */
    static inline Point Polar(double r, double t) {
        return Point(Point::polarToX(r, t), Point::polarToY(r, t));
    }

    //! returns the x value in a cartesian point
    inline double getX() const      { return x; }

    //! returns the y value in a cartesian point
    inline double getY() const      { return y; }

    //! returns the rho value in a point represented in polar coordinates
    inline double getRho() const    { return Point::cartesianToRho(x, y); }

    //! returns the theta value in a point represented in polar coordinates
    inline double getTheta() const  { return Point::cartesianToTheta(x, y); }

    //! Sets the x value in a cartesian point
    inline void setX(double x)      { this->x = x; }

    //! Sets the y value in a cartesian point
    inline void setY(double y)      { this->y = y; }

    //! Sets the rho value in a point represented in polar coordinates
    inline void setRho(double r) {
        double x = Point::polarToX(r, this->getTheta());
        double y = Point::polarToY(r, this->getTheta());
        this->x = x;
        this->y = y;
    }

    //! Set the theta value in a point represented in polar coordinates
    inline void setTheta(double t) {
        double x = Point::polarToX(this->getRho(), t);
        double y = Point::polarToY(this->getRho(), t);
        this->x = x;
        this->y = y;
    }

    friend std::ostream &operator<<(std::ostream &stream, const Point &p);

};

inline std::ostream &operator<<(std::ostream &stream, const Point &p) {
    return stream << "{" << p.getX() << "," << p.getY() << "}";
}


#endif /* POINT_H_ */
