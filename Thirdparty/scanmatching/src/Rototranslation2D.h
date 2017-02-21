/*
 * Rototranslation2D.h
 *
 *  Created on: 23/nov/2009
 *      Author: mallo
 */

#ifndef ROTOTRANSLATION2D_H_
#define ROTOTRANSLATION2D_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

//! A two dimensional rototranslation in homogeneous coordinates
class Rototranslation2D: public Eigen::Transform<double, 2, Eigen::Affine> {

private:
    void initRototranslation2D(const Eigen::Rotation2D<Scalar> &r,const Eigen::Matrix<Scalar,2,1> &vect);

public:
    //! Creates a two dimensional rototranslation
    Rototranslation2D();
    //! Initialize the rototranslation with a vector whose values represent x,y and angle
    /*!
     * \param vect the three dimensional vector with x,y and angle
     */
    Rototranslation2D(const Eigen::Vector3d &vect);

    //! Initialize the rototranslation with the rotation and the translation given separately
    /*!
     * \param r the two dimensional rotation part of the rototranslation
     * \param vect the translational part of the rototranslation
     */
    Rototranslation2D(const Eigen::Rotation2D<double> &r, const Eigen::Vector2d &vect);

    //! Copy constructor
    Rototranslation2D(const Rototranslation2D &rt2);

    Rototranslation2D &operator=(const Rototranslation2D &rt2);
    Rototranslation2D &operator=(const Eigen::Matrix3d &mat);
    Rototranslation2D operator*(const Rototranslation2D &rt2) const;

    //! Returns the inverse transformation
    Rototranslation2D inverse() const;

    //! Returns the rotational part of the rototranslation
    Eigen::Rotation2D<double> getRotation() const;

    //! Returns the rotation angle
    double getAngle() const;

    //! Returns the translational part of the rototranslation
    Eigen::Vector2d getTranslation() const;

    //! Returns the vector form with x, y, angle of the rototranslation
    Eigen::Vector3d getVectorForm() const;

    operator Eigen::Vector3d() const;

    //! Rototranslate the input values x,y and return the rototranslated values
    /*!
     * \param x the first value that will be rototranslated
     * \param y the second value that will be rototranslated
     * \param nx the return value representing the value x rototranslated
     * \param ny the return value representing the value y rototranslated
     */
    inline void doRotoTranslation(double x, double y, double &nx, double &ny) const {
        nx = this->m_matrix(0, 0) * x + this->m_matrix(0, 1) * y + this->m_matrix(0, 2);
        ny = this->m_matrix(1, 0) * x + this->m_matrix(1, 1) * y + this->m_matrix(1, 2);
    }
};

#endif /* ROTOTRANSLATION2D_H_ */
