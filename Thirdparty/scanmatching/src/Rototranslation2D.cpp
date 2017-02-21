/*
 * RotoTranslation2D.cpp
 *
 *  Created on: 16/giu/2010
 *      Author: Mladen Mazuran
 */

#include "Rototranslation2D.h"

void Rototranslation2D::initRototranslation2D(
        const Eigen::Rotation2D<double> &r,
        const Eigen::Vector2d &vect) {

    this->setIdentity();
    this->rotate(r);
    this->translation() = vect;
}

Rototranslation2D::Rototranslation2D() {
    this->setIdentity();
}

Rototranslation2D::Rototranslation2D(const Eigen::Vector3d &vect) {
   Eigen::Vector2d vect2(vect[0], vect[1]);
   initRototranslation2D(Eigen::Rotation2D<double>(vect[2]),vect2);
}

Rototranslation2D::Rototranslation2D(
        const Eigen::Rotation2D<double> &r,
        const Eigen::Matrix<double,2,1> &vect) {

    initRototranslation2D(r,vect);
}

Rototranslation2D::Rototranslation2D(const Rototranslation2D &rt2):
        Eigen::Transform<double, 2, 2>() {
    *this = rt2;
}

Rototranslation2D &Rototranslation2D::operator=(const Rototranslation2D &rt2) {
    if (this == &rt2) {      // Same object?
        return *this;        // Yes, so skip assignment, and just return *this.
    }
    m_matrix = rt2.m_matrix;
    //Eigen::Transform<double, 2, Eigen::Affine>::m_matrix = rt2.Eigen::Transform<double, 2, Eigen::Affine>::m_matrix;
    return *this;
}

Rototranslation2D &Rototranslation2D::operator=(const Eigen::Matrix3d &mat) {
    m_matrix = mat;
    //Eigen::Transform<double,2>::m_matrix = mat;
    return *this;
}

Rototranslation2D Rototranslation2D::operator*(const Rototranslation2D &rt2) const {
    Rototranslation2D product;
    product.matrix() = this->matrix() * rt2.matrix();
    return product;
}

Rototranslation2D Rototranslation2D::inverse() const {
    Eigen::Rotation2D<double> rot(-this->getAngle());
    Eigen::Vector2d tNew(rot * this->getTranslation());
    tNew = -tNew;
    return Rototranslation2D(rot, tNew);
}

Eigen::Rotation2D<double> Rototranslation2D::getRotation() const {
    Eigen::Rotation2D<double> rot(0);
    rot = rot.fromRotationMatrix(this->linear());
    return rot;
}

double Rototranslation2D::getAngle() const {
    Eigen::Rotation2D<double> rot = this->getRotation();
    return rot.angle();
}

Eigen::Vector2d Rototranslation2D::getTranslation() const {
    return this->matrix().block(0, 2, 2, 1);
}

Eigen::Vector3d Rototranslation2D::getVectorForm() const {
    Eigen::Vector3d vect;
    vect[0] = this->matrix()(0, 2);
    vect[1] = this->matrix()(1, 2);
    vect[2] = this->getAngle();
    return vect;
}

Rototranslation2D::operator Eigen::Vector3d() const {
    return getVectorForm();
}
