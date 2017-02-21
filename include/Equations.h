/*
 * Equations.h
 *
 *  Created on: 22/lug/2012
 *      Author: Mladen Mazuran
 */

#ifndef EQUATIONS_H_
#define EQUATIONS_H_

namespace LaserSlam{

/* Provides an estimate of the next pose from the previous one plus a motion delta */
static Rototranslation2D updatePose(
        const Eigen::Vector3d &oldPose, const Eigen::Vector3d &motion) {
    return Rototranslation2D(oldPose) * Rototranslation2D(motion);
}

/*
    Provides an estimate (linearised) of the next pose covariance from the previous pose and its
    covariance, plus a motion delta and its covariance
*/
static Eigen::Matrix3d updatePoseCovariance(
        const Eigen::Vector3d &oldPose, const Eigen::Matrix3d &oldCov,
        const Eigen::Vector3d &motion,  const Eigen::Matrix3d &motionCov) {

    Eigen::Matrix3d Jp, Jm;

    double c = std::cos(oldPose[2]), s = std::sin(oldPose[2]);

    /* Jacobian wrt previous pose */
    Jp <<
            1, 0, - motion[1] * c - motion[0] * s,
            0, 1, - motion[1] * s + motion[0] * c,
            0, 0, 1;

    /* Jacobian wrt motion delta */
    Jm <<
            c, -s, 0,
            s,  c, 0,
            0,  0, 1;

    return Jp * oldCov * Jp.transpose() + Jm * motionCov * Jm.transpose();
}

} /* namespace equations */

#endif /* EQUATIONS_H_ */
