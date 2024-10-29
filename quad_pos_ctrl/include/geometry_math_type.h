#ifndef GEOMETRY_MATH_TYPE_H_
#define GEOMETRY_MATH_TYPE_H_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

void get_dcm_from_q(Eigen::Matrix3d &dcm, const Eigen::Quaterniond &q) {
    float a = q.w();
    float b = q.x();
    float c = q.y();
    float d = q.z();
    float aSq = a*a;
    float bSq = b*b;
    float cSq = c*c;
    float dSq = d*d;
    dcm(0, 0) = aSq + bSq - cSq - dSq; 
    dcm(0, 1) = 2 * (b * c - a * d);
    dcm(0, 2) = 2 * (a * c + b * d);
    dcm(1, 0) = 2 * (b * c + a * d);
    dcm(1, 1) = aSq - bSq + cSq - dSq;
    dcm(1, 2) = 2 * (c * d - a * b);
    dcm(2, 0) = 2 * (b * d - a * c);
    dcm(2, 1) = 2 * (a * b + c * d);
    dcm(2, 2) = aSq - bSq - cSq + dSq;
}

void get_q_from_dcm(Eigen::Quaterniond &q, const Eigen::Matrix3d &dcm) {
    float t = dcm.trace();
    if ( t > 0.0f ) {
        t = sqrt(1.0f + t);
        q.w() = 0.5f * t;
        t = 0.5f / t;
        q.x() = (dcm(2,1) - dcm(1,2)) * t;
        q.y() = (dcm(0,2) - dcm(2,0)) * t;
        q.z() = (dcm(1,0) - dcm(0,1)) * t;
    } else if (dcm(0,0) > dcm(1,1) && dcm(0,0) > dcm(2,2)) {
        t = sqrt(1.0f + dcm(0,0) - dcm(1,1) - dcm(2,2));
        q.x() = 0.5f * t;
        t = 0.5f / t;
        q.w() = (dcm(2,1) - dcm(1,2)) * t;
        q.y() = (dcm(1,0) + dcm(0,1)) * t;
        q.z() = (dcm(0,2) + dcm(2,0)) * t;
    } else if (dcm(1,1) > dcm(2,2)) {
        t = sqrt(1.0f - dcm(0,0) + dcm(1,1) - dcm(2,2));
        q.y() = 0.5f * t;
        t = 0.5f / t;
        q.w() = (dcm(0,2) - dcm(2,0)) * t;
        q.x() = (dcm(1,0) + dcm(0,1)) * t;
        q.z() = (dcm(2,1) + dcm(1,2)) * t;
    } else {
        t = sqrt(1.0f - dcm(0,0) - dcm(1,1) + dcm(2,2));
        q.z() = 0.5f * t;
        t = 0.5f / t;
        q.w() = (dcm(1,0) - dcm(0,1)) * t;
        q.x() = (dcm(0,2) + dcm(2,0)) * t;
        q.y() = (dcm(2,1) + dcm(1,2)) * t;
    }
}

void get_euler_from_R(Eigen::Vector3d &e, const Eigen::Matrix3d &R) {
   float phi = atan2(R(2, 1), R(2, 2));
   float theta = asin(-R(2, 0));
   float psi = atan2(R(1, 0), R(0, 0));
   float pi = M_PI;

   if (fabs(theta - pi/2.0f) < 1.0e-3) {
       phi = 0.0f;
       psi = atan2(R(1, 2), R(0, 2));
   } else if (fabs(theta + pi/2.0f) < 1.0e-3) {
       phi = 0.0f;
       psi = atan2(-R(1, 2), -R(0, 2));
   }
   e(0) = phi;
   e(1) = theta;
   e(2) = psi;
}

void get_euler_from_q(Eigen::Vector3d &e, const Eigen::Quaterniond &q) {
    Eigen::Matrix3d temp_R;
    get_dcm_from_q(temp_R, q);
    get_euler_from_R(e, temp_R);
}

#endif
