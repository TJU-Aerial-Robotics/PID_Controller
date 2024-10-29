#ifndef LOWPASS_FILTER_H_
#define LOWPASS_FILTER_H_

#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

template<class T>
class Lowpass_Filter{
    public:

        Lowpass_Filter(double cut_param) {
            if ((cut_param > 0) && (cut_param < 1)) {
                _cut_param = cut_param;
            } else {
                ROS_INFO("wrong param");
                _cut_param = 0.0f;
            }
            has_init = false;
        }
        ~Lowpass_Filter() {}

        void input(T new_value) {
            if (has_init) {
                last_value = new_value + (last_value-new_value)*_cut_param;
            } else {
                last_value = new_value;
                has_init = true;
            }
        }

        void getu(T &out) {
            out = last_value;
        }

    private:
        double _cut_param;
        T last_value;
        bool has_init;
};

#endif