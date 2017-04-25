#include "driver/PTUDriverMock.h"
#include "ros/ros.h"

#define STEPS_UNTIL_GOAL_REACHED 20

namespace asr_flir_ptu_driver {
    PTUDriverMock::PTUDriverMock(const char* port, int baud, bool speedControl):
        PTUDriver::PTUDriver()
    {
        ROS_DEBUG("port: %s, baud: %d, speeControl: %d", port, baud, speedControl);
        is_stopped_count = 0;
        pan_angle = 0;
        tilt_angle = 0;
        pan_speed = 0;
        tilt_speed = 0;
        setLimitAnglesToHardwareConstraints();
    }

    bool PTUDriverMock::isConnected()
    {
        return true;
    }

    void PTUDriverMock::setSettings(int pan_base, int tilt_base,
                                    int pan_speed, int tilt_speed,
                                    int pan_upper, int tilt_upper,
                                    int pan_accel, int tilt_accel,
                                    int pan_hold, int tilt_hold,
                                    int pan_move, int tilt_move)
    {
        this->pan_speed = pan_speed;
        this->tilt_speed = tilt_speed;
        this->pan_base = pan_base;
        this->tilt_base = tilt_base;
        this->pan_acceleration = pan_accel;
        this->tilt_acceleration = tilt_accel;
        ROS_DEBUG("setSettings");
        //this->forbiddenAreas = forbiddenAreas;

    }
    void PTUDriverMock::setLimitAngles(double pan_min, double pan_max, double tilt_min, double tilt_max) {
        ROS_DEBUG("setLimitAngles");
        ROS_DEBUG("double panMin:%f, double panMax:%f",  pan_min,  pan_max);
        ROS_DEBUG("double tiltMin:%f, double tiltMax:%f", tilt_min,  tilt_max);
        this->pan_min_limit = pan_min;
        this->pan_max_limit = pan_max;
        this->tilt_min_limit = tilt_min;
        this->tilt_max_limit = tilt_max;
    }

    void PTUDriverMock::setAbsoluteAngleSpeeds(double pan_speed, double tilt_speed) {
        ROS_DEBUG("setAbsoluteAngleSpeeds");
        ROS_DEBUG("double pan_speed:%f, double tilt_speed:%f", pan_speed,  tilt_speed);
        this->pan_speed = pan_speed;
        this->tilt_speed = tilt_speed;

    }

    void PTUDriverMock::setAbsoluteAngleSpeeds(signed short pan_speed, signed short tilt_speed) {
        ROS_ERROR("This mock function should not be used");
    }

    bool PTUDriverMock::setAbsoluteAngles(double pan_angle, double tilt_angle, bool no_forbidden_area_check) {
        if (isInForbiddenArea(pan_angle, tilt_angle)) {
            ROS_ERROR("PAN and TILT lie within forbidden area");
            return false;
        }
        if (!isWithinPanTiltLimits(pan_angle, tilt_angle)){
            ROS_ERROR("PAN/TILT out of pan/tilt bounds");
            return false;
        }
        ROS_DEBUG("setAbsoluteAngles");
        ROS_DEBUG("double pan_angle:%f, double tilt_angle:%f", pan_angle,  tilt_angle);
        this->pan_distance = pan_angle - this->pan_angle;
        this->tilt_distance = tilt_angle - this->tilt_angle;
        this->pan_angle = pan_angle;
        this->tilt_angle = tilt_angle;
        is_stopped_count++;

        ROS_INFO("Successfull set Pan and Tilt. PAN: %f, TILT: %f\n", pan_angle, tilt_angle);
        return true;
    }
    void PTUDriverMock::setSpeedControlMode(bool speed_control_mode) {
        ROS_DEBUG("setSpeedControlMode");
    }
    bool PTUDriverMock::isInSpeedControlMode() {
        return false;
    }
    /*
    bool PTUDriverMock::isStopped() {
        isStoppedCount++;
        if (isStoppedCount % 50 == 0) {
            isStoppedCount = 1;
            ROS_DEBUG("is NOT stopped");
            return true;
        }
        return false;
    }
    */

    bool PTUDriverMock::isWithinPanTiltLimits(double pan, double tilt){
        if ((pan < pan_min_limit)
            || (pan > pan_max_limit)
            || (tilt < tilt_min_limit)
            || (tilt > tilt_max_limit)){
            return false;
        }
        else {
            return true;
        }
    }

    bool PTUDriverMock::hasHalted() {
        if(is_stopped_count != 0) {
            is_stopped_count++;
        }
        if(is_stopped_count % STEPS_UNTIL_GOAL_REACHED == 0) {
            is_stopped_count = 0;
            return true;
        }
        else {
            return false;
        }
    }

    bool PTUDriverMock::reachedGoal() {
        if(is_stopped_count % STEPS_UNTIL_GOAL_REACHED == 0) {
            return true;
        }
        else {
            return false;
        }
    }

    bool PTUDriverMock::hasHaltedAndReachedGoal() {
        return hasHalted() && reachedGoal();
    }


    double PTUDriverMock::getCurrentAngle(char type) {
        if (type == PAN) {
            return pan_angle - (((STEPS_UNTIL_GOAL_REACHED - is_stopped_count) % STEPS_UNTIL_GOAL_REACHED)/STEPS_UNTIL_GOAL_REACHED * pan_distance);
        }
        return tilt_angle - (((STEPS_UNTIL_GOAL_REACHED - is_stopped_count) % STEPS_UNTIL_GOAL_REACHED)/STEPS_UNTIL_GOAL_REACHED * tilt_distance);
    }

    double PTUDriverMock::getDesiredAngle(char type) {
        if (type == PAN) {
            return pan_angle;
        }
        return tilt_angle;
    }

    //WARNING: IN PTUDriver this returns the current angle speed. In MOCK this is not possible (no real movement, simulation would take a lot of time etc.)
    //Therefore the desired speed is returned if the ptu is "in movment" and 0 is returned otherwise
    double PTUDriverMock::getAngleSpeed(char type) {
        if (type == PAN) {
            if(is_stopped_count % STEPS_UNTIL_GOAL_REACHED == 0) {
                return 0.0;
            }
            else {
                return pan_speed;
            }
        }
        if(is_stopped_count % STEPS_UNTIL_GOAL_REACHED == 0) {
            return 0.0;
        }
        else {
            return tilt_speed;
        }
    }




    void PTUDriverMock::setLimitAnglesToHardwareConstraints() {
        ros::NodeHandle n("~");
        double pan_min_limit, pan_max_limit, tilt_min_limit, tilt_max_limit;
        n.getParam("mock_pan_min_hardware_limit", pan_min_limit);
        n.getParam("mock_pan_max_hardware_limit", pan_max_limit);
        n.getParam("mock_tilt_min_hardware_limit", tilt_min_limit);
        n.getParam("mock_tilt_max_hardware_limit", tilt_max_limit);

        this->pan_min_limit = pan_min_limit;
        this->pan_max_limit = pan_max_limit;
        this->tilt_min_limit = tilt_min_limit;
        this->tilt_max_limit = tilt_max_limit;

        ROS_DEBUG("pan_min_limit: %f, pan_max_limit: %f, tilt_min_limit: %f, tilt_max_limit: %f", pan_min_limit, pan_max_limit, tilt_min_limit, tilt_max_limit);

    }

    //DO NOT USE, NOT INTENDED TO USE WITH PTU DRIVER MOCK
    std::vector<double> PTUDriverMock::determineLegitEndPoint(double end_point_pan_candidate, double end_point_tilt_candidate) {
        ROS_ERROR("DO NOT USE determineLegitEndPoint WITH PTU DRIVER MOCK");
        ROS_ERROR("Maybe you enabled path_prediction in one of the launch files you use. Path prediction is not intended to be used with the mock launch files");
        std::vector<double> dummy;
        dummy.push_back(end_point_pan_candidate);
        dummy.push_back(end_point_tilt_candidate);
        return dummy;
    }

    bool PTUDriverMock::setValuesOutOfLimitsButWithinMarginToLimit(double * pan, double * tilt, double margin) {
        ROS_DEBUG("PAN: %f, TILT: %f\n", *pan, *tilt);
        if(((pan_max_limit - pan_min_limit) <= margin) || ((tilt_max_limit - tilt_min_limit) <= margin)) {
            ROS_ERROR("Margin Degree: %f is  too big. Bigger than distance between TILT max and TILT min (%f Degree) or PAN max and PAN min (%f Degree)",margin, (tilt_max_limit - tilt_min_limit), (pan_max_limit - pan_min_limit));
            return false;
        }
        double pan_initial = *pan;
        double tilt_initial = *tilt;
        if((pan_initial < pan_min_limit) && (pan_initial >= (pan_min_limit - margin))) {
            *pan = pan_min_limit;
            ROS_WARN("PAN was out of limits, but within margin, so instead of the initial value %f the value %f is used", pan_initial, *pan);
        }
        else if ((pan_initial > pan_max_limit) && (pan_initial <= (pan_max_limit + margin))) {
            *pan = pan_max_limit;
            ROS_WARN("PAN was out of limits, but within margin, so instead of the initial value %f the value %f is used", pan_initial, *pan);
        }
        if((tilt_initial < tilt_min_limit) && (tilt_initial >= (tilt_min_limit - margin))) {
            *tilt = tilt_min_limit;
            ROS_WARN("TILT was out of limits, but within margin, so instead of the initial value %f the value %f is used", tilt_initial, *tilt);
        }
        else if((tilt_initial > tilt_max_limit) && (tilt_initial <= (tilt_max_limit + margin))) {
            *tilt = tilt_max_limit;
            ROS_WARN("TILT was out of limits, but within margin, so instead of the initial value %f the value %f is used", tilt_initial, *tilt);
        }

        if(isWithinPanTiltLimits(*pan, *tilt)) {
            ROS_DEBUG("PAN: %f, TILT: %f\n", *pan, *tilt);
            return true;
        }
        else {
            return false;
        }
    }



    long PTUDriverMock::getLimitAngle(char pan_or_tilt, char upper_or_lower) {
        if(pan_or_tilt == 'p') {
            if (upper_or_lower == 'l') {
                return pan_min_limit;
            }
            else if (upper_or_lower == 'u') {
                return pan_max_limit;
            }
            else {
                return std::numeric_limits<double>::max();
            }
        }
        else if (pan_or_tilt == 't') {
            if (upper_or_lower == 'l') {
                return tilt_min_limit;
            }
            else if (upper_or_lower == 'u') {
                return tilt_max_limit;
            }
            else {
                return std::numeric_limits<double>::max();
            }
        }
        else {
            return std::numeric_limits<double>::max();
        }
    }
}
