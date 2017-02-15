#ifndef PTUDRIVERMOCK_H
#define PTUDRIVERMOCK_H
#include "driver/PTUDriver.h"

namespace asr_flir_ptu_driver {

class PTUDriverMock : public PTUDriver{
    public:
        PTUDriverMock(const char* port, int baud, bool speed_control);
        ~PTUDriverMock();

        bool isConnected();

        void virtual setSettings(int pan_base, int tilt_base,
                                 int pan_speed, int tilt_speed,
                                 int pan_upper, int tilt_upper,
                                 int pan_accel, int tilt_accel,
                                 int pan_hold, int tilt_hold,
                                 int pan_move, int tilt_move);

        void virtual setLimitAngles(double pan_min, double pan_max, double tilt_min, double tilt_max);

        //dynamic settings
        void virtual setAbsoluteAngleSpeeds(double pan_speed, double tilt_speed); //in deg/s

        /**
         * @brief determineLegitEndPoint DO NOT USE, NOT SUPPORTED IN MOCK
         * @param end_point_pan_candidate -
         * @param end_point_tilt_candidate -
         */
        void virtual setAbsoluteAngleSpeeds(signed short pan_speed, signed short tilt_speed); //in angle seconds
        bool virtual setAbsoluteAngles(double pan_angle, double tilt_angle, bool no_forbidden_area_check); //in deg

        void virtual setSpeedControlMode(bool speed_control_mode);
        bool virtual isInSpeedControlMode();

        double virtual getCurrentAngle(char type);
        double virtual getDesiredAngle(char type);
        double virtual getAngleSpeed(char type);

        bool virtual hasHalted();
        bool virtual reachedGoal();
        bool virtual hasHaltedAndReachedGoal();
        bool virtual isWithinPanTiltLimits(double pan, double tilt);

        void virtual setLimitAnglesToHardwareConstraints();

        /**
         * @brief determineLegitEndPoint DO NOT USE, NOT SUPPORTED IN MOCK
         * @param end_point_pan_candidate -
         * @param end_point_tilt_candidate -
         * @return -
         */
        std::vector<double> virtual determineLegitEndPoint(double end_point_pan_candidate, double end_point_tilt_candidate);

        bool virtual setValuesOutOfLimitsButWithinMarginToLimit(double * pan, double * tilt, double margin);
        long virtual getLimitAngle(char pan_or_tilt, char upper_or_lower);

   private:
        int is_stopped_count;
        double pan_angle;
        double tilt_angle;
        double pan_speed;
        double tilt_speed;
        double current_pan_speed;
        double current_tilt_speed;
        double pan_base;
        double tilt_base;
        double pan_acceleration;
        double tilt_acceleration;
        double pan_max_limit;
        double pan_min_limit;
        double tilt_max_limit;
        double tilt_min_limit;

        double pan_distance;
        double tilt_distance;
};

}
#endif // PTUDRIVERMOCK_H
