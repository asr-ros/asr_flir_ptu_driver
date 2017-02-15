/**
 * PTUDriver class.
 *
 * @author Valerij Wittenbeck, Pascal Meissner
 * @version See SVN
 */

#ifndef PTUDRIVER_H_
#define PTUDRIVER_H_

#include <string.h>
#include <math.h>
#include "ros/ros.h"
#include <limits>
#include "PTUFree.h"
/*
extern "C" {
#include "../../cpi-v1.09.15/include/ptu.h"
}
*/
#define STOPPED_THRESHOLD 0.1


#ifdef __PTU_FREE_INCLUDED__
#define PAN 0
#define TILT 1
#define RESOLUTION 2
#define POSITION_LIMITS_MODE 3
#define OFF_MODE 4
#define BASE 5
#define ABSOLUTE 6
#define UPPER_SPEED_LIMIT 7
#define SPEED 8
#define ACCELERATION 9
#define HOLD_POWER_LEVEL 10
#define MOVE_POWER_LEVEL 11
#define MINIMUM_POSITION 12
#define MAXIMUM_POSITION 13
#define LOWER_SPEED_LIMIT 14
#define POSITION 15
#define PTU_LOW_POWER 16
#define PTU_REG_POWER 17
#define PTU_HI_POWER 18
#define PTU_PURE_VELOCITY_SPEED_CONTROL_MODE 19
#define PTU_OFF_POWER 20
#define SPEED_CONTROL_MODE 21
#define PTU_INDEPENDENT_SPEED_CONTROL_MODE 22
#define PTU_OK 23
#define PTU_NOT_OK 24
#endif


//TODO: Test Methods in pure speed control mode

namespace asr_flir_ptu_driver {
#define HOLZ

class PTUDriver {
    #define HOLZ
    public:
        /**
         * @brief PTUDriver Constructor for PTUDriver class.
         * @param port identifing string for port where ptu is plugged in
         * @param baud baud rate for port
         * @param speed_control True if PTUDriver shall be launched in pure speed control mode
         */
        PTUDriver(const char* port, int baud, bool speed_control);
        /**
         * @brief PTUDriver Do not use, needed for mock.
         */
        PTUDriver();

        /**
         * @brief ~PTUDriver Do not use, necessary because of some compiler issues.
         */
        ~PTUDriver();


        /**
         * @brief isConnected Method to check if ptu is connected.
         * @return True if connected
         */
        bool virtual isConnected();

        /**
         * @brief setSettings Method to configure various settings of the ptu
         * @param pan_base Base speed of pan axis
         * @param tilt_base Base speed of tilt axis
         * @param pan_speed Desired speed on pan axis
         * @param tilt_speed Desired speed on tilt axis
         * @param pan_upper Upper speed limit on pan axis
         * @param tilt_upper Upper speed limit on tilt axis
         * @param pan_accel Acceleration on pan axis
         * @param tilt_accel Acceleration on tilt axis
         * @param pan_hold Powerlevel of pan axis when not moving
         * @param tilt_hold Powerlevel of tilt axis when not moving
         * @param pan_move Powerlevel of pan axis when moving
         * @param tilt_move Powerlevel of tilt axis when moving
         */
        void virtual setSettings(int pan_base, int tilt_base, int pan_speed, int tilt_speed, int pan_upper, int tilt_upper,
                        int pan_accel, int tilt_accel, int pan_hold, int tilt_hold, int pan_move, int tilt_move);


        /**
         * @brief setSettings Method to configure limits for pan and tilt
         * @param pan_min Minimum pan value in degree
         * @param pan_max Maxiumum pan value in degree
         * @param tilt_min Minimum tilt value in degree
         * @param tilt_max Maximum tilt value in degree
         */
        void virtual setLimitAngles(double pan_min, double pan_max, double tilt_min, double tilt_max);

        /**
         * @brief setAbsoluteAngleSpeeds Method to set desired speed for pan/tilt movement absolute in deg/s
         * @param pan_speed Absolute desired speed for pan axis
         * @param tilt_speed Absolute desired speed for tilt axis
         */
        void virtual setAbsoluteAngleSpeeds(double pan_speed, double tilt_speed); //in deg/s

        /**
         * @brief setAbsoluteAngleSpeeds Method to set desired speed for pan/tilt movement absolute in position/s
         * @param pan_speed Absolute desired speed for pan axis
         * @param tilt_speed Absolute desired speed for pan axis
         */
        void virtual setAbsoluteAngleSpeeds(signed short pan_speed, signed short tilt_speed);

        /**
         * @brief setAbsoluteAngles Method to set pan and tilt angle for the PTU
         * @param pan_angle Angle to move the pan axis to
         * @param tilt_angle Angle to move the tilt axis to
         * @param no_forbidden_area_check True if path prediction shall be used (nearest point on path outside orbidden areas is used) or False if it shall be checked if point is inside forbidden area
         * (rejected then)
         * @return True if successfull, false otherwise (especially if no_forbidden_are_check is true and point is in forbidden area)
         */
        bool virtual setAbsoluteAngles(double pan_angle, double tilt_angle, bool no_forbidden_area_check); //in deg

        /**
         * @brief setValuesOutOfLimitsButWithinMarginToLimit This method is used to set pan/tilt values that are slighly out of pan/tilt limits (but within a margin) to these limits
         * @param pan Pan value to check if it is within pan limits and to set to limit if it is outside but within margin
         * @param tilt Tilt value to check if it is within tilt limits and to set to limit if it is outside but within margin
         * @param margin Margin for values outside pan/tilt limits
         * @return True if successfull, false if values are more out of pan/tilt limits than margin
         */
        bool virtual setValuesOutOfLimitsButWithinMarginToLimit(double * pan, double * tilt, double margin);

        /**
         * @brief isWithinPanTiltLimits Method that checks if pan and tilt value are within the chosen pan and tilt limits
         * @param pan Value for pan to be checked to be within limits
         * @param tilt Value for tilt to be checke to be within limits
         * @return True if pan and tilt are within the limits
         */
        bool virtual isWithinPanTiltLimits(double pan, double tilt);

        /**
         * @brief setLimitAnglesToHardwareConstraints Method that sets the pan and tilt limits to the maximum values that the hardware can provide.
         */
        void virtual setLimitAnglesToHardwareConstraints();

        /**
         * @brief setForbiddenAreas Method to set all forbidden areas for the PTU. Former forbidden areas will be discarded when invoking this method
         * @param forbidden_areas Vector of std::map. Each map needs to contain 4 entries specified by the strings "pan_min",  "pan_max", "tilt_min" and "tilt_max" with the corresponding values for the forbidden area
         */
        void virtual setForbiddenAreas(std::vector< std::map< std::string, double> > forbidden_areas);

        /**
         * @brief setSpeedControlMode Method to set and disable pure speed control mode
         * @param speed_control_mode True to make the PTU controllable by speed values only (no angles, only speed), false for hybrid mode (movement via angles by specified speed)
         */
        void virtual setSpeedControlMode(bool speed_control_mode);

        /**
         * @brief isInSpeedControlMode Method to determine if ptu is in pure speed control mode
         * @return True if ptu is in pure speed control mode
         */
        bool virtual isInSpeedControlMode();

        /**
         * @brief getCurrentAngle Method to get the current pan/tilt angle
         * @param type Axis to get the current angle from. PAN or TILT as value allowed.
         * @return The current angle of the chosen axis
         */
        double virtual getCurrentAngle(char type);

        /**
         * @brief getDesiredAngle Method to get the desired pan/tilt angle (where the ptu moves to)
         * @param type Axis to get the desired angle from. PAN or TILT as value allowed.
         * @return The desired angle of the chosen axis
         */
        double virtual getDesiredAngle(char type);

        /**
         * @brief getAngleSpeed Method to get the current angle speed of an axisa (deg/s)
         * @param type Axis to get the current angle speed from. PAN or TILT as value allowed.
         * @return Current angle speed of the axis (deg/s)
         */
        double virtual getAngleSpeed(char type);

        /**
         * @brief isInForbiddenArea method to determine if a pan + tilt value pair lies within a forbidden area.
         * @param pan_angle Pan angle to check
         * @param tilt_angle Tilt angle to check
         * @return True if pan + tilt position lies in forbidden area
         */
        bool virtual isInForbiddenArea(double pan_angle, double tilt_angle);

        /**
         * @brief setComputationTolerance Sets the tolerance value for errors due to the imprecise nature of float, double, sqrt, ... computations (default is 0.00005 on startup)
         * @param computation_tolerance Tolerance for calculation errors on floating-point numbers
         */
        void virtual setComputationTolerance(double computation_tolerance);

        /**
         * @brief setDistanceFactor Method to set the factor which determines how much samples are going to be taken for path prediction. The higher the distance factor is the less samples get taken (the maximum
         * time for changing a single position (step on step motor) is taken and scaled by distance factor. This is the time between two samples). Default is 5.
         * @param distance_factor Factor for scaling the time period between two samples of the path prediction. Default 5. Higher Value -> Less Samples. Decrease to boost precision, Increase to boost performance.
         */
        void virtual setDistanceFactor(long distance_factor);

        //!!!!!!!!!!Müssen nachfolgende public sein? Nach erstellen von mock überprüfen!!!!!!!!!!!


        /**
         * @brief getLimitAngle Method to get one of the limit angles for pan/tilt movement
         * @param pan_or_tilt Value to specify the axes from which you want the limit angle. Can be 'p' for pan or 't' for tilt
         * @param upper_or_lower Value to specifie if you want the upper or lower limit on specified axis. Can be 'l' for lower and 'u' for upper limit.
         * @return Desired limit angle; in error case (none legit input) maximum value for double is returned
         */
        long virtual getLimitAngle(char pan_or_tilt, char upper_or_lower);

        /**
         * @brief hasHaltedAndReachedGoal Method to determine if the PTU has halted and reached its goal
         * @return True if halted and reached goal
         */
        bool virtual hasHaltedAndReachedGoal();

        /**
         * @brief hasHalted Method to determine if PTU movement has stopped
         * @return True if halted
         */
        bool virtual hasHalted();

        /**
         * @brief reachedGoal Method to determine if the PTU has reached its goal.
         * @return True if halted
         */
        bool virtual reachedGoal();

        /**
         * @brief determineLegitEndPoint Method to determine an end point which is not inside a forbidden area using path prediction to get the point where the path would hit the first forbidden area. ASSUMES that the point is within pan-tilt-limits.
         * @param end_point_pan_candidate Pan value of desired end point
         * @param end_point_tilt_candidate Tilt value of desired end point
         * @return Vector with nearest legit end point on the path to the desired end point. Pan at [0], Tilt at [1].
         */
        std::vector<double> virtual determineLegitEndPoint(double end_point_pan_candidate, double end_point_tilt_candidate);



   protected:

        std::string virtual getErrorString(char status_code);
        std::vector< std::map< std::string, double> > forbidden_areas;
        long pan_min, pan_max;
        long tilt_min, tilt_max;

        long prefetched_pan_current_base;
        long prefetched_pan_desired_acceleration;
        long prefetched_pan_desired_speed;
        long prefetched_tilt_current_base;
        long prefetched_tilt_desired_acceleration;
        long prefetched_tilt_desired_speed;
        long prefetched_pan_current_position;
        long prefetched_tilt_current_position;
        double pan_resolution, tilt_resolution;
    private:
        void setValuesToBackupValues(int & pan_base, int & tilt_base, int & pan_speed, int & tilt_speed,
                        int & pan_upper, int & tilt_upper, int & pan_accel, int & tilt_accel,
                        int & pan_hold, int & tilt_hold, int & pan_move, int & tilt_move);
        void createSettingsBackup();
        bool checkReturnCode(char return_code);
        void restoreSettingsFromBackup();
        void prefetchValues();

        double double_computation_tolerance;
        bool speed_control;
        static short int POW_VAL_MOVE[3];
        static short int POW_VAL_HOLD[3];
        std::map<std::string,int> backup_settings;
        long convertPanFromAngleToPosition(double angle);
        double convertPanFromPositionToAngle(long position);
        long convertTiltFromAngleToPosition(double angle);
        double convertTiltFromPositionToAngle(long position);
        double pan_acceleration_time;
        double pan_slew_speed_time;
        double tilt_acceleration_time;
        double tilt_slew_speed_time;
        std::vector<std::vector<double> > forbidden_area_first_line_coordinate_forms;
        std::vector<std::vector<double> > forbidden_area_second_line_coordinate_forms;
        std::vector<std::vector<double> > forbidden_area_third_line_coordinate_forms;
        std::vector<std::vector<double> > forbidden_area_fourth_line_coordinate_forms;

        std::vector<std::vector<double> > max_pan_max_tilt_points;
        std::vector<std::vector<double> > max_pan_min_tilt_points;
        std::vector<std::vector<double> > min_pan_max_tilt_points;
        std::vector<std::vector<double> > min_pan_min_tilt_points;

        long backup_pan_base;
        long backup_pan_upper;
        long backup_pan_speed;
        long backup_pan_accel;
        long backup_pan_hold;
        long backup_pan_move;
        long backup_tilt_base;
        long backup_tilt_upper;
        long backup_tilt_speed;
        long backup_tilt_accel;
        long backup_tilt_hold;
        long backup_tilt_move;


        std::vector<double> solveSecondDegreePolynomial(double a, double b, double c);
        std::vector<double> getAccelerationTimeAndSlewSpeedTime(double distance_in_steps, double base_speed, double acceleration, double slew_speed);
        std::vector<double> predictPositionInTime(std::vector<double> start_point, std::vector<double> end_point, double point_in_time);
        double calculateCoveredDistance(double acceleration_time, double slew_speed_time, double decceleration_time, bool is_pan);
        std::vector<double> calculatePointOfIntersectionWithForbiddenAreas(std::vector<double> start_point, std::vector<double> end_point);
        std::vector<double> calculateIntersectionPoint(std::vector<double> first_line_coordiante_form, std::vector<double> second_line_coordiante_form);
        bool isOnLineSegmentBetweenTwoPoints(std::vector<double> start_point, std::vector<double> end_point, std::vector<double> line_coordinate_form, std::vector<double> point_to_check, double tolerance);
        std::vector<double> calculateCoordinateForm(std::vector<double> start_point, std::vector<double> end_point);
        double getVectorLength(std::vector<double> input_vector);
        double getVectorLength(std::vector<double> start_point, std::vector<double> end_point);
        std::vector<double> checkForPossibleKollision(double new_pan_angle, double new_tilt_angle);
        double distance_factor;
        void precalculateForbiddenAreaCoodinateForms();

        #ifndef __PTU_FREE_INCLUDED__
        portstream_fd COMstream;
        #endif

        #ifdef __PTU_FREE_INCLUDED__
        long get_current(char pan_or_tilt, char what);
        char set_desired(char pan_or_tilt, char what, short int * value, char type);
        long get_desired(char pan_or_tilt, char what);
        char set_mode(char mode_type, char mode);
        ptu_free::PTUFree free_ptu;
        #endif




};

}

#endif /* PTU46DRIVER_H_ */
