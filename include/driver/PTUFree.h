#ifndef __PTU_FREE_INCLUDED__
#define __PTU_FREE_INCLUDED__
//#include <serial/serial.h>
//#include "ros/ros.h"
//#include <boost/asio/io_service.hpp>
//#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include "limits.h"

#define ERROR LONG_MIN
#define DISABLED ( LONG_MIN + 1 )
#define ENABELED ( LONG_MIN + 2 )
#define FACTORY_LIMITS_ENABLED ( LONG_MIN + 3 )
#define USER_DEFINED_LIMITS_ENABLED ( LONG_MIN + 4 )
#define LIMITS_DISABLED ( LONG_MIN + 5 )
#define USER_DEFINED_PAN_LIMIT_ENABLED ( LONG_MIN + 6 )
#define IMMEDIATE_POSITION_EXECUTION_MODE ( LONG_MIN + 7 )
#define SLAVED_POSITION_EXECUTION_MODE ( LONG_MIN + 8 )
#define HALT_PAN_ONLY ( LONG_MIN + 9 )
#define HALT_TILT_ONLY ( LONG_MIN + 10 )
#define HALT_BOTH ( LONG_MIN + 11 )
#define INDEPENDENT_SPEED_MODE ( LONG_MIN + 12 )
#define PURE_VELOCITY_CONTROL_MODE ( LONG_MIN + 13 )
#define NO_RESET_MODE ( LONG_MIN + 14 )
#define PAN_ONLY_RESET_MODE ( LONG_MIN + 15 )
#define TILT_ONLY_RESET_MODE ( LONG_MIN + 16 )
#define BOTH_RESET_MODE ( LONG_MIN + 17 )
#define REGULAR_HOLD_POWER_MODE ( LONG_MIN + 18 )
#define LOW_HOLD_POWER_MODE ( LONG_MIN + 19 )
#define OFF_HOLD_POWER_MODE ( LONG_MIN + 20 )
#define HIGH_MOVE_POWER_MODE ( LONG_MIN + 21 )
#define REGULAR_MOVE_POWER_MODE ( LONG_MIN + 22 )
#define LOW_MOVE_POWER_MODE ( LONG_MIN + 23 )

#define ERROR_CODES_BELOW ( LONG_MIN + 24)




namespace ptu_free {
    /**
    *   A class for accessing flir ptu via serial port
    */
    class PTUFree {
        public:

            /**
             * @brief PTUFree Constructor for PTUFree object. Not connected to any port.
             */
            PTUFree();

            /**
             * @brief PTUFree Constructor for PTUFree object with existing io_service.
             * @param io Existing io_service object to be used for communication by PTUFree
             */
            PTUFree(boost::asio::io_service& io);

            /**
             * @brief setBaudRate Sets the baud rate for the serial port that is used. Only use of serial connection is established
             * @param baud Baud rate for serial port
             * @return True if setting was successfull
             */
            bool setBaudRate(int baud);

            /**
             * @brief setNewSerialConnection Establishes a new connection via serial port specified by 'port' to a target device using baud rate 'baud'. The ptu is set up with IMMEDIATE_POSITION_EXECUTION_MODE and FACTORY_LIMITS_ENABLED.
             * @param port identifier of serial port that will be used
             * @param baud baud rate for serial port
             * @return True if establishing a connection was successfull
             */
            bool setNewSerialConnection(std::string port, int baud);

            /**
             * @brief closeSerialConnection Closes currently used serial port
             */
            void closeSerialConnection();

            /**
             * @brief communicate Method to send a command 'request' to the serial port (and the device) and recieve the answer
             * @param request Command to execute on device connected via serial port
             * @return Execution result message
             */
            std::string communicate(std::string request);

            /**
             * @brief readPTUResponse Method that reads from the PTU until delimitor ('\n') appears. Then the read data is returned.
             * @return Data from serial port until next delimitor ('\n').
             */
            std::string readPTUResponse();

            /**
             * @brief evaluateResponse Method that preprocesses the answer of the PTU. Splits the PTU to a maximum of 3 parts. At position 0 is '*' or '!' (successfull or failed command),
             * at position 1 is the full return message without the qualifier at start (might be "", for example if a set command only was performed) and at 2 (optional) there is the number only
             * if the response contained one (e.g. for casts)
             * @param response full ptu response string
             * @return string "split" into 3 parts. [0] is prefix for success('*') or failure ('!'), [1] is the full message without prefix, [2] (optional) is number only if one is contained (e.g. for cast to int/long/...)
             */
            std::vector<std::string> evaluateResponse(std::string response);

            /**
             * @brief setDesiredPanPositionAbsolute Method that sets absolute pan position
             * @param position absolute pan position ptu shall move to
             * @return True if successfull
             */
            bool setDesiredPanPositionAbsolute(short int position);

            /**
             * @brief setDesiredPanPositionRelative Method that sets current pan position by offset (current pos + offset = new position)
             * @param position_offset Offset of current position to be added for new pan position
             * @return True if successfull
             */
            bool setDesiredPanPositionRelative(short int position_offset);

            /**
             * @brief setDesiredPanSpeedAbsolute Method that sets absolute pan speed (position/second)
             * @param speed absolute value in position/second for pan speed
             * @return True if successfull
             */
            bool setDesiredPanSpeedAbsolute(short int speed);

            /**
             * @brief setDesiredPanSpeedRelative Method that sets the desired pan speed relative to the CURRENT (not the desired!) speed with an offset (position/second)
             * @param speed_offset offset in position/second from CURRENT speed to set new desired speed
             * @return True if successfull
             */
            bool setDesiredPanSpeedRelative(short int speed_offset);
            /**
             * @brief setDesiredPanAccelerationAbsolute Method that sets the absolute pan acceleration (position/second^2)
             * @param acceleration value in position/second^2 for pan acceleration
             * @return True if successfull
             */
            bool setDesiredPanAccelerationAbsolute(short int acceleration);
            /**
             * @brief setDesiredPanUpperSpeedLimit Method that sets the upper speed limit for pan (position/second). WARNING: Takes extremly long or does not work on every ptu
             * @param upper_speed_limit value for pan upper speed limit in position/second
             * @return True if successfull
             */
            bool setDesiredPanUpperSpeedLimit(short int upper_speed_limit);
            /**
             * @brief setDesiredPanLowerSpeedLimit Method that sets the lower speed limit for pan (position/second). WARNING: Takes extremly long or does not work on every ptu
             * @param lower_speed_limit value for pan lower speed limit in position/second. MUST BE 0 or >= 57 (second value depending on used ptu)
             * @return True if successfull
             */
            bool setDesiredPanLowerSpeedLimit(short int lower_speed_limit);
            /**
             * @brief setPanBaseSpeed Method that sets the base speed for pan (position/second)
             * @param base_speed value for pan base speed in position/second
             * @return True if successfull
             */
            bool setPanBaseSpeed(short int base_speed);
            /**
             * @brief getCurrentPanPosition Method that queries the current pan position
             * @return pan position
             */
            long getCurrentPanPosition();
            /**
             * @brief getCurrentPanSpeed Method that queries the current pan speed
             * @return pan speed
             */
            long getCurrentPanSpeed();
            /**
             * @brief getPanUpperSpeedLimit Method that queries the pan upper speed limit WARNING: This Method consumes either extreme amounts of time to execute or does not work properly with all PTU
             * @return pan upper speed limit in position/second
             */
            long getPanUpperSpeedLimit();
            /**
             * @brief getPanLowerSpeedLimit Method that queries the pan lower speed limit. WARNING: This Method consumes either extreme amounts of time to execute or does not work properly with all PTU
             * @return pan lower speed limit in position/second
             */
            long getPanLowerSpeedLimit();
            /**
             * @brief getPanResolution Method that queries the pan resolution (seconds/arc per position). Divide by 3600 to get Degree.
             * @return seconds/arc per position of step motor on pan axis
             */
            double getPanResolution();
            /**
             * @brief getDesiredPanPosition Method that queries the desired pan position
             * @return desired pan position
             */
            long getDesiredPanPosition();
            /**
             * @brief getDesiredPanSpeed Method that queries the desired pan speed
             * @return desired pan speed
             */
            long getDesiredPanSpeed();
            /**
             * @brief getPanAcceleartion Method that queries the pan acceleration (no current or desired here)
             * @return pan acceleration
             */
            long getPanAcceleartion();
            /**
             * @brief getPanBaseSpeed Returns the current pan base speed
             * @return pan base speed
             */
            long getPanBaseSpeed();
            /**
             * @brief setDesiredTiltPositionAbsolute Method that sets absolute tilt position
             * @param position absolute tilt position ptu shall move to
             * @return True if successfull
             */
            bool setDesiredTiltPositionAbsolute(short int position);

            /**
             * @brief setDesiredTiltPositionRelative Method that sets current tilt position by offset (current pos + offset = new position)
             * @param position_offset Offset of current position to be added for new tilt position
             * @return True if successfull
             */
            bool setDesiredTiltPositionRelative(short int position_offset);

            /**
             * @brief setDesiredTiltSpeedAbsolute Method that sets absolute tilt speed (position/second)
             * @param speed absolute value in position/second for tilt speed
             * @return True if successfull
             */
            bool setDesiredTiltSpeedAbsolute(short int speed);

            /**
             * @brief setDesiredTiltSpeedRelative Method that sets the desired tilt speed relative to the CURRENT (not the desired!) speed with an offset (position/second)
             * @param speed_offset offset in position/second from CURRENT speed to set new desired speed
             * @return True if successfull
             */
            bool setDesiredTiltSpeedRelative(short int speed_offset);
            /**
             * @brief setDesiredTiltAccelerationAbsolute Method that sets the absolute tilt acceleration (position/second^2)
             * @param acceleration value in position/second^2 for tilt acceleration
             * @return True if successfull
             */
            bool setDesiredTiltAccelerationAbsolute(short int acceleration);
            /**
             * @brief setDesiredTiltUpperSpeedLimit Method that sets the upper speed limit for tilt (position/second). WARNING: Takes extremly long or does not work on every ptu
             * @param upper_speed_limit value for tilt upper speed limit in position/second
             * @return True if successfull
             */
            bool setDesiredTiltUpperSpeedLimit(short int upper_speed_limit);
            /**
             * @brief setDesiredTiltLowerSpeedLimit Method that sets the lower speed limit for tilt (position/second). WARNING: Takes extremly long or does not work on every ptu
             * @param lower_speed_limit value for tilt lower speed limit in position/second. MUST BE 0 or >= 57 (second value depending on used ptu)
             * @return True if successfull
             */
            bool setDesiredTiltLowerSpeedLimit(short int lower_speed_limit);
            /**
             * @brief setTiltBaseSpeed Method that sets the base speed for tilt (position/second)
             * @param base_speed value for tilt base speed in position/second
             * @return True if successfull
             */
            bool setTiltBaseSpeed(short int base_speed);
            /**
             * @brief getCurrentTiltPosition Method that queries the current tilt position
             * @return tilt position
             */
            long getCurrentTiltPosition();
            /**
             * @brief getCurrentTiltSpeed Method that queries the current tilt speed
             * @return tilt speed
             */
            long getCurrentTiltSpeed();
            /**
             * @brief getTiltUpperSpeedLimit Method that queries the tilt upper speed limit WARNING: This Method consumes either extreme amounts of time to execute or does not work properly with all PTU
             * @return tilt upper speed limit in position/second
             */
            long getTiltUpperSpeedLimit();
            /**
             * @brief getTiltLowerSpeedLimit Method that queries the tilt lower speed limit. WARNING: This Method consumes either extreme amounts of time to execute or does not work properly with all PTU
             * @return tilt lower speed limit in position/second
             */
            long getTiltLowerSpeedLimit();
            /**
             * @brief getTiltResolution Method that queries the tilt resolution (seconds/arc per position). Divide by 3600 to get Degree.
             * @return seconds/arc per position of step motor on tilt axis
             */
            double getTiltResolution();
            /**
             * @brief getDesiredTiltPosition Method that queries the desired tilt position
             * @return desired tilt position
             */
            long getDesiredTiltPosition();
            /**
             * @brief getDesiredTiltSpeed Method that queries the desired tilt speed
             * @return desired tilt speed
             */
            long getDesiredTiltSpeed();
            /**
             * @brief getTiltAcceleartion Method that queries the tilt acceleration (no current or desired here)
             * @return tilt acceleration
             */
            long getTiltAcceleartion();
            /**
             * @brief getTiltBaseSpeed Returns the current tilt base speed
             * @return tilt base speed
             */
            long getTiltBaseSpeed();

            /**
             * @brief setPositionLimitEnforcementMode Method to set the position limit enforcement mode. Warning: USER_DEFINED_LIMITS_ENABLED does not work on older PTUs and setting of this value is not tested.
             * @param enable Type of position limit enforcement to enable. Possible values are FACTORY_LIMITS_ENABLED, USER_DEFINED_LIMITS_ENABLED (warning: does not work on older PTU's) and LIMITS_DISABLED
             * @return True if successfull
             */
            bool setPositionLimitEnforcementMode(long enable);

            /**
             * @brief setMinimumPanPositionLimit Method that is used to set a user defined minimum pan position limit. WARNING: Does not work on older PTUs and is not tested.
             * @param position Value for the minimum pan position allowed
             * @return True if successfull
             */
            bool setMinimumPanPositionLimit(short int position);

            /**
             * @brief setMaximumPanPositionLimit Method that is used to set a user defined maximum pan position limit. WARNING: Does not work on older PTUs and is not tested.
             * @param position Value for the maximum pan position allowed
             * @return True if successfull
             */
            bool setMaximumPanPositionLimit(short int position);

            /**
             * @brief setMinimumTiltPositionLimit Method that is used to set a user defined minimum tilt position limit. WARNING: Does not work on older PTUs and is not tested.
             * @param position Value for the minimum tilt position allowed
             * @return True if successfull
             */
            bool setMinimumTiltPositionLimit(short int position);

            /**
             * @brief setMaximumTiltPositionLimit Method that is used to set a user defined maximum tilt position limit. WARNING: Does not work on older PTUs and is not tested.
             * @param position Value for the maximum tilt position allowed
             * @return True if successfull
             */
            bool setMaximumTiltPositionLimit(short int position);

            /**
             * @brief getUserMinimumPanPositionLimit Method to get the user defined minimum pan position. WARNING: Does not work on older PTUs and is not tested.
             * @return Minimum pan position defined by user
             */
            long getUserMinimumPanPositionLimit();

            /**
             * @brief getUserMaximumPanPositionLimit Method to get the user defined maximum pan position. WARNING: Does not work on older PTUs and is not tested.
             * @return Maximum pan position defined by user
             */
            long getUserMaximumPanPositionLimit();

            /**
             * @brief getUserMinimumTiltPositionLimit Method to get the user defined minimum tilt position. WARNING: Does not work on older PTUs and is not tested.
             * @return Minimum tilt position defined by user
             */
            long getUserMinimumTiltPositionLimit();

            /**
             * @brief getUserMaximumTiltPositionLimit Method to get the user defined maximum tilt position. WARNING: Does not work on older PTUs and is not tested.
             * @return Maximum tilt position defined by user
             */
            long getUserMaximumTiltPositionLimit();

            /**
             * @brief getFactoryMinimumPanPositionLimit Method to get the factory defined minimum pan position.
             * @return Minimum pan position defined by factory default
             */
            long getFactoryMinimumPanPositionLimit();

            /**
             * @brief getFactoryMaximumPanPositionLimit Method to get the factory defined maximum pan position.
             * @return Maximum pan position defined by factory default
             */
            long getFactoryMaximumPanPositionLimit();

            /**
             * @brief getFactoryMinimumTiltPositionLimit Method to get the factory defined minimum tilt position.
             * @return Minimum tilt position defined by factory default
             */
            long getFactoryMinimumTiltPositionLimit();

            /**
             * @brief getFactoryMaximumTiltPositionLimit Method to get the factory defined maximum tilt position.
             * @return Maximum tilt position defined by factory default
             */
            long getFactoryMaximumTiltPositionLimit();

            /**
             * @brief getCurrentUsedMinimumPanPositionLimit Method to get the currently used minimum pan position.
             * @return Minimum pan position being currently used
             */
            long getCurrentUsedMinimumPanPositionLimit();

            /**
             * @brief getCurrentUsedMaximumPanPositionLimit Method to get the currently used maximum pan position.
             * @return Maximum pan position being currently used
             */
            long getCurrentUsedMaximumPanPositionLimit();

            /**
             * @brief getCurrentUsedMinimumTiltPositionLimit Method to get the currently used minimum tilt position.
             * @return Minimum tilt position being currently used
             */
            long getCurrentUsedMinimumTiltPositionLimit();

            /**
             * @brief getCurrentUsedMaximumTiltPositionLimit Method to get the currently used maximum tilt position.
             * @return Maximum tilt position being currently used
             */
            long getCurrentUsedMaximumTiltPositionLimit();

            /**
             * @brief getPositionLimitEnforcementMode Method to get the currently used position limit enforcement mode. Can return FACTORY_LIMITS_ENABLED, USER_DEFINED_LIMITS_ENABLED or FACTORY_LIMITS_ENABLED.
             * @return Currently used position limit enforcement mode
             */
            long getPositionLimitEnforcementMode();

            /**
             * @brief setPositionExecutionMode Method to set the position execution mode.
             * @param mode Position execution mode. Can be IMMEDIATE_POSITION_EXECUTION_MODE or SLAVED_POSITION_EXECUTION_MODE.
             * @return True if successfull
             */
            bool setPositionExecutionMode(long mode);

            /**
             * @brief getPositionExecutionMode Method to get the currently used position execution mode. Can return IMMEDIATE_POSITION_EXECUTION_MODE or SLAVED_POSITION_EXECUTION_MODE.
             * @return current position execution mode, can be IMMEDIATE_POSITION_EXECUTION_MODE or SLAVED_POSITION_EXECUTION_MODE
             */
            long getPositionExecutionMode();

            /**
             * @brief awaitPositionCommandCompletion Method to wait for the completion of the last issued pan and tilt position command (waits until ptu stops).
             * @return True if successfull
             */
            bool awaitPositionCommandCompletion();

            /**
             * @brief halt Method that halts movement on specified axis.
             * @param axis Axis to halt, can be PAN or TILT
             * @return True if successfull
             */
            bool halt(long axis);

            /**
             * @brief setDesiredPanTiltPositionAbsoluteSlaved Method that allows a movement of pan and tilt axis to specified absolute position where both start at the same time. This method waits until movement is completed.
             * @param pan Absolute position for pan
             * @param tilt Absolute position for tilt
             * @return True if successfull
             */
            bool setDesiredPanTiltPositionAbsoluteSlaved(short int pan, short int tilt);

            /**
             * @brief setPreset Method that allows to associate a pan and tilt position with a preset index. Moves to the pan and tilt position and saves the position as a preset. WARNING: Does not work on older PTUs and is not tested.
             * @param preset_index Number to identify preset. Must be between 1 and 32
             * @param pan Absolute position for pan for preset
             * @param tilt Absolute position for tilt for preset
             * @return True if successfull
             */
            bool setPreset(int preset_index, short int pan, short int tilt);

            /**
             * @brief setPreset Method that allows to associate a pan and tilt position with a preset index. Saves the current pan and tilt position as a preset. WARNING: Does not work on older PTUs and is not tested.
             * @param preset_index Number to identify preset. Must be between 1 and 32
             * @return True if successfull
             */
            bool setPreset(int preset_index);

            /**
             * @brief gotoPreset Method that moves the ptu to a existing preset of pan and tilt corrdiantes. WARNING: Does not work on older PTUs and is not tested.
             * @param preset_index Number to identify preset. Must be between 1 and 32
             * @return True if successfull
             */
            bool gotoPreset(int preset_index);

            /**
             * @brief clearPreset Method to delete all existing presets. WARNING: Does not work on older PTUs and is not tested.
             * @return True if successfull
             */
            bool clearPreset();

            /**
             * @brief setSpeedControlMode Method to set the sped control mode of the ptu.
             * @param mode Speed control mode to set, possible values are INDEPENDENT_SPEED_MODE or PURE_VELOCITY_CONTROL_MODE
             * @return True if successfull
             */
            bool setSpeedControlMode(long mode);

            /**
             * @brief getSpeedControlMode Method to get the currently used sped control mode of the ptu.
             * @return Used speed control mode. Can be INDEPENDENT_SPEED_MODE or PURE_VELOCITY_CONTROL_MODE.
             */
            long getSpeedControlMode();

            /**
             * @brief reset Method to reset the ptu (pan and/or tilt axis depending on reset mode)
             * @return True if successfull
             */
            bool reset();

            /**
             * @brief setResetMode Method to set the reset mode of the ptu. Saved to EEPROM, do not use too often. WARNING: Not tested.
             * @param mode Ptu reset mode. Possibles Values are NO_RESET_MODE, PAN_ONLY_RESET_MODE, TILT_ONLY_RESET_MODE or BOTH_RESET_MODE
             * @return True if successfull
             */
            bool setResetMode(long mode);

            /**
             * @brief saveDefault Method to save the current axis settings as default at power up. Note: This class sets the ptu by invoking setNewSerialConnection to some standard values. To get the values saved
                by this method invoke restore_defaults after setting the ptu connection. Saved to EEPROM, do not use too often. WARNING: Not tested.
             * @return True if successfull
             */
            bool saveDefault();

            /**
             * @brief restoreDefault Method to restore default settings. WARNING: Not tested.
             * @return True if successfull
             */
            bool restoreDefault();

            /**
             * @brief restoreFactoryDefault Method to set default settings to factory defaults. Saved to EEPROM, do not use too often. WARNING: Not tested.
             * @return True if successfull
             */
            bool restoreFactoryDefault();

            /**
             * @brief setPanStationaryPowerMode Method to set the stationary power mode for pan axis.
             * @param mode Stationary power mode for pan axis, can be REGULAR_HOLD_POWER_MODE, LOW_HOLD_POWER_MODE or OFF_HOLD_POWER_MODE
             * @return True if successfull
             */
            bool setPanStationaryPowerMode(long mode);

            /**
             * @brief setTiltStationaryPowerMode Method to set the stationary power mode for tilt axis.
             * @param mode Stationary power mode for tilt axis, can be REGULAR_HOLD_POWER_MODE, LOW_HOLD_POWER_MODE or OFF_HOLD_POWER_MODE
             * @return True if successfull
             */
            bool setTiltStationaryPowerMode(long mode);

            /**
             * @brief getPanStationaryPowerMode Method to get the stationary power mode for pan axis.
             * @return Stationary power mode for pan axis, can be REGULAR_HOLD_POWER_MODE, LOW_HOLD_POWER_MODE or OFF_HOLD_POWER_MODE
             */
            long getPanStationaryPowerMode();

            /**
             * @brief getTiltStationaryPowerMode Method to get the stationary power mode for tilt axis.
             * @return Stationary power mode for tilt axis, can be REGULAR_HOLD_POWER_MODE, LOW_HOLD_POWER_MODE or OFF_HOLD_POWER_MODE
             */
            long getTiltStationaryPowerMode();

            /**
             * @brief setPanInMotionPowerMode Method to set the move power mode for pan axis.
             * @param mode Move power mode for pan axis, can be REGULAR_MOVE_POWER_MODE, LOW_MOVE_POWER_MODE or HIGH_MOVE_POWER_MODE
             * @return True if successfull
             */
            bool setPanInMotionPowerMode(long mode);

            /**
             * @brief setTiltInMotionPowerMode Method to set the move power mode for tilt axis.
             * @param mode Move power mode for tilt axis, can be REGULAR_MOVE_POWER_MODE, LOW_MOVE_POWER_MODE or HIGH_MOVE_POWER_MODE
             * @return True if successfull
             */
            bool setTiltInMotionPowerMode(long mode);

            /**
             * @brief getPanInMotionPowerMode Method to get the move power mode for pan axis.
             * @return Move power mode for pan axis, can be REGULAR_MOVE_POWER_MODE, LOW_MOVE_POWER_MODE or HIGH_MOVE_POWER_MODE
             */
            long getPanInMotionPowerMode();

            /**
             * @brief getTiltInMotionPowerMode Method to get the move power mode for tilt axis.
             * @return Move power mode for tilt axis, can be REGULAR_MOVE_POWER_MODE, LOW_MOVE_POWER_MODE or HIGH_MOVE_POWER_MODE
             */
            long getTiltInMotionPowerMode();

            /**
             * @brief test Method to test the methods of this program, for DEBUG/DEVELOPMENT purpose, changes PTU settings for test purpose (restart before normal use)
             */
            void test();

            /**
             * @brief isOpen Method to determine if used port is open or closed.
             * @return True if open
             */
            bool isOpen();

        private:
            boost::asio::io_service ptu_io_service;
            boost::asio::io_service timer_io_service;
            boost::asio::serial_port ptu_port;
            bool timeout_occured;
            long factory_pan_min;
            long factory_pan_max;
            long factory_tilt_min;
            long factory_tilt_max;
            long position_execution_mode;
            std::string getErrorString(boost::system::error_code error);
        };
}
#endif
