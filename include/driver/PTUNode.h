#ifndef PTUNODE_H
#define PTUNODE_H
/**
 * PTUNode class.
 *
 * @author Valerij Wittenbeck, Pascal Meissner
 * @version See SVN
 */

#include <math.h>
#include <string.h>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include "asr_flir_ptu_driver/Validate.h"
#include "driver/PTUDriver.h"
#include "driver/PTUDriverMock.h"
#include "asr_flir_ptu_driver/State.h"
#include "asr_flir_ptu_driver/Predict.h"

//Test porpouse only
#include "PTUFree.h"

class PTUNode {
    public:
        /**
         * @brief PTUNode Constructor for PTUNode class
         * @param n noehandle to use
         */
        PTUNode(ros::NodeHandle& n);
        ~PTUNode();
        /**
         * @brief ok Method to check if the PTUNode is still working
         * @return True if PTUNode stil works.
         */
        bool ok();
        /**
         * @brief spinOnce Method that is used to spin the PTUNode once, including ros::spinOnce(). Run this in a loop.
         */
        void spinOnce();


    private:
        void setState(const asr_flir_ptu_driver::State::ConstPtr& msg);
        bool updateSettings(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
        bool updateSpeedControl(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

        bool setSettings();
        bool setSpeedControl();
        bool validatePanTilt(asr_flir_ptu_driver::Validate::Request &req, asr_flir_ptu_driver::Validate::Response &res);
        bool emptyAlive(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool predict(asr_flir_ptu_driver::Predict::Request &req, asr_flir_ptu_driver::Predict::Response &res);
        asr_flir_ptu_driver::PTUDriver* ptu;
        ros::NodeHandle node_handle;

        ros::Subscriber joint_sub;
        ros::Publisher joint_pub;
	ros::Publisher joint_pub_old;
        ros::ServiceServer validate_service;
        ros::ServiceServer alive_service;
        ros::ServiceServer settings_service;
        ros::ServiceServer speedmode_service;
        ros::ServiceServer path_prediction_service;
        bool goal_not_reached;
        int seq_num;
	
	std::string ptu_topic_state;

        // broadcasters for PTU coordinate frame
        tf::TransformBroadcaster tb_pan;
        tf::TransformBroadcaster tb_tilt;

        // descriptors for PTU coordinate frames
        std::string ptu_pan_frame;
        std::string ptu_pan_frame_rotated;
        std::string ptu_tilt_frame;
        std::string ptu_tilt_frame_rotated;

        //ROS topics
        std::string topic_state_command;
        std::string topic_state;

        //ROS services
        std::string service_settings_update;
        std::string service_speed_control_update;
        std::string service_range;

        // used for conversion between radian and degrees
        static const double DEG_TO_RAD = M_PI / 180.0;
};

#endif // PTUNODE_H
