#pragma once

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <asr_flir_ptu_controller/PTUMovementAction.h>
#include "asr_flir_ptu_driver/Validate.h"
#include <std_srvs/Empty.h>
#include "asr_flir_ptu_driver/State.h"
namespace asr_flir_ptu_controller
{
    class PTUController
    {
        public:
            PTUController(ros::NodeHandle& n, std::string name);
            ~PTUController();
            double getMaximumPan();
            double getMinimumPan();

            double getMaximumTilt();
            double getMinimumTilt();

            double getCurrentPan();
            double getCurrentTilt();

            std::string getDefaultStateCmdTopicName();
            std::string getDefaultStateTopicName();
            double getTimeToWait();
            double getToleranceValue();
            int getMaxSteps();
            static double getRadianToDegree() {return RAD_TO_DEG;}

            double getMargin() const;
            void setMargin(double value);

    private:
            void goalCB();
            void preemptCB();
            void setSettings();
            void currentStateArrived(const asr_flir_ptu_driver::State::ConstPtr& msg);
            bool validate(double pan, double tilt);
            int count;
            double startDistance;

            sensor_msgs::JointState target_joint;
            ros::NodeHandle nodeHandle;
            ros::Time lastStateTime;

            ros::Publisher stateCommandPublisher;
            ros::Subscriber stateSubscriber;
            ros::ServiceClient validate_client;
            ros::ServiceClient alive;
            sensor_msgs::JointState stateCommandMessage;

            static const double RAD_TO_DEG = 180.0 / M_PI;

            double current_pan;
            double desired_pan;
            double max_pan;
            double min_pan;

            double current_tilt;
            double desired_tilt;
            double max_tilt;
            double min_tilt;
            int seq_num;

            int maxSteps;
            double timeToWait;
            double tolerance;
            double margin;
            std::vector< std::map< std::string, double> > forbiddenAreas;
            std::string topicName;
            std::string commandTopicName;
            actionlib::SimpleActionServer<asr_flir_ptu_controller::PTUMovementAction> simpleActionServer;
            asr_flir_ptu_controller::PTUMovementFeedback simpleActionServerFeedback;
            asr_flir_ptu_controller::PTUMovementResult simpleActionServerResult;
    };
}

