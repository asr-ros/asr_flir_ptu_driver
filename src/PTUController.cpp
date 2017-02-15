#include "ptu_controller/PTUController.h"
namespace asr_flir_ptu_controller
{
    PTUController::PTUController(ros::NodeHandle& node_handle, std::string name):
        nodeHandle(node_handle),
        simpleActionServer(name, false) {
        //register the goal and feeback callbacks
        simpleActionServer.registerGoalCallback(boost::bind(&PTUController::goalCB, this));
        simpleActionServer.registerPreemptCallback(boost::bind(&PTUController::preemptCB, this));

        ROS_DEBUG("PTUController, %s", ros::this_node::getName().c_str());
        PTUController::setSettings();
        stateCommandPublisher = nodeHandle.advertise<asr_flir_ptu_driver::State>(getDefaultStateCmdTopicName(), 100);
        stateSubscriber =  nodeHandle.subscribe<asr_flir_ptu_driver::State>(getDefaultStateTopicName(), 100, &PTUController::currentStateArrived, this);

        stateCommandMessage.header.seq = 0;
        stateCommandMessage.name.push_back("pan");
        stateCommandMessage.name.push_back("tilt");
        stateCommandMessage.velocity.push_back(0);
        stateCommandMessage.velocity.push_back(0);
        lastStateTime = ros::Time::now();
        simpleActionServer.start();
        validate_client= nodeHandle.serviceClient<asr_flir_ptu_driver::Validate>("/validation_service");
        alive = nodeHandle.serviceClient<std_srvs::Empty>("/alive_service", true);
        seq_num == 0;
    }

    PTUController::~PTUController() {
    }
    void PTUController::currentStateArrived(const asr_flir_ptu_driver::State::ConstPtr& msg) {
        if (msg->seq_num == seq_num) {
            current_pan = msg->state.position[0];
            current_tilt = msg->state.position[1];
            lastStateTime = ros::Time::now();
            if (simpleActionServer.isActive()) {
                //update joint state
                sensor_msgs::JointState state;
                state.header.stamp = lastStateTime;
                state.name.resize(2);
                state.position.resize(2);
                state.velocity.resize(2);
                state.name[0] = "ptu_pan";
                state.position[0] = getCurrentPan();
                state.velocity[0] = 0;
                state.name[1] = "ptu_tilt";
                state.position[1] = getCurrentTilt();
                state.velocity[1] = 0;
                simpleActionServerFeedback.base_joint = state;

                double squaredSums = pow( desired_pan - current_pan , 2.0) + pow(desired_tilt - current_tilt, 2.0);
                double currentDistance = sqrt(squaredSums);
                simpleActionServerFeedback.percentage = 1.0 - currentDistance / startDistance;
                ROS_DEBUG_STREAM("Covered " << simpleActionServerFeedback.percentage << "% of the way to my goal.");
                simpleActionServer.publishFeedback(simpleActionServerFeedback);

                //if ((fabs(getCurrentPan() - desired_pan) < getToleranceValue() ) &&
                //    (fabs(getCurrentTilt() - desired_tilt) < getToleranceValue() )) {

                //This might cause a problem when a new goal is sent but the variable is still true from the previouse one - not sure atm if this can happen
                //bool reached_goal_and_halted;
                if(msg->finished) {
                    simpleActionServerResult.end_joint = state;
                    ROS_INFO("Succeeded");
                    simpleActionServer.setSucceeded(simpleActionServerResult);
                } else {
                    count++;
                    if (count > getMaxSteps()) {
                        simpleActionServerResult.end_joint = state;
                        ROS_ERROR("Aborted: More steps taken than allowed. MaxSteps: %d", getMaxSteps());
                        simpleActionServer.setAborted(simpleActionServerResult);
                    }
                }
            }
        }
    }

    void PTUController::goalCB() {
        ROS_DEBUG("Goal callback");
        target_joint = simpleActionServer.acceptNewGoal()->target_joint;
        double pan_candidate          = target_joint.position[0];
        double tilt_candidate         = target_joint.position[1];
        double pan_speed     = target_joint.velocity[0];
        double tilt_speed    = target_joint.velocity[1];
        //seems it only uses non-speed mode; not sure why this is needed
        if (pan_speed != 0 || tilt_speed != 0)
        {
            ROS_ERROR("The speed is not 0,0!");
            simpleActionServer.setAborted(simpleActionServerResult);
        }

        asr_flir_ptu_driver::Validate values_for_validation;
        values_for_validation.request.pan = pan_candidate;
        values_for_validation.request.tilt = tilt_candidate;
        values_for_validation.request.margin = getMargin();
        if(validate_client.call(values_for_validation)) {
            if(values_for_validation.response.is_valid)
            {
                desired_pan = values_for_validation.response.new_pan;
                desired_tilt = values_for_validation.response.new_tilt;
            }
            else {
                ROS_ERROR("Values for pan %f and tilt %f do not lie within the working area of the ptu, neither do the lie within the margin %f out of the working are"
                          , pan_candidate, tilt_candidate, getMargin());
                simpleActionServer.setAborted(simpleActionServerResult);
            }
        }
        else {
            ROS_ERROR("Failed to contact Validate service");
            simpleActionServer.setAborted(simpleActionServerResult);
        }


        //ros::Time current_time = ros::Time::now();
        //double time_difference = current_time.toSec() - lastStateTime.toSec();

        //Falsch: Hier bitte separaten watchdog einbauen
        //if (time_difference > getTimeToWait()) {


        //Alive is a service client with steady connection to the supplying PTUNode. A check with "if" of this client indicates if the PTUNode instance is still alive or not
        //For more information please check http://wiki.ros.org/roscpp/Overview/Services point 2.1
        if(!alive)
        {
            ROS_ERROR("The PTU does not responde. Please check the PTU-Node");
            simpleActionServer.setAborted(simpleActionServerResult);
        }
        else {
            seq_num++;
            asr_flir_ptu_driver::State msg;
            stateCommandMessage.position.clear();
            stateCommandMessage.position.push_back(desired_pan);
            stateCommandMessage.position.push_back(desired_tilt);
            //Wenn panSpeed und tiltSpeed sowieso nur übergeben wird wenn beide = 0 sind, warum werden sie dann überhaupt übergeben?
            stateCommandMessage.velocity.push_back(pan_speed);
            stateCommandMessage.velocity.push_back(tilt_speed);
            stateCommandMessage.header.stamp = ros::Time::now();
            msg.state = stateCommandMessage;
            msg.seq_num = seq_num;
            msg.no_check_forbidden_area = false;
            stateCommandPublisher.publish(msg);
            ROS_DEBUG("pan:%f , tilt:%f, panSpeed:%f, tiltSpeed:%f", desired_pan, desired_tilt, pan_speed, tilt_speed);
            count = 0;
            double squared_sums = pow( desired_pan - current_pan , 2.0) + pow(desired_tilt - current_tilt, 2.0);
            startDistance = sqrt(squared_sums);
        }
    }

    void PTUController::preemptCB()  {
        ROS_INFO("Preemted");
        simpleActionServer.setPreempted();
    }
    /**
    *   Get all requiredParameters from the nodehandle
    */
    void PTUController::setSettings() {
        nodeHandle.getParam("timeToWait", timeToWait);
        nodeHandle.getParam("margin", margin);
        nodeHandle.getParam("maxSteps", maxSteps);
        nodeHandle.getParam("tolerance", tolerance);
        nodeHandle.getParam("topicName", topicName);
        nodeHandle.getParam("commandTopicName", commandTopicName);
    }
    double PTUController::getMaximumPan(){
        return max_pan;
    }
    double PTUController::getMinimumPan() {
        return min_pan;
    }

    double PTUController::getMaximumTilt(){
        return max_tilt;
    }
    double PTUController::getMinimumTilt(){
        return min_tilt;
    }

    double PTUController::getCurrentPan() {
        return current_pan;
    }
    double PTUController::getCurrentTilt(){
        return current_tilt;
    }

    std::string PTUController::getDefaultStateCmdTopicName() {
        return topicName;
    }
    std::string PTUController::getDefaultStateTopicName() {
        return commandTopicName;
    }
    double PTUController::getToleranceValue() {
        return tolerance;
    }
    double PTUController::getTimeToWait() {
        return timeToWait;
    }
    int PTUController::getMaxSteps() {
        return maxSteps;
    }
    double PTUController::getMargin() const
    {
        return margin;
    }

    void PTUController::setMargin(double value)
    {
        margin = value;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ptu_controller_actionlib");
    ros::NodeHandle n("~");
    std::string actionServerName;
    n.getParam("actionServerName", actionServerName);
    asr_flir_ptu_controller::PTUController * nodeActionlib = new asr_flir_ptu_controller::PTUController(n, actionServerName);
    ros::spin();
    return 0;
}
