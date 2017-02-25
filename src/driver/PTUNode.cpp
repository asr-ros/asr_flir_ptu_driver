#include "driver/PTUNode.h"

PTUNode::PTUNode(ros::NodeHandle& node_handle):node_handle(node_handle) {
	std::string port;
	int baud;
    bool speed_control;
    bool mock_PTU = false;
    goal_not_reached = false;
    node_handle.getParam("port", port);
    node_handle.getParam("baud", baud);
    node_handle.getParam("speed_control", speed_control);
    node_handle.getParam("mockPTU", mock_PTU);
    ptu = NULL;
    if (mock_PTU) {
        ptu = new asr_flir_ptu_driver::PTUDriverMock(port.c_str(), baud, speed_control);
    }
    else {
        ptu = new asr_flir_ptu_driver::PTUDriver(port.c_str(), baud, speed_control);
    }

    if (ptu->isConnected()) {
        ROS_INFO("PTU is connected");
    } else {
        ROS_ERROR("Couldn't connect to PTU");
        exit(1);
    }

    setSettings();

    ROS_INFO("PTU is fully initialized.");

    joint_sub = node_handle.subscribe<asr_flir_ptu_driver::State>(topic_state_command, 1, &PTUNode::setState, this);
    joint_pub = node_handle.advertise<asr_flir_ptu_driver::State>(ptu_topic_state, 1);

    //Required for some external packages, not asr_flir_ptu_controller (maybe fix there if time left, they expect the old sensor_msgs::JointState instead of asr_flir_ptu_driver::State)
    joint_pub_old = node_handle.advertise<sensor_msgs::JointState>(topic_state, 1);

    //name is a placeholder for testing
    validate_service = node_handle.advertiseService("/validation_service", &PTUNode::validatePanTilt, this);
    alive_service = node_handle.advertiseService("/alive_service", &PTUNode::emptyAlive, this);
    path_prediction_service = node_handle.advertiseService("/path_prediction", &PTUNode::predict, this);
    //goal_fulfilled_service = node_handle.advertiseService("/goal_fulfilled", &PTUNode::goalFulfilled, this);

    //Required for some external packages (not asr_flir_ptu_driver). If time left, check if really necessary in the other packages.
    range_service = node_handle.advertiseService("/get_range", &PTUNode::getRange, this);


    settings_service = node_handle.advertiseService(service_settings_update, &PTUNode::updateSettings, this);
    speedmode_service = node_handle.advertiseService(service_speed_control_update, &PTUNode::updateSpeedControl, this);

}

PTUNode::~PTUNode() {
	if (ptu != NULL && ptu->isConnected()) {
		ptu->setAbsoluteAngleSpeeds(0.0, 0.0);
	}
}


void PTUNode::setState(const asr_flir_ptu_driver::State::ConstPtr& msg) {
    ROS_ERROR("STATE RECIVIED");
	ROS_DEBUG("SETSTATE REQUEST - START");
    double pan = msg->state.position[0];
    double tilt = msg->state.position[1];
    double pan_speed = msg->state.velocity[0];
    double tilt_speed = msg->state.velocity[1];
    seq_num = msg->seq_num;
    ROS_DEBUG_STREAM("Node received state_cmd message: (p,t)=(" << pan << "," << tilt << ") and (v_p,v_t)=(" << pan_speed << "," << tilt_speed << ").");

	if (ptu->isInSpeedControlMode()) {
        ROS_DEBUG("PTU is in SpeedcontrolMode");
        ptu->setAbsoluteAngleSpeeds(pan_speed, tilt_speed);
	} else {
        ROS_DEBUG("PTU is NOT in SpeedcontrolMode");
        ptu->setAbsoluteAngles(pan, tilt, msg->no_check_forbidden_area);
        goal_not_reached = true;
        node_handle.setParam("reached_desired_position", false);

	}
	ROS_DEBUG("SETSTATE REQUEST - END");

}

bool PTUNode::validatePanTilt(asr_flir_ptu_driver::Validate::Request &req, asr_flir_ptu_driver::Validate::Response &res)
{
    double pan = req.pan;
    double tilt = req.tilt;
    double margin = req.margin;
    //Reicht das, brauch ich die normale Überprüfung nicht?
    if(ptu->setValuesOutOfLimitsButWithinMarginToLimit(&pan, &tilt, margin)) {
        if(ptu->isInForbiddenArea(pan, tilt)) {
            res.is_valid = false;
            ROS_ERROR("FORBIDDEN");
        }
        else {
            res.is_valid = true;
            ROS_ERROR("VALID");
        }
    }
    else {
        res.is_valid = false;
        ROS_ERROR("OUT OF BOUNDS");
    }
    res.new_pan = pan;
    res.new_tilt = tilt;
    return true;
}

bool PTUNode::predict(asr_flir_ptu_driver::Predict::Request &req, asr_flir_ptu_driver::Predict::Response &res) {
    std::vector<double> legit_end_point = ptu->determineLegitEndPoint(req.pan, req.tilt);
    res.new_pan = legit_end_point[0];
    res.new_tilt = legit_end_point[1];
    return true;
}


bool PTUNode::emptyAlive(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    //Empty because service is meant to be used with a steady connection (see http://wiki.ros.org/roscpp/Overview/Services point 2.1) to check if PTUNode is working/running
    return true;
}


bool PTUNode::updateSettings(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	return setSettings();
}

bool PTUNode::validatePanTilt(asr_flir_ptu_driver::Range::Request &req, asr_flir_ptu_driver::Range::Response &res)
{
    res.pan_min_angle = ptu->getLimitAngle('p', 'l');
    res.pan_max_angle = ptu->getLimitAngle('p', 'u');
    res.tilt_min_angle = ptu->getLimitAngle('t', 'l');
    res.tilt_max_angle = ptu->getLimitAngle('t', 'u');
    res.forbidden_pan_min = node_handle.getParam("forbidden_pan_min", forbidden_pan_min);
    res.forbidden_pan_max = node_handle.getParam("forbidden_pan_max", forbidden_pan_max);
    res.forbidden_tilt_min = node_handle.getParam("forbidden_tilt_min", forbidden_tilt_min);
    res.forbidden_tilt_max = node_handle.getParam("forbidden_tilt_max", forbidden_tilt_max);
    return true;

}

bool PTUNode::setSettings() {
      int pan_base, pan_speed, pan_upper, pan_accel;
      int tilt_base, tilt_speed, tilt_upper, tilt_accel;
      int pan_hold, pan_move;
      int tilt_hold, tilt_move;
      double pan_min, pan_max;
      double tilt_min, tilt_max;
      bool speed_control;

      double computation_tolerance;
      int distance_factor;


      //get all ptu settings
      node_handle.getParam("speed_control", speed_control);
      node_handle.getParam("pan_min_angle", pan_min);
      node_handle.getParam("pan_max_angle", pan_max);
      node_handle.getParam("pan_base_speed", pan_base);
      node_handle.getParam("pan_target_speed", pan_speed);
      node_handle.getParam("pan_upper_speed", pan_upper);
      node_handle.getParam("pan_accel", pan_accel);
      node_handle.getParam("pan_hold", pan_hold);
      node_handle.getParam("pan_move", pan_move);
      node_handle.getParam("tilt_min_angle", tilt_min);
      node_handle.getParam("tilt_max_angle", tilt_max);
      node_handle.getParam("tilt_base_speed", tilt_base);
      node_handle.getParam("tilt_target_speed", tilt_speed);
      node_handle.getParam("tilt_upper_speed", tilt_upper);
      node_handle.getParam("tilt_accel", tilt_accel);
      node_handle.getParam("tilt_hold", tilt_hold);
      node_handle.getParam("tilt_move", tilt_move);
      node_handle.getParam("computation_tolerance", computation_tolerance);
      node_handle.getParam("distance_factor", distance_factor);

      std::vector<double> forbidden_pan_min;
      std::vector<double> forbidden_pan_max;
      std::vector<double> forbidden_tilt_min;
      std::vector<double> forbidden_tilt_max;
      node_handle.getParam("forbidden_pan_min", forbidden_pan_min);
      node_handle.getParam("forbidden_pan_max", forbidden_pan_max);
      node_handle.getParam("forbidden_tilt_min", forbidden_tilt_min);
      node_handle.getParam("forbidden_tilt_max", forbidden_tilt_max);

      std::vector< std::map< std::string, double> > forbidden_areas;
      for (unsigned int i = 0; i < forbidden_pan_min.size(); i++)
	{
	  std::map< std::string, double> area;
	  area["pan_min"] = forbidden_pan_min.at(i);
	  area["pan_max"] = forbidden_pan_max.at(i);
	  area["tilt_min"] = forbidden_tilt_min.at(i);
	  area["tilt_max"] = forbidden_tilt_max.at(i);
	  if (forbidden_pan_min.at(i) < forbidden_pan_max.at(i) &&
	      forbidden_tilt_min.at(i) < forbidden_tilt_max.at(i))
        forbidden_areas.push_back(area);
	  ROS_DEBUG("forbidden area: %d", i+1);
	  ROS_DEBUG("forbidden_pan_min:%f, forbidden_pan_min:%f",  forbidden_pan_min.at(i),  forbidden_pan_max.at(i));
	  ROS_DEBUG("forbidden_tilt_min:%f, forbidden_tilt_max:%f",  forbidden_tilt_min.at(i),  forbidden_tilt_max.at(i));

	}

      ROS_DEBUG("double minPan:%f, double maxPan:%f",  pan_min,  pan_max);
      ROS_DEBUG("double tiltMin:%f, double tiltMax:%f",  tilt_min,  tilt_max);


      // configurate PTU driver with parameters loaded via node handle
      ptu->setSettings(pan_base, tilt_base, pan_speed, tilt_speed, pan_upper, tilt_upper, pan_accel,
               tilt_accel, pan_hold, tilt_hold, pan_move, tilt_move);
      ptu->setForbiddenAreas(forbidden_areas);
      ptu->setLimitAngles(pan_min, pan_max, tilt_min, tilt_max);
      ptu->setComputationTolerance(computation_tolerance);
      ptu->setDistanceFactor(distance_factor);

      node_handle.setParam("pan_min_angle", (int) ptu->getLimitAngle('p','l'));
      node_handle.setParam("pan_max_angle", (int) ptu->getLimitAngle('p','u'));
      node_handle.setParam("pan_base_speed", pan_base);
      node_handle.setParam("pan_target_speed", pan_speed);
      node_handle.setParam("pan_upper_speed", pan_upper);
      node_handle.setParam("pan_accel", pan_accel);
      node_handle.setParam("pan_hold", pan_hold);
      node_handle.setParam("pan_move", pan_move);
      node_handle.setParam("tilt_min_angle", (int) ptu->getLimitAngle('t','l'));
      node_handle.setParam("tilt_max_angle", (int) ptu->getLimitAngle('t','u'));
      node_handle.setParam("tilt_base_speed", tilt_base);
      node_handle.setParam("tilt_target_speed", tilt_speed);
      node_handle.setParam("tilt_upper_speed", tilt_upper);
      node_handle.setParam("tilt_accel", tilt_accel);
      node_handle.setParam("tilt_hold", tilt_hold);
      node_handle.setParam("tilt_move", tilt_move);

      // load names of coordinate systems
      node_handle.getParam("ptu_pan_frame", ptu_pan_frame);
      node_handle.getParam("ptu_pan_frame_rotated", ptu_pan_frame_rotated);
      node_handle.getParam("ptu_tilt_frame", ptu_tilt_frame);
      node_handle.getParam("ptu_tilt_frame_rotated", ptu_tilt_frame_rotated);

      // load names of ROS topics
      node_handle.getParam("topicStateCommand", topic_state_command);
      node_handle.getParam("topicState", topic_state);
      node_handle.getParam("ptuTopicState", ptu_topic_state);

      ROS_DEBUG("topicStateCommand: %s", topic_state_command.c_str());
      ROS_DEBUG("topicState: %s", topic_state.c_str());

      // load names of ROS services
      node_handle.getParam("serviceSettingsUpdate", service_settings_update);
      node_handle.getParam("serviceSpeedControlUpdate", service_speed_control_update);
      node_handle.getParam("serviceRange", service_range);

      ROS_DEBUG("serviceSettingsUpdate: %s", service_settings_update.c_str());
      ROS_DEBUG("serviceSpeedControlUpdate: %s", service_speed_control_update.c_str());
      ROS_DEBUG("serviceRange: %s", service_range.c_str());

      //add the namespace of the asr_flir_ptu_driver to the list of ptu namespaces. This is necessary to 'find' the PTU with the gui
      if (node_handle.hasParam("ptu_namespaces"))
	{
	  ROS_INFO("Set namespace");
      node_handle.setParam("/ptu_namespaces",node_handle.getNamespace());
	}

      return true;
}

bool PTUNode::updateSpeedControl(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	return setSpeedControl();
}

bool PTUNode::setSpeedControl() {
    int pan_speed, tilt_speed;
    bool speed_control;
    node_handle.getParam("speed_control", speed_control);
    node_handle.getParam("pan_target_speed", pan_speed);
    node_handle.getParam("tilt_target_speed", tilt_speed);

	//not critical, but needs more testing: does PTU keep current speed when it's switched to position mode?
	//this code assumes it does

    if (ptu->isInSpeedControlMode() & !speed_control) {
		ptu->setAbsoluteAngleSpeeds(0.0, 0.0);
        ptu->setSpeedControlMode(speed_control);
        ptu->setAbsoluteAngleSpeeds((signed short)pan_speed, (signed short)tilt_speed);
    } else if (!ptu->isInSpeedControlMode() & speed_control) {
        ptu->setSpeedControlMode(speed_control);
	}
	return true;
}

//check to see if PTU is still connected
bool PTUNode::ok() {
	return ros::ok() && ptu->isConnected();
}

void PTUNode::spinOnce() {
    //set stopped and whether or not the ptu is in speed control mode
    //PROBLEM HERE: When new state arrives it can have wrong value on the first publishing when goal that was executed before succeeded

    //if(goal_not_reached || ptu->isInSpeedControlMode()) {
        bool done = ptu->hasHaltedAndReachedGoal();
        node_handle.setParam("reached_desired_position", done);
        if(done) goal_not_reached == false;

        //update joint state
        sensor_msgs::JointState state;
        asr_flir_ptu_driver::State msg;
        state.header.stamp = ros::Time::now();
        state.header.frame_id = (ptu->isInSpeedControlMode())? "speed_control" : "position_control"; //mostly for debugging
        state.name.resize(2);
        state.position.resize(2);
        state.velocity.resize(2);
        state.name[0] = "ptu_pan";
        state.position[0] = ptu->getCurrentAngle(PAN);
        state.velocity[0] = ptu->getAngleSpeed(PAN);
        state.name[1] = "ptu_tilt";
        state.position[1] = ptu->getCurrentAngle(TILT);
        state.velocity[1] = ptu->getAngleSpeed(TILT);
        msg.state = state;
        msg.seq_num = seq_num;
        msg.finished = done;
        joint_pub.publish(msg);
        joint_pub_old.publish(state);
    //}

    // publish updated PTU coordinate system
    tf::Transform pan(tf::Quaternion(tf::Vector3(0,0,1), ptu->getCurrentAngle(PAN) * DEG_TO_RAD));
    tf::StampedTransform panStamped(pan, ros::Time::now(), ptu_pan_frame, ptu_pan_frame_rotated);
    tb_pan.sendTransform(panStamped);
	
    tf::Transform tilt(tf::Quaternion(tf::Vector3(0,-1,0), ptu->getCurrentAngle(TILT) * DEG_TO_RAD));
    tf::StampedTransform tiltStamped(tilt, ros::Time::now(), ptu_tilt_frame, ptu_tilt_frame_rotated);
    tb_tilt.sendTransform(tiltStamped);

    ros::spinOnce();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "asr_flir_ptu_driver");
    ros::NodeHandle n("~");
    PTUNode* node = new PTUNode(n);

	double update_rate = 100;
    n.getParam("update_rate", update_rate);
	ros::Rate loop_rate(update_rate);
	while (node->ok()) {
		node->spinOnce();
		loop_rate.sleep();
	}
	delete node;

}
