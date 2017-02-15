#include "ptu_controller/PTUControllerClient.h"
#include <iostream>
#include <fstream>

void spinThread()
{
  ros::spin();
}
PTUControllerClient::PTUControllerClient(std::string name):
    actionClient(name) {
    ROS_INFO("Waiting for action server to start.");
    actionClient.waitForServer();
    goal.target_joint.header.seq = 0;
    goal.target_joint.name.push_back("pan");
    goal.target_joint.name.push_back("tilt");
    goal.target_joint.velocity.push_back(0);
    goal.target_joint.velocity.push_back(0);
}
void PTUControllerClient::sendJoint(double pan, double tilt, bool wait) {
    ROS_DEBUG("Sending goal.");
    goal.target_joint.position.clear();
    goal.target_joint.position.push_back(pan);
    goal.target_joint.position.push_back(tilt);
    actionClient.sendGoal(goal);
    if (wait) {
        bool finished_before_timeout = actionClient.waitForResult(ros::Duration(30.0));
        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = actionClient.getState();
          ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
          ROS_INFO("Action did not finish before the time out.");
    }

}
void PTUControllerClient::checkValidation(double maxPan, double minPan, double maxTilt, double minTilt) {
    double epsilon = 0.1;
    double greaterMaxPan = maxPan + epsilon;
    double smallerMinPan = minPan - epsilon;
    double greaterMaxTilt = maxTilt + epsilon;
    double smallerMinTilt = minTilt - epsilon;

    double pans[3] = {0.0, greaterMaxPan, smallerMinPan};
    double tilts[3] = {0.0, greaterMaxTilt, smallerMinTilt};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i != 0 || j != 0)
                this->sendJoint(pans[i],tilts[j], true);
        }
    }
}

actionlib::SimpleClientGoalState PTUControllerClient::testFunction(double pan, double tilt) {
    goal.target_joint.position.clear();
    goal.target_joint.position.push_back(pan);
    goal.target_joint.position.push_back(tilt);
    actionClient.sendGoal(goal);
    bool finished_before_timeout = actionClient.waitForResult(ros::Duration(60.0));
    if (finished_before_timeout)
    {
        return actionClient.getState();
    }
    else
    {
        return actionlib::SimpleClientGoalState::ACTIVE;
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ptu_controller_client");

    // create the action client
    boost::thread spin_thread(&spinThread);
    std::string actionServerName;
    ros::NodeHandle n("~");
    n.getParam("actionServerName", actionServerName);
    PTUControllerClient* ptu_controller = new PTUControllerClient(actionServerName);
    bool checkValidation;
    n.getParam("checkValidation", checkValidation);
    //############## Testing Only, Code untested ####################
    bool testing;
    n.getParam("testing", testing);
    if(testing) {
        std::ofstream testfile;
        std::string filepath;
        n.getParam("testlog_filepath", filepath);
        testfile.open(filepath.c_str(), std::ios::trunc);
        ROS_DEBUG("Test Nr. 00: Test initialization\n");
        testfile << "Test Nr. 00:\nIt is tried to bring the PTU in starting position with pan = 0 and tilt = 0\n";
        testfile << "It is expected that the Actionserver accepts the goal\n";
        testfile << "Expected return state: SUCCEEDED\n";
        testfile << "Recieved return state: ";
        actionlib::SimpleClientGoalState state(ptu_controller->testFunction(0,0).state_, ptu_controller->testFunction(0,0).text_);
        testfile << state.toString() << "\n";
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_DEBUG("Correct state returned, for pan/tilt check look into the testlog\n");
            testfile << "Expected pan: 0\n";
            testfile << "Recieved pan: " << ptu_controller->actionClient.getResult()->end_joint.position[0] << "\n";
            testfile << "Expected tilt: 0\n";
            testfile << "Recieved tilt: " << ptu_controller->actionClient.getResult()->end_joint.position[1] << "\n";
        }
        else {
            ROS_DEBUG("Failure\n");
        }
        testfile << "\n";

        ROS_DEBUG("Test Nr. 01: No Movement\n");
        testfile << "Test Nr. 01:\nIt is tried if the Actionserver works with a command that moves the PTU to the same position where it is located at the moment\n";
        testfile << "It is expected that the Actionserver takes the goal and executes it\n";
        testfile << "Expected return state: SUCCEEDED\n";
        testfile << "Recieved return state: ";
        state = ptu_controller->testFunction(0,0);
        testfile << state.toString() << "\n";
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_DEBUG("Correct state returned, for pan/tilt check look into the testlog\n");
            testfile << "Expected pan: 0\n";
            testfile << "Recieved pan: " << ptu_controller->actionClient.getResult()->end_joint.position[0] << "\n";
            testfile << "Expected tilt: 0\n";
            testfile << "Recieved tilt: " << ptu_controller->actionClient.getResult()->end_joint.position[1] << "\n";
        }
        else {
            ROS_DEBUG("Failure\n");
        }
        testfile << "\n";

        ROS_DEBUG("Test Nr. 02: Pan Movement\n");
        testfile << "Test Nr. 02:\nIt is tried to move the pan-unit of the PTU from 0 to 5 degree\n";
        testfile << "It is expected that the Actionserver accepts the goal and executes it\n";
        testfile << "Expected return state: SUCCEEDED\n";
        testfile << "Recieved return state: ";
        state = ptu_controller->testFunction(5,0);
        testfile << state.toString() << "\n";
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_DEBUG("Correct state returned, for pan/tilt check look into the testlog\n");
            testfile << "Expected pan: ~5\n";
            testfile << "Recieved pan: " << ptu_controller->actionClient.getResult()->end_joint.position[0] << "\n";
            testfile << "Expected tilt: 0\n";
            testfile << "Recieved tilt: " << ptu_controller->actionClient.getResult()->end_joint.position[1] << "\n";
        }
        else {
            ROS_DEBUG("Failure\n");
        }
        testfile << "\n";

        ROS_DEBUG("Test Nr. 03: Tilt Movement\n");
        testfile << "Test Nr 03:\nIt is tried to move the tilt-unit of the PTU from 0 to 5 degree\n";
        testfile << "It is expected that the Actionserver accepts the goal and executes it\n";
        testfile << "Expected return state: SUCCEEDED\n";
        testfile << "Recieved return state: ";
        state = ptu_controller->testFunction(5,5);
        testfile << state.toString() << "\n";
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_DEBUG("Correct state returned, for pan/tilt check look into the testlog\n");
            testfile << "Expected pan: ~5\n";
            testfile << "Recieved pan: " << ptu_controller->actionClient.getResult()->end_joint.position[0] << "\n";
            testfile << "Expected tilt: ~5\n";
            testfile << "Recieved tilt: " << ptu_controller->actionClient.getResult()->end_joint.position[1] << "\n";
        }
        else {
            ROS_DEBUG("Failure\n");
        }
        testfile << "\n";

        ROS_DEBUG("Test Nr. 04: Pan + Tilt Movement\n");
        testfile << "Test Nr. 04:\nIt is tried to move the pan and the tilt unit within the same command from 5 to 10\n";
        testfile << "It is expected that the Actionserver takes the command and executes it\n";
        testfile << "Expected return state: SUCCEEDED\n";
        testfile << "Recieved return state: ";
        state = ptu_controller->testFunction(10,10);
        testfile << state.toString() << "\n";
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_DEBUG("Correct state returned, for pan/tilt check look into the testlog\n");
            testfile << "Expected pan: ~10\n";
            testfile << "Recieved pan: " << ptu_controller->actionClient.getResult()->end_joint.position[0] << "\n";
            testfile << "Expected tilt: ~10\n";
            testfile << "Recieved tilt: " << ptu_controller->actionClient.getResult()->end_joint.position[1] << "\n";
        }
        else {
            ROS_DEBUG("Failure\n");
        }
        testfile << "\n";

        ROS_DEBUG("Test Nr. 05: Pan out of bounds \n");
        testfile << "Test Nr. 05:\nIt is tested if it is possible to pass a pan position to the Actionserver which is out of the working area of the PTU (pan 178)\n";
        testfile << "It is expected that the server rejects (at the moment instant abort) the request and that no movement happens\n";
        testfile << "Expected return state: ABORTED\n";
        testfile << "Recieved return state: ";
        state = ptu_controller->testFunction(178,10);
        testfile << state.toString() << "\n";
        if(state == actionlib::SimpleClientGoalState::ABORTED) {
            ROS_DEBUG("Correct state returned, for pan/tilt check look into the testlog\n");
            testfile << "Expected pan: ~10\n";
            testfile << "Recieved pan: " << ptu_controller->actionClient.getResult()->end_joint.position[0] << "\n";
            testfile << "Expected tilt: ~10\n";
            testfile << "Recieved tilt: " << ptu_controller->actionClient.getResult()->end_joint.position[1] << "\n";
        }
        else {
            ROS_DEBUG("Failure\n");
        }
        testfile << "\n";

        ROS_DEBUG("Test Nr. 06: Tilt out of bounds \n");
        testfile << "Test Nr. 06:\nIt is tested if it is possible to pass a pan position to the Actionserver which is out of the working area of the PTU (tilt 178)\n";
        testfile << "It is expected that the server rejects (at the moment instant abort) the request and that no movement happens\n";
        testfile << "Expected return state: ABORTED\n";
        testfile << "Recieved return state: ";
        state = ptu_controller->testFunction(10,178);
        testfile << state.toString() << "\n";
        if(state == actionlib::SimpleClientGoalState::ABORTED) {
            ROS_DEBUG("Correct state returned, for pan/tilt check look into the testlog\n");
            testfile << "Expected pan: ~10\n";
            testfile << "Recieved pan: " << ptu_controller->actionClient.getResult()->end_joint.position[0] << "\n";
            testfile << "Expected tilt: ~10\n";
            testfile << "Recieved tilt: " << ptu_controller->actionClient.getResult()->end_joint.position[1] << "\n";
        }
        else {
            ROS_DEBUG("Failure\n");
        }
        testfile << "\n";

        ROS_DEBUG("Test Nr. 07: In forbidden area \n");
        testfile << "Test Nr. 07\nIt is tested if it is possible to pass a position within a forbidden area but within the working area of the PTU to the Actionserver (pan -55, tilt - 30)";
        testfile << "It is expected that the server rejects (at the moment instant abort) the request and that no movement happens\n";
        testfile << "Expected return state: ABORTED\n";
        testfile << "Recieved return state: ";
        state = ptu_controller->testFunction(-55,-30);
        testfile << state.toString() << "\n";
        if(state == actionlib::SimpleClientGoalState::ABORTED) {
            ROS_DEBUG("Correct state returned, for pan/tilt check look into the testlog\n");
            testfile << "Expected pan: ~10\n";
            testfile << "Recieved pan: " << ptu_controller->actionClient.getResult()->end_joint.position[0] << "\n";
            testfile << "Expected tilt: ~10\n";
            testfile << "Recieved tilt: " << ptu_controller->actionClient.getResult()->end_joint.position[1] << "\n";
        }
        else {
            ROS_DEBUG("Failure\n");
        }
        testfile << "\n";

        testfile.flush();
        testfile.close();
        return 0;
        
    }
    //##################################################
    if (checkValidation) {
        ROS_DEBUG("checkValidation");
        double maxPan;
        double minPan;
        double maxTilt;
        double minTilt;

        n.getParam("minPan", minPan);
        n.getParam("maxPan", maxPan);
        n.getParam("minTilt", minTilt);
        n.getParam("maxTilt", maxTilt);

        ptu_controller->checkValidation(maxPan, minPan, maxTilt, minTilt);
    } else {
        std::vector<double> pans;
        std::vector<double> tilts;
        std::vector<bool> waits;
        n.getParam("pans",pans);
        n.getParam("tilts", tilts);
        n.getParam("waits",waits);

        if (pans.size() != tilts.size() || pans.size() != waits.size())
            ROS_ERROR("ERROR in commands.yaml");
        for(int i = 0; i < waits.size(); i++) {
            ptu_controller->sendJoint(pans[i], tilts[i], waits[0]);
        }
    }

  ros::shutdown();
  spin_thread.join();
  return 0;
}



