#ifndef PTUCONTROLLERCLIENT_H
#define PTUCONTROLLERCLIENT_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <asr_flir_ptu_controller/PTUMovementAction.h>
#include <boost/thread.hpp>
class PTUControllerClient {
    public:
        PTUControllerClient(std::string name);
        void sendJoint(double pan, double tilt, bool wait);
        void checkValidation(double maxPan, double minPan, double maxTilt, double minTilt);
        actionlib::SimpleClientGoalState testFunction(double pan, double tilt);
        actionlib::SimpleActionClient<asr_flir_ptu_controller::PTUMovementAction> actionClient;

    private:
        asr_flir_ptu_controller::PTUMovementGoal goal;
};
#endif // PTUCONTROLLERCLIENT_H
