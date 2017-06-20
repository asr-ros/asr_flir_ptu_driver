#include "driver/PTUFree.h"

namespace ptu_free {

    PTUFree::PTUFree():ptu_io_service(), timer_io_service(), ptu_port(ptu_io_service) {

    }
    PTUFree::PTUFree(boost::asio::io_service& io): timer_io_service(), ptu_port(io) {

    }

    bool PTUFree::setBaudRate(int baud) {
        printf("Setting baud rate\n");
        boost::system::error_code set_option_error_code;
        ptu_port.set_option(boost::asio::serial_port_base::baud_rate(baud), set_option_error_code);
        if(set_option_error_code.value() != boost::system::errc::success) {
            std::string error_reason = getErrorString(set_option_error_code);
            printf("Setting baud to %d failed with error message: %s\n", baud, error_reason.c_str());
            return false;
        }
        else {
            printf("Success\n");
            return true;
        }
    }

    bool PTUFree::setNewSerialConnection(std::string port, int baud)
    {
        printf("Setting new Connection\n");
        if(ptu_port.is_open()) {
            printf("Closing current Serial Port Connection\n");
            closeSerialConnection();
        }

        boost::system::error_code open_error_code;
        ptu_port.open(port, open_error_code);
        if(open_error_code.value() != boost::system::errc::success) {
            std::string error_reason = getErrorString(open_error_code);
            printf("Opening port %s failed with error message: %s", port.c_str(), error_reason.c_str());
            return false;
        }
        //PTU sends some data at the beginning, eg. distrubuter and model name. This sleep command makes sure that everything has been send before buffer is cleared
        sleep(2);
        //To get rid of possible remaing stuff in the buffer of the new port used (from possible former usage, e.g. through other program)
        tcflush(ptu_port.lowest_layer().native_handle(), TCIOFLUSH);

        ptu_port.set_option(boost::asio::serial_port_base::baud_rate(9600));
        ptu_port.set_option(boost::asio::serial_port_base::character_size(8));
        ptu_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        ptu_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        ptu_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        communicate("ed ");
        reset();
        //Are those neccesary or rather bad because it works against the defaults if set? NOTE: temporarily solved by comments in the method documentation
        awaitPositionCommandCompletion();
        setPositionLimitEnforcementMode(FACTORY_LIMITS_ENABLED);
        setPositionExecutionMode(IMMEDIATE_POSITION_EXECUTION_MODE);
        //necessary to get initial values for the internal variables
        factory_pan_min = getCurrentUsedMinimumPanPositionLimit();
        factory_pan_max = getCurrentUsedMaximumPanPositionLimit();
        factory_tilt_min = getCurrentUsedMinimumTiltPositionLimit();
        factory_tilt_max = getCurrentUsedMaximumTiltPositionLimit();
        return true;
    }

    void PTUFree::closeSerialConnection() {
        ptu_port.close();
    }

    std::string PTUFree::communicate(std::string request) {
        boost::asio::write(ptu_port, boost::asio::buffer(request.c_str(), request.size()));
        std::string response = readPTUResponse();
        return response;
    }



    std::string PTUFree::readPTUResponse() {
        std::string recieved_message;
        char current_letter;
        while(true) {
            boost::asio::read(ptu_port, boost::asio::buffer(&current_letter,1));
            if(current_letter == '\n') {
                //HOTFIX solution: Asynchronous responses in successfull messages get cut out. In unsuccessfull messages they stay in because they gonna return false/ERROR later on anyway
                //Only printf message can be messed up in error case a little bit. Since that error does not occure often it is tolerated for the moment.
                size_t start_pos = recieved_message.find('*');
                if(start_pos != std::string::npos) {
                    recieved_message = recieved_message.substr(start_pos);
                }
                return recieved_message;
            }
            else {
                recieved_message.push_back(current_letter);
            }
        }
    }

    std::vector<std::string> PTUFree::evaluateResponse(std::string response) {
        std::vector<std::string> result;
        result.push_back(response.substr(0, 1));
        if(response.at(0) == '*') {
            result.push_back(response.substr(1, response.length() - 1));
            int start_position = 10000;
            int end_position = 0;
            size_t first_found_position = 0;
            size_t last_found_position = 0;
            for (int i = 0; i < 10; i++) {
                first_found_position = response.find(boost::lexical_cast<std::string>(i));
                if(first_found_position != std::string::npos) {
                        if((int) first_found_position < start_position) {
                            start_position = first_found_position;
                        }
                }
                last_found_position = response.rfind(boost::lexical_cast<std::string>(i));
                if( last_found_position != std::string::npos) {
                        if((int) last_found_position > end_position) {
                            end_position = last_found_position;
                        }
                }
            }
            if(start_position <= end_position) {
                if(response.at(start_position - 1) == '-') {
                    start_position -= 1;
                }
                result.push_back(response.substr(start_position, end_position - start_position + 1));
            }
        }
        //In this case the first character is '!'
        else {
            //First character identifies if it was successfull or not and therefore is not needed here.
            result.push_back(response.substr(1, response.length() - 1));
        }
        return result;
    }



    bool PTUFree::setDesiredPanPositionAbsolute(short int position) {
        std::string command = "pp" + boost::lexical_cast<std::string>(position) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting desired absolute pan position to %d: %s\n", position, response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setDesiredPanPositionRelative(short int position_offset) {
        std::string command = "po" + boost::lexical_cast<std::string>(position_offset) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while adding desired pan position offset of %d: %s\n", position_offset, response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setDesiredPanSpeedAbsolute(short int speed) {
        std::string command = "ps" + boost::lexical_cast<std::string>(speed) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting desired absolute pan speed to %d: %s\n", speed, response_parts[1].c_str());
            return false;
        }
    }

    //WARNING: OFFSET FROM CURRENT (NOT DESIRED) SPEED IS SET TO DESIRED SPEED (Example: Current speed = 0, Desired Speed = 500. set_desired_pan_speed_realtive(100) will result in
    //current speed being 0 and desired speed being 0 + 100 = 100
    bool PTUFree::setDesiredPanSpeedRelative(short int speed_offset) {
        std::string command = "pd" + boost::lexical_cast<std::string>(speed_offset) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting desired pan speed offset to %d: %s\n", speed_offset, response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setDesiredPanAccelerationAbsolute(short int acceleration) {
        std::string command = "pa" + boost::lexical_cast<std::string>(acceleration) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting absolute pan acceleration to %d: %s\n", acceleration, response_parts[1].c_str());
            return false;
        }
    }


    //WARNING: TAKES EXTREMLY LONG OR DOES NOT WORK. ACCORDING TO MANUAL THIS DOES NOT WORK ON THE FLY, IN TESTING EVEN 300 SECONDS WERE NOT ENOUGH. THEREFORE NOT SUCCESSFULLY TESTED, DO NOT USE
    bool PTUFree::setDesiredPanUpperSpeedLimit(short int upper_speed_limit) {
        std::string command = "pu" + boost::lexical_cast<std::string>(upper_speed_limit) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting absolute pan base speed to %d: %s\n", upper_speed_limit, response_parts[1].c_str());
            return false;
        }
    }


    //WARNING: MUST BE 0 or >= 57 (last value depending on ptu)
    //WARNING: Consumes a lot of time (usually >6 seconds) until changes are in place (not possible on the fly)
    bool PTUFree::setDesiredPanLowerSpeedLimit(short int lower_speed_limit) {
        std::string command = "pl" + boost::lexical_cast<std::string>(lower_speed_limit) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting absolute lower speed limit to %d: %s\n", lower_speed_limit, response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setPanBaseSpeed(short int base_speed) {
        std::string command = "pb" + boost::lexical_cast<std::string>(base_speed) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting pan base speed to %d: %s\n", base_speed, response_parts[1].c_str());
            return false;
        }
    }



    long PTUFree::getCurrentPanPosition() {
        std::string command = "pp ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting current pan position: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getCurrentPanSpeed() {
        std::string command = "pd ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting current pan speed: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }




    long PTUFree::getPanUpperSpeedLimit() {
        std::string command = "pu ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting current pan upper speed limit: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getPanLowerSpeedLimit() {
        std::string command = "pl ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting curent pan lower speed limit: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    double PTUFree::getPanResolution() {
        std::string command = "pr ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<double>(response_parts[2]);
        }
        else {
            printf("Error while getting pan resolution: %s\n", response_parts[1].c_str());
            return -1.0;
        }
    }

    long PTUFree::getDesiredPanPosition() {
        std::string command = "po ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting desired pan position: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getDesiredPanSpeed() {
        std::string command = "ps ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting desired pan speed: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getPanAcceleartion() {
        std::string command = "pa ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting desired pan acceleration (desired == current in this case): %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getPanBaseSpeed() {
        std::string command = "pb ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting desired pan base speed: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }




    bool PTUFree::setDesiredTiltPositionAbsolute(short int position) {
        std::string command = "tp" + boost::lexical_cast<std::string>(position) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting desired absolute tilt position to %d: %s\n", position, response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setDesiredTiltPositionRelative(short int position_offset) {
        std::string command = "to" + boost::lexical_cast<std::string>(position_offset) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while adding desired tilt position offset of %d: %s\n", position_offset, response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setDesiredTiltSpeedAbsolute(short int speed) {
        std::string command = "ts" + boost::lexical_cast<std::string>(speed) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting desired absolute tilt speed to %d: %s\n", speed, response_parts[1].c_str());
            return false;
        }
    }

    //WARNING: OFFSET FROM CURRENT (NOT DESIRED) SPEED IS SET TO DESIRED SPEED (Example: Current speed = 0, Desired Speed = 500. setDesiredTiltSpeedRelative(100) will result in
    //current speed being 0 and desired speed being 0 + 100 = 100
    bool PTUFree::setDesiredTiltSpeedRelative(short int speed_offset) {
        std::string command = "td" + boost::lexical_cast<std::string>(speed_offset) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting desired tilt speed offset to %d: %s\n", speed_offset, response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setDesiredTiltAccelerationAbsolute(short int acceleration) {
        std::string command = "ta" + boost::lexical_cast<std::string>(acceleration) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting absolute tilt acceleration to %d: %s\n", acceleration, response_parts[1].c_str());
            return false;
        }
    }


    //WARNING: TAKES EXTREMLY LONG OR DOES NOT WORK. ACCORDING TO MANUAL THIS DOES NOT WORK ON THE FLY, IN TESTING EVEN 300 SECONDS WERE NOT ENOUGH. THEREFORE NOT SUCCESSFULLY TESTED, DO NOT USE
    bool PTUFree::setDesiredTiltUpperSpeedLimit(short int upper_speed_limit) {
        std::string command = "tu" + boost::lexical_cast<std::string>(upper_speed_limit) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting desired tilt base speed to %d: %s\n", upper_speed_limit, response_parts[1].c_str());
            return false;
        }
    }


    //WARNING: MUST BE 0 or >= 57 (last value depending on ptu)
    //WARNING: Consumes a lot of time (usually >6 seconds) until changes are in place (not possible on the fly)
    bool PTUFree::setDesiredTiltLowerSpeedLimit(short int lower_speed_limit) {
        std::string command = "tl" + boost::lexical_cast<std::string>(lower_speed_limit) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting desired tilt lower speed limit to %d: %s\n", lower_speed_limit, response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setTiltBaseSpeed(short int base_speed) {
        std::string command = "tb" + boost::lexical_cast<std::string>(base_speed) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting tilt base speed to %d: %s\n", base_speed, response_parts[1].c_str());
            return false;
        }
    }



    long PTUFree::getCurrentTiltPosition() {
        std::string command = "tp ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting current tilt position: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getCurrentTiltSpeed() {
        std::string command = "td ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting current tilt speed: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }




    long PTUFree::getTiltUpperSpeedLimit() {
        std::string command = "tu ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting current tilt upper speed limit: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getTiltLowerSpeedLimit() {
        std::string command = "tl ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting curent tilt lower speed limit: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    double PTUFree::getTiltResolution() {
        std::string command = "tr ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<double>(response_parts[2]);
        }
        else {
            printf("Error while getting tilt resolution: %s\n", response_parts[1].c_str());
            return -1.0;
        }
    }

    long PTUFree::getDesiredTiltPosition() {
        std::string command = "to ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting desired tilt position: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getDesiredTiltSpeed() {
        std::string command = "ts ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting desired tilt speed: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getTiltAcceleartion() {
        std::string command = "ta ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting desired tilt acceleration (desired == current in this case): %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }

    long PTUFree::getTiltBaseSpeed() {
        std::string command = "tb ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting desired tilt  base speed: %s\n", response_parts[1].c_str());
            return LONG_MIN;
        }
    }


    //WARNING: USER_DEFINED_LIMITS_ENABLED NOT SUPPORTED IN OLDER PTU VERSIONS, CASE WITH THAT LIMIT MODE NOT TESTED (PTU VERSION TOO OLD)
    bool PTUFree::setPositionLimitEnforcementMode(long enable) {
        std::string command;
        if(enable == FACTORY_LIMITS_ENABLED) {
            command = "le ";
        }
        else if (enable == USER_DEFINED_LIMITS_ENABLED) {
            command = "lu ";
        }
        else if (enable == LIMITS_DISABLED) {
            command = "ld ";
        }
        else {
            printf("Error while setting limit enforcement mode: Unknown mode\n");
            return false;
        }
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting limit enforcement: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setMinimumPanPositionLimit(short int position) {
        std::string command = "pn" + boost::lexical_cast<std::string>(position) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting minimum user pan position limit to %d: %s\n", position, response_parts[1].c_str());
            return false;
        }
    }
    bool PTUFree::setMaximumPanPositionLimit(short int position) {
        std::string command = "px" + boost::lexical_cast<std::string>(position) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting maximum user pan position limit to %d: %s\n", position, response_parts[1].c_str());
            return false;
        }
    }
    bool PTUFree::setMinimumTiltPositionLimit(short int position) {
        std::string command = "tn" + boost::lexical_cast<std::string>(position) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting minimum user tilt position limit to %d: %s\n", position, response_parts[1].c_str());
            return false;
        }
    }
    bool PTUFree::setMaximumTiltPositionLimit(short int position) {
        std::string command = "tx" + boost::lexical_cast<std::string>(position) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting maximum user tilt position limit to %d: %s\n", position, response_parts[1].c_str());
            return false;
        }
    }

    //WARNING: NOT SUPPORTED IN OLDER PTU VERSIONS, NOT TESTED (PTU VERSION TOO OLD)
    long PTUFree::getUserMinimumPanPositionLimit() {
        std::string command = "pnu ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting minimum user pan position limit: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }

    //WARNING: NOT SUPPORTED IN OLDER PTU VERSIONS, NOT TESTED (PTU VERSION TOO OLD)
    long PTUFree::getUserMaximumPanPositionLimit() {
        std::string command = "pxu ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting maximum user pan position limit: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }

    //WARNING: NOT SUPPORTED IN OLDER PTU VERSIONS, NOT TESTED (PTU VERSION TOO OLD)
    long PTUFree::getUserMinimumTiltPositionLimit() {
        std::string command = "tnu ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting minimum user tilt position limit: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }

    //WARNING: NOT SUPPORTED IN OLDER PTU VERSIONS, NOT TESTED (PTU VERSION TOO OLD)
    long PTUFree::getUserMaximumTiltPositionLimit() {
        std::string command = "txu ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting maximum user tilt position limit: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }
    //Needed because there is no specific command to always get the factory limits, only a command to get the limits depending on selected mode
    long PTUFree::getFactoryMinimumPanPositionLimit() {
        return factory_pan_min;
    }
    long PTUFree::getFactoryMaximumPanPositionLimit() {
        return factory_pan_max;
    }
    long PTUFree::getFactoryMinimumTiltPositionLimit() {
        return factory_tilt_min;
    }
    long PTUFree::getFactoryMaximumTiltPositionLimit() {
        return factory_tilt_max;
    }
    long PTUFree::getCurrentUsedMinimumPanPositionLimit() {
        std::string command = "pn ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting currently used minimum pan position limit: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }
    long PTUFree::getCurrentUsedMaximumPanPositionLimit() {
        std::string command = "px ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting currently used maximum pan position limit: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }
    long PTUFree::getCurrentUsedMinimumTiltPositionLimit() {
        std::string command = "tn ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting currently used minimum tilt position limit: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }
    long PTUFree::getCurrentUsedMaximumTiltPositionLimit() {
        std::string command = "tx ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return boost::lexical_cast<long>(response_parts[2]);
        }
        else {
            printf("Error while getting currently used maximum tilt position limit: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }



    long PTUFree::getPositionLimitEnforcementMode() {
        std::string command = "l ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response.at(0) == '*') {
            if(response_parts[1].find("DISABLED") != std::string::npos) {
                return LIMITS_DISABLED;
            }
            else if (response_parts[1].find("user") != std::string::npos) {
                return USER_DEFINED_LIMITS_ENABLED;
            }
            else {
                return FACTORY_LIMITS_ENABLED;
            }
        }
        else {
            printf("Error while getting currently used limit enforcement mode, %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }

    bool PTUFree::setPositionExecutionMode(long mode) {
        std::string command;
        long new_mode;
        if(mode == IMMEDIATE_POSITION_EXECUTION_MODE) {
            command = "i ";
            new_mode = IMMEDIATE_POSITION_EXECUTION_MODE;
        }
        else if (mode == SLAVED_POSITION_EXECUTION_MODE) {
            command = "s ";
            new_mode = SLAVED_POSITION_EXECUTION_MODE;
        }
        else {
            printf("Error while setting position execution mode: Unknown mode\n");
            return false;
        }
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            position_execution_mode = new_mode;
            return true;
        }
        else {
            printf("Error while setting position execution mode: %s\n", response_parts[1].c_str());
            return false;
        }
    }


    //no method found on ptu interface to get this mode
    long PTUFree::getPositionExecutionMode() {
        return position_execution_mode;
    }

    //PTU waits until last position command is fully executed
    bool PTUFree::awaitPositionCommandCompletion() {
        std::string command = "a ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while awaiting command completion: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::halt(long axis) {
        std::string command;
        if(axis == HALT_PAN_ONLY) {
            command = "hp ";
        }
        else if (axis == HALT_TILT_ONLY) {
            command = "ht ";
        }
        else if (axis == HALT_BOTH) {
            command = "h ";
        }
        else {
            printf("Error while halting axis: Unknown mode\n");
            return false;
        }
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while halting axis: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    //Typical errors (e.g. pan/tilt out of bounds) get handled properly, untypical errors can not be handled really well due to the usage of serial port, in that case false is returned to indicate the error
    //and the functions print the kind of error to console.
    bool PTUFree::setDesiredPanTiltPositionAbsoluteSlaved(short int pan, short int tilt) {
        long previous_mode = getPositionExecutionMode();
        bool worked = setPositionExecutionMode(SLAVED_POSITION_EXECUTION_MODE);
        if(!worked) {
            return false;
        }
        short int prev_pan = getDesiredPanPosition();
        worked = setDesiredPanPositionAbsolute(pan);
        if(!worked) {
            setPositionExecutionMode(previous_mode);
            return false;
        }
        short int prev_tilt = getDesiredTiltPosition();
        worked = setDesiredTiltPositionAbsolute(tilt);
        if(!worked) {
            setDesiredPanPositionAbsolute(prev_pan);
            setPositionExecutionMode(previous_mode);
            return false;
        }
        worked = awaitPositionCommandCompletion();
        if(!worked) {
            setDesiredPanPositionAbsolute(prev_pan);
            setDesiredTiltPositionAbsolute(prev_tilt);
            setPositionExecutionMode(previous_mode);
            return false;
        }
        worked = setPositionExecutionMode(previous_mode);
        if(!worked) {
            return false;
        }
        return true;
    }



    //WARNING: NOT SUPPORTED IN OLDER PTU VERSIONS, NOT TESTED (PTU VERSION TOO OLD)
    bool PTUFree::setPreset(int preset_index, short int pan, short int tilt) {
        short int prev_pan = getDesiredPanPosition();
        if(!setDesiredPanPositionAbsolute(pan)) {
            return false;
        }
        if(!setDesiredTiltPositionAbsolute(tilt)) {
            setDesiredPanPositionAbsolute(prev_pan);
            return false;
        }
        awaitPositionCommandCompletion();
        return setPreset(preset_index);

    }


    //WARNING: NOT SUPPORTED IN OLDER PTU VERSIONS, NOT TESTED (PTU VERSION TOO OLD)
    bool PTUFree::setPreset(int preset_index) {
        if((0 > preset_index) || (preset_index > 32)) {
            return false;
        }
        std::string command = "xs" + boost::lexical_cast<std::string>(preset_index) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting position preset: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    //WARNING: NOT SUPPORTED IN OLDER PTU VERSIONS, NOT TESTED (PTU VERSION TOO OLD)
    bool PTUFree::gotoPreset(int preset_index) {
        std::string command = "xg" + boost::lexical_cast<std::string>(preset_index) + " ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while going to position preset: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    //WARNING: NOT SUPPORTED IN OLDER PTU VERSIONS, NOT TESTED (PTU VERSION TOO OLD)
    bool PTUFree::clearPreset() {
        std::string command = "xc ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while clearing position presets: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setSpeedControlMode(long mode) {
        std::string command;
        if(mode == INDEPENDENT_SPEED_MODE) {
            command = "ci ";
        }
        else if (mode == PURE_VELOCITY_CONTROL_MODE) {
            command = "cv ";
        }
        else {
            printf("Error while setting speed control mode: Unknown command\n");
            return false;
        }
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting speed control mode: %s\n", response_parts[1].c_str());
            return false;
        }
    }


    long PTUFree::getSpeedControlMode() {
        std::string command = "c ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            if(response_parts[1].find("independent") != std::string::npos) {
                return INDEPENDENT_SPEED_MODE;
            }
            else {
                return PURE_VELOCITY_CONTROL_MODE;
            }
        }
        else {
            printf("Error while getting speed control mode: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }

    bool PTUFree::reset() {
        std::string command = "r ";
        std::string response = communicate(command);
        printf("RESET MESSAGE: %s\n", response.c_str());
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while resetting the ptu unit: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setResetMode(long mode) {
        std::string command;
        if(mode == NO_RESET_MODE) {
            command = "rd ";
        }
        else if (mode == PAN_ONLY_RESET_MODE) {
            command = "rp ";
        }
        else if (mode == TILT_ONLY_RESET_MODE) {
            command = "rt ";
        }
        else if (mode == BOTH_RESET_MODE) {
            command = "re ";
        }
        else {
            printf("Error while setting reset mode: Unknown command\n");
            return false;
        }
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting reset mode: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::saveDefault() {
        std::string command = "ds ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while saving current axis settings as default: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::restoreDefault() {
        std::string command = "dr ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while restoring default axis settings: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::restoreFactoryDefault() {
        std::string command = "df ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while restoring factory default axis settings: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setPanStationaryPowerMode(long mode) {
        std::string command;
        if(mode == REGULAR_HOLD_POWER_MODE) {
            command = "phr ";
        }
        else if (mode == LOW_HOLD_POWER_MODE) {
            command = "phl ";
        }
        else if (mode == OFF_HOLD_POWER_MODE) {
            command = "pho ";
        }
        else {
            printf("Error while setting pan stationary power mode: Unknown command\n");
            return false;
        }
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting pan stationary power mode: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    bool PTUFree::setTiltStationaryPowerMode(long mode) {
        std::string command;
        if(mode == REGULAR_HOLD_POWER_MODE) {
            command = "thr ";
        }
        else if (mode == LOW_HOLD_POWER_MODE) {
            command = "thl ";
        }
        else if (mode == OFF_HOLD_POWER_MODE) {
            command = "tho ";
        }
        else {
            printf("Error while setting pan stationary power mode: Unknown command\n");
            return false;
        }
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting pan stationary power mode: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    long PTUFree::getPanStationaryPowerMode() {
        std::string command = "ph ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            if(response_parts[1].find("REGULAR") != std::string::npos) {
                return REGULAR_HOLD_POWER_MODE;
            }
            if(response_parts[1].find("LOW") != std::string::npos) {
                return LOW_HOLD_POWER_MODE;
            }
            else {
                return OFF_HOLD_POWER_MODE;
            }
        }
        else {
            printf("Error while getting pan stationary power mode: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }
    long PTUFree::getTiltStationaryPowerMode() {
        std::string command = "th ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            if(response_parts[1].find("REGULAR") != std::string::npos) {
                return REGULAR_HOLD_POWER_MODE;
            }
            if(response_parts[1].find("LOW") != std::string::npos) {
                return LOW_HOLD_POWER_MODE;
            }
            else {
                return OFF_HOLD_POWER_MODE;
            }
        }
        else {
            printf("Error while getting tilt stationary power mode: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }

    bool PTUFree::setPanInMotionPowerMode(long mode) {
        std::string command;
        if(mode == HIGH_MOVE_POWER_MODE) {
            command = "pmh ";
        }
        else if (mode == REGULAR_MOVE_POWER_MODE) {
            command = "pmr ";
        }
        else if (mode == LOW_MOVE_POWER_MODE) {
            command = "pml ";
        }
        else {
            printf("Error while setting pan in-motion power mode: Unknown command\n");
            return false;
        }
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting pan in-motion power mode: %s\n", response_parts[1].c_str());
            return false;
        }
    }
    bool PTUFree::setTiltInMotionPowerMode(long mode) {
        std::string command;
        if(mode == HIGH_MOVE_POWER_MODE) {
            command = "tmh ";
        }
        else if (mode == REGULAR_MOVE_POWER_MODE) {
            command = "tmr ";
        }
        else if (mode == LOW_MOVE_POWER_MODE) {
            command = "tml ";
        }
        else {
            printf("Error while setting tilt in-motion power mode: Unknown command\n");
            return false;
        }
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            return true;
        }
        else {
            printf("Error while setting tilt in-motion power mode: %s\n", response_parts[1].c_str());
            return false;
        }
    }

    long PTUFree::getPanInMotionPowerMode() {
        std::string command = "pm ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            if(response_parts[1].find("REGULAR") != std::string::npos) {
                return REGULAR_MOVE_POWER_MODE;
            }
            if(response_parts[1].find("LOW") != std::string::npos) {
                return LOW_MOVE_POWER_MODE;
            }
            else {
                return HIGH_MOVE_POWER_MODE;
            }
        }
        else {
            printf("Error while getting pan in-motion power mode: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }
    long PTUFree::getTiltInMotionPowerMode() {
        std::string command = "tm ";
        std::string response = communicate(command);
        std::vector<std::string> response_parts = evaluateResponse(response);
        if(response_parts[0].at(0) == '*') {
            if(response_parts[1].find("REGULAR") != std::string::npos) {
                return REGULAR_MOVE_POWER_MODE;
            }
            if(response_parts[1].find("LOW") != std::string::npos) {
                return LOW_MOVE_POWER_MODE;
            }
            else {
                return HIGH_MOVE_POWER_MODE;
            }
        }
        else {
            printf("Error while getting tilt in-motion power mode: %s\n", response_parts[1].c_str());
            return ERROR;
        }
    }

    std::string PTUFree::getErrorString(boost::system::error_code error) {
        switch (error.value()) {
            case boost::system::errc::success: {
                return "success";
            }
            break;
            case boost::system::errc::address_family_not_supported: {
                return "address_family_not_supported | EAFNOSUPPORT";
            }
            break;
            case boost::system::errc::address_in_use: {
                return "address_in_use | EADDRINUSE";
            }
            break;
            case boost::system::errc::address_not_available: {
                return "address_not_available | EADDRNOTAVAIL";
            }
            break;
            case boost::system::errc::already_connected: {
                return "already_connected | EISCONN";
            }
            break;
            case boost::system::errc::argument_list_too_long: {
                return "argument_list_too_long | E2BIG";
            }
            break;
            case boost::system::errc::argument_out_of_domain: {
                return "argument_out_of_domain | EDOM";
            }
            break;
            case boost::system::errc::bad_address: {
                return "bad_address | EFAULT";
            }
            break;
            case boost::system::errc::bad_file_descriptor: {
                return "bad_file_descriptor | EBADF";
            }
            break;
            case boost::system::errc::bad_message: {
                return "bad_message | EBADMSG";
            }
            break;
            case boost::system::errc::broken_pipe: {
                return "broken_pipe | EPIPE";
            }
            break;
            case boost::system::errc::connection_aborted: {
                return "connection_aborted | ECONNABORTED";
            }
            break;
            case boost::system::errc::connection_already_in_progress: {
                return "connection_already_in_progress | EALREADY";
            }
            break;
            case boost::system::errc::connection_refused: {
                return "connection_refused | ECONNREFUSED";
            }
            break;
            case boost::system::errc::connection_reset: {
                return "connection_reset | ECONNRESET";
            }
            break;
            case boost::system::errc::cross_device_link: {
                return "cross_device_link | EXDEV";
            }
            break;
            case boost::system::errc::destination_address_required: {
                return "destination_address_required | EDESTADDRREQ";
            }
            break;
            case boost::system::errc::device_or_resource_busy: {
                return "device_or_resource_busy | EBUSY";
            }
            break;
            case boost::system::errc::directory_not_empty: {
                return "directory_not_empty | ENOTEMPTY";
            }
            break;
            case boost::system::errc::executable_format_error: {
                return "executable_format_error | ENOEXEC";
            }
            break;
            case boost::system::errc::file_exists: {
                return "file_exists | EEXIST";
            }
            break;
            case boost::system::errc::file_too_large: {
                return "file_too_large | EFBIG";
            }
            break;
            case boost::system::errc::filename_too_long: {
                return "filename_too_long | ENAMETOOLONG";
            }
            break;
            case boost::system::errc::function_not_supported: {
                return "function_not_supported | ENOSYS";
            }
            break;
            case boost::system::errc::host_unreachable: {
                return "host_unreachable | EHOSTUNREACH";
            }
            break;
            case boost::system::errc::identifier_removed: {
                return "identifier_removed | EIDRM";
            }
            break;
            case boost::system::errc::illegal_byte_sequence: {
                return "illegal_byte_sequence | EILSEQ";
            }
            break;
            case boost::system::errc::inappropriate_io_control_operation: {
                return "inappropriate_io_control_operation | ENOTTY";
            }
            break;
            case boost::system::errc::interrupted: {
                return "interrupted | EINTR";
            }
            break;
            case boost::system::errc::invalid_argument: {
                return "invalid_argument | EINVAL";
            }
            break;
            case boost::system::errc::invalid_seek: {
                return "invalid_seek | ESPIPE";
            }
            break;
            case boost::system::errc::io_error: {
                return "io_error | EIO";
            }
            break;
            case boost::system::errc::is_a_directory: {
                return "is_a_directory | EISDIR";
            }
            break;
            case boost::system::errc::message_size: {
                return "message_size | EMSGSIZE";
            }
            break;
            case boost::system::errc::network_down: {
                return "network_down | ENETDOWN";
            }
            break;
            case boost::system::errc::network_reset: {
                return "network_reset | ENETRESET";
            }
            break;
            case boost::system::errc::network_unreachable: {
                return "network_unreachable | ENETUNREACH";
            }
            break;
            case boost::system::errc::no_buffer_space: {
                return "no_buffer_space | ENOBUFS";
            }
            break;
            case boost::system::errc::no_child_process: {
                return "no_child_process | ECHILD";
            }
            break;
            case boost::system::errc::no_link: {
                return "no_link | ENOLINK";
            }
            break;
            case boost::system::errc::no_lock_available: {
                return "no_lock_available | ENOLCK";
            }
            break;
            case boost::system::errc::no_message_available: {
                return "no_message_available | ENODATA";
            }
            break;
            case boost::system::errc::no_message: {
                return "no_message | ENOMSG";
            }
            break;
            case boost::system::errc::no_protocol_option: {
                return "no_protocol_option | ENOPROTOOPT";
            }
            break;
            case boost::system::errc::no_space_on_device: {
                return "no_space_on_device | ENOSPC";
            }
            break;
            case boost::system::errc::no_stream_resources: {
                return "no_stream_resources | ENOSR";
            }
            break;
            case boost::system::errc::no_such_device_or_address: {
                return "no_such_device_or_address | ENXIO";
            }
            break;
            case boost::system::errc::no_such_device: {
                return "no_such_device | ENODEV";
            }
            break;
            case boost::system::errc::no_such_file_or_directory: {
                return "no_such_file_or_directory | ENOENT";
            }
            break;
            case boost::system::errc::no_such_process: {
                return "no_such_process | ESRCH";
            }
            break;
            case boost::system::errc::not_a_directory: {
                return "not_a_directory | ENOTDIR";
            }
            break;
            case boost::system::errc::not_a_socket: {
                return "not_a_socket | ENOTSOCK";
            }
            break;
            case boost::system::errc::not_a_stream: {
                return "not_a_stream | ENOSTR";
            }
            break;
            case boost::system::errc::not_connected: {
                return "not_connected | ENOTCONN";
            }
            break;
            case boost::system::errc::not_enough_memory: {
                return "not_enough_memory | ENOMEM";
            }
            break;
            case boost::system::errc::not_supported: {
                return "not_supported | ENOTSUP";
            }
            break;
            case boost::system::errc::operation_canceled: {
                return "operation_canceled | ECANCELED";
            }
            break;
            case boost::system::errc::operation_in_progress: {
                return "operation_in_progress | EINPROGRESS";
            }
            break;
            case boost::system::errc::operation_not_permitted: {
                return "operation_not_permitted | EPERM";
            }
            break;
            //Uses internal same error code as operation_not_supported
            //case boost::system::errc::operation_not_supported: {
            //    return "operation_not_supported | EOPNOTSUPP";
            //}
            //break;
            //Uses internal same error code as recources_unavailable_try_again
            //case boost::system::errc::operation_would_block: {
            //    return "operation_would_block | EWOULDBLOCK";
            //}
            //break;
            case boost::system::errc::owner_dead: {
                return "owner_dead | EOWNERDEAD";
            }
            break;
            case boost::system::errc::permission_denied: {
                return "permission_denied | EACCES";
            }
            break;
            case boost::system::errc::protocol_error: {
                return "protocol_error | EPROTO";
            }
            break;
            case boost::system::errc::protocol_not_supported: {
                return "protocol_not_supported | EPROTONOSUPPORT";
            }
            break;
            case boost::system::errc::read_only_file_system: {
                return "read_only_file_system | EROFS";
            }
            break;
            case boost::system::errc::resource_deadlock_would_occur: {
                return "resource_deadlock_would_occur | EDEADLK";
            }
            break;
            case boost::system::errc::resource_unavailable_try_again: {
                return "resource_unavailable_try_again | EAGAIN";
            }
            break;
            case boost::system::errc::result_out_of_range: {
                return "result_out_of_range | ERANGE";
            }
            break;
            case boost::system::errc::state_not_recoverable: {
                return "state_not_recoverable | ENOTRECOVERABLE";
            }
            break;
            case boost::system::errc::stream_timeout: {
                return "stream_timeout | ETIME";
            }
            break;
            case boost::system::errc::text_file_busy: {
                return "text_file_busy | ETXTBSY";
            }
            break;
            case boost::system::errc::timed_out: {
                return "timed_out | ETIMEDOUT";
            }
            break;
            case boost::system::errc::too_many_files_open_in_system: {
                return "too_many_files_open_in_system | ENFILE";
            }
            break;
            case boost::system::errc::too_many_files_open: {
                return "too_many_files_open | EMFILE";
            }
            break;
            case boost::system::errc::too_many_links: {
                return "too_many_links | EMLINK";
            }
            break;
            //Most likely not available in current boost version (compiler error)
            //case boost::system::errc::too_many_synbolic_link_levels: {
            //    return "too_many_synbolic_link_levels | ELOOP";
            //}
            //break;
            case boost::system::errc::value_too_large: {
                return "value_too_large | EOVERFLOW";
            }
            break;
            case boost::system::errc::wrong_protocol_type: {
                return "wrong_protocol_type | EPROTOTYPE";
            }
            break;
            default: {
                return "unknown error";
            }
            break;
        }
    }

    void PTUFree::test() {

            long pan_pos = getCurrentPanPosition();
            if(pan_pos != 0) {
                printf("Error getting initial pan position. Expected 0, got %ld\n", pan_pos);
                return;
            }

            if(!setDesiredPanPositionAbsolute(300)) {
                printf("Error while setting pan position to 300");
                return;
            }

            long desired_pan_position = getDesiredPanPosition();
            if(desired_pan_position != 300) {
                    printf("Error while setting or getting (desired) pan position. Expected 300, desired %ld", desired_pan_position);
                    return;
            }

            pan_pos = getCurrentPanPosition();
            if((pan_pos < 0) || (pan_pos >= 300)) {
                printf("Error getting pan position while ptu is moving position. Expected value between 0 and 300, got %ld\n", pan_pos);
                if(pan_pos == 300) {
                    printf("Pan Position was 300 but it should not be at the moment. This can also happen due to to slow serial port or sceduling, testing will be continued therefore.");
                }
                else {
                    return;
                }
            }

            pan_pos = getCurrentPanPosition();
            if((pan_pos < 0) || (pan_pos >= 300)) {
                printf("Error getting pan position while ptu is moving position. Expected value between 0 and 300, got %ld\n", pan_pos);
                if(pan_pos == 300) {
                    printf("Pan Position was 300 but it should not be at the moment. This can also happen due to to slow serial port or sceduling, testing will be continued therefore.");
                }
                else {
                    return;
                }
            }

            if(!awaitPositionCommandCompletion()) {
                printf("Execution of await command failed");
                return;
            }

            pan_pos = getCurrentPanPosition();
            if((pan_pos != 300)) {
                printf("Setting pan position, getting pan position or await command failed. Expected pan position 300, but got %ld\n", pan_pos);
                return;
            }

            long tilt_pos = getCurrentTiltPosition();
            if(tilt_pos != 0) {
                printf("Error getting initial tilt position. Expected 0, got %ld\n", tilt_pos);
                return;
            }

            if(!setDesiredTiltPositionAbsolute(-350)) {
                printf("Error while setting tilt position to -350");
                return;
            }

            long desired_tilt_position = getDesiredTiltPosition();
            if(desired_tilt_position != -350) {
                    printf("Error while setting or getting (desired) tilt position. Expected -350, desired %ld", desired_tilt_position);
                    return;
            }


            tilt_pos = getCurrentTiltPosition();
            if((tilt_pos > 0) || (tilt_pos <= -350)) {
                printf("Error getting tilt position while ptu is moving position. Expected value between 0 and -350, got %ld\n", tilt_pos);
                if(pan_pos == -350) {
                    printf("Tilt Position was -350 but it should not be at the moment. This can also happen due to to slow serial port or sceduling, testing will be continued therefore.");
                }
                else {
                    return;
                }
            }

            tilt_pos = getCurrentTiltPosition();
            if((tilt_pos > 0) || (tilt_pos <= -350)) {
                printf("Error getting tilt position while ptu is moving position. Expected value between 0 and -350, got %ld\n", tilt_pos);
                if(tilt_pos == -350) {
                    printf("Tilt Position was -350 but it should not be at the moment. This can also happen due to to slow serial port or sceduling, testing will be continued therefore.");
                }
                else {
                    return;
                }
            }

            if(!awaitPositionCommandCompletion()) {
                printf("Execution of await command failed");
                return;
            }

            tilt_pos = getCurrentTiltPosition();
            if((tilt_pos != -350)) {
                printf("Setting tilt position, getting tilt position or await command failed. Expected tilt position -350, but got %ld\n", tilt_pos);
                return;
            }

            if(!setDesiredPanPositionRelative(100)) {
                printf("Setting pan position realtiv failed.");
                return;
            }

            if(!awaitPositionCommandCompletion()) {
                printf("Execution of await command failed");
                return;
            }

            pan_pos = getCurrentPanPosition();
            if(pan_pos != 400) {
                printf("After setting relative pan position, position == 400 was expected, but position was %ld\n", pan_pos);
            }


            if(!setDesiredTiltPositionRelative(100)) {
                printf("Setting pan position realtiv failed.");
                return;
            }

            if(!awaitPositionCommandCompletion()) {
                printf("Execution of await command failed");
                return;
            }

            tilt_pos = getCurrentTiltPosition();
            if(tilt_pos != -250) {
                printf("After setting relative pan position, position == -250 was expected, but position was %ld\n", tilt_pos);
            }



            printf("SUCCESS: Setting pan and tilt testing successfull\n");



            if(!setPositionLimitEnforcementMode(FACTORY_LIMITS_ENABLED)) {
                printf("Setting position limit enforcement to factory limits enabled failed\n");
                return;
            }
            //some of those methods can take a while
            sleep(6);
            long limit_enforcement_mode = getPositionLimitEnforcementMode();
            if(limit_enforcement_mode != FACTORY_LIMITS_ENABLED) {
                if(limit_enforcement_mode == USER_DEFINED_LIMITS_ENABLED) {
                    printf("setPositionLimitEnforcementMode or the corresponding get method failed. FACOTRY_LIMITS_ENABLED was expected, USER_DEFINED_LIMITS_ENABLE was retrieved\n");
                }
                else if (limit_enforcement_mode == LIMITS_DISABLED) {
                    printf("setPositionLimitEnforcementMode or the corresponding get method failed. FACOTRY_LIMITS_ENABLED was expected, LIMITS_DISABLED was retrieved\n");
                }
                else if(limit_enforcement_mode == ERROR) {
                    printf("getPositionLimitEnforcementMode failed with ERROR\n");
                }
                else {
                    printf("get_posiion_limit_enforcement_mode returned unexpected value after setting FACTORY_LIMITS_ENABLED\n");
                }
                return;
            }


            long pan_upper_limit = getFactoryMaximumPanPositionLimit();
            long pan_lower_limit = getFactoryMinimumPanPositionLimit();
            long tilt_upper_limit = getFactoryMaximumTiltPositionLimit();
            long tilt_lower_limit = getFactoryMinimumTiltPositionLimit();

            //The following also tells if the get_current methods with Factory Limits work, because the values returned by get_factory... are recieved from that function at launch
            if(pan_upper_limit == ERROR) {
                printf("Function getCurrentUsedMaximumPanPositionLimit returned ERROR when used in FACTORY_LIMITS_ENABLES position enforcement mode\n");
                return;
            }
            if(pan_lower_limit == ERROR) {
                printf("Function getCurrentUsedMinimumPanPositionLimit returned ERROR when used in FACTORY_LIMITS_ENABLES position enforcement mode\n");
                return;
            }
            if(tilt_upper_limit == ERROR) {
                printf("Function getCurrentUsedMaximumTiltPositionLimit returned ERROR when used in FACTORY_LIMITS_ENABLES position enforcement mode\n");
                return;
            }
            if(tilt_lower_limit == ERROR) {
                printf("Function getCurrentUsedMinimumTiltPositionLimit returned ERROR when used in FACTORY_LIMITS_ENABLES position enforcement mode\n");
                return;
            }

            if(setDesiredPanPositionAbsolute(pan_upper_limit + 1)) {
                printf("FACTORY_LIMITS_ENABLED mode or any of the get functions did not work properly. Trying to move pan unit over the upper limit did not result in error\n");
                return;
            }
            if(setDesiredPanPositionAbsolute(pan_lower_limit - 1)) {
                printf("FACTORY_LIMITS_ENABLED mode or any of the get functions did not work properly. Trying to move pan unit over the lower limit did not result in error\n");
                return;
            }
            if(setDesiredTiltPositionAbsolute(tilt_upper_limit + 1)) {
                printf("FACTORY_LIMITS_ENABLED mode or any of the get functions did not work properly. Trying to move tilt unit over the upper limit did not result in error\n");
                return;
            }
            if(setDesiredTiltPositionAbsolute(tilt_lower_limit - 1)) {
                printf("FACTORY_LIMITS_ENABLED mode or any of the get functions did not work properly. Trying to move tilt unit over the lower limit did not result in error\n");
                return;
            }

            sleep(6);
            //Does not work on older ptus
            /*
            if(!setMaximumPanPositionLimit(1000)) {
                printf("Error while setting maximum_pan_position_limit to 1000\n");
                return;
            }
            if(!setMinimumPanPositionLimit(-1000)) {
                printf("Error while setting minimum_pan_position_limit to -1000\n");
                return;
            }
            if(!setMinimumTiltPositionLimit(-300)) {
                printf("Error while setting minimum_tilt_position_limit to -300\n");
                return;
            }
            if(!setMaximumTiltPositionLimit(300)) {
                printf("Error while setting maximum_tilt_position_limit to 300\n");
                return;
            }

            sleep(6);

            long min_user_pan_limit = getUserMinimumPanPositionLimit();
            long max_user_pan_limit = getUserMaximumPanPositionLimit();
            long min_user_tilt_limit = getUserMinimumTiltPositionLimit();
            long max_user_tilt_limit = getUserMaximumTiltPositionLimit();
            if(min_user_pan_limit != -1000) {
                printf("Error while getting minimum user pan position limit: -1000 expected, got %ld\n", min_user_pan_limit);
                return;
            }
            if(max_user_pan_limit != 1000) {
                printf("Error while getting maximum user pan position limit: 1000 expected, got %ld\n", max_user_pan_limit);
                return;
            }
            if(min_user_tilt_limit != -300) {
                printf("Error while getting minimum user tilt position limit: -300 expected, got %ld\n", min_user_tilt_limit);
                return;
            }
            if(max_user_tilt_limit != 300) {
                printf("Error while getting maximum user tilt position limit: 300 expected, got %ld\n", max_user_tilt_limit);
                return;
            }

            if(!setPositionLimitEnforcementMode(USER_DEFINED_LIMITS_ENABLED)) {
                printf("Setting position limit enforcement mode to USER_DEFINED_LIMITS_ENABLED failed");
                return;
            }
            limit_enforcement_mode = getPositionLimitEnforcementMode();

            if(limit_enforcement_mode != USER_DEFINED_LIMITS_ENABLED) {
                printf("Getting position limit enforcement mode failed. Expected: USER_DEFINED_LIMITS_ENABLED, Got: %ld\n", limit_enforcement_mode);
                return;
            }

            long min_current_pan_limit = getCurrentUsedMinimumPanPositionLimit();
            long max_current_pan_limit = getCurrentUsedMaximumPanPositionLimit();
            long min_current_tilt_limit = getCurrentUsedMinimumTiltPositionLimit();
            long max_current_tilt_limit = getCurrentUsedMaximumTiltPositionLimit();
            if(min_current_pan_limit != -1000) {
                printf("Error while getting minimum current used pan position limit: -1000 expected, got %ld\n", min_current_pan_limit);
                return;
            }
            if(max_current_pan_limit != 1000) {
                printf("Error while getting maximum current used pan position limit: 1000 expected, got %ld\n", max_current_pan_limit);
                return;
            }
            if(min_current_tilt_limit != -300) {
                printf("Error while getting minimum current used tilt position limit: -300 expected, got %ld\n", min_current_tilt_limit);
                return;
            }
            if(max_current_tilt_limit != 300) {
                printf("Error while getting maximum current used tilt position limit: 300 expected, got %ld\n", max_current_tilt_limit);
                return;
            }


            */
            if(!setPositionLimitEnforcementMode(LIMITS_DISABLED)) {
                printf("Setting position limit enforcement mode to LIMITS_DISABLED failed");
                return;
            }

            limit_enforcement_mode = getPositionLimitEnforcementMode();

            if(limit_enforcement_mode != LIMITS_DISABLED) {
                printf("Getting position limit enforcement mode failed. Expected: LIMITS_DISABLED, Got: %ld\n", limit_enforcement_mode);
                return;
            }

            if(!setDesiredPanPositionAbsolute(4000)) {
                printf("Setting position out of limits with LIMITS_DISABLED failed\n");
                return;
            }

            sleep(6);

            setDesiredPanPositionAbsolute(300);

            sleep(6);

            printf("Testing Limit Enforcement succeeded\n");


            setDesiredPanPositionAbsolute(3000);
            awaitPositionCommandCompletion();


            long cur_pan_speed = getCurrentPanSpeed();
            long cur_tilt_speed = getCurrentTiltSpeed();
            if(cur_pan_speed != 0) {
                printf("Error while getting current pan speed. 0 expected but got %ld\n", cur_pan_speed);
                return;
            }
            if(cur_tilt_speed != 0) {
                printf("Error while getting current tilt speed. 0 expected but got %ld\n", cur_tilt_speed);
                return;
            }

            long des_pan_speed = getDesiredPanSpeed();
            long des_tilt_speed = getDesiredTiltSpeed();
            if(des_pan_speed <= 0) {
                printf("Error while getting desired pan speed. Value >= 0 expected but got %ld\n", des_pan_speed);
                return;
            }
            if(des_tilt_speed <= 0) {
                printf("Error while getting desired tilt speed. Value >= 0 expected but got %ld\n", des_tilt_speed);
                return;
            }

            if(!setDesiredPanSpeedAbsolute((des_pan_speed - 1))) {
                printf("Error while setting desired pan speed absolute\n");
                return;
            }

            long mod_desired_pan_speed = getDesiredPanSpeed();
            if(mod_desired_pan_speed != (des_pan_speed - 1)) {
                printf("Error while getting or setting desired pan speed (absolute set). Expected %ld - 1, but got %ld\n", des_pan_speed, mod_desired_pan_speed);
                return;
            }

            if(!setDesiredTiltSpeedAbsolute((des_tilt_speed - 2))) {
                printf("Error while setting desired tilt speed absolute\n");
                return;
            }

            long mod_desired_tilt_speed = getDesiredTiltSpeed();
            if(mod_desired_tilt_speed != (des_tilt_speed - 2)) {
                printf("Error while getting or setting desired tilt speed (absolute set). Expected %ld - 2, but got %ld\n", des_tilt_speed, mod_desired_tilt_speed);
                return;
            }

            //WARNING: OFFSET FROM CURRENT (NOT DESIRED) SPEED IS SET TO DESIRED SPEED (Example: Current speed = 0, Desired Speed = 500. setDesiredPanSpeedRelative will result in
            //current speed being 0 and desired speed being 0 + 100 = 100
            if(!setDesiredPanSpeedRelative(des_pan_speed - 2)) {
                printf("Error while setting desired pan speed realtive\n");
                return;
            }

            mod_desired_pan_speed = getDesiredPanSpeed();
            if(mod_desired_pan_speed != (des_pan_speed - 2)) {
                printf("Error while getting or setting desired pan speed (realtive set). Expected %ld - 2, but got %ld\n", des_pan_speed, mod_desired_pan_speed);
                return;
            }

            if(!setDesiredTiltSpeedRelative(des_tilt_speed - 4)) {
                printf("Error while setting desired tilt speed relative\n");
                return;
            }

            mod_desired_tilt_speed = getDesiredTiltSpeed();
            if(mod_desired_tilt_speed != (des_tilt_speed - 4)) {
                printf("Error while getting or setting desired tilt speed (relative set). Expected %ld - 4, but got %ld\n", des_tilt_speed, mod_desired_tilt_speed);
                return;
            }

            long upper_limit_pan = getPanUpperSpeedLimit();
            long lower_limit_pan = getPanLowerSpeedLimit();
            long upper_limit_tilt = getTiltUpperSpeedLimit();
            long lower_limit_tilt = getTiltLowerSpeedLimit();

            if(upper_limit_pan == ERROR) {
                printf("Error while getting upper speed limit pan.\n");
                return;
            }
            if(lower_limit_pan == ERROR) {
                printf("Error while getting lower speed limit pan.\n");
                return;
            }
            if(upper_limit_tilt == ERROR) {
                printf("Error while getting upper speed limit tilt.\n");
                return;
            }
            if(lower_limit_tilt == ERROR) {
                printf("Error while getting lower speed limit tilt.\n");
                return;
            }

            if(!setDesiredPanUpperSpeedLimit(upper_limit_pan - 1)) {
                printf("Error while setting upper speed limit pan\n");
                return;
            }
            if(!setDesiredPanLowerSpeedLimit(57)) {
                printf("Error while setting lower speed limit pan\n");
                return;
            }
            if(!setDesiredTiltUpperSpeedLimit(upper_limit_tilt - 2)) {
                printf("Error while setting upper speed limit tilt\n");
                return;
            }
            if(!setDesiredTiltLowerSpeedLimit(58)) {
                printf("Error while setting lower speed limit tilt\n");
                return;
            }

            //Changing speed bounds is not possible on the fly, needs long break
            sleep(6);

            //long mod_upper_limit_pan = getPanUpperSpeedLimit();
            long mod_lower_limit_pan = getPanLowerSpeedLimit();
            //long mod_upper_limit_tilt = getTiltUpperSpeedLimit();
            long mod_lower_limit_tilt = getTiltLowerSpeedLimit();
            /*
            if(mod_upper_limit_pan != (upper_limit_pan - 1)) {
                printf("Error while getting or setting upper speed limit pan. Expected %ld - 1, but got %ld\n", upper_limit_pan, mod_upper_limit_pan);
                return;
            }
            */
            if(mod_lower_limit_pan != (57)) {
                printf("Error while getting or setting lower speed limit pan. Expected 57, but got %ld\n", mod_lower_limit_pan);
                return;
            }
            /*
            if(mod_upper_limit_tilt != (upper_limit_tilt - 2)) {
                printf("Error while getting or setting upper speed limit tilt. Expected %ld - 2, but got %ld\n", upper_limit_tilt, mod_upper_limit_tilt);
                return;
            }
            */
            if(mod_lower_limit_tilt != (58)) {
                printf("Error while getting or setting lower speed limit tilt. Expected 58, but got %ld\n", mod_lower_limit_tilt);
                return;
            }

            long pan_base_speed = getPanBaseSpeed();
            long tilt_base_speed = getTiltBaseSpeed();
            if(pan_base_speed <= 0) {
                printf("Error while getting pan_base_speed\n");
                return;
            }
            if(tilt_base_speed <= 0) {
                printf("Error while getting tilt_base_speed\n");
                return;
            }
            if(!setPanBaseSpeed((pan_base_speed - 1))) {
                printf("Error while setting pan_base_speed\n");
                return;
            }
            if(!setTiltBaseSpeed((tilt_base_speed - 1))) {
                printf("Error while setting tilt_base_speed\n");
                return;
            }
            long mod_pan_base_speed = getPanBaseSpeed();
            long mod_tilt_base_speed = getTiltBaseSpeed();
            if((pan_base_speed - 1) != mod_pan_base_speed) {
                printf("Error while getting pan_base_speed. Expected %ld - 1, got %ld\n", pan_base_speed, mod_pan_base_speed);
                return;
            }
            if((tilt_base_speed - 1) != mod_tilt_base_speed) {
                printf("Error while getting tilt_base_speed. Expected %ld - 1, got %ld\n", tilt_base_speed, mod_tilt_base_speed);
                return;
            }

            long pan_accel = getPanAcceleartion();
            long tilt_accel = getTiltAcceleartion();
            if(pan_accel <= 0) {
                printf("Error while getting pan acceleration\n");
                return;
            }
            if(tilt_accel <= 0) {
                printf("Error while getting tilt acceleration\n");
                return;
            }
            if(!setDesiredPanAccelerationAbsolute((pan_accel - 1))) {
                printf("Setting desired pan acceleration absolute failed\n");
                return;
            }
            if(!setDesiredTiltAccelerationAbsolute((tilt_accel - 1))) {
                printf("Setting desired pan acceleration absolute failed\n");
                return;
            }
            long mod_pan_accel = getPanAcceleartion();
            long mod_tilt_accel = getTiltAcceleartion();
            if((pan_accel - 1) != mod_pan_accel) {
                printf("Error while getting pan acceleration. Expected %ld - 1, got %ld\n", pan_accel, mod_pan_accel);
                return;
            }
            if((tilt_accel - 1) != mod_tilt_accel) {
                printf("Error while getting tilt acceleration. Expected %ld - 1, got %ld\n", tilt_accel, mod_tilt_accel);
                return;
            }

            printf("Speed testing successfull\n");

            double pan_res = getPanResolution();
            double tilt_res = getTiltResolution();

            if(pan_res <= 0.0) {
                printf("Error while getting pan resolution");
                return;
            }
            if(tilt_res <= 0.0) {
                printf("Error while getting tilt resolution");
                return;
            }

            printf("Resolution testing successfull with Pan Resolution %f und Tilt Resolution %f\n", pan_res, tilt_res);


            if(!setPositionExecutionMode(SLAVED_POSITION_EXECUTION_MODE)) {
                printf("Error while setting position execution mode to SLAVED_POSITION_EXECUTION_MODE\n");
                return;
            }
            long pos_ex_mode = getPositionExecutionMode();
            if(pos_ex_mode != SLAVED_POSITION_EXECUTION_MODE) {
                printf("Error while getting or setting position execution mode. SLAVED_POSITION_EXECUTION_MODE expected, got %ld\n", pos_ex_mode);
                return;
            }
            if(!setPositionExecutionMode(IMMEDIATE_POSITION_EXECUTION_MODE)) {
                printf("Error while setting position execution mode to IMMEDIATE_POSITION_EXECUTION_MODE\n");
                return;
            }
            pos_ex_mode = getPositionExecutionMode();
            if(pos_ex_mode != IMMEDIATE_POSITION_EXECUTION_MODE) {
                printf("Error while getting or setting position execution mode. IMMEDIATE_POSITION_EXECUTION_MODE expected, got %ld\n", pos_ex_mode);
                return;
            }



            if(!halt(HALT_BOTH)) {
                printf("Error while executing halt command with HALT_BOTH\n");
                return;
            }
            if(!halt(HALT_PAN_ONLY)) {
                printf("Error while executing halt command with HALT_PAN_ONLY\n");
                return;
            }
            if(!halt(HALT_TILT_ONLY)) {
                printf("Error while executing halt command with HALT_TILT_ONLY\n");
                return;
            }

            if(!setDesiredPanTiltPositionAbsoluteSlaved(300, 300)) {
                printf("Problem while setting preset with current location\n");
                return;
            }
            awaitPositionCommandCompletion();
            long cur_pan_pos = getCurrentPanPosition();
            long cur_tilt_pos = getCurrentTiltPosition();
            if(cur_pan_pos != 300 || cur_tilt_pos != 300) {
                printf("Synchronous setting of pan and tilt axis in setDesiredPanTiltPositionAbsoluteSlaved did not work properly, false position was set\n");
                return;
            }
            /*
            if(!setPreset(1)) {
                printf("Problem with setting implicit preset\n");
                return;
            }
            setDesiredTiltPositionAbsolute(0);
            sleep(6);
            if(getCurrentTiltPosition()!= 0) {
                printf("Probably problem after setting pan_tilt synchronous with setDesiredPanTiltPositionAbsoluteSlaved. Next regular pan only setting command returned wrong result or was not executed\n");
                return;
            }
            if(!setPreset(2, -300, -300)) {
                printf("Problem while setting preset with explizit pan and tilt pasing\n");
                return;
            }
            if(!gotoPreset(1)) {
                printf("Error while going to preset with Number 1\n");
                return;
            }
            awaitPositionCommandCompletion();
            cur_pan_pos = getCurrentPanPosition();
            cur_tilt_pos = getCurrentTiltPosition();
            if(cur_pan_pos != 300 || cur_tilt_pos != 300) {
                printf("setPreset did not work properly. When going to preset 1 wrong position is loaded\n");
                return;
            }
            if(!gotoPreset(2)) {
                printf("Error while going to preset with Number 1\n");
                return;
            }
            awaitPositionCommandCompletion();
            cur_pan_pos = getCurrentPanPosition();
            cur_tilt_pos = getCurrentTiltPosition();
            if(cur_pan_pos != -300 || cur_tilt_pos != -300) {
                printf("setPreset did not work properly. When going to preset 2 wrong position is loaded\n");
                return;
            }

            clearPreset();

            if(gotoPreset(1)) {
                printf("Goto preset 1 after clearing preset worked but it should not have worked\n");
                return;
            }
            */

            if(!setSpeedControlMode(PURE_VELOCITY_CONTROL_MODE)) {
                printf("Error while setting speed control mode to PURE_VELOCITY_CONTROL_MODE\n");
                return;
            }
            long cur_speed_control_mode = getSpeedControlMode();
            if(cur_speed_control_mode != PURE_VELOCITY_CONTROL_MODE) {
                printf("Error while getting or setting speed controle mode. Expected PURE_VELOCITY_CONTROL_MODE, got %ld\n", cur_speed_control_mode);
                return;
            }

            if(!setSpeedControlMode(INDEPENDENT_SPEED_MODE)) {
                printf("Error while setting speed control mode to INDEPENDENT_SPEED_MODE\n");
                return;
            }
            cur_speed_control_mode = getSpeedControlMode();
            if(cur_speed_control_mode != INDEPENDENT_SPEED_MODE) {
                printf("Error while getting or setting speed controle mode. Expected INDEPENDENT_SPEED_MODE, got %ld\n", cur_speed_control_mode);
                return;
            }


            //NOTE: Only visual check on reset modes they are written to eeprom and there is no way to query the current mode.
            //Folowing commented out functions where also only visually checked (code review)
            //bool setResetMode(long mode);
            //bool saveDefault();
            //bool restoreDefault();
            //bool restoreFactoryDefault();

            long prev_pan_stat_power_mode = getPanStationaryPowerMode();
            long prev_tilt_stat_power_mode = getTiltStationaryPowerMode();
            if(!setPanStationaryPowerMode(REGULAR_HOLD_POWER_MODE)) {
                printf("Error while setting pan stationary power mode to REGULAR_HOLD_POWER_MODE\n");
                return;
            }
            if(!setTiltStationaryPowerMode(REGULAR_HOLD_POWER_MODE)) {
                printf("Error while setting tilt stationary power mode to REGULAR_HOLD_POWER_MODE\n");
                return;
            }
            long cur_pan_stat_power_mode = getPanStationaryPowerMode();
            long cur_tilt_stat_power_mode = getTiltStationaryPowerMode();
            if(cur_pan_stat_power_mode != REGULAR_HOLD_POWER_MODE) {
                printf("Error while getting or setting pan stationary power mode. Expected: REGULAR_HOLD_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }
            if(cur_tilt_stat_power_mode != REGULAR_HOLD_POWER_MODE) {
                printf("Error while getting or setting tilt stationary power mode. Expected: REGULAR_HOLD_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }

            if(!setPanStationaryPowerMode(LOW_HOLD_POWER_MODE)) {
                printf("Error while setting pan stationary power mode to LOW_HOLD_POWER_MODE\n");
                return;
            }
            if(!setTiltStationaryPowerMode(LOW_HOLD_POWER_MODE)) {
                printf("Error while setting tilt stationary power mode to LOW_HOLD_POWER_MODE\n");
                return;
            }
            cur_pan_stat_power_mode = getPanStationaryPowerMode();
            cur_tilt_stat_power_mode = getTiltStationaryPowerMode();
            if(cur_pan_stat_power_mode != LOW_HOLD_POWER_MODE) {
                printf("Error while getting or setting pan stationary power mode. Expected: LOW_HOLD_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }
            if(cur_tilt_stat_power_mode != LOW_HOLD_POWER_MODE) {
                printf("Error while getting or setting tilt stationary power mode. Expected: LOW_HOLD_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }

            if(!setPanStationaryPowerMode(OFF_HOLD_POWER_MODE)) {
                printf("Error while setting pan stationary power mode to OFF_HOLD_POWER_MODE\n");
                return;
            }
            if(!setTiltStationaryPowerMode(OFF_HOLD_POWER_MODE)) {
                printf("Error while setting tilt stationary power mode to OFF_HOLD_POWER_MODE\n");
                return;
            }
            cur_pan_stat_power_mode = getPanStationaryPowerMode();
            cur_tilt_stat_power_mode = getTiltStationaryPowerMode();
            if(cur_pan_stat_power_mode != OFF_HOLD_POWER_MODE) {
                printf("Error while getting or setting pan stationary power mode. Expected: OFF_HOLD_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }
            if(cur_tilt_stat_power_mode != OFF_HOLD_POWER_MODE) {
                printf("Error while getting or setting tilt stationary power mode. Expected: OFF_HOLD_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }

            setPanStationaryPowerMode(prev_pan_stat_power_mode);
            setTiltStationaryPowerMode(prev_tilt_stat_power_mode);

            long prev_pan_move_power_mode = getPanInMotionPowerMode();
            long prev_tilt_move_power_mode = getTiltInMotionPowerMode();
            if(!setPanInMotionPowerMode(REGULAR_MOVE_POWER_MODE)) {
                printf("Error while setting pan in_motion power mode to REGULAR_MOVE_POWER_MODE\n");
                return;
            }
            if(!setTiltInMotionPowerMode(REGULAR_MOVE_POWER_MODE)) {
                printf("Error while setting tilt in_motion power mode to REGULAR_MOVE_POWER_MODE\n");
                return;
            }
            long cur_pan_move_power_mode = getPanInMotionPowerMode();
            long cur_tilt_move_power_mode = getTiltInMotionPowerMode();
            if(cur_pan_move_power_mode != REGULAR_MOVE_POWER_MODE) {
                printf("Error while getting or setting pan in_motion power mode. Expected: REGULAR_MOVE_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }
            if(cur_tilt_move_power_mode != REGULAR_MOVE_POWER_MODE) {
                printf("Error while getting or setting tilt in_motion power mode. Expected: REGULAR_MOVE_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }

            if(!setPanInMotionPowerMode(LOW_MOVE_POWER_MODE)) {
                printf("Error while setting pan in_motion power mode to LOW_MOVE_POWER_MODE\n");
                return;
            }
            if(!setTiltInMotionPowerMode(LOW_MOVE_POWER_MODE)) {
                printf("Error while setting tilt in_motion power mode to LOW_MOVE_POWER_MODE\n");
                return;
            }
            cur_pan_move_power_mode = getPanInMotionPowerMode();
            cur_tilt_move_power_mode = getTiltInMotionPowerMode();
            if(cur_pan_move_power_mode != LOW_MOVE_POWER_MODE) {
                printf("Error while getting or setting pan in_motion power mode. Expected: LOW_MOVE_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }
            if(cur_tilt_move_power_mode != LOW_MOVE_POWER_MODE) {
                printf("Error while getting or setting tilt in_motion power mode. Expected: LOW_MOVE_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }

            if(!setPanInMotionPowerMode(HIGH_MOVE_POWER_MODE)) {
                printf("Error while setting pan in_motion power mode to HIGH_MOVE_POWER_MODE\n");
                return;
            }
            if(!setTiltInMotionPowerMode(HIGH_MOVE_POWER_MODE)) {
                printf("Error while setting tilt in_motion power mode to HIGH_MOVE_POWER_MODE\n");
                return;
            }
            cur_pan_move_power_mode = getPanInMotionPowerMode();
            cur_tilt_move_power_mode = getTiltInMotionPowerMode();
            if(cur_pan_move_power_mode != HIGH_MOVE_POWER_MODE) {
                printf("Error while getting or setting pan in_motion power mode. Expected: HIGH_MOVE_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }
            if(cur_tilt_move_power_mode != HIGH_MOVE_POWER_MODE) {
                printf("Error while getting or setting tilt in_motion power mode. Expected: HIGH_MOVE_POWER_MODE, got %ld\n", cur_pan_stat_power_mode);
                return;
            }

            setPanInMotionPowerMode(prev_pan_move_power_mode);
            setTiltInMotionPowerMode(prev_tilt_move_power_mode);


            printf("Testing successfull\n");
    }

    bool PTUFree::isOpen() {
        return ptu_port.is_open();
    }
}
