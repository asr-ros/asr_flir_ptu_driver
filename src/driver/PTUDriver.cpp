/**
* PTUDriver class.
*
* @author Valerij Wittenbeck, Pascal Meissner
* @version See SVN
*/

#include "driver/PTUDriver.h"

namespace asr_flir_ptu_driver {

short int PTUDriver::POW_VAL_MOVE[3] = {PTU_LOW_POWER, PTU_REG_POWER, PTU_HI_POWER};
short int PTUDriver::POW_VAL_HOLD[3] = {PTU_OFF_POWER, PTU_LOW_POWER, PTU_REG_POWER};

PTUDriver::PTUDriver(const char* port, int baud, bool speed_control) {
   ROS_DEBUG("PTUDriverConstructor");
   double_computation_tolerance = 0.00005;
   distance_factor = 10;
   this->speed_control = speed_control;

   #ifndef __PTU_FREE_INCLUDED__
   set_baud_rate(baud);
   COMstream = open_host_port((char*)port);
   if (isConnected()) {
       char c = reset_ptu();
       ROS_DEBUG("ptu_status: %d", c);
       char d = reset_PTU_parser(1000);
       ROS_DEBUG("reset_PTU_parser: %d", d);
   } else {
       ROS_ERROR("PTU is not connected!");
       exit(1);
   }
   #endif // __PTU_FREE_INCLUDED__
   #ifdef __PTU_FREE_INCLUDED__
       long return_code = free_ptu.setNewSerialConnection(port, baud);
       ROS_DEBUG("Free PTU serial connection set with return code: %ld", return_code);
   #endif // __PTU_FREE_INCLUDED__

   /*
    * With the PTU comes a file, test.c, to be found with the purchasable driver version for the flir ptu.
    * In Line 632 a test function for the resolution is declared.
    * The value get_current delivers is devided by 3600 there to get min arc
    * Therefore it needs to be devided by another 60 to recieve degree.
    *
    * Be carefull: Calculation shown in the manual is NOT correct
    * If PTUFree is used the result that is returned via serial port and via PTUFree sticks to the manual,
    * but gets scaled by 60 in the code of this class to match the format that ptu.h returns
    */
   pan_resolution = get_current(PAN, RESOLUTION) / (3600 * 60.0);
   tilt_resolution = get_current(TILT, RESOLUTION) / (3600 * 60.0);

   //Important, let it in, do not change (other mode caused problems with our ptu)
   set_mode(POSITION_LIMITS_MODE, OFF_MODE);

   setSpeedControlMode(speed_control);
   ROS_DEBUG("port: %s, baud: %d, speed_control: %d", port, baud, speed_control);
   setLimitAnglesToHardwareConstraints();
}

PTUDriver::PTUDriver() {
   ROS_DEBUG("This constructor is only for the mock!");
}

PTUDriver::~PTUDriver() {
    //empty
}


bool PTUDriver::isConnected() {
   #ifndef __PTU_FREE_INCLUDED__
   return COMstream > 0;
   #endif
   #ifdef __PTU_FREE_INCLUDED__
   return free_ptu.isOpen();
   #endif
}

void PTUDriver::setComputationTolerance(double computation_tolerance) {
    this->double_computation_tolerance = computation_tolerance;
}

void PTUDriver::setDistanceFactor(long distance_factor) {
    this->distance_factor = distance_factor;
}


void PTUDriver::setSettings(int pan_base, int tilt_base, int pan_speed, int tilt_speed,
               int pan_upper, int tilt_upper,int pan_accel, int tilt_accel,
               int pan_hold, int tilt_hold, int pan_move, int tilt_move) {
   createSettingsBackup();
   char return_val;
   bool no_error_occured = true;
   if(no_error_occured)
   {
       return_val = set_desired(PAN, BASE, (short int *)&pan_base, ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(PAN, BASE, " << pan_base << ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }
   if(no_error_occured)
   {
       return_val = set_desired(PAN, UPPER_SPEED_LIMIT, (short int *)&pan_upper, ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(PAN, UPPER_SPEED_LIMIT, " << pan_upper << ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }
   if(no_error_occured)
   {
       return_val = set_desired(PAN, SPEED, (short int *)&pan_speed, ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(PAN, SPEED, " <<pan_speed<< ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }

   if(no_error_occured)
   {
       return_val = set_desired(PAN, ACCELERATION, (short int *)&pan_accel, ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(PAN, ACCELERATION, " << pan_accel << ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }

   if(no_error_occured)
   {
       return_val = set_desired(PAN, HOLD_POWER_LEVEL, &POW_VAL_HOLD[pan_hold], ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(PAN, HOLD_POWER_LEVEL, " << POW_VAL_HOLD[pan_hold] << ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }

   if(no_error_occured)
   {
       return_val = set_desired(PAN, MOVE_POWER_LEVEL, &POW_VAL_MOVE[pan_move], ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(PAN, MOVE_POWER_LEVEL, " << POW_VAL_MOVE[pan_move] <<", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }

   if(no_error_occured)
   {
       return_val = set_desired(TILT, BASE, (short int *)&tilt_base, ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(TILT, BASE, " << tilt_base << ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }

   if(no_error_occured)
   {
       return_val = set_desired(TILT, UPPER_SPEED_LIMIT, (short int *)&tilt_upper, ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(TILT, UPPER_SPEED_LIMIT, " << tilt_upper << ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }

   if(no_error_occured)
   {
       if(no_error_occured) return_val = set_desired(TILT, SPEED, (short int *)&tilt_speed, ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(TILT, SPEED, " <<tilt_speed<< ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }


   if(no_error_occured)
   {
       return_val = set_desired(TILT, ACCELERATION, (short int *)&tilt_accel, ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(TILT, ACCELERATION, " << tilt_accel << ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }

   if(no_error_occured)
   {
       return_val = set_desired(TILT, HOLD_POWER_LEVEL, &POW_VAL_HOLD[tilt_hold], ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(TILT, HOLD_POWER_LEVEL, " << POW_VAL_HOLD[tilt_hold] << ", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }

   if(no_error_occured)
   {
       return_val = set_desired(TILT, MOVE_POWER_LEVEL, &POW_VAL_MOVE[tilt_move], ABSOLUTE);
       ROS_DEBUG_STREAM("set_desired(TILT, MOVE_POWER_LEVEL, " << POW_VAL_MOVE[tilt_move] <<", ABSOLUTE) returned " << getErrorString(return_val) );
       if(!checkReturnCode(return_val)) no_error_occured = false;
   }


   if(!no_error_occured)
   {
       ROS_ERROR("Error occured while setting new settings! Old settings will be restored.");
       restoreSettingsFromBackup();
       setValuesToBackupValues(pan_base, tilt_base, pan_speed, tilt_speed, pan_upper, tilt_upper, pan_accel, tilt_accel, pan_hold, tilt_hold, pan_move, tilt_move);
   }
}



void PTUDriver::setValuesToBackupValues(int & pan_base, int & tilt_base, int & pan_speed, int & tilt_speed,
               int & pan_upper, int & tilt_upper, int & pan_accel, int & tilt_accel,
               int & pan_hold, int & tilt_hold, int & pan_move, int & tilt_move)
{
   pan_base = (int) backup_pan_base;
   pan_upper = (int) backup_pan_upper;
   pan_speed = (int) backup_pan_speed;
   pan_accel = (int) backup_pan_accel;
   if(POW_VAL_HOLD[0] == backup_pan_hold) {
       pan_hold = 0;
   }
   else if (POW_VAL_HOLD[1] == backup_pan_hold) {
       pan_hold = 1;
   }
   else {
       pan_hold = 2;
   }

   if(POW_VAL_MOVE[0] == backup_pan_move) {
       pan_move = 0;
   }
   else if (POW_VAL_MOVE[1] == backup_pan_move) {
       pan_move = 1;
   }
   else {
       pan_move = 2;
   }
   tilt_base = (int) backup_tilt_base;
   tilt_upper = (int) backup_tilt_upper;
   tilt_speed = (int) backup_tilt_speed;
   tilt_accel = (int) backup_tilt_accel;
   if(POW_VAL_HOLD[0] == backup_tilt_hold) {
       tilt_hold = 0;
   }
   else if (POW_VAL_HOLD[1] == backup_tilt_hold) {
       tilt_hold = 1;
   }
   else {
       tilt_hold = 2;
   }

   if(POW_VAL_MOVE[0] == backup_tilt_move) {
       tilt_move = 0;
   }
   else if (POW_VAL_MOVE[1] == backup_tilt_move) {
       tilt_move = 1;
   }
   else {
       tilt_move = 2;
   }
}


void PTUDriver::createSettingsBackup()
{
   backup_pan_base = get_desired(PAN, BASE);
   backup_pan_upper = get_desired(PAN, UPPER_SPEED_LIMIT);
   backup_pan_speed = get_desired(PAN, SPEED);
   backup_pan_accel = get_desired(PAN, ACCELERATION);
   backup_pan_hold = get_desired(PAN, HOLD_POWER_LEVEL);
   backup_pan_move = get_desired(PAN, MOVE_POWER_LEVEL);
   backup_tilt_base = get_desired(TILT, BASE);
   backup_tilt_upper = get_desired(TILT, UPPER_SPEED_LIMIT);
   backup_tilt_speed = get_desired(TILT, SPEED);
   backup_tilt_accel = get_desired(TILT, ACCELERATION);
   backup_tilt_hold = get_desired(TILT, HOLD_POWER_LEVEL);
   backup_tilt_move = get_desired(TILT, MOVE_POWER_LEVEL);
}

//Possible extensions for error handling could be done here (e.g. extra info output)
bool PTUDriver::checkReturnCode(char return_code) {
   if(return_code == PTU_OK) return true;
   return false;
}


void PTUDriver::restoreSettingsFromBackup() {
 set_desired(PAN, BASE, (short int *)&backup_pan_base, ABSOLUTE);
 set_desired(PAN, UPPER_SPEED_LIMIT, (short int *)&backup_pan_upper, ABSOLUTE);
 set_desired(PAN, SPEED, (short int *)&backup_pan_speed, ABSOLUTE);
 set_desired(PAN, ACCELERATION, (short int *)&backup_pan_accel, ABSOLUTE);
 set_desired(PAN, HOLD_POWER_LEVEL, (short int *) &backup_pan_hold, ABSOLUTE);
 set_desired(PAN, MOVE_POWER_LEVEL, (short int *) &backup_pan_move, ABSOLUTE);
 set_desired(TILT, BASE, (short int *)&backup_tilt_base, ABSOLUTE);
 set_desired(TILT, UPPER_SPEED_LIMIT, (short int *)&backup_tilt_upper, ABSOLUTE);
 set_desired(TILT, SPEED, (short int *)&backup_tilt_speed, ABSOLUTE);
 set_desired(TILT, ACCELERATION, (short int *)&backup_tilt_accel, ABSOLUTE);
 set_desired(TILT, HOLD_POWER_LEVEL, (short int *)&backup_tilt_hold, ABSOLUTE);
 set_desired(TILT, MOVE_POWER_LEVEL, (short int *)&backup_tilt_move, ABSOLUTE);
}

long PTUDriver::getLimitAngle(char pan_or_tilt, char upper_or_lower) {
   if(pan_or_tilt == 'p') {
       if (upper_or_lower == 'l') {
           return this->convertPanFromPositionToAngle(pan_min);
       }
       else if (upper_or_lower == 'u') {
           return this->convertPanFromPositionToAngle(pan_max);
       }
       else {
           return std::numeric_limits<double>::max();
       }
   }
   else if (pan_or_tilt == 't') {
       if (upper_or_lower == 'l') {
           return this->convertTiltFromPositionToAngle(tilt_min);
       }
       else if (upper_or_lower == 'u') {
           return this->convertTiltFromPositionToAngle(tilt_max);
       }
       else {
           return std::numeric_limits<double>::max();
       }
   }
   else {
       return std::numeric_limits<double>::max();
   }
}



void PTUDriver::setForbiddenAreas(std::vector< std::map< std::string, double> > forbidden_areas) {
   std::vector< std::map< std::string, double> > legit_forbidden_areas;
   for (unsigned int i = 0; i < forbidden_areas.size(); i++)
   {
       std::map< std::string, double> area = forbidden_areas.at(i);
       double area_pan_min = area["pan_min"];
       double area_pan_max = area["pan_max"];
       double area_tilt_min = area["tilt_min"];
       double area_tilt_max = area["tilt_max"];
       if ( area_pan_min <= area_pan_max  &&
            area_tilt_min <= area_tilt_max)
       {
               legit_forbidden_areas.push_back(area);
       }
       else
       {
           ROS_ERROR("Forbidden Area with pan_min %f, pan_max %f, tilt_min %f and tilt_max %f is not valid. One of the max values is larger than the related min value.\n", area_pan_min, area_pan_max, area_tilt_min, area_tilt_max);
       }
   }
   this->forbidden_areas = legit_forbidden_areas;
   precalculateForbiddenAreaCoodinateForms();
}

void PTUDriver::setAbsoluteAngleSpeeds(double pan_speed, double tilt_speed) {
 ROS_DEBUG("Driver::SetAbsoluteAngleSpeed()");
 signed short pan_spd = (signed short) convertPanFromAngleToPosition(pan_speed);
 signed short tilt_spd = (signed short) convertTiltFromAngleToPosition(tilt_speed);

 if ((getCurrentAngle(PAN) <= pan_min && pan_spd < 0) ||
     (getCurrentAngle(PAN) >= pan_max && pan_spd > 0)) {
     pan_spd = 0;
     ROS_DEBUG("Driver::PanSpeed=0");
 }
 if ((getCurrentAngle(TILT) <= tilt_min && tilt_spd < 0) ||
         (getCurrentAngle(TILT) >= tilt_max && tilt_spd > 0)) {
     tilt_spd = 0;
     ROS_DEBUG("Driver::TiltSpeed=0");
 }
 setAbsoluteAngleSpeeds(pan_spd, tilt_spd);
 ROS_DEBUG("Driver::SetAbsoluteAngleSpeed");
}

void PTUDriver::setAbsoluteAngleSpeeds(signed short pan_speed, signed short tilt_speed) {
   char return_code;
   return_code = set_desired(PAN, SPEED, &pan_speed, ABSOLUTE);
   ROS_DEBUG_STREAM("set_desired(PAN, SPEED, " << pan_speed << ", ABSOLUTE) returned "
                    << getErrorString(return_code) );
   return_code = set_desired(TILT, SPEED, &tilt_speed, ABSOLUTE);
   ROS_DEBUG_STREAM("set_desired(TILT, SPEED, " << tilt_speed << ", ABSOLUTE) returned "
                    << getErrorString(return_code) );

}

//Do not use if you use a point determinded by checkForPossibleKollision or determineLegitEndPoint because it could lie minimal within a forbidden area (unprecise double calculations) or (normally) directly on the border.
bool PTUDriver::isInForbiddenArea(double pan_angle, double tilt_angle)
{
 ROS_DEBUG("Driver::isInForbiddenArea()");
 for (unsigned int i = 0; i < forbidden_areas.size(); i++)
 {
     ROS_DEBUG("area %d", i);
     std::map< std::string, double> area = forbidden_areas.at(i);
     double area_pan_min = area["pan_min"];
     double area_pan_max = area["pan_max"];
     double area_tilt_min = area["tilt_min"];
     double area_tilt_max = area["tilt_max"];
     ROS_DEBUG_STREAM("Checking forbidden area " << i << " with pan_min " << area_pan_min << ", pan_max " << area_pan_max << ", tilt_min " << area_tilt_min << ", tilt_max " << area_tilt_max);

     if (area_pan_min < pan_angle && pan_angle < area_pan_max  &&
          area_tilt_min < tilt_angle && tilt_angle < area_tilt_max)
     {
         ROS_ERROR("Value lies in forbidden area: %d", i);
         return true;
     }
 }
 return false;
}

bool PTUDriver::isWithinPanTiltLimits(double pan, double tilt) {
   if((pan_min <= convertPanFromAngleToPosition(pan)) && (convertPanFromAngleToPosition(pan) <= pan_max)) {
       if((tilt_min <= convertTiltFromAngleToPosition(tilt)) && (convertTiltFromAngleToPosition(tilt) <= tilt_max)) {
           return true;
       }
       else{
           ROS_DEBUG("TILT %f is not between the lower limit %f and the upper limit %f", tilt, convertTiltFromPositionToAngle(tilt_min), convertTiltFromPositionToAngle(tilt_max));
       }
   }
   else {
       ROS_DEBUG("PAN %f is not between the lower limit %f and the upper limit %f", pan, convertPanFromPositionToAngle(pan_min), convertPanFromPositionToAngle(pan_max));
   }
   return false;
}

bool PTUDriver::setValuesOutOfLimitsButWithinMarginToLimit(double * pan, double * tilt, double margin) {
   ROS_DEBUG("PAN: %f, TILT: %f\n", *pan, *tilt);
   long margin_as_position = convertPanFromAngleToPosition(margin);
   if(((pan_max - pan_min) <= margin_as_position) || ((tilt_max - tilt_min) <= margin_as_position)) {

       ROS_ERROR("Margin in Positions (%ld) (Degree: (%f)) is  too big. Bigger than distance between TILT max and TILT min (%ld Positions) or PAN max and PAN min (%ld Positions)",margin_as_position, margin, (tilt_max - tilt_min), (pan_max - pan_min));
       return false;
   }
   double pan_initial = *pan;
   double tilt_initial = *tilt;
   short int pan_as_position = convertPanFromAngleToPosition(*pan);
   short int tilt_as_position = convertTiltFromAngleToPosition(*tilt);
   if((pan_as_position < pan_min) && (pan_as_position >= (pan_min - margin_as_position))) {
       pan_as_position = pan_min;
       *pan = convertPanFromPositionToAngle(pan_min);
       ROS_WARN("PAN was out of limits, but within margin, so instead of the initial value %f the value %f is used", pan_initial, *pan);
   }
   else if ((pan_as_position > pan_max) && (pan_as_position <= (pan_max + margin_as_position))) {
       pan_as_position = pan_max;
       *pan = convertPanFromPositionToAngle(pan_max);
       ROS_WARN("PAN was out of limits, but within margin, so instead of the initial value %f the value %f is used", pan_initial, *pan);
   }
   if((tilt_as_position < tilt_min) && (tilt_as_position >= (tilt_min - margin_as_position))) {
       tilt_as_position = tilt_min;
       *tilt = convertTiltFromPositionToAngle(tilt_min);
       ROS_WARN("TILT was out of limits, but within margin, so instead of the initial value %f the value %f is used", tilt_initial, *tilt);
   }
   else if((tilt_as_position > tilt_max) && (tilt_as_position <= (tilt_max + margin_as_position))) {
       tilt_as_position = tilt_max;
       *tilt = convertTiltFromPositionToAngle(tilt_max);
       ROS_WARN("TILT was out of limits, but within margin, so instead of the initial value %f the value %f is used", tilt_initial, *tilt);
   }

   if(isWithinPanTiltLimits(*pan, *tilt)) {
       ROS_ERROR("PAN: %f, TILT: %f\n", *pan, *tilt);
       return true;
   }
   else {
       return false;
   }
}

void PTUDriver::setLimitAnglesToHardwareConstraints() {
   this->pan_min = (double) get_current(PAN, MINIMUM_POSITION);
   this->pan_max = (double) get_current(PAN, MAXIMUM_POSITION);
   this->tilt_min = (double) get_current(TILT, MINIMUM_POSITION);
   this->tilt_max = (double) get_current(TILT, MAXIMUM_POSITION);
   ROS_DEBUG("Limits set to Hardware Limits: PAN_MINIMUM: %f, PAN_MAXIMUM: %f, TILT_MINIMUM: %f, TILT_MAXIMUM: %f", convertPanFromPositionToAngle(pan_min), convertPanFromPositionToAngle(pan_max), convertTiltFromPositionToAngle(tilt_min), convertTiltFromPositionToAngle(tilt_max));
}

//No Check should only be used if you use a point that can be minimal within a forbidden area and when you know that getting in touch with the border of the forbidden area is no problem
//A typical case for that problem is a point derived from 'checkForPossibleKollision' or 'determineLegitEndPoint' - these can lay onto the border of a forbidden area or minimal within
bool PTUDriver::setAbsoluteAngles(double pan_angle, double tilt_angle, bool no_forbidden_area_check) {

   // Maybe for later: use with path prediction, the problem that the new position must be returned to the layers above is currently only solved for the GUI, not for the PTU Controller
   //PTU Controller must be modified to support new end points to do so

   ROS_DEBUG("Driver::SetAbsoluteAngles()");
   if (isInForbiddenArea(pan_angle, tilt_angle) && !no_forbidden_area_check)
   {
       return false;
   }
   short int pan_short_angle = convertPanFromAngleToPosition(pan_angle);
   short int tilt_short_angle = convertTiltFromAngleToPosition(tilt_angle);

   if ((pan_short_angle < pan_min)
         || (pan_short_angle > pan_max)
         || (tilt_short_angle < tilt_min)
         || (tilt_short_angle > tilt_max))
   {
       return false;
   }
   char return_code;
   ROS_DEBUG_STREAM("panShortAngle = " << pan_short_angle);
   return_code = set_desired(PAN, POSITION, &pan_short_angle, ABSOLUTE);
   ROS_DEBUG_STREAM("set_desired(PAN, POSITION, " << pan_short_angle << ", ABSOLUTE) returned "
                  << getErrorString(return_code) );
   ROS_DEBUG_STREAM("tiltShortAngle = " << tilt_short_angle);
   return_code = set_desired(TILT, POSITION, &tilt_short_angle, ABSOLUTE);
   ROS_DEBUG_STREAM("set_desired(TILT, POSITION, " << tilt_short_angle << ", ABSOLUTE) returned "
                  << getErrorString(return_code) );
   return true;
}

void PTUDriver::setLimitAngles(double pan_min, double pan_max, double tilt_min, double tilt_max) {
   long pan_min_candidate = convertPanFromAngleToPosition(pan_min);
   if(pan_min_candidate >= get_current(PAN, MINIMUM_POSITION)) {
       this->pan_min = pan_min_candidate;
   }
   else {
       this->pan_min = get_current(PAN, MINIMUM_POSITION);
       ROS_ERROR("Desired PAN_MIN Position was smaller than the hardware limit -> PAN_MIN set to hardware limit");
   }
   double pan_max_candidate = convertPanFromAngleToPosition(pan_max);
   if(pan_max_candidate <= get_current(PAN, MAXIMUM_POSITION)) {
       this->pan_max = pan_max_candidate;
   }
   else {
       this->pan_max = get_current(PAN, MAXIMUM_POSITION);
       ROS_ERROR("Desired PAN_MAX Position was greater than the hardware limit -> PAN_MAX set to hardware limit");
   }
   double tilt_min_candidate = convertTiltFromAngleToPosition(tilt_min);
   if(tilt_min_candidate >= get_current(TILT, MINIMUM_POSITION)) {
       this->tilt_min = tilt_min_candidate;
   }
   else {
       this->tilt_min = get_current(TILT, MINIMUM_POSITION);
       ROS_ERROR("Desired TILT_MIN Position was smaller than the hardware limit -> TILT_MIN set to hardware limit");
   }
   double tilt_max_candidate = convertTiltFromAngleToPosition(tilt_max);
   if(tilt_max_candidate <= get_current(TILT, MAXIMUM_POSITION)) {
       this->tilt_max = tilt_max_candidate;
   }
   else {
       this->tilt_max = get_current(TILT, MAXIMUM_POSITION);
       ROS_ERROR("Desired TILT_MAX Position was greater than the hardware limit -> TILT_MAX set to hardware limit");
   }
   ROS_DEBUG("Limits set to Virtual Limits: PAN_MINIMUM: %f, PAN_MAXIMUM: %f, TILT_MINIMUM: %f, TILT_MAXIMUM: %f", pan_min, pan_max, tilt_min, tilt_max);
}

void PTUDriver::setSpeedControlMode(bool speed_control) {
   this->speed_control = speed_control;
   if (speed_control) {
       set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE);
   } else {
       set_mode(SPEED_CONTROL_MODE, PTU_INDEPENDENT_SPEED_CONTROL_MODE);
   }
}

bool PTUDriver::isInSpeedControlMode() {
   return speed_control;
}

double PTUDriver::getCurrentAngle(char type) {
    if (type == PAN) {
        return convertPanFromPositionToAngle(get_current(PAN, POSITION));
    }
    else if(type == TILT){
        return convertTiltFromPositionToAngle(get_current(TILT, POSITION));
    }
    else {
        throw std::exception();
    }

}

double PTUDriver::getDesiredAngle(char type) {
   if (type == PAN) {
       return convertPanFromPositionToAngle(get_desired(PAN, POSITION));
   }
   else if (type == TILT) {
       return convertTiltFromPositionToAngle(get_desired(TILT, POSITION));
   }
   else {
       throw std::exception();
   }
}

double PTUDriver::getAngleSpeed(char type) {
   if (type == PAN) {
       return convertPanFromPositionToAngle(get_current(PAN, SPEED));
   }
   else if (type == TILT) {
       return convertTiltFromPositionToAngle(get_current(TILT, SPEED));
   }
   else {
       throw std::exception();
   }
}

long PTUDriver::convertPanFromAngleToPosition(double angle) {
   return ((long) round(angle / pan_resolution));
}

double PTUDriver::convertPanFromPositionToAngle(long position) {
   return ((double)position) * pan_resolution;
}

long PTUDriver::convertTiltFromAngleToPosition(double angle) {
   return ((long) round(angle / tilt_resolution));
}

double PTUDriver::convertTiltFromPositionToAngle(long position) {
   return ((double)position) * tilt_resolution;
}

std::string PTUDriver::getErrorString(char status_code)
{
   #ifndef __PTU_FREE_INCLUDED__
   switch (status_code)
   {
    case  PTU_OK:
          return "PTU_OK"; 		   break;
    case  PTU_ILLEGAL_COMMAND:
          return "PTU_ILLEGAL_COMMAND"; 		   break;
    case  PTU_ILLEGAL_POSITION_ARGUMENT:
          return "PTU_ILLEGAL_POSITION_ARGUMENT"; 		   break;
    case  PTU_ILLEGAL_SPEED_ARGUMENT:
          return "PTU_ILLEGAL_SPEED_ARGUMENT"; 		   break;
    case  PTU_ACCEL_TABLE_EXCEEDED:
          return "PTU_ACCEL_TABLE_EXCEEDED"; 		   break;
    case  PTU_DEFAULTS_EEPROM_FAULT:
          return "PTU_DEFAULTS_EEPROM_FAULT"; 		   break;
    case  PTU_SAVED_DEFAULTS_CORRUPTED:
          return "PTU_SAVED_DEFAULTS_CORRUPTED"; 		   break;
    case  PTU_LIMIT_HIT:
          return "PTU_LIMIT_HIT"; 		   break;
    case  PTU_CABLE_DISCONNECTED:
          return "PTU_CABLE_DISCONNECTED"; 		   break;
    case  PTU_ILLEGAL_UNIT_ID:
          return"PTU_ILLEGAL_UNIT_ID"; 		   break;
    case  PTU_ILLEGAL_POWER_MODE:
          return "PTU_ILLEGAL_POWER_MODE"; 		   break;
    case  PTU_RESET_FAILED:
          return "PTU_RESET_FAILED"; 		   break;
    case  PTU_ILLEGAL_PARAMETERS:
          return "PTU_ILLEGAL_PARAMETERS"; 		   break;
    case  PTU_DUART_ERROR:
          return "PTU_DUART_ERROR"; 		   break;
    case  PTU_ERROR:
          return "PTU_ERROR"; 		   break;
    case  NOT_SUPPORTED_BY_THIS_FIRMWARE_VERSION:
          return "NOT_SUPPORTED_BY_THIS_FIRMWARE_VERSION"; 		   break;
    case  PTU_TILT_VANE_OUT_OF_RANGE_ERROR:
          return "PTU_TILT_VANE_OUT_OF_RANGE_ERROR"; 		   break;
    default:
          return "UNDEFINED_RETURN_STATUS_ERROR"; 		   break;
   }
   #endif
   #ifdef __PTU_FREE_INCLUDED__
   if(status_code == PTU_OK) {
       return "PTU_OK";
   }
   else {
       return "PTU_ERROR";
   }
   #endif
}

bool PTUDriver::hasHaltedAndReachedGoal() {
   return hasHalted() && reachedGoal();
}

bool PTUDriver::hasHalted() {
   long current_tilt_speed = get_current(TILT, SPEED);
   long current_pan_speed = get_current(PAN, SPEED);
   return (((current_tilt_speed == 0) && (current_pan_speed == 0)));
}

//TODO: why is computation tolerance not used here? necessary?
bool PTUDriver::reachedGoal() {
   long current_tilt = get_current(TILT, POSITION);
   long desired_tilt = get_desired(TILT, POSITION);
   long current_pan = get_current(PAN, POSITION);
   long desired_pan = get_desired(PAN, POSITION);
   return ((current_tilt == desired_tilt) && (current_pan == desired_pan));
}


/*
* Needs to have the form a * x^2 + b * x + c = 0, where x is the variable who needs to be solved and a, b and c are constants
*
*/
std::vector<double> PTUDriver::solveSecondDegreePolynomial(double a, double b, double c) {
   std::vector<double> solutions;
   //midnight formula used (b +- sqrt(b^2 - 4 * a * c))/2a
   double second_part_before_root = pow(b, 2) - 4 * a * c;
   if(second_part_before_root < 0){
       return solutions;
   }
   if(second_part_before_root == 0) {
       solutions.push_back(-b / (2 * a));
       return solutions;
   }
   double root_part = sqrt(second_part_before_root);
   solutions.push_back((-b + root_part) / (2 * a));
   solutions.push_back((-b - root_part) / (2 * a));
   return solutions;
}

//Returns a vector consisting of acceleration time (at position 0) and slew speed time (position 1). Do not forget that acceleration time is ONLY for acceleration and not for acceleration and deceleration.
//Though, acceleration time and deceleration time are the same because ramp profile is used.
//Returns empty vector on error.
//All needs to be entered in steps!!!
std::vector<double> PTUDriver::getAccelerationTimeAndSlewSpeedTime(double distance_in_steps, double base_speed, double acceleration, double slew_speed) {
   std::vector<double> solution;
   if(distance_in_steps == 0.0) {
       solution.push_back(0.0);
       solution.push_back(0.0);
       return solution;
   }
   //Because acceleration to BASE SPEED happens instantly according to the manual it needs to be subtracted from the desired speed (slew speed)
   double speed_to_accelerate = slew_speed - base_speed;
   double acceleration_time_to_desired_speed = speed_to_accelerate / acceleration;
   double distance_for_full_acceleration_and_deceleration = 2.0 * (acceleration_time_to_desired_speed * base_speed
                                                           + 1.0/2.0 * acceleration * pow(acceleration_time_to_desired_speed, 2));
   //In that case the PTU can not accelerate to full desired speed
   if(distance_for_full_acceleration_and_deceleration >= distance_in_steps) {
       std::vector<double> speed_candidates = solveSecondDegreePolynomial(1.0 / acceleration, 2.0 * base_speed / acceleration, -distance_in_steps);
       if(speed_candidates.size() == 0) {
           ROS_DEBUG("PAN ACCELERATION TIME CALCULATION FAILED (getAccelerationTime-Method)");
           return solution;
       }
       else if(speed_candidates.size() == 1) {
           solution.push_back(speed_candidates[0]);
       }
       else {
           double new_acceleration_time_to_desired_speed_candidate_one = speed_candidates[0] / acceleration;
           double new_acceleration_time_to_desired_speed_candidate_two = speed_candidates[1] / acceleration;
           if(speed_candidates[0] > 0.0) {
               solution.push_back(new_acceleration_time_to_desired_speed_candidate_one);
               solution.push_back(0.0);
               return solution;
           }
           else {
               solution.push_back(new_acceleration_time_to_desired_speed_candidate_two);
               solution.push_back(0.0);
               return solution;
           }
       }
   }
   else {
       solution.push_back(acceleration_time_to_desired_speed);
       double remaining_distance_at_slew_speed = distance_in_steps - distance_for_full_acceleration_and_deceleration;
       double time_at_slew_speed = remaining_distance_at_slew_speed / slew_speed;
       solution.push_back(time_at_slew_speed);
       return solution;
   }
}


//Pan at position 0, tilt at position 1, input in degree, output in degree!
std::vector<double> PTUDriver::predictPositionInTime(std::vector<double> start_point, std::vector<double> end_point, double point_in_time) {
   double pan_distance = end_point[0] - start_point[0];
   double tilt_distance = end_point[1] - start_point[1];
   double pan_distance_in_steps = convertPanFromAngleToPosition(pan_distance);
   double tilt_distance_in_steps = convertTiltFromAngleToPosition(tilt_distance);

   std::vector<double> pan_time = getAccelerationTimeAndSlewSpeedTime(std::abs(pan_distance_in_steps), prefetched_pan_current_base, prefetched_pan_desired_acceleration, prefetched_pan_desired_speed);
   std::vector<double> tilt_time = getAccelerationTimeAndSlewSpeedTime(std::abs(tilt_distance_in_steps), prefetched_pan_current_base, prefetched_pan_desired_acceleration, prefetched_pan_desired_speed);

   double pan_complete_time = 2.0 * pan_time[0] + pan_time[1];
   double tilt_complete_time = 2.0 * tilt_time[0] + tilt_time[1];
   double max_time = std::max(pan_complete_time, tilt_complete_time);
   if(max_time <= point_in_time) {
       return end_point;
   }
   else {
       std::vector<double> pan_tilt_distance;
       if(pan_time[0] >= point_in_time) {
           double acceleration_time = point_in_time;
           double slew_speed_time = 0.0;
           double decceleration_time = 0.0;
           if(pan_distance >= 0) {
               pan_tilt_distance.push_back(convertPanFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,true)));
           }
           else {
               pan_tilt_distance.push_back(-1.0 * convertPanFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,true)));
           }
       }
       else if((pan_time[0] + pan_time[1]) >= point_in_time) {
           double acceleration_time = pan_time[0];
           double slew_speed_time = point_in_time - pan_time[0];
           double decceleration_time = 0.0;
           if(pan_distance >= 0) {
               pan_tilt_distance.push_back(convertPanFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,true)));
           }
           else {
               pan_tilt_distance.push_back(-1.0 * convertPanFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,true)));
           }
       }
       //neccessary because before only max of pan and tilt complete time was checked
       else if((2.0 * pan_time[0] + pan_time[1]) >= point_in_time){
           double acceleration_time = pan_time[0];
           double slew_speed_time = pan_time[1];
           double decceleration_time = point_in_time - pan_time[0] - pan_time[1];
           if(pan_distance >= 0) {
               pan_tilt_distance.push_back(convertPanFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,true)));
           }
           else {
               pan_tilt_distance.push_back(-1.0 * convertPanFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,true)));
           }
       }
       else {
           pan_tilt_distance.push_back(end_point[0] - start_point[0]);
       }


       if(tilt_time[0] >= point_in_time) {
           double acceleration_time = point_in_time;
           double slew_speed_time = 0.0;
           double decceleration_time = 0.0;
           if(tilt_distance >= 0) {
               pan_tilt_distance.push_back(convertTiltFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,false)));
           }
           else {
               pan_tilt_distance.push_back(-1.0 * convertTiltFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,false)));
           }
       }
       else if((tilt_time[0] + tilt_time[1]) >= point_in_time) {
           double acceleration_time = tilt_time[0];
           double slew_speed_time = point_in_time - tilt_time[0];
           double decceleration_time = 0.0;
           if(tilt_distance >= 0) {
               pan_tilt_distance.push_back(convertTiltFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,false)));
           }
           else {
               pan_tilt_distance.push_back(-1.0 * convertTiltFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,false)));
           }
       }
       //neccessary because before only max of pan and tilt complete time was checked
       else if(((2.0 * tilt_time[0] + tilt_time[1]) >= point_in_time)){
           double acceleration_time = tilt_time[0];
           double slew_speed_time = tilt_time[1];
           double decceleration_time = point_in_time - tilt_time[0] - tilt_time[1];
           if(tilt_distance >= 0) {
               pan_tilt_distance.push_back(convertTiltFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,false)));
           }
           else {
               pan_tilt_distance.push_back(-1.0 * convertTiltFromPositionToAngle(calculateCoveredDistance(acceleration_time,slew_speed_time,decceleration_time,false)));
           }
       }
       else {
           pan_tilt_distance.push_back(end_point[1] - start_point[1]);
       }

       std::vector<double> new_position;
       new_position.push_back(start_point[0] + pan_tilt_distance[0]);
       new_position.push_back(start_point[1] + pan_tilt_distance[1]);


       return new_position;
   }
}

//It is assumed that deccelleration_time is maximum as large as acceleration_time
double PTUDriver::calculateCoveredDistance(double acceleration_time, double slew_speed_time, double decceleration_time, bool is_pan) {
   char type;
   long prefetched_base;
   long prefetched_accel;
   if(is_pan) {
       prefetched_base = prefetched_pan_current_base;
       prefetched_accel = prefetched_pan_desired_acceleration;
   }
   else {
       prefetched_base = prefetched_tilt_current_base;
       prefetched_accel = prefetched_tilt_desired_acceleration;
   }
   double covered_distance = 0.0;
   double covered_distance_acceleration = prefetched_base * acceleration_time + 1.0/2.0 * prefetched_accel * pow(acceleration_time, 2);
   covered_distance += covered_distance_acceleration;
   if(slew_speed_time != 0.0) {
       double covered_distance_slew_speed = (prefetched_base + acceleration_time * prefetched_accel) * slew_speed_time;
       covered_distance += covered_distance_slew_speed;
   }
   if(decceleration_time != 0) {
       double covered_distance_decceleration = covered_distance_acceleration - 1.0/2.0 * pow((acceleration_time - decceleration_time),2) * prefetched_accel - (acceleration_time - decceleration_time) * prefetched_base;
       covered_distance += covered_distance_decceleration;
   }
   return covered_distance;
}









//If non empty: First Value is Pan of intersection Point, second point is tilt of intersection point
std::vector<double> PTUDriver::calculatePointOfIntersectionWithForbiddenAreas(std::vector<double> start_point, std::vector<double> end_point) {

   double distance_from_start = std::numeric_limits<double>::max();
   std::vector<double> intersection_point;
   std::vector<double> route_coordiante_form = calculateCoordinateForm(start_point, end_point);

   for (int i = 0; i < forbidden_areas.size(); i++) {

       std::vector<double> first_intersection_point = calculateIntersectionPoint(route_coordiante_form, forbidden_area_first_line_coordinate_forms[i]);
       std::vector<double> second_intersection_point = calculateIntersectionPoint(route_coordiante_form, forbidden_area_second_line_coordinate_forms[i]);
       std::vector<double> third_intersection_point = calculateIntersectionPoint(route_coordiante_form, forbidden_area_third_line_coordinate_forms[i]);
       std::vector<double> fourth_intersection_point = calculateIntersectionPoint(route_coordiante_form, forbidden_area_fourth_line_coordinate_forms[i]);
       double new_distance_from_start;
       if((first_intersection_point.size() != 0)
               && isOnLineSegmentBetweenTwoPoints(start_point, end_point, route_coordiante_form, first_intersection_point, double_computation_tolerance)
               && isOnLineSegmentBetweenTwoPoints(max_pan_max_tilt_points[i], max_pan_min_tilt_points[i], forbidden_area_first_line_coordinate_forms[i], first_intersection_point, double_computation_tolerance)) {
           new_distance_from_start = getVectorLength(start_point, first_intersection_point);
           if(new_distance_from_start < distance_from_start) {
               distance_from_start = new_distance_from_start;
               intersection_point = first_intersection_point;
           }
       }
       if((second_intersection_point.size() != 0)
               && isOnLineSegmentBetweenTwoPoints(start_point, end_point, route_coordiante_form, second_intersection_point, double_computation_tolerance)
               && isOnLineSegmentBetweenTwoPoints(max_pan_min_tilt_points[i], min_pan_min_tilt_points[i], forbidden_area_second_line_coordinate_forms[i], second_intersection_point, double_computation_tolerance)) {
           new_distance_from_start = getVectorLength(start_point, second_intersection_point);
           if(new_distance_from_start < distance_from_start) {
               distance_from_start = new_distance_from_start;
               intersection_point = second_intersection_point;
           }
       }
       if((third_intersection_point.size() != 0)
               && isOnLineSegmentBetweenTwoPoints(start_point, end_point, route_coordiante_form, third_intersection_point, double_computation_tolerance)
               && isOnLineSegmentBetweenTwoPoints(min_pan_min_tilt_points[i], min_pan_max_tilt_points[i], forbidden_area_third_line_coordinate_forms[i], third_intersection_point, double_computation_tolerance)) {
           new_distance_from_start = getVectorLength(start_point, third_intersection_point);
           if(new_distance_from_start < distance_from_start) {
               distance_from_start = new_distance_from_start;
               intersection_point = third_intersection_point;
           }
       }
       if((fourth_intersection_point.size() != 0)
               && isOnLineSegmentBetweenTwoPoints(start_point, end_point, route_coordiante_form, fourth_intersection_point, double_computation_tolerance)
               && isOnLineSegmentBetweenTwoPoints(min_pan_max_tilt_points[i], max_pan_max_tilt_points[i], forbidden_area_fourth_line_coordinate_forms[i], fourth_intersection_point, double_computation_tolerance)) {
           new_distance_from_start = getVectorLength(start_point, fourth_intersection_point);
           if(new_distance_from_start < distance_from_start) {
               distance_from_start = new_distance_from_start;
               intersection_point = fourth_intersection_point;
           }
       }
   }
   if(intersection_point.size() != 0) {
       printf("Start Point: (%f,%f)\n", start_point[0], start_point[1]);
       printf("End Point: (%f,%f)\n", end_point[0], end_point[1]);
       printf("Intersection Point: (%f,%f)\n", intersection_point[0], intersection_point[1]);
   }
   return intersection_point;
}

//must have form a pan + b tilt = c   a at 0, b at 1, c at 2; returns empty vector if parallel (-> no intersection point or they are identical)
std::vector<double> PTUDriver::calculateIntersectionPoint(std::vector<double> first_line_coordiante_form, std::vector<double> second_line_coordiante_form) {
   //Calculated with Cramer's rule
   std::vector<double> intersection_point;
   double denominator = first_line_coordiante_form[0] * second_line_coordiante_form[1] - first_line_coordiante_form[1] * second_line_coordiante_form[0];
   if((denominator >= -0.00001 && denominator <= 0.00001)) {
       return intersection_point;
   }
   double pan_intersection_point = first_line_coordiante_form[2] * second_line_coordiante_form[1] - first_line_coordiante_form[1] * second_line_coordiante_form[2];
   double tilt_intersection_point = first_line_coordiante_form[0] * second_line_coordiante_form[2] - first_line_coordiante_form[2] * second_line_coordiante_form[0];
   pan_intersection_point /= denominator;
   tilt_intersection_point /= denominator;
   intersection_point.push_back(pan_intersection_point);
   intersection_point.push_back(tilt_intersection_point);
   return intersection_point;
}

bool PTUDriver::isOnLineSegmentBetweenTwoPoints(std::vector<double> start_point, std::vector<double> end_point, std::vector<double> line_coordinate_form, std::vector<double> point_to_check, double tolerance) {
   double distance_from_start_point = sqrt(pow((point_to_check[0] - start_point[0]), 2) + pow((point_to_check[1] - start_point[1]), 2));
   double distance_from_end_point = sqrt(pow((point_to_check[0] - end_point[0]), 2) + pow((point_to_check[1] - end_point[1]), 2));
   if((distance_from_start_point <= tolerance) || (distance_from_end_point <= tolerance)) {
       return true;
   }

   //needed to calculate distance with Hesse normal form
   double normal_vector_length = sqrt(pow(line_coordinate_form[0], 2) + pow(line_coordinate_form[1], 2));

   //Calculating distance with Hesse normal form
   double distance_from_line = (1/normal_vector_length) * (point_to_check[0] * line_coordinate_form[0] + point_to_check[1] * line_coordinate_form[1] - line_coordinate_form[2]);

   if(distance_from_line > tolerance) {
       return false;
   }

   //!!!!!!!!!!!!!!Problem: x or y is zero -> this value is freely changeable!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   //Check if problem still exists, comment above seems old


   //We have already determinded that the point is less than tolerance from the line that passes through start_point and end_point.
   //To determine that the found point lies between start and endpoint we check if it inside the rectange induced by start and endpoint (+ tolerances)
   //If it is inside the rectangle it is also on the line between both points, because that it is "on" the line was already checked before and all in the rectangle is between the both points
   double max_pan = std::max(start_point[0], end_point[0]);
   double min_pan = std::min(start_point[0], end_point[0]);
   double max_tilt = std::max(start_point[1], end_point[1]);
   double min_tilt = std::min(start_point[1], end_point[1]);
   max_pan += tolerance;
   min_pan -= tolerance;
   max_tilt += tolerance;
   min_tilt -= tolerance;

   if((point_to_check[0] > max_pan) || (point_to_check[0] < min_pan) || (point_to_check[1] > max_tilt) || (point_to_check[1] < min_tilt)) {
       return false;
   }
   else {
       return true;
   }
}

//coordinate form a pan + b tilt = c
std::vector<double> PTUDriver::calculateCoordinateForm(std::vector<double> start_point, std::vector<double> end_point) {
   std::vector<double> coordinate_form;
   if((start_point[0] == end_point[0]) && (start_point[1] == end_point[1])) {
       return coordinate_form;
   }
   double a = start_point[1] - end_point[1];
   double b = end_point[0] - start_point[0];
   double c = end_point[0] * start_point[1] - start_point[0] * end_point[1];
   coordinate_form.push_back(a);
   coordinate_form.push_back(b);
   coordinate_form.push_back(c);
   return coordinate_form;
}


double PTUDriver::getVectorLength(std::vector<double> input_vector) {
   double length = 0.0;
   for(int i = 0; i < input_vector.size(); i++) {
       length += pow(input_vector[i], 2);
   }
   return sqrt(length);
}

double PTUDriver::getVectorLength(std::vector<double> start_point, std::vector<double> end_point) {
   if (start_point.size() != end_point.size()) {
       return -1.0;
   }
   std::vector<double> length_vector(start_point.size());
   for (int i = 0; i < start_point.size(); i++) {
       length_vector[i] = end_point[i] - start_point[i];
   }
   return getVectorLength(length_vector);
}

std::vector<double> PTUDriver::determineLegitEndPoint(double end_point_pan_candidate, double end_point_tilt_candidate) {
   prefetchValues();
   std::vector<double> possible_end_point = checkForPossibleKollision(end_point_pan_candidate, end_point_tilt_candidate);
   if(possible_end_point.size() == 0) {
       std::vector<double> end_point;
       end_point.push_back(end_point_pan_candidate);
       end_point.push_back(end_point_tilt_candidate);
       return end_point;
   }
   //The path that is taken changes because acceleration time and slew speed time changes - therefore, another collision that did not occure previously can now occure.
   while(true) {
       std::vector<double> new_possible_end_point = checkForPossibleKollision(possible_end_point[0], possible_end_point[1]);
       if(new_possible_end_point.size() == 0) {
           return possible_end_point;
       }
       //In this case, possible_end_point and new_possible_end_point are litterally the same because they only differe because of unprecise double value (from the calculations).
       else if (getVectorLength(possible_end_point, new_possible_end_point) < double_computation_tolerance) {
           //At this point any of the 2 points could be returned because they only differe less than 'double_computation_tolerance' and therefore are literally the same
           return new_possible_end_point;
       }
       else {
           possible_end_point = new_possible_end_point;
       }
   }
}

//Returns empty vector if no collision is found, collsision point with forbidden area otherwise
std::vector<double> PTUDriver::checkForPossibleKollision(double new_pan_angle, double new_tilt_angle) {
   std::vector<double> start_point;
   double start_pan_value = convertPanFromPositionToAngle(prefetched_pan_current_position);
   start_point.push_back(start_pan_value);
   double start_tilt_value = convertTiltFromPositionToAngle(prefetched_tilt_current_position);
   start_point.push_back(start_tilt_value);

   std::vector<double> end_point;
   end_point.push_back(new_pan_angle);
   end_point.push_back(new_tilt_angle);


   double max_base_speed = std::max(prefetched_pan_current_base, prefetched_tilt_current_base);

   //This is the maximum time needed to switch a pan and a tilt position while the ptu is moving
   double time_per_position = 1.0 / max_base_speed;



   double time_between_samples = time_per_position * distance_factor;

   double counter = 1.0;
   std::vector<double> last_point = start_point;
   std::vector<double> intersection_point;
   std::vector<double> new_point;
   while(true){
       new_point = predictPositionInTime(start_point, end_point, (counter * time_between_samples));
       if((new_point[0] == last_point[0]) && (new_point[1] == last_point[1])) {
           return intersection_point;
       }
       intersection_point = calculatePointOfIntersectionWithForbiddenAreas(last_point, new_point);
       if(intersection_point.size() != 0) {
           return intersection_point;
       }
       else {
           last_point = new_point;
           counter += 1.0;
       }
   }
}


void PTUDriver::precalculateForbiddenAreaCoodinateForms() {
    for (int i = 0; i < forbidden_areas.size(); i++) {
       std::vector<double> max_pan_max_tilt_point;
       max_pan_max_tilt_point.push_back(forbidden_areas[i]["pan_max"]);
       max_pan_max_tilt_point.push_back(forbidden_areas[i]["tilt_max"]);
       max_pan_max_tilt_points.push_back(max_pan_max_tilt_point);
       std::vector<double> max_pan_min_tilt_point;
       max_pan_min_tilt_point.push_back(forbidden_areas[i]["pan_max"]);
       max_pan_min_tilt_point.push_back(forbidden_areas[i]["tilt_min"]);
       max_pan_min_tilt_points.push_back(max_pan_min_tilt_point);
       std::vector<double> min_pan_max_tilt_point;
       min_pan_max_tilt_point.push_back(forbidden_areas[i]["pan_min"]);
       min_pan_max_tilt_point.push_back(forbidden_areas[i]["tilt_max"]);
       min_pan_max_tilt_points.push_back(min_pan_max_tilt_point);
       std::vector<double> min_pan_min_tilt_point;
       min_pan_min_tilt_point.push_back(forbidden_areas[i]["pan_min"]);
       min_pan_min_tilt_point.push_back(forbidden_areas[i]["tilt_min"]);
       min_pan_min_tilt_points.push_back(min_pan_min_tilt_point);
       forbidden_area_first_line_coordinate_forms.push_back(calculateCoordinateForm(max_pan_max_tilt_point, max_pan_min_tilt_point));
       forbidden_area_second_line_coordinate_forms.push_back(calculateCoordinateForm(max_pan_min_tilt_point, min_pan_min_tilt_point));
       forbidden_area_third_line_coordinate_forms.push_back(calculateCoordinateForm(min_pan_min_tilt_point, min_pan_max_tilt_point));
       forbidden_area_fourth_line_coordinate_forms.push_back(calculateCoordinateForm(min_pan_max_tilt_point, max_pan_max_tilt_point));
    }
}

void PTUDriver::prefetchValues() {
    prefetched_pan_current_base = get_current(PAN, BASE);
    prefetched_pan_desired_acceleration = get_desired(PAN, ACCELERATION);
    prefetched_pan_desired_speed = get_desired(PAN, SPEED);
    prefetched_pan_current_position = get_current(PAN, POSITION);
    prefetched_tilt_current_base = get_current(TILT, BASE);
    prefetched_tilt_desired_acceleration = get_desired(TILT, ACCELERATION);
    prefetched_tilt_desired_speed = get_desired(TILT, SPEED);
    prefetched_tilt_current_position = get_current(TILT, POSITION);
}


#ifdef __PTU_FREE_INCLUDED__
long PTUDriver::get_current(char pan_or_tilt, char what) {
   if(pan_or_tilt == PAN) {
       switch(what) {
       case POSITION:
           return free_ptu.getCurrentPanPosition();
           break;
       case RESOLUTION:
           return (free_ptu.getPanResolution() * 60.0);
           break;
       case MINIMUM_POSITION:
           return free_ptu.getCurrentUsedMinimumPanPositionLimit();
           break;
       case MAXIMUM_POSITION:
           return free_ptu.getCurrentUsedMaximumPanPositionLimit();
           break;
       case SPEED:
           return free_ptu.getCurrentPanSpeed();
           break;
       case BASE:
           return free_ptu.getPanBaseSpeed();
           break;
       default:
           ROS_DEBUG("bad use of internal get_current method: input combination not legit");
           return ERROR;
       }
   }
   //Case TILT
   else if(pan_or_tilt == TILT) {
       switch(what) {
       case POSITION:
           return free_ptu.getCurrentTiltPosition();
           break;
       case RESOLUTION:
           return (free_ptu.getTiltResolution() * 60);
           break;
       case MINIMUM_POSITION:
           return free_ptu.getCurrentUsedMinimumTiltPositionLimit();
           break;
       case MAXIMUM_POSITION:
           return free_ptu.getCurrentUsedMaximumTiltPositionLimit();
           break;
       case SPEED:
           return free_ptu.getCurrentTiltSpeed();
           break;
       case BASE:
           return free_ptu.getTiltBaseSpeed();
           break;
       default:
           ROS_DEBUG("bad use of internal get_current method: input combination not legit");
           return ERROR;
       }
   }
   else {
       ROS_DEBUG("bad use of internal get_current method: input combination not legit");
       return ERROR;
   }
}

//TYPE is atm almost never needed because only ABSOLUTE is used. If that changes, edit this method accordingly
char PTUDriver::set_desired(char pan_or_tilt, char what, short int * value, char type) {
   if(pan_or_tilt == PAN) {
       switch(what) {
       case BASE:
           if(free_ptu.setPanBaseSpeed(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       case UPPER_SPEED_LIMIT:
           if(free_ptu.setDesiredPanUpperSpeedLimit(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       case SPEED:
           if(free_ptu.setDesiredPanSpeedAbsolute(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       case ACCELERATION:
           if(free_ptu.setDesiredPanAccelerationAbsolute(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       case HOLD_POWER_LEVEL:
           if(*value == PTU_OFF_POWER){
               if(free_ptu.setPanStationaryPowerMode(OFF_HOLD_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else if (*value == PTU_LOW_POWER){
               if(free_ptu.setPanStationaryPowerMode(LOW_HOLD_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else if(*value == PTU_REG_POWER) {
               if(free_ptu.setPanStationaryPowerMode(REGULAR_HOLD_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else {
                ROS_DEBUG("bad use of internal set_desired method: input combination not legit");
                return PTU_NOT_OK;
           }
           break;
       case MOVE_POWER_LEVEL:
           if(*value == PTU_LOW_POWER){
               if(free_ptu.setPanInMotionPowerMode(LOW_MOVE_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else if (*value == PTU_REG_POWER){
               if(free_ptu.setPanInMotionPowerMode(REGULAR_MOVE_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else if (*value = PTU_HI_POWER) {
               if(free_ptu.setPanInMotionPowerMode(HIGH_MOVE_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else {
               ROS_DEBUG("bad use of internal set_desired method: input combination not legit");
               return PTU_NOT_OK;
           }
           break;
       case POSITION:
           if(free_ptu.setDesiredPanPositionAbsolute(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       default:
           ROS_DEBUG("bad use of internal set_desired method: input combination not legit");
           return PTU_NOT_OK;
       }
   }
   else if(pan_or_tilt == TILT) {
       switch(what) {
       case BASE:
           if(free_ptu.setTiltBaseSpeed(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       case UPPER_SPEED_LIMIT:
           if(free_ptu.setDesiredTiltUpperSpeedLimit(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       case SPEED:
           if(free_ptu.setDesiredTiltSpeedAbsolute(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       case ACCELERATION:
           if(free_ptu.setDesiredTiltAccelerationAbsolute(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       case HOLD_POWER_LEVEL:
           if(*value == PTU_OFF_POWER){
               if(free_ptu.setTiltStationaryPowerMode(OFF_HOLD_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else if (*value == PTU_LOW_POWER){
               if(free_ptu.setTiltStationaryPowerMode(LOW_HOLD_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else if (*value == PTU_REG_POWER) {
               if(free_ptu.setTiltStationaryPowerMode(REGULAR_HOLD_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else {
               ROS_DEBUG("bad use of internal set_desired method: input combination not legit");
               return PTU_NOT_OK;
           }
           break;
       case MOVE_POWER_LEVEL:
           if(*value == PTU_LOW_POWER){
               if(free_ptu.setTiltInMotionPowerMode(LOW_MOVE_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else if (*value == PTU_REG_POWER){
               if(free_ptu.setTiltInMotionPowerMode(REGULAR_MOVE_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else if (*value == PTU_HI_POWER) {
               if(free_ptu.setTiltInMotionPowerMode(HIGH_MOVE_POWER_MODE)) {
                   return PTU_OK;
               }
               else {
                   return PTU_NOT_OK;
               }
           }
           else {
               ROS_DEBUG("bad use of internal set_desired method: input combination not legit");
               return PTU_NOT_OK;
           }
           break;
       case POSITION:
           if(free_ptu.setDesiredTiltPositionAbsolute(*value)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       default:
           ROS_DEBUG("bad use of internal set_desired method: input combination not legit");
           return PTU_NOT_OK;
       }
   }
   else {
       ROS_DEBUG("bad use of internal set_desired method: input combination not legit");
       return PTU_NOT_OK;
   }

}


long PTUDriver::get_desired(char pan_or_tilt, char what) {
   long pow_mode;
   if(pan_or_tilt == PAN) {
       switch(what) {
       case UPPER_SPEED_LIMIT:
           return free_ptu.getPanUpperSpeedLimit();
           break;
       case SPEED:
           return free_ptu.getDesiredPanSpeed();
           break;
       case ACCELERATION:
           return free_ptu.getPanAcceleartion();
           break;
       case HOLD_POWER_LEVEL:
           pow_mode = free_ptu.getPanStationaryPowerMode();
           if(pow_mode == LOW_HOLD_POWER_MODE) {
               return PTU_LOW_POWER;
           }
           else if(pow_mode == OFF_HOLD_POWER_MODE) {
               return PTU_OFF_POWER;
           }
           else if(pow_mode == REGULAR_HOLD_POWER_MODE) {
               return PTU_REG_POWER;
           }
           else {
               return ERROR;
           }
           break;
       case MOVE_POWER_LEVEL:
           pow_mode = free_ptu.getPanInMotionPowerMode();
           if(pow_mode == LOW_MOVE_POWER_MODE) {
               return PTU_LOW_POWER;
           }
           else if(pow_mode == HIGH_MOVE_POWER_MODE) {
               return PTU_HI_POWER;
           }
           else if(pow_mode == REGULAR_MOVE_POWER_MODE) {
               return PTU_REG_POWER;
           }
           else {
               return ERROR;
           }
           break;
       case POSITION:
           return free_ptu.getDesiredPanPosition();
           break;
       case BASE:
           return free_ptu.getPanBaseSpeed();
           break;
       default:
           return ERROR;
       }
   }
   else if (pan_or_tilt == TILT) {
       switch(what) {
       case UPPER_SPEED_LIMIT:
           return free_ptu.getTiltUpperSpeedLimit();
           break;
       case SPEED:
           return free_ptu.getDesiredTiltSpeed();
           break;
       case ACCELERATION:
           return free_ptu.getTiltAcceleartion();
           break;
       case HOLD_POWER_LEVEL:
           pow_mode = free_ptu.getTiltStationaryPowerMode();
           if(pow_mode == LOW_HOLD_POWER_MODE) {
               return PTU_LOW_POWER;
           }
           else if(pow_mode == OFF_HOLD_POWER_MODE) {
               return PTU_OFF_POWER;
           }
           else if(pow_mode == REGULAR_HOLD_POWER_MODE) {
               return PTU_REG_POWER;
           }
           else {
               return ERROR;
           }
           break;
       case MOVE_POWER_LEVEL:
           pow_mode = free_ptu.getTiltInMotionPowerMode();
           if(pow_mode == LOW_MOVE_POWER_MODE) {
               return PTU_LOW_POWER;
           }
           else if(pow_mode == HIGH_MOVE_POWER_MODE) {
               return PTU_HI_POWER;
           }
           else if(pow_mode == REGULAR_MOVE_POWER_MODE) {
               return PTU_REG_POWER;
           }
           else {
               return ERROR;
           }
           break;
       case POSITION:
           return free_ptu.getDesiredTiltPosition();
           break;
       case BASE:
           return free_ptu.getTiltBaseSpeed();
           break;
       default:
           return ERROR;
       }
   }
   else {
       ROS_DEBUG("bad use of internal get_desired method: input combination not legit");
       return ERROR;
   }
}

char PTUDriver::set_mode(char mode_type, char mode) {
   if(mode_type == POSITION_LIMITS_MODE) {
       switch(mode) {
       case OFF_MODE:
           if(free_ptu.setPositionLimitEnforcementMode(LIMITS_DISABLED)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       default:
           return PTU_NOT_OK;
       }
   }
   else if (mode_type == SPEED_CONTROL_MODE) {
       switch(mode) {
       case PTU_PURE_VELOCITY_SPEED_CONTROL_MODE:
           if(free_ptu.setSpeedControlMode(PURE_VELOCITY_CONTROL_MODE)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       case PTU_INDEPENDENT_SPEED_CONTROL_MODE:
           if(free_ptu.setSpeedControlMode(INDEPENDENT_SPEED_MODE)) {
               return PTU_OK;
           }
           else {
               return PTU_NOT_OK;
           }
           break;
       default:
           return PTU_NOT_OK;
       }
   }
   else {
       ROS_DEBUG("bad use of internal set_mode method: input combination not legit");
       return PTU_NOT_OK;
   }
}

#endif // __PTU_FREE_INCLUDED__

}
