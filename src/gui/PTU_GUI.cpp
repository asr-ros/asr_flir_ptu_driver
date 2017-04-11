/**
 * PTU_GUI class.
 *
 * @author Valerij Wittenbeck, Pascal Meissner
 * @version See SVN
 */

#include "gui/PTU_GUI.h"

namespace GUI_PTU {

PTU_GUI::PTU_GUI( wxWindow* parent ) : GUIDialog( parent )
{
	update_timer = new wxTimer(this);
	update_timer->Start(16);

	immUpdate = false;
	listenForUpdates = false;

	Connect(update_timer->GetId(), wxEVT_TIMER, wxTimerEventHandler(PTU_GUI::onUpdate), NULL, this);

	initTopicList();

//	ros::NodeHandle priv("~");

	panSlider->SetValue(0);
	panSpinner->SetValue(0);
	tiltSlider->SetValue(0);
	tiltSpinner->SetValue(0);

	pan_min->SetValue(0);
	pan_max->SetValue(0);
	pan_base->SetValue(0);
	pan_target->SetValue(0);
	pan_upper->SetValue(0);
	pan_accel->SetValue(0);
	pan_hold->SetValue(0);
	pan_move->SetValue(0);

	tilt_min->SetValue(0);
	tilt_max->SetValue(0);
	tilt_base->SetValue(0);
	tilt_target->SetValue(0);
	tilt_upper->SetValue(0);
	tilt_accel->SetValue(0);
	tilt_hold->SetValue(0);
	tilt_move->SetValue(0);

	leftImageSubscriber = nh.subscribe<sensor_msgs::Image>("left/image_color", 1, &PTU_GUI::onLeftImage, this);
	rightImageSubscriber = nh.subscribe<sensor_msgs::Image>("right/image_color", 1, &PTU_GUI::onRightImage, this);

    predict_client = nh.serviceClient<asr_flir_ptu_driver::Predict>("/asr_flir_ptu_driver/path_prediction");

	wxBitmap* imageLeft = new wxBitmap(wxImage(640, 480));
	leftPanel->setImage(imageLeft);
	leftPanel->paintNow();

	wxBitmap* imageRight = new wxBitmap(wxImage(640, 480));
	rightPanel->setImage(imageRight);
	rightPanel->paintNow();
    seq_num = 0;
    ptu.getParam("asr_flir_ptu_gui/path_prediction", use_path_prediction);
}

void PTU_GUI::updatePTUInfo() {
    jointStatePublisher = ptu.advertise<asr_flir_ptu_driver::State>("asr_flir_ptu_driver/state_cmd", 1);
    jointStateSubscriber = ptu.subscribe<asr_flir_ptu_driver::State>("asr_flir_ptu_driver/ptu_state", 1, &PTU_GUI::onStateCommand, this);
    updater = ptu.serviceClient<std_srvs::Empty>("asr_flir_ptu_driver/update_settings");

    updateSlidersFromParamServer();

	panSlider->SetValue(0);
	panSpinner->SetValue(0);
	tiltSlider->SetValue(0);
	tiltSpinner->SetValue(0);
}

void PTU_GUI::updateSlidersFromParamServer()
{
    int panBase = 0, panSpeed = 0, panUpper = 0, panAccel = 0;
    int tiltBase = 0, tiltSpeed = 0, tiltUpper = 0, tiltAccel = 0;
    int panHold = 0, panMove = 0;
    int tiltHold = 0, tiltMove = 0;
    double panMin = 0, panMax = 0;
    double tiltMin = 0, tiltMax = 0;

    ptu.getParam("asr_flir_ptu_driver/pan_min_angle", panMin);
    ptu.getParam("asr_flir_ptu_driver/pan_max_angle", panMax);
    ptu.getParam("asr_flir_ptu_driver/pan_base_speed", panBase);
    ptu.getParam("asr_flir_ptu_driver/pan_target_speed", panSpeed);
    ptu.getParam("asr_flir_ptu_driver/pan_upper_speed", panUpper);
    ptu.getParam("asr_flir_ptu_driver/pan_accel", panAccel);
    ptu.getParam("asr_flir_ptu_driver/pan_hold", panHold);
    ptu.getParam("asr_flir_ptu_driver/pan_move", panMove);
    ptu.getParam("asr_flir_ptu_driver/tilt_min_angle", tiltMin);
    ptu.getParam("asr_flir_ptu_driver/tilt_max_angle", tiltMax);
    ptu.getParam("asr_flir_ptu_driver/tilt_base_speed", tiltBase);
    ptu.getParam("asr_flir_ptu_driver/tilt_target_speed", tiltSpeed);
    ptu.getParam("asr_flir_ptu_driver/tilt_upper_speed", tiltUpper);
    ptu.getParam("asr_flir_ptu_driver/tilt_accel", tiltAccel);
    ptu.getParam("asr_flir_ptu_driver/tilt_hold", tiltHold);
    ptu.getParam("asr_flir_ptu_driver/tilt_move", tiltMove);

    pan_min->SetValue((int) panMin);
    pan_max->SetValue((int) panMax);
    pan_base->SetValue(panBase);
    pan_target->SetValue(panSpeed);
    pan_upper->SetValue(panUpper);
    pan_accel->SetValue(panAccel);
    pan_hold->SetValue(panHold);
    pan_move->SetValue(panMove);

    tilt_min->SetValue((int) tiltMin);
    tilt_max->SetValue((int) tiltMax);
    tilt_base->SetValue(tiltBase);
    tilt_target->SetValue(tiltSpeed);
    tilt_upper->SetValue(tiltUpper);
    tilt_accel->SetValue(tiltAccel);
    tilt_hold->SetValue(tiltHold);
    tilt_move->SetValue(tiltMove);
}


void PTU_GUI::initTopicList() {
	 ros::master::V_TopicInfo topics;
	 ros::master::getTopics(topics);
	 ros::master::V_TopicInfo::iterator it = topics.begin();
	 ros::master::V_TopicInfo::iterator end = topics.end();

	 int topicCount = 0;

	 for (; it != end; ++it) {
		 const ros::master::TopicInfo& ti = *it;

		 if (strcmp(ti.datatype.c_str(), "sensor_msgs/Image") == 0) {
			 leftImageTopic->Append(wxString::FromUTF8(ti.name.c_str()));
			 rightImageTopic->Append(wxString::FromUTF8(ti.name.c_str()));
			 topicCount++;
         } else if (strcmp(ti.datatype.c_str(), "asr_flir_ptu_driver/State") == 0 &&
				 ti.name.find("state") != std::string::npos) {
			 wxString entryName = wxString::FromUTF8(ti.name.substr(0, ti.name.find("state")).c_str());
			 if (ptuChoice->FindString(entryName) == wxNOT_FOUND) ptuChoice->Append(entryName);
		 }
	 }

	 //these lines assume that the combo boxes are sorted, which they aren't
	 for (long int i = 0; i < topicCount; i++) {
		 if (leftImageTopic->GetString(i).Find(wxString::FromUTF8(DEFAULT_PREFFEREDLEFT)) >= 0) {
			 leftImageTopic->SetSelection(i);
			 break;
		 }
	 }

	 for (long int i = 0; i < topicCount; i++) {
		 if (rightImageTopic->GetString(i).Find(wxString::FromUTF8(DEFAULT_PREFFEREDRIGHT)) >= 0) {
			 rightImageTopic->SetSelection(i);
			 break;
		 }
	 }
}

void PTU_GUI::OnDialogClose( wxCloseEvent& event ) {
	//setting the checkbox seems to suppress the bug where the cam tries to turn around 180°
	immCheck->SetValue(false);
	wxTheApp->Exit();
}

void PTU_GUI::OnLeftTopicChoice( wxCommandEvent& event ) {
	leftImageSubscriber.shutdown();
	std::string str = std::string(leftImageTopic->GetStringSelection().mb_str());
	leftImageSubscriber = nh.subscribe<sensor_msgs::Image>(str, 1, &PTU_GUI::onLeftImage, this);
}

void PTU_GUI::OnRightTopicChoice( wxCommandEvent& event ) {
	rightImageSubscriber.shutdown();
	std::string str = std::string(rightImageTopic->GetStringSelection().mb_str());
	rightImageSubscriber = nh.subscribe<sensor_msgs::Image>(str, 1, &PTU_GUI::onRightImage, this);
}

void PTU_GUI::OnPTUChoice( wxCommandEvent& event ) {
	std::string new_ptu_name = std::string(ptuChoice->GetStringSelection().mb_str());
	if (new_ptu_name.size() == 0) return;
	else ptu_name = new_ptu_name;
	updatePTUInfo();
}

void PTU_GUI::OnPanScroll( wxScrollEvent& event ) {
	panSpinner->SetValue(panSlider->GetValue());
}

void PTU_GUI::OnPanSpin( wxSpinEvent& event ) {
	panSlider->SetValue(panSpinner->GetValue());
}

void PTU_GUI::OnPanSpinText( wxCommandEvent& event ) {
	panSlider->SetValue(panSpinner->GetValue());
}

void PTU_GUI::OnTiltScroll( wxScrollEvent& event ) {
	tiltSpinner->SetValue(tiltSlider->GetValue());
}

void PTU_GUI::OnTiltSpin( wxSpinEvent& event ) {
	tiltSlider->SetValue(tiltSpinner->GetValue());
}

void PTU_GUI::OnTiltSpinText( wxCommandEvent& event ) {
	tiltSlider->SetValue(tiltSpinner->GetValue());
}

void PTU_GUI::OnImmChecked( wxCommandEvent& event ) {
	immUpdate = immCheck->IsChecked();
}

void PTU_GUI::OnListenChecked( wxCommandEvent& event ) {
	listenForUpdates = listenCheck->IsChecked();
	if (listenForUpdates) immCheck->SetValue(false);
	immCheck->Enable(!listenForUpdates);
	updateButton->Enable(!listenForUpdates);
}

void PTU_GUI::OnUpdateClicked( wxCommandEvent& event ) {
	if (ptu_name.size() == 0) return;
    ptu.setParam("asr_flir_ptu_driver/pan_min_angle", (double) pan_min->GetValue());
    ptu.setParam("asr_flir_ptu_driver/pan_max_angle", (double) pan_max->GetValue());
    ptu.setParam("asr_flir_ptu_driver/pan_base_speed", pan_base->GetValue());
    ptu.setParam("asr_flir_ptu_driver/pan_target_speed", pan_target->GetValue());
    ptu.setParam("asr_flir_ptu_driver/pan_upper_speed", pan_upper->GetValue());
    ptu.setParam("asr_flir_ptu_driver/pan_accel", pan_accel->GetValue());
    ptu.setParam("asr_flir_ptu_driver/pan_hold", pan_hold->GetValue());
    ptu.setParam("asr_flir_ptu_driver/pan_move", pan_move->GetValue());
    ptu.setParam("asr_flir_ptu_driver/tilt_min_angle", (double) tilt_min->GetValue());
    ptu.setParam("asr_flir_ptu_driver/tilt_max_angle", (double) tilt_max->GetValue());
    ptu.setParam("asr_flir_ptu_driver/tilt_base_speed", tilt_base->GetValue());
    ptu.setParam("asr_flir_ptu_driver/tilt_target_speed", tilt_target->GetValue());
    ptu.setParam("asr_flir_ptu_driver/tilt_upper_speed", tilt_upper->GetValue());
    ptu.setParam("asr_flir_ptu_driver/tilt_accel", tilt_accel->GetValue());
    ptu.setParam("asr_flir_ptu_driver/tilt_hold", tilt_hold->GetValue());
    ptu.setParam("asr_flir_ptu_driver/tilt_move", tilt_move->GetValue());
	std_srvs::Empty empty;
	updater.call(empty);

    //############### Überprüfen ob funktioniert, z.B. indem man Wert übergibt der außerhalb der phys. Limits liegt ###############################
    updateSlidersFromParamServer();
    //#############################################################################################################################################

	if (!immUpdate) {
        sensor_msgs::JointState joint_state;
        asr_flir_ptu_driver::State msg;
        if(use_path_prediction) {
            asr_flir_ptu_driver::Predict end_point_prediction;
            end_point_prediction.request.pan = panSlider->GetValue();
            end_point_prediction.request.tilt = tiltSlider->GetValue();
            predict_client.call(end_point_prediction);
            joint_state = createJointCommand(end_point_prediction.response.new_pan, end_point_prediction.response.new_tilt, 0, 0);
            msg.no_check_forbidden_area = true;
        }
        else {
            joint_state = createJointCommand(panSlider->GetValue(), tiltSlider->GetValue(), 0, 0);
            msg.no_check_forbidden_area = false;
        }
        msg.state = joint_state;
        seq_num++;
        msg.seq_num = seq_num;
        jointStatePublisher.publish(msg);
	}

}

void PTU_GUI::onStateCommand(const asr_flir_ptu_driver::State::ConstPtr& msg) {
	if (!listenForUpdates) return;

    panSpinner->SetValue(msg->state.position[0]);
	OnPanSpin(((wxSpinEvent&)wxEVT_NULL));
    tiltSpinner->SetValue(msg->state.position[1]);
	OnTiltSpin(((wxSpinEvent&)wxEVT_NULL));
}

void PTU_GUI::onUpdate(wxTimerEvent& evt) {
	if (immUpdate) {
		sensor_msgs::JointState joint_state = createJointCommand(panSlider->GetValue(), tiltSlider->GetValue(), 0, 0);
        asr_flir_ptu_driver::State msg;
        msg.state = joint_state;
        seq_num++;
        msg.seq_num = seq_num;
        jointStatePublisher.publish(msg);
	}

	ros::spinOnce();
	if (!ros::ok())
	{
		Close();
	}
}

wxBitmap* PTU_GUI::createBitmap(const sensor_msgs::Image::ConstPtr& msg) {
	//not a particularly good approach; ignores encoding/image size/etc.
//	wxImage* img = new wxImage(msg->width / 2, msg->height / 2);
	int sizeMod = msg->width / 640;
	wxImage img(msg->width / sizeMod, msg->height / sizeMod);
	int bpp = msg->step / msg->width;

	for (int i = 0; i < img.GetHeight(); i++) {
		for (int j = 0; j < img.GetWidth(); j++) {
			img.SetRGB(j, i,
					msg->data[sizeMod * i * msg->width * bpp + sizeMod * bpp * j + 2],
					msg->data[sizeMod * i * msg->width * bpp + sizeMod * bpp * j + 1],
					msg->data[sizeMod * i * msg->width * bpp + sizeMod * bpp * j]);
		}
	}

	wxBitmap* image = new wxBitmap(img);
	return image;
}

void PTU_GUI::onLeftImage(const sensor_msgs::Image::ConstPtr& msg) {
	leftPanel->setImage(createBitmap(msg));
	leftPanel->paintNow();
}

void PTU_GUI::onRightImage(const sensor_msgs::Image::ConstPtr& msg) {
	rightPanel->setImage(createBitmap(msg));
	rightPanel->paintNow();
}

sensor_msgs::JointState PTU_GUI::createJointCommand(double pan, double tilt, double panSpeed, double tiltSpeed) {
	sensor_msgs::JointState joint_state;
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.push_back("pan");
	joint_state.position.push_back(pan);
	joint_state.velocity.push_back(panSpeed);
	joint_state.name.push_back("tilt");
	joint_state.position.push_back(tilt);
	joint_state.velocity.push_back(tiltSpeed);
	return joint_state;
}

}
