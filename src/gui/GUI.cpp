///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "gui/GUI.h"

///////////////////////////////////////////////////////////////////////////

namespace GUI_PTU {

GUIDialog::GUIDialog( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxFlexGridSizer* dialogSizer;
	dialogSizer = new wxFlexGridSizer( 2, 1, 0, 0 );
	dialogSizer->SetFlexibleDirection( wxBOTH );
	dialogSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	cameraPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( -1,-1 ), wxTAB_TRAVERSAL );
	wxFlexGridSizer* cameraImageSizer;
	cameraImageSizer = new wxFlexGridSizer( 2, 2, 0, 0 );
	cameraImageSizer->SetFlexibleDirection( wxBOTH );
	cameraImageSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	leftImageTopic = new wxComboBox( cameraPanel, ID_LEFTTOPICCHOICE, wxEmptyString, wxDefaultPosition, wxSize( -1,-1 ), 0, NULL, wxCB_READONLY|wxCB_SORT );
	cameraImageSizer->Add( leftImageTopic, 0, wxALL|wxEXPAND, 5 );

	rightImageTopic = new wxComboBox( cameraPanel, ID_RIGHTTOPICCHOICE, wxEmptyString, wxDefaultPosition, wxSize( -1,-1 ), 0, NULL, wxCB_READONLY|wxCB_SORT );
	cameraImageSizer->Add( rightImageTopic, 0, wxALL|wxEXPAND, 5 );

	leftPanel = new wxImagePanel( cameraPanel);
	cameraImageSizer->Add( leftPanel, 1, wxEXPAND | wxALL, 5 );
	
	rightPanel = new wxImagePanel( cameraPanel);
	cameraImageSizer->Add( rightPanel, 1, wxEXPAND | wxALL, 5 );
	
	cameraPanel->SetSizer( cameraImageSizer );
	cameraPanel->Layout();
	cameraImageSizer->Fit( cameraPanel );
	dialogSizer->Add( cameraPanel, 1, wxEXPAND | wxALL, 5 );
	
	settingsPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( -1,-1 ), wxTAB_TRAVERSAL );
	wxFlexGridSizer* settingsContentSizer;
	settingsContentSizer = new wxFlexGridSizer( 6, 1, 0, 0 );
	settingsContentSizer->SetFlexibleDirection( wxBOTH );
	settingsContentSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	ptuChoice = new wxComboBox( settingsPanel, ID_PTUCHOICE, wxEmptyString, wxDefaultPosition, wxSize( 380,-1 ), 0, NULL, wxCB_READONLY|wxCB_SORT );
	settingsContentSizer->Add( ptuChoice, 0, wxALL, 5 );

	wxFlexGridSizer* angleSettingsSizer;
	angleSettingsSizer = new wxFlexGridSizer( 2, 8, 0, 0 );
	angleSettingsSizer->SetFlexibleDirection( wxBOTH );
	angleSettingsSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	panAngleLabel = new wxStaticText( settingsPanel, wxID_ANY, wxT("pan angle"), wxDefaultPosition, wxDefaultSize, 0 );
	panAngleLabel->Wrap( -1 );
	angleSettingsSizer->Add( panAngleLabel, 0, wxALL, 10 );
	
	panSlider = new wxSlider( settingsPanel, ID_PANSLIDER, 0, -180, 180, wxPoint( -1,-1 ), wxSize( 200,25 ), wxSL_HORIZONTAL );
	panSlider->SetMinSize( wxSize( 200,25 ) );
	
	angleSettingsSizer->Add( panSlider, 0, wxALL, 5 );
	
	panSpinner = new wxSpinCtrl( settingsPanel, ID_PANSPINNER, wxEmptyString, wxDefaultPosition, wxSize( 80,25 ), wxSP_ARROW_KEYS, -180, 180, -18 );
	angleSettingsSizer->Add( panSpinner, 0, wxALL, 5 );
	
	m_staticline2 = new wxStaticLine( settingsPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_VERTICAL );
	angleSettingsSizer->Add( m_staticline2, 0, wxEXPAND | wxALL, 5 );
	
	m_staticText3 = new wxStaticText( settingsPanel, wxID_ANY, wxT("pan min angle"), wxPoint( -1,-1 ), wxSize( -1,-1 ), 0 );
	m_staticText3->Wrap( -1 );
	angleSettingsSizer->Add( m_staticText3, 0, wxALL, 10 );
	
	pan_min = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -180, 180, 0 );
	angleSettingsSizer->Add( pan_min, 0, wxALL, 5 );
	
	m_staticText4 = new wxStaticText( settingsPanel, wxID_ANY, wxT("pan max angle"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText4->Wrap( -1 );
	angleSettingsSizer->Add( m_staticText4, 0, wxALL, 10 );
	
	pan_max = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -180, 180, 0 );
	angleSettingsSizer->Add( pan_max, 0, wxALL, 5 );
	
	tiltAngleLabel = new wxStaticText( settingsPanel, wxID_ANY, wxT("tilt angle"), wxDefaultPosition, wxDefaultSize, 0 );
	tiltAngleLabel->Wrap( -1 );
	angleSettingsSizer->Add( tiltAngleLabel, 0, wxALL, 10 );
	
	tiltSlider = new wxSlider( settingsPanel, ID_TILTSLIDER, 0, -180, 180, wxDefaultPosition, wxSize( 200,25 ), wxSL_HORIZONTAL );
	tiltSlider->SetMinSize( wxSize( 200,25 ) );
	
	angleSettingsSizer->Add( tiltSlider, 0, wxALL, 5 );
	
	tiltSpinner = new wxSpinCtrl( settingsPanel, ID_TILTSPINNER, wxEmptyString, wxDefaultPosition, wxSize( 80,25 ), wxSP_ARROW_KEYS, -180, 180, -1 );
	angleSettingsSizer->Add( tiltSpinner, 0, wxALL, 5 );
	
	m_staticline3 = new wxStaticLine( settingsPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_VERTICAL );
	angleSettingsSizer->Add( m_staticline3, 0, wxEXPAND | wxALL, 5 );
	
	m_staticText5 = new wxStaticText( settingsPanel, wxID_ANY, wxT("tilt min angle"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText5->Wrap( -1 );
	angleSettingsSizer->Add( m_staticText5, 0, wxALL, 10 );
	
	tilt_min = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -180, 180, 0 );
	angleSettingsSizer->Add( tilt_min, 0, wxALL, 5 );
	
	m_staticText6 = new wxStaticText( settingsPanel, wxID_ANY, wxT("tilt max angle"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText6->Wrap( -1 );
	angleSettingsSizer->Add( m_staticText6, 0, wxALL, 10 );
	
	tilt_max = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -180, 180, 0 );
	angleSettingsSizer->Add( tilt_max, 0, wxALL, 5 );
	
	settingsContentSizer->Add( angleSettingsSizer, 1, wxEXPAND, 5 );
	
	settingsDivider = new wxStaticLine( settingsPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	settingsContentSizer->Add( settingsDivider, 0, wxEXPAND | wxALL, 5 );
	
	wxFlexGridSizer* speedSettingsSizer;
	speedSettingsSizer = new wxFlexGridSizer( 2, 12, 0, 0 );
	speedSettingsSizer->SetFlexibleDirection( wxBOTH );
	speedSettingsSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText10 = new wxStaticText( settingsPanel, wxID_ANY, wxT("pan base speed"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText10->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText10, 0, wxALL, 10 );
	
	pan_base = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 0 );
	speedSettingsSizer->Add( pan_base, 0, wxALL, 5 );
	
	m_staticText11 = new wxStaticText( settingsPanel, wxID_ANY, wxT("pan target speed"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText11->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText11, 0, wxALL, 10 );
	
	pan_target = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 0 );
	speedSettingsSizer->Add( pan_target, 0, wxALL, 5 );
	
	m_staticText13 = new wxStaticText( settingsPanel, wxID_ANY, wxT("pan upper speed"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText13->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText13, 0, wxALL, 10 );
	
	pan_upper = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 0 );
	speedSettingsSizer->Add( pan_upper, 0, wxALL, 5 );
	
	m_staticText14 = new wxStaticText( settingsPanel, wxID_ANY, wxT("pan accel"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText14->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText14, 0, wxALL, 10 );
	
	pan_accel = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 0 );
	speedSettingsSizer->Add( pan_accel, 0, wxALL, 5 );
	
	m_staticText15 = new wxStaticText( settingsPanel, wxID_ANY, wxT("pan hold pwr"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText15->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText15, 0, wxALL, 10 );
	
	pan_hold = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 2, 0 );
	speedSettingsSizer->Add( pan_hold, 0, wxALL, 5 );
	
	m_staticText16 = new wxStaticText( settingsPanel, wxID_ANY, wxT("pan move pwr"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText16->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText16, 0, wxALL, 10 );
	
	pan_move = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 2, 0 );
	speedSettingsSizer->Add( pan_move, 0, wxALL, 5 );
	
	m_staticText20 = new wxStaticText( settingsPanel, wxID_ANY, wxT("tilt base speed"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText20->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText20, 0, wxALL, 10 );
	
	tilt_base = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 0 );
	speedSettingsSizer->Add( tilt_base, 0, wxALL, 5 );
	
	m_staticText21 = new wxStaticText( settingsPanel, wxID_ANY, wxT("tilt target speed"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText21->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText21, 0, wxALL, 10 );
	
	tilt_target = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 0 );
	speedSettingsSizer->Add( tilt_target, 0, wxALL, 5 );
	
	m_staticText22 = new wxStaticText( settingsPanel, wxID_ANY, wxT("tilt upper speed"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText22->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText22, 0, wxALL, 10 );
	
	tilt_upper = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 0 );
	speedSettingsSizer->Add( tilt_upper, 0, wxALL, 5 );
	
	m_staticText23 = new wxStaticText( settingsPanel, wxID_ANY, wxT("tilt accel"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText23->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText23, 0, wxALL, 10 );
	
	tilt_accel = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 0 );
	speedSettingsSizer->Add( tilt_accel, 0, wxALL, 5 );
	
	m_staticText24 = new wxStaticText( settingsPanel, wxID_ANY, wxT("tilt hold pwr"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText24->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText24, 0, wxALL, 10 );
	
	tilt_hold = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 2, 0 );
	speedSettingsSizer->Add( tilt_hold, 0, wxALL, 5 );
	
	m_staticText25 = new wxStaticText( settingsPanel, wxID_ANY, wxT("tilt move pwr"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText25->Wrap( -1 );
	speedSettingsSizer->Add( m_staticText25, 0, wxALL, 10 );
	
	tilt_move = new wxSpinCtrl( settingsPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 2, 0 );
	speedSettingsSizer->Add( tilt_move, 0, wxALL, 5 );
	
	settingsContentSizer->Add( speedSettingsSizer, 1, wxEXPAND, 5 );
	
	wxFlexGridSizer* miscSettingsSizer;
	miscSettingsSizer = new wxFlexGridSizer( 1, 2, 0, 0 );
	miscSettingsSizer->SetFlexibleDirection( wxBOTH );
	miscSettingsSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	immCheck = new wxCheckBox( settingsPanel, ID_UPDATEIMMCHECKBOX, wxT("update current angle immediately"), wxDefaultPosition, wxDefaultSize, 0 );
	miscSettingsSizer->Add( immCheck, 0, wxALL, 5 );
	
	listenCheck = new wxCheckBox( settingsPanel, ID_LISTENFORUPDATES, wxT("listen for updates"), wxDefaultPosition, wxDefaultSize, 0 );
	miscSettingsSizer->Add( listenCheck, 0, wxALL, 5 );

	settingsContentSizer->Add( miscSettingsSizer, 1, wxEXPAND, 5 );
	
	wxFlexGridSizer* buttonSizer;
	buttonSizer = new wxFlexGridSizer( 2, 2, 0, 0 );
	buttonSizer->SetFlexibleDirection( wxBOTH );
	buttonSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	updateButton = new wxButton( settingsPanel, ID_UPDATEBUTTON, wxT("Update"), wxDefaultPosition, wxDefaultSize, 0 );
	buttonSizer->Add( updateButton, 0, wxALL, 5 );
	
	settingsContentSizer->Add( buttonSizer, 1, wxEXPAND, 5 );
	
	settingsPanel->SetSizer( settingsContentSizer );
	settingsPanel->Layout();
	settingsContentSizer->Fit( settingsPanel );
	dialogSizer->Add( settingsPanel, 1, wxEXPAND | wxALL, 5 );
	
	this->SetSizer( dialogSizer );
	this->Layout();
	dialogSizer->Fit( this );
	
	this->Centre( wxBOTH );
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( GUIDialog::OnDialogClose ) );
	leftImageTopic->Connect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( GUIDialog::OnLeftTopicChoice ), NULL, this );
	rightImageTopic->Connect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( GUIDialog::OnRightTopicChoice ), NULL, this );
	ptuChoice->Connect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( GUIDialog::OnPTUChoice ), NULL, this );
	panSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSpinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( GUIDialog::OnPanSpin ), NULL, this );
	panSpinner->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( GUIDialog::OnPanSpinText ), NULL, this );
	tiltSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSpinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( GUIDialog::OnTiltSpin ), NULL, this );
	tiltSpinner->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( GUIDialog::OnTiltSpinText ), NULL, this );
	immCheck->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( GUIDialog::OnImmChecked ), NULL, this );
	listenCheck->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( GUIDialog::OnListenChecked ), NULL, this );
	updateButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GUIDialog::OnUpdateClicked ), NULL, this );
}

GUIDialog::~GUIDialog()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( GUIDialog::OnDialogClose ) );
	leftImageTopic->Disconnect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( GUIDialog::OnLeftTopicChoice ), NULL, this );
	rightImageTopic->Disconnect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( GUIDialog::OnRightTopicChoice ), NULL, this );
	ptuChoice->Disconnect( wxEVT_COMMAND_COMBOBOX_SELECTED, wxCommandEventHandler( GUIDialog::OnPTUChoice ), NULL, this );
	panSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( GUIDialog::OnPanScroll ), NULL, this );
	panSpinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( GUIDialog::OnPanSpin ), NULL, this );
	panSpinner->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( GUIDialog::OnPanSpinText ), NULL, this );
	tiltSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( GUIDialog::OnTiltScroll ), NULL, this );
	tiltSpinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( GUIDialog::OnTiltSpin ), NULL, this );
	tiltSpinner->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( GUIDialog::OnTiltSpinText ), NULL, this );
	immCheck->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( GUIDialog::OnImmChecked ), NULL, this );
	listenCheck->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( GUIDialog::OnListenChecked ), NULL, this );
	updateButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GUIDialog::OnUpdateClicked ), NULL, this );
	
}

}
