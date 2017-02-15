///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __GUI__
#define __GUI__

#include <wx/string.h>
#include <wx/combobox.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/slider.h>
#include <wx/spinctrl.h>
#include <wx/statline.h>
#include <wx/checkbox.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include "wxImagePanel.h"

///////////////////////////////////////////////////////////////////////////

#define ID_LEFTTOPICCHOICE 1000
#define ID_RIGHTTOPICCHOICE 1001
#define ID_PTUCHOICE 1002
#define ID_PANSLIDER 1003
#define ID_PANSPINNER 1004
#define ID_TILTSLIDER 1005
#define ID_TILTSPINNER 1006
#define ID_UPDATEIMMCHECKBOX 1007
#define ID_LISTENFORUPDATES 1008
#define ID_UPDATEBUTTON 1009

namespace GUI_PTU {

///////////////////////////////////////////////////////////////////////////////
/// Class GUIDialog
///////////////////////////////////////////////////////////////////////////////
class GUIDialog : public wxDialog 
{
	private:
	
	protected:
		wxPanel* cameraPanel;
		wxComboBox* leftImageTopic;
		wxComboBox* rightImageTopic;
		wxImagePanel* leftPanel;
		wxImagePanel* rightPanel;
		wxPanel* settingsPanel;
		wxComboBox* ptuChoice;
		wxStaticText* panAngleLabel;
		wxSlider* panSlider;
		wxSpinCtrl* panSpinner;
		wxStaticLine* m_staticline2;
		wxStaticText* m_staticText3;
		wxSpinCtrl* pan_min;
		wxStaticText* m_staticText4;
		wxSpinCtrl* pan_max;
		wxStaticText* tiltAngleLabel;
		wxSlider* tiltSlider;
		wxSpinCtrl* tiltSpinner;
		wxStaticLine* m_staticline3;
		wxStaticText* m_staticText5;
		wxSpinCtrl* tilt_min;
		wxStaticText* m_staticText6;
		wxSpinCtrl* tilt_max;
		wxStaticLine* settingsDivider;
		wxStaticText* m_staticText10;
		wxSpinCtrl* pan_base;
		wxStaticText* m_staticText11;
		wxSpinCtrl* pan_target;
		wxStaticText* m_staticText13;
		wxSpinCtrl* pan_upper;
		wxStaticText* m_staticText14;
		wxSpinCtrl* pan_accel;
		wxStaticText* m_staticText15;
		wxSpinCtrl* pan_hold;
		wxStaticText* m_staticText16;
		wxSpinCtrl* pan_move;
		wxStaticText* m_staticText20;
		wxSpinCtrl* tilt_base;
		wxStaticText* m_staticText21;
		wxSpinCtrl* tilt_target;
		wxStaticText* m_staticText22;
		wxSpinCtrl* tilt_upper;
		wxStaticText* m_staticText23;
		wxSpinCtrl* tilt_accel;
		wxStaticText* m_staticText24;
		wxSpinCtrl* tilt_hold;
		wxStaticText* m_staticText25;
		wxSpinCtrl* tilt_move;
		wxCheckBox* immCheck;
		wxCheckBox* listenCheck;
		wxButton* updateButton;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnDialogClose( wxCloseEvent& event ) { event.Skip(); }
		virtual void OnLeftTopicChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnRightTopicChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnPTUChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnPanScroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnPanSpin( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnPanSpinText( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnTiltScroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnTiltSpin( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnTiltSpinText( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnImmChecked( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnListenChecked( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnUpdateClicked( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		GUIDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("PTU Visualisation"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_DIALOG_STYLE );
		~GUIDialog();
	
};

}

#endif //__GUI__
