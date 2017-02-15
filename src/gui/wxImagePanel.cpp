/**
 * wxImagePanel class.
 *
 * @author Valerij Wittenbeck, Pascal Meissner
 * @version See SVN
 */

#include "gui/wxImagePanel.h"
#include <ros/ros.h>

namespace GUI_PTU {

BEGIN_EVENT_TABLE(wxImagePanel, wxPanel)
/*
 EVT_MOTION(wxImagePanel::mouseMoved)
 EVT_LEFT_DOWN(wxImagePanel::mouseDown)
 EVT_LEFT_UP(wxImagePanel::mouseReleased)
 EVT_RIGHT_DOWN(wxImagePanel::rightClick)
 EVT_LEAVE_WINDOW(wxImagePanel::mouseLeftWindow)
 EVT_KEY_DOWN(wxImagePanel::keyPressed)
 EVT_KEY_UP(wxImagePanel::keyReleased)
 EVT_MOUSEWHEEL(wxImagePanel::mouseWheelMoved)
 */

// catch paint events
EVT_PAINT(wxImagePanel::paintEvent)

END_EVENT_TABLE()

wxImagePanel::wxImagePanel(
		wxWindow* parent) :
	wxPanel(parent)
{
	SetSize(wxSize(640, 480));
	image = new wxBitmap(640, 480, -1);
}

/*
 * Called by the system of by wxWidgets when the panel needs
 * to be redrawn. You can also trigger this call by
 * calling Refresh()/Update().
 */

void wxImagePanel::paintEvent(wxPaintEvent & evt)
{
    // depending on your system you may need to look at double-buffered dcs
    wxPaintDC dc(this);
    render(dc);
}

/*
 * Alternatively, you can use a clientDC to paint on the panel
 * at any time. Using this generally does not free you from
 * catching paint events, since it is possible that e.g. the window
 * manager throws away your drawing when the window comes to the
 * background, and expects you will redraw it when the window comes
 * back (by sending a paint event).
 */
void wxImagePanel::paintNow()
{
    // depending on your system you may need to look at double-buffered dcs
    wxClientDC dc(this);
    render(dc);
}

/*
 * Here we do the actual rendering. I put it in a separate
 * method so that it can work no matter what type of DC
 * (e.g. wxPaintDC or wxClientDC) is used.
 */
void wxImagePanel::render(wxDC&  dc)
{
    dc.DrawBitmap( *image, 0, 0, false );
}

void wxImagePanel::setImage(wxBitmap* image) {
	delete this->image;
	this->image = image;
}

}
