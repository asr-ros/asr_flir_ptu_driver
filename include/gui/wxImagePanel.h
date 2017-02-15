/**
 * wxImagePanel class.
 *
 * @author Valerij Wittenbeck, Pascal Meissner
 * @version See SVN
 */

#ifndef WXIMAGEPANEL_H_
#define WXIMAGEPANEL_H_

#include <wx/wx.h>

namespace GUI_PTU {

class wxImagePanel : public wxPanel {
	private:
        wxBitmap* image;

    public:
        wxImagePanel(wxWindow* parent);

        void paintEvent(wxPaintEvent & evt);
        void paintNow();

        void render(wxDC& dc);

        void setImage(wxBitmap* image);

        /*
         void mouseMoved(wxMouseEvent& event);
         void mouseDown(wxMouseEvent& event);
         void mouseWheelMoved(wxMouseEvent& event);
         void mouseReleased(wxMouseEvent& event);
         void rightClick(wxMouseEvent& event);
         void mouseLeftWindow(wxMouseEvent& event);
         void keyPressed(wxKeyEvent& event);
         void keyReleased(wxKeyEvent& event);
         */

        DECLARE_EVENT_TABLE()
    };

}

#endif /* WXIMAGEPANEL_H_ */
