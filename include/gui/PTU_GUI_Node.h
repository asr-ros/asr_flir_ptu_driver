/**
 * PTU_GUI_Node class.
 *
 * @author Valerij Wittenbeck, Pascal Meissner
 * @version See SVN
 */

#ifndef PTU_GUI_NODE_H_
#define PTU_GUI_NODE_H_

#include <wx/wx.h>
#include "ros/ros.h"
#include "PTU_GUI.h"

namespace GUI_PTU {

class PTU_GUI_Node : public wxApp {
public:
	PTU_GUI_Node();
	char** local_argv;
	ros::NodeHandlePtr nh;

	virtual ~PTU_GUI_Node();
	virtual bool OnInit();
	virtual int OnExit();
};

DECLARE_APP(PTU_GUI_Node)

}

#endif /* PTU_GUI_NODE_H_ */
