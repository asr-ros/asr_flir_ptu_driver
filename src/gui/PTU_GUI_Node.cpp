/**
 * PTU_GUI_Node class.
 *
 * @author Valerij Wittenbeck, Pascal Meissner
 * @version See SVN
 */

#include "gui/PTU_GUI_Node.h"

IMPLEMENT_APP(GUI_PTU::PTU_GUI_Node)

namespace GUI_PTU {


PTU_GUI_Node::PTU_GUI_Node()
{
}

PTU_GUI_Node::~PTU_GUI_Node()
{
}

bool PTU_GUI_Node::OnInit()
{
	local_argv =  new char*[ argc ];
	for ( int i = 0; i < argc; ++i ) {
		local_argv[ i ] = strdup( wxString( argv[ i ] ).char_str() );
	}

	ros::init(argc, local_argv, "PTU_GUI");
	nh.reset(new ros::NodeHandle);
	PTU_GUI* dialog = new PTU_GUI( (wxWindow*)NULL);
    dialog ->Show();
    SetTopWindow( dialog );
    return true;
}

int PTU_GUI_Node::OnExit() {
    for ( int i = 0; i < argc; ++i ) {
    	free( local_argv[ i ] );
    }
    delete [] local_argv;
    return 0;
}

}



