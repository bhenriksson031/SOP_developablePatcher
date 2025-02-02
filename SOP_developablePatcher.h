/*
 * This SOP builds a voronoi diagram.
 */

#ifndef __SOP_VoronoiGraph_h__
#define __SOP_VoronoiGraph_h__

#include <SOP/SOP_Node.h>
#include <UT/UT_StringHolder.h>

namespace HDK_Beh {
/// This is the SOP class definition.  It doesn't need to be in a separate
/// file like this.  This is just an example of a header file, in case
/// another file needs to reference something in here.
/// You shouldn't have to change anything in here except the name of the class.
class SOP_developablePatcher : public SOP_Node
{
public:
	static PRM_Template		 myTemplateList[];
    static PRM_Template *buildTemplates();
    static OP_Node *myConstructor(OP_Network *net, const char *name, OP_Operator *op)
    {
        return new SOP_developablePatcher(net, name, op);
    }

    static const UT_StringHolder theSOPTypeName;


protected:
	SOP_developablePatcher(OP_Network *net, const char *name, OP_Operator *op)
        : SOP_Node(net, name, op)
    {
        mySopFlags.setManagesDataIDs(false); //manual edit, was set to false!
    }
	/// Method to cook geometry for the SOP
	virtual OP_ERROR cookMySop(OP_Context &context);

    virtual ~SOP_developablePatcher() {}

private: 

	double GETITERS() { return evalInt("iters", 0, 0); }
	double GETMAG() { return evalFloat("mag", 0, 0); }

};
} // End HDK_Sample namespace

#endif
