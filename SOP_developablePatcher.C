

#include "SOP_developablePatcher.h"

// This is an automatically generated header file based on theDsFile, below,
// to provide SOP_VoronoiGraphParms, an easy way to access parameter values from
// SOP_VoronoiGraphVerb::cook with the correct type.
#include "SOP_developablePatcher.proto.h"

#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <GEO/GEO_PrimPoly.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_TemplateBuilder.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_StringHolder.h>
#include <SYS/SYS_Math.h>
#include <limits.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>



#include <OP/OP_AutoLockInputs.h>
#include <include/converters.h>
#include <include/devFlow_types.h>
#include <include/devFlow_meshstruct.h>
#include "include/devFlow_timestepstruct.h"

#include <developableflow/energy_selector.h>
#include <developableflow/curvature_energy.h>
#include <include/developableflow/timestep.h>



using namespace HDK_Beh;

/// This is the internal name of the SOP type.
/// It isn't allowed to be the same as any other SOP's type name.
const UT_StringHolder SOP_developablePatcher::theSOPTypeName("beh_developablePatcher"_sh);

/// newSopOperator is the hook that Houdini grabs from this dll
/// and invokes to register the SOP.  In this case, we add ourselves
/// to the specified operator table.
void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "beh_developablePatcher",   // Internal name
        "Developable Patcher",                     // UI name
		SOP_developablePatcher::myConstructor,    // How to build the SOP
        SOP_developablePatcher::myTemplateList, // My parameters
        1,                          // Min # of sources
        1,                          // Max # of sources
        nullptr,                    // Custom local variables (none)
        NULL));        // Flag it as generator
}

/// This is a multi-line raw string specifying the parameter interface
/// for this SOP.
static const char *theDsFile = R"THEDSFILE(
{
    name        parameters
    parm {
        name    "divs"      // Internal parameter name
        label   "Divisions" // Descriptive parameter name for user interface
        type    integer
        default { "5" }     // Default for this parameter on new nodes
        range   { 2! 50 }   // The value is prevented from going below 2 at all.
                            // The UI slider goes up to 50, but the value can go higher.
        export  all         // This makes the parameter show up in the toolbox
                            // above the viewport when it's in the node's state.
    }
    }
}
)THEDSFILE";

static PRM_Name names[] = {
	PRM_Name("iters",	"Iterations"),
	PRM_Name("bound",	"Boundary Distance"),
	PRM_Name("mag",	"Mag"),
	PRM_Name("doBoundaryGeo",	"Boundary Geo"),
	PRM_Name("resampleCurves",	"resampleCurves"),
	PRM_Name("keepOutsideSegments",	"keepOutsideSegments"),
	PRM_Name("removePointSegments",	"removePointSegments"),

};
PRM_Template
SOP_developablePatcher::myTemplateList[] = {
	PRM_Template(PRM_INT,  1, &names[0], PRMzeroDefaults),
	PRM_Template(PRM_FLT_J,	1, &names[1], PRMzeroDefaults, 0, &PRMscaleRange),
	PRM_Template(PRM_FLT_J,	1, &names[2], PRMzeroDefaults, 0, &PRMscaleRange),
	PRM_Template(PRM_TOGGLE,    1, &names[3]),
	PRM_Template(PRM_TOGGLE,    1, &names[4]),
	PRM_Template(PRM_TOGGLE,    1, &names[5]),
	PRM_Template(PRM_TOGGLE,    1, &names[6]),
	PRM_Template(PRM_DIRECTION, 3, &PRMdirectionName, PRMzaxisDefaults),
	PRM_Template(),
};
void mesh_to_matrix(GU_Detail &gdp, OMatrixXs &points, OMatrixXi &faces)
{
	printf("in mesh_to_matrix...\n");
	points.resize(0,0); faces.resize(0, 0);
	points.resize(gdp.getNumPoints()*3,1);
	printf("size %i, %i \n", points.rows(), points.cols());
	GA_Offset ptoff;
	printf("getting points\n");
	int i = 0;
	int j = 0;
	GA_FOR_ALL_PTOFF(&gdp, ptoff) {
		i+=1;
		if ((int)ptoff<0 || (int)ptoff*3 > points.rows() ) {
			printf("too many ptoffs!!\n");
			break;
		}
		//printf("pt %i\n", (int)ptoff);
		const UT_Vector3 pos = gdp.getPos3(ptoff);
		j+=1;
		points(static_cast<uint>(ptoff)*3,	0) = pos.x();
		points(static_cast<uint>(ptoff)*3+1,0) = pos.y();
		points(static_cast<uint>(ptoff)*3+2,0) = pos.z();
	}

	printf("resizing faces\n");
	faces.resize(gdp.getNumPrimitives(), 3);
	printf("done resizing faces\n");
	GEO_Primitive *prim;
	GA_FOR_ALL_PRIMITIVES(&gdp, prim)
	{
		const int vertex_count = prim->getVertexCount();
		for (int vt = 0; vt < vertex_count; ++vt) {
			const GA_Offset voff = prim->getPointOffset(vt);
			faces(prim->getMapIndex(), SYSmin(vt, 2)) = static_cast<int>(voff);
		}
	}
}

void
matrix_to_mesh(const OMatrixXs &points, const OMatrixXi &faces, GU_Detail &gdp)
{
	printf("matrix_to_mesh...\n");
	GA_Offset ptoff;
	UT_Vector3 pos;
	printf("appending points...\n");
	gdp.clear();
	gdp.appendPointBlock((GA_Size) points.rows()/3);
	printf("in points:	%i...\n", int(gdp.getNumPoints()));
	printf("out points: %i...\n", int(points.rows()/3));
	printf("in faces:	%i...\n", int(gdp.getNumPrimitives())); 
	printf("out faces:	%i...\n", int(faces.rows()));
	printf("setting point positions...\n");
	GA_FOR_ALL_PTOFF(&gdp, ptoff) {
		//printf("pt: %i...\n", int(ptoff) );
		pos.x() = points(static_cast<uint>(ptoff)*3);
		pos.y() = points(static_cast<uint>(ptoff)*3+1);
		pos.z() = points(static_cast<uint>(ptoff)*3+2);
		//printf("set pos: %lf...\n", pos.x());
		gdp.setPos3(ptoff, pos);
	}

	// TODO: expand to size of matrix
	//gdp.appendPrimitiveBlock(GA_PRIMPOLY, (GA_Size)faces.rows());
	printf("doing faces (%i)...\n", int(faces.rows()) );
	for (uint i = 0; i < faces.rows(); ++i) {
		//printf("face %i\n", i);
		GU_PrimPoly *prim = GU_PrimPoly::build(&gdp, 0, false, false);
		prim->appendVertex((GA_Index)faces(i, 0));
		prim->appendVertex((GA_Index)faces(i, 1));
		prim->appendVertex((GA_Index)faces(i, 2));
	}
	//return error();
}


OP_ERROR
SOP_developablePatcher::cookMySop(OP_Context &context)
{
	printf("SOP_developablePatcher::cook...\n");
	OP_AutoLockInputs inputs(this);
	if (inputs.lock(context) >= UT_ERROR_ABORT)
		return error();

	fpreal now = context.getTime();

	duplicateSource(0, context);
	// 2. Copy input geometry into our gdp
	// 3. Parse and create myGroup
	if (cookInputGroups(context) >= UT_ERROR_ABORT)
		return error();

	setCurGdh(0, myGdpHandle);

	//boostVoronoiGraph vor_diagram;
	double iters = GETITERS();
	double mag = GETMAG();
	double do_boundary_geo = GETDOBOUNDARYGEO();
	bool do_resample = GETRESAMPLE();
	bool keep_outside_segments = GETKEEPOUTIDES();
	bool remove_point_segments = GETREMOVEPOINTSEGS();
	printf("declare F and V...\n");
	OMatrixXi F; //was  Eigen::MatrixXi in edgeyEggs
	OMatrixXs V; //was  Eigen::MatrixXd 
	printf("mesh_to_matrix...\n");
	mesh_to_matrix(*gdp, V, F);
	printf("set m...\n");
	Developables::Mesh m; //Mesh struct
	m = Developables::Mesh(V, F);
	printf("declare timestap data...\n");
	Timestep t_step; //Timestep struct
	Linesearch linesearchMode = LINESEARCH_NONE;
	StepType stepType = STEP_TYPE_GRADDESC;
	EnergyType energyMode = ENERGY_TYPE_HINGE;
	t_step.t = mag;
	printf("solving...\n");
	for (int i = 0; i < iters; i++) {
		printf("iteration %i..\n", i);
		int success = timestep(m.V, m.F, m.VF, m.VFi, m.isB, t_step.t, t_step.p, t_step.energy, t_step.energyGrad, linesearchMode, stepType, energyMode);
	}
	printf("remeshing...\n");
	matrix_to_mesh(m.V, m.F, *gdp);
	inputs.unlock();
	return error();

}

