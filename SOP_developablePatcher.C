

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
//#include <include/converters.h>
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
	PRM_Name("mag",	"Mag"),
};
PRM_Template
SOP_developablePatcher::myTemplateList[] = {
	PRM_Template(PRM_INT,  1, &names[0], PRMzeroDefaults),
	PRM_Template(PRM_FLT_J,	1, &names[2], PRMzeroDefaults, 0, &PRMscaleRange),
	PRM_Template(PRM_DIRECTION, 3, &PRMdirectionName, PRMzaxisDefaults),
	PRM_Template(),
};
void mesh_to_matrix(GU_Detail &gdp, OMatrixXs &points, OMatrixXi &faces)
{
	printf("in mesh_to_matrix...\n");
	points = OMatrixXs(); 
	faces = OMatrixXi();
	points.resize(gdp.getNumPoints(),3);
	printf("npoints: %i\n", (int)gdp.getNumPoints());
	printf("size %i, %i \n", (int)points.rows(), (int)points.cols() );
	printf("getting points\n");
	int i = 0;
	int j = 0;

	GA_Offset start; GA_Offset end;
	for (GA_Iterator ptit(gdp.getPointRange()); ptit.blockAdvance(start, end); ) {
		for (GA_Offset ptoff = start; ptoff < end; ++ptoff) {
			/*
			if ((int)ptoff<0 || (int)ptoff * 3 > points.rows()) {
				printf("too many ptoffs!!\n");
				break;
			}*/
			//printf("pt %i\n", (int)ptoff);
			const UT_Vector3 pos = gdp.getPos3(ptoff);
			int ptindex = gdp.pointIndex(ptoff);
			points(ptindex, 0) = pos.x();
			points(ptindex, 1) = pos.y();
			points(ptindex, 2) = pos.z();
			i += 1;
		}
	}

	printf("resizing faces\n");
	faces.resize(gdp.getNumPrimitives(), 3);
	printf("done resizing faces\n");
	GA_Primitive *prim;
	i = 0;
	GA_Range index_map = gdp.getPointRange();
	for (GA_Iterator primit(gdp.getPrimitiveRange()); !primit.atEnd(); ++primit) {
		// getPrimitiveVertexList is new in 16.0; most people probably use getPrimitive,
		// then call getVertexCount and getVertexOffset on it.
		prim = gdp.getPrimitive(*primit);
		const GA_OffsetListRef vertices = gdp.getPrimitiveVertexList(*primit);
		for (GA_Size j = 0, n = vertices.size(); j < n; ++j) {
			const GA_Offset ptoff = prim->getPointOffset(j);
			if(vertices.size() != 3)
			{
				printf("non triangulated polygon, will break!\n");
			}
			//printf("i=%i, primit=%i, ptoff=%i, ", i, (int)*primit, (int)ptoff;
			int ptindex = gdp.pointIndex(ptoff);
			faces(i,j) = ptindex;
			//faces(prim->getMapIndex(), SYSmin(vt, 2)) = static_cast<int>(voff);
		}
		++i;
	}
	if (false){
		printf("points, size(%i, %i): [\n", (int)points.rows(), (int)points.cols());
		for (int r = 0; r < (int)points.rows(); ++r) {
			printf("%lf, ", points(r, 0));
		}
		printf("]\n");
		printf("faces, size(%i, %i): [\n", (int)faces.rows(), (int)faces.cols());
		for (int r = 0; r < (int)faces.rows(); ++r) {
			printf("%i, ", faces(r, 0));
		}
		printf("]\n");
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
	gdp.appendPointBlock((GA_Size) points.rows());
	printf("in points:	%i...\n", int(gdp.getNumPoints()));
	printf("out points: %i...\n", int(points.rows()));
	printf("in faces:	%i...\n", int(gdp.getNumPrimitives())); 
	printf("out faces:	%i...\n", int(faces.rows()));
	printf("setting point positions...\n");
	GA_FOR_ALL_PTOFF(&gdp, ptoff) {
		//printf("pt: %i...\n", int(ptoff) );
		pos.x() = points(static_cast<uint>(ptoff), 0);
		pos.y() = points(static_cast<uint>(ptoff), 1);
		pos.z() = points(static_cast<uint>(ptoff), 2);
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
//#include <igl/read_triangle_mesh.h>

OP_ERROR
SOP_developablePatcher::cookMySop(OP_Context &context)
{
	OP_AutoLockInputs inputs(this);
	if (inputs.lock(context) >= UT_ERROR_ABORT)
		return error();

	UT_AutoInterrupt progress("SOP_developablePatcher::cook...\n");

	fpreal now = context.getTime();

	// 2. Copy input geometry into our gdp
	// 3. Parse and create myGroup
	if (cookInputGroups(context) >= UT_ERROR_ABORT)
		return error();

	// Duplicate incoming geometry.
	duplicateSource(0, context);

	setCurGdh(0, myGdpHandle);

	//boostVoronoiGraph vor_diagram;
	double iters = GETITERS();
	double mag = GETMAG();
	printf("declare F and V...\n");
	OMatrixXi F = OMatrixXi(); //   V  eigen double matrix #V by 3 (from igl::read_triangle_mesh)  //was  Eigen::MatrixXi in edgeyEggs
	OMatrixXs V = OMatrixXs();; //  F  eigen int matrix #F by 3    (from igl::read_triangle_mesh)  //was  Eigen::MatrixXd 
	  
  


	printf("mesh_to_matrix...\n");
	mesh_to_matrix(*gdp, V, F);
	printf("F size(%i, %i): [\n", (int)F.rows(), (int)F.cols());
	printf("V size(%i, %i): [\n", (int)V.rows(), (int)V.cols());
	printf("set m...\n");
	Developables::Mesh m; //Mesh struct

	m = Developables::Mesh(V, F);
	printf("declare timestap data...\n");
	
	Linesearch linesearchMode = LINESEARCH_NONE;
	StepType stepType = STEP_TYPE_GRADDESC;
	EnergyType energyMode = ENERGY_TYPE_HINGE;
	
	printf("solving...\n");
	for (int i = 0; i < iters; i++) {
		printf("iteration %i..\n", i);
		Timestep t_step; //Timestep struct, reset between iterations
		t_step.t = mag;
		if (progress.wasInterrupted())
			break;
		int success = timestep(m.V, m.F, m.VF, m.VFi, m.isB, t_step.t, t_step.p, t_step.energy, t_step.energyGrad, linesearchMode, stepType, energyMode);
	}
	printf("remeshing...\n");
	matrix_to_mesh(m.V, m.F, *gdp);
	//inputs.unlock();
	return error();

}

