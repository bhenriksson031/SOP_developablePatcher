#pragma once
class GU_Detail;
class GU_PrimPoly;

#ifndef __developblePatcher_converters__
#define __developblePatcher_converters__


namespace HDK_Beh {

	void detail_to_eigen(const GU_Detail &gdp, Eigen::MatrixXd &points, Eigen::MatrixXi &faces);
	void detail_to_eigen(const GU_Detail &gdp, Eigen::MatrixXd &points, Eigen::MatrixXi &faces, Eigen::MatrixXi &tetras);
	void attribPointF_to_eigen(const GU_Detail &gdp, Eigen::VectorXd &vec, GA_RWHandleF &attrib);
	void boundaryAttrib_to_eigen(const GU_Detail &gdp, Eigen::VectorXi &b, GA_ROHandleF attrib);
	void eigen_to_detail(const Eigen::MatrixXd &points, const Eigen::MatrixXi &faces, GU_Detail &gdp);
	void eigen_to_detail_points(const Eigen::MatrixXd &points, GU_Detail &gdp);
	void eigen_to_point_attribF(const Eigen::MatrixXd &M, GU_Detail *gdp, const char* attribName);
	void eigen_to_point_attribV(const Eigen::MatrixXd &M, GU_Detail *gdp, const char* attribName);

} // end of namespace SOP_IGL
#endif