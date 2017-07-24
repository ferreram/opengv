#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <Eigen/Eigen>
#include <opengv/triangulation/methods.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/math/cayley.hpp>
#include <sstream>
#include <fstream>
#include <boost/concept_check.hpp>

#include "random_generators.hpp"
#include "experiment_helpers.hpp"
#include "time_measurement.hpp"


using namespace std;
using namespace Eigen;
using namespace opengv;

int main( int argc, char** argv )
{
	//initialize random seed
	initializeRandomSeed();
	
	//set experiment parameters
	double noise = 2.5;
	double outlierFraction = 0.3;
	size_t numberPoints = 200;
	
	//generate a random pose for viewpoint 1
	translation_t position1 = Eigen::Vector3d::Zero();
	rotation_t rotation1 = Eigen::Matrix3d::Identity();
	
	//generate a random pose for viewpoint 2
	translation_t position2 = generateRandomTranslation(2.0);
	rotation_t rotation2 = generateRandomRotation(0.25);
	
	//create a fake central camera
	translations_t camOffsets;
	rotations_t camRotations;
	generateCentralCameraSystem( camOffsets, camRotations );
	
	//derive correspondences based on random point-cloud
	bearingVectors_t bearingVectors1;
	bearingVectors_t bearingVectors2;
	std::vector<int> camCorrespondences1; //unused in the central case
	std::vector<int> camCorrespondences2; //unused in the central case
	Eigen::MatrixXd gt(3,numberPoints);
	generateRandom2D2DCorrespondences(
		position1, rotation1, position2, rotation2,
		camOffsets, camRotations, numberPoints, noise, outlierFraction,
		bearingVectors1, bearingVectors2,
		camCorrespondences1, camCorrespondences2, gt );
	
	
	std::cout << "*************************************************" << std::endl;
	std::cout << "		Ground Truth" << std::endl;
	std::cout << "*************************************************" << std::endl;
	
	//print the experiment characteristics
	printExperimentCharacteristics(
		position2 / position2.norm(), rotation2, noise, outlierFraction );
	
	/*********************************
	 * 		Compute Relative Pose	 *
	 * *******************************/
	
	std::cout << "*************************************************" << std::endl;
	std::cout << "Computing relative Pose between View 0 and View 1" << std::endl;
	std::cout << "*************************************************" << std::endl;
	
	relative_pose::CentralRelativeAdapter rel_adapter(
		bearingVectors1,
		bearingVectors2);
	
	sac::Ransac<
	sac_problems::relative_pose::CentralRelativePoseSacProblem> rel_ransac;
	std::shared_ptr<
	sac_problems::relative_pose::CentralRelativePoseSacProblem> relposeproblem_ptr(
		new sac_problems::relative_pose::CentralRelativePoseSacProblem(
			rel_adapter,
			sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER));
	rel_ransac.sac_model_ = relposeproblem_ptr;
	rel_ransac.threshold_ = 2.*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)));
	rel_ransac.max_iterations_ = 1000;
	
	//Run the experiment
	struct timeval tic;
	struct timeval toc;
	gettimeofday( &tic, 0 );
	rel_ransac.computeModel();
	gettimeofday( &toc, 0 );
	double ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));
	
	//print results for ransac 1
	std::cout << "the ransac threshold is: " << rel_ransac.threshold_ << std::endl;
	std::cout << "the normalized translation result is: " << std::endl;
	std::cout << rel_ransac.model_coefficients_.col(3)/
	rel_ransac.model_coefficients_.col(3).norm() << std::endl << std::endl;
	std::cout << "Ransac needed " << rel_ransac.iterations_ << " iterations and ";
	std::cout << ransac_time << " seconds" << std::endl << std::endl;
	std::cout << "the number of inliers is: " << rel_ransac.inliers_.size() << " out of " << numberPoints;;
	std::cout << std::endl << std::endl;
	
	translation_t t = rel_ransac.model_coefficients_.col(3) / rel_ransac.model_coefficients_.col(3).norm();
	rotation_t R = rel_ransac.model_coefficients_.block<3,3>(0,0);
	
	bearingVectors1.clear();
	bearingVectors2.clear();
	
	for(size_t i = 0 ; i < rel_ransac.inliers_.size() ; ++i)
	{
		bearingVectors1.push_back(rel_adapter.getBearingVector1(rel_ransac.inliers_[i]));
		bearingVectors2.push_back(rel_adapter.getBearingVector2(rel_ransac.inliers_[i]));
	}
	
	relative_pose::CentralRelativeAdapter adapter(
		bearingVectors1,
		bearingVectors2,
		rel_adapter.gett12() / rel_adapter.gett12().norm(),
		rel_adapter.getR12());

	/*********************************
	 * 		Triangulate Points		 *
	 * *******************************/
	
	std::cout << "*************************************************" << std::endl;
	std::cout << "Triangulating Points between View 0 and View 1" << std::endl;
	std::cout << "*************************************************" << std::endl;
	
	std::cout << "running triangulation algorithm 2" << std::endl << std::endl;
	MatrixXd triangulate2_results(3,(int) rel_ransac.inliers_.size());
	gettimeofday( &tic, 0 );
	
	for(size_t j = 0; j < rel_ransac.inliers_.size(); j++)
	{
		triangulate2_results.block<3,1>(0,j) = triangulation::triangulate2(adapter,j);
	}
	
	gettimeofday( &toc, 0 );
	
	ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));
	
	/*
	std::cout << "triangulation truth: " << std::endl;
	std::cout << gt << std::endl;
	*/
	
	std::cout << "triangulation result: " << std::endl;
	std::cout << triangulate2_results.col(0) << std::endl << std::endl;

	std::cout << " in " << ransac_time << " seconds" << std::endl << std::endl;
	
	/*
	MatrixXd error(1,numberPoints);
	for(size_t i = 0; i < numberPoints; i++)
	{
		Vector3d singleError = triangulate2_results.col(i) - gt.col(i);
		error(0,i) = singleError.norm();
	}
	std::cout << "triangulation error is: " << std::endl << error << std::endl;
	*/
	
	/*********************************
	 * 		Compute Absolute Pose	 *
	 * *******************************/
	
	
	std::cout << "*************************************************" << std::endl;
	std::cout << "Computing absolute Pose of View 1 from 3D points" << std::endl;
	std::cout << "*************************************************" << std::endl;
	
	points_t points;
	
	for(size_t i = 0 ; i < rel_ransac.inliers_.size() ; ++i)
	{
		//store the point
		points.push_back(triangulate2_results.col(i));
	}
	
	//create a central absolute adapter
	absolute_pose::CentralAbsoluteAdapter abs_adapter(
		bearingVectors2,
		points);
	
	transformation_t optimized_pose;
	
	//Create an AbsolutePoseSac problem and Ransac
	//The method can be set to KNEIP, GAO or EPNP
	sac::Ransac<sac_problems::absolute_pose::AbsolutePoseSacProblem> abs_ransac;
	std::shared_ptr<
	sac_problems::absolute_pose::AbsolutePoseSacProblem> abspose_kneip_ptr(
		new sac_problems::absolute_pose::AbsolutePoseSacProblem(
			abs_adapter,
			sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
	abs_ransac.sac_model_ = abspose_kneip_ptr;
	abs_ransac.threshold_ = 1.0*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)));
	abs_ransac.max_iterations_ = 1000;
	
	//Run the experiment
	gettimeofday( &tic, 0 );
	abs_ransac.computeModel();
	gettimeofday( &toc, 0 );
	ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));
	
	//print the results
	std::cout << "KNEIP: " << std::endl;
	std::cout << "the ransac results is: " << std::endl;
	std::cout << abs_ransac.model_coefficients_ << std::endl << std::endl;
	std::cout << "Ransac needed " << abs_ransac.iterations_ << " iterations and ";
	std::cout << ransac_time << " seconds" << std::endl << std::endl;
	std::cout << "the number of inliers is: " << abs_ransac.inliers_.size() << " out of " << rel_ransac.inliers_.size();
	std::cout << std::endl << std::endl;
	
	abs_ransac.sac_model_->optimizeModelCoefficients(abs_ransac.inliers_, abs_ransac.model_coefficients_, optimized_pose);
	
	std::cout << "the sac optimized ransac results is: " << std::endl;
	std::cout <<  optimized_pose << std::endl << std::endl;
	
	t = abs_ransac.model_coefficients_.col(3);
	R = abs_ransac.model_coefficients_.block<3,3>(0,0);
	
	abs_adapter.sett(t);
	abs_adapter.setR(R);
	
	transformation_t nonlinear_transformation =
	absolute_pose::optimize_nonlinear(abs_adapter);
	
	std::cout << "results from nonlinear algorithm with every points:" << std::endl;
	std::cout << nonlinear_transformation << std::endl << std::endl;
	
	std::cout << "Truth is: " << std::endl  << std::endl;
	std::cout << position2 / position2.norm() << std::endl << std::endl;
	std::cout << rotation2 << std::endl << std::endl;
	
	
	/*********************************
	 * 		Simulation of view 3	 *
	 * *******************************/
	
	
	std::cout << "*************************************************" << std::endl;
	std::cout << "		Simulating a 3rd View " << std::endl;
	std::cout << "*************************************************" << std::endl;
	
	//generate a random pose for viewpoint 3
	translation_t position3 = position2 + generateRandomTranslation(5.0);
	rotation_t rotation3 = generateRandomRotation(0.25);

	//derive correspondences based on random point-cloud
	bearingVectors_t bearingVectors3;
	std::vector<int> camCorrespondences22; //unused in the central case
	std::vector<int> camCorrespondences33; //unused in the central case
	
	for( size_t i = 0; i < points.size(); i++ )
	{
		//get the point in viewpoint 3
		point_t bodyPoint3 = rotation3.transpose()*(points[i] - position3);

		//get the point in the camera in viewpoint 2
		bearingVectors3.push_back(bodyPoint3);
		
		//normalize the bearing-vectors
		bearingVectors3[i] = bearingVectors3[i] / bearingVectors3[i].norm();
		
		//add noise
		bearingVectors3[i] = addNoise(2.5,bearingVectors3[i]);
	}

	//print the experiment characteristics
	printExperimentCharacteristics(
		position3, rotation3, noise, outlierFraction );
	
	//create a central absolute adapter
	absolute_pose::CentralAbsoluteAdapter next_abs_adapter(
		bearingVectors3,
		points);
	
	//Create an AbsolutePoseSac problem and Ransac
	//The method can be set to KNEIP, GAO or EPNP
	sac::Ransac<sac_problems::absolute_pose::AbsolutePoseSacProblem> next_abs_ransac;
	std::shared_ptr<
	sac_problems::absolute_pose::AbsolutePoseSacProblem> next_abspose_kneip_ptr(
		new sac_problems::absolute_pose::AbsolutePoseSacProblem(
			next_abs_adapter,
			sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
	next_abs_ransac.sac_model_ = next_abspose_kneip_ptr;
	next_abs_ransac.threshold_ = 2.0*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)));
	next_abs_ransac.max_iterations_ = 1000;
	
	//Run the experiment
	gettimeofday( &tic, 0 );
	next_abs_ransac.computeModel();
	gettimeofday( &toc, 0 );
	ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));
	
	//print the results
	std::cout << "KNEIP: " << std::endl;
	std::cout << "the ransac results is: " << std::endl;
	std::cout << next_abs_ransac.model_coefficients_ << std::endl << std::endl;
	std::cout << "Ransac needed " << next_abs_ransac.iterations_ << " iterations and ";
	std::cout << ransac_time << " seconds" << std::endl << std::endl;
	std::cout << "the number of inliers is: " << next_abs_ransac.inliers_.size() << " out of " << rel_ransac.inliers_.size();
	std::cout << std::endl << std::endl;
	
	next_abs_ransac.sac_model_->optimizeModelCoefficients(next_abs_ransac.inliers_, next_abs_ransac.model_coefficients_, optimized_pose);
	
	std::cout << "the sac optimized ransac results is: " << std::endl;
	std::cout <<  optimized_pose << std::endl << std::endl;
	
	t = next_abs_ransac.model_coefficients_.col(3);
	R = next_abs_ransac.model_coefficients_.block<3,3>(0,0);
	
	next_abs_adapter.sett(t);
	next_abs_adapter.setR(R);
	
	nonlinear_transformation =
	absolute_pose::optimize_nonlinear(next_abs_adapter);
	
	std::cout << "results from nonlinear algorithm with every points:" << std::endl;
	std::cout << nonlinear_transformation << std::endl << std::endl;
	
	std::cout << "Truth is: " << std::endl  << std::endl;
	std::cout << position3 << std::endl << std::endl;
	std::cout << rotation3 << std::endl << std::endl;
	
}
	