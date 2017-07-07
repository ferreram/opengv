/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <Eigen/Eigen>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <sstream>
#include <fstream>

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
  double noise = 5.0;
  double outlierFraction = 0.3;
  size_t numberPoints = 400;

  //create a random viewpoint pose
  translation_t position = generateRandomTranslation(2.0);
  rotation_t rotation = generateRandomRotation(0.5);
  
  //create a fake central camera
  translations_t camOffsets;
  rotations_t camRotations;
  generateCentralCameraSystem( camOffsets, camRotations );
  
  //derive correspondences based on random point-cloud
  bearingVectors_t bearingVectors;
  points_t points;
  std::vector<int> camCorrespondences; //unused in the central case!
  Eigen::MatrixXd gt(3,numberPoints);
  generateRandom2D3DCorrespondences(
      position, rotation, camOffsets, camRotations, numberPoints, noise, outlierFraction,
      bearingVectors, points, camCorrespondences, gt );

  //print the experiment characteristics
  printExperimentCharacteristics(
      position, rotation, noise, outlierFraction );

  //create a central absolute adapter
  absolute_pose::CentralAbsoluteAdapter adapter(
      bearingVectors,
      points,
      rotation);
  
  transformation_t optimized_pose;

  //Create an AbsolutePoseSac problem and Ransac
  //The method can be set to KNEIP, GAO or EPNP
  sac::Ransac<sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
  std::shared_ptr<
      sac_problems::absolute_pose::AbsolutePoseSacProblem> abspose_kneip_ptr(
      new sac_problems::absolute_pose::AbsolutePoseSacProblem(
      adapter,
      sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
	  ransac.sac_model_ = abspose_kneip_ptr;
  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
  ransac.max_iterations_ = 50;

  //Run the experiment
  struct timeval tic;
  struct timeval toc;
  gettimeofday( &tic, 0 );
  ransac.computeModel();
  gettimeofday( &toc, 0 );
  double ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));

  //print the results
  std::cout << "KNEIP: " << std::endl;
  std::cout << "the ransac results is: " << std::endl;
  std::cout << ransac.model_coefficients_ << std::endl << std::endl;
  std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  std::cout << ransac_time << " seconds" << std::endl << std::endl;
  std::cout << "the number of inliers is: " << ransac.inliers_.size() << " out of " << numberPoints;
  std::cout << std::endl << std::endl;
  
  ransac.sac_model_->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, optimized_pose);
  
  std::cout << "the optimized ransac results is: " << std::endl;
  std::cout <<  optimized_pose << std::endl << std::endl;
  
  //Create an AbsolutePoseSac problem and Ransac
  //The method can be set to KNEIP, GAO or EPNP
  std::shared_ptr<
  sac_problems::absolute_pose::AbsolutePoseSacProblem> abspose_gao_ptr(
	  new sac_problems::absolute_pose::AbsolutePoseSacProblem(
		  adapter,
		  sac_problems::absolute_pose::AbsolutePoseSacProblem::GAO));
  ransac.sac_model_ = abspose_gao_ptr;
  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
  ransac.max_iterations_ = 50;
  
  //Run the experiment
  gettimeofday( &tic, 0 );
  ransac.computeModel();
  gettimeofday( &toc, 0 );
  ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));
  
  //print the results
  std::cout << "GAO: " << std::endl;
  std::cout << "the ransac results is: " << std::endl;
  std::cout << ransac.model_coefficients_ << std::endl << std::endl;
  std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  std::cout << ransac_time << " seconds" << std::endl << std::endl;
  std::cout << "the number of inliers is: " << ransac.inliers_.size() << " out of " << numberPoints;
  std::cout << std::endl << std::endl;
  
  ransac.sac_model_->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, optimized_pose);
  
  std::cout << "the optimized ransac results is: " << std::endl;
  std::cout <<  optimized_pose << std::endl << std::endl;

  //Create an AbsolutePoseSac problem and Ransac
  //The method can be set to KNEIP, GAO or EPNP
  std::shared_ptr<
  sac_problems::absolute_pose::AbsolutePoseSacProblem> abspose_epnp_ptr(
	  new sac_problems::absolute_pose::AbsolutePoseSacProblem(
		  adapter,
		  sac_problems::absolute_pose::AbsolutePoseSacProblem::EPNP));
  ransac.sac_model_ = abspose_epnp_ptr;
  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
  ransac.max_iterations_ = 50;
  
  gettimeofday( &tic, 0 );
  ransac.computeModel();
  gettimeofday( &toc, 0 );
  ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));
  
  //print the results
  std::cout << "EPNP: " << std::endl;
  std::cout << "the ransac results is: " << std::endl;
  std::cout << ransac.model_coefficients_ << std::endl << std::endl;
  std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  std::cout << ransac_time << " seconds" << std::endl << std::endl;
  std::cout << "the number of inliers is: " << ransac.inliers_.size() << " out of " << numberPoints;
  std::cout << std::endl << std::endl;

  ransac.sac_model_->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, optimized_pose);
  
  std::cout << "the optimized ransac results is: " << std::endl;
  std::cout <<  optimized_pose << std::endl << std::endl;
  /*
  std::vector<int> indices10 = getNindices(numberPoints);
  adapter.sett(ransac.model_coefficients_);
  adapter.setR(R_perturbed);
  transformation_t nonlinear_transformation_10 =
  absolute_pose::optimize_nonlinear(adapter,indices10);
  
  adapter.sett(ransac.sac_model_->optimizeModelCoefficients());
  
  adapter.setR(ransac.sac_model_->);
  
  transformation_t nonlinear_transformation_10 =
  absolute_pose::optimize_nonlinear(adapter,indices10);
  */
  
}
