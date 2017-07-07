# CMake generated Testfile for 
# Source directory: /home/mferrera/libs/opengv
# Build directory: /home/mferrera/libs/opengv/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(test_absolute_pose "/home/mferrera/libs/opengv/build/bin/test_absolute_pose")
SET_TESTS_PROPERTIES(test_absolute_pose PROPERTIES  WORKING_DIRECTORY "/home/mferrera/libs/opengv/build/bin")
ADD_TEST(test_absolute_pose_sac "/home/mferrera/libs/opengv/build/bin/test_absolute_pose_sac")
ADD_TEST(test_noncentral_absolute_pose "/home/mferrera/libs/opengv/build/bin/test_noncentral_absolute_pose")
ADD_TEST(test_noncentral_absolute_pose_sac "/home/mferrera/libs/opengv/build/bin/test_noncentral_absolute_pose_sac")
ADD_TEST(test_multi_noncentral_absolute_pose_sac "/home/mferrera/libs/opengv/build/bin/test_multi_noncentral_absolute_pose_sac")
ADD_TEST(test_relative_pose "/home/mferrera/libs/opengv/build/bin/test_relative_pose")
ADD_TEST(test_relative_pose_rotationOnly "/home/mferrera/libs/opengv/build/bin/test_relative_pose_rotationOnly")
ADD_TEST(test_relative_pose_rotationOnly_sac "/home/mferrera/libs/opengv/build/bin/test_relative_pose_rotationOnly_sac")
ADD_TEST(test_relative_pose_sac "/home/mferrera/libs/opengv/build/bin/test_relative_pose_sac")
ADD_TEST(test_noncentral_relative_pose "/home/mferrera/libs/opengv/build/bin/test_noncentral_relative_pose")
ADD_TEST(test_noncentral_relative_pose_sac "/home/mferrera/libs/opengv/build/bin/test_noncentral_relative_pose_sac")
ADD_TEST(test_multi_noncentral_relative_pose_sac "/home/mferrera/libs/opengv/build/bin/test_multi_noncentral_relative_pose_sac")
ADD_TEST(test_eigensolver_sac "/home/mferrera/libs/opengv/build/bin/test_eigensolver_sac")
ADD_TEST(test_triangulation "/home/mferrera/libs/opengv/build/bin/test_triangulation")
ADD_TEST(test_eigensolver "/home/mferrera/libs/opengv/build/bin/test_eigensolver")
ADD_TEST(test_point_cloud "/home/mferrera/libs/opengv/build/bin/test_point_cloud")
ADD_TEST(test_point_cloud_sac "/home/mferrera/libs/opengv/build/bin/test_point_cloud_sac")
ADD_TEST(test_Sturm "/home/mferrera/libs/opengv/build/bin/test_Sturm")
ADD_TEST(test_odom "/home/mferrera/libs/opengv/build/bin/test_odom")
SUBDIRS(python)
