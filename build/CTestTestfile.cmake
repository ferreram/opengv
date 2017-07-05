# CMake generated Testfile for 
# Source directory: /home/maxime/libraries/opengv
# Build directory: /home/maxime/libraries/opengv/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_absolute_pose "/home/maxime/libraries/opengv/build/bin/test_absolute_pose")
set_tests_properties(test_absolute_pose PROPERTIES  WORKING_DIRECTORY "/home/maxime/libraries/opengv/build/bin")
add_test(test_absolute_pose_sac "/home/maxime/libraries/opengv/build/bin/test_absolute_pose_sac")
add_test(test_noncentral_absolute_pose "/home/maxime/libraries/opengv/build/bin/test_noncentral_absolute_pose")
add_test(test_noncentral_absolute_pose_sac "/home/maxime/libraries/opengv/build/bin/test_noncentral_absolute_pose_sac")
add_test(test_multi_noncentral_absolute_pose_sac "/home/maxime/libraries/opengv/build/bin/test_multi_noncentral_absolute_pose_sac")
add_test(test_relative_pose "/home/maxime/libraries/opengv/build/bin/test_relative_pose")
add_test(test_relative_pose_rotationOnly "/home/maxime/libraries/opengv/build/bin/test_relative_pose_rotationOnly")
add_test(test_relative_pose_rotationOnly_sac "/home/maxime/libraries/opengv/build/bin/test_relative_pose_rotationOnly_sac")
add_test(test_relative_pose_sac "/home/maxime/libraries/opengv/build/bin/test_relative_pose_sac")
add_test(test_noncentral_relative_pose "/home/maxime/libraries/opengv/build/bin/test_noncentral_relative_pose")
add_test(test_noncentral_relative_pose_sac "/home/maxime/libraries/opengv/build/bin/test_noncentral_relative_pose_sac")
add_test(test_multi_noncentral_relative_pose_sac "/home/maxime/libraries/opengv/build/bin/test_multi_noncentral_relative_pose_sac")
add_test(test_eigensolver_sac "/home/maxime/libraries/opengv/build/bin/test_eigensolver_sac")
add_test(test_triangulation "/home/maxime/libraries/opengv/build/bin/test_triangulation")
add_test(test_eigensolver "/home/maxime/libraries/opengv/build/bin/test_eigensolver")
add_test(test_point_cloud "/home/maxime/libraries/opengv/build/bin/test_point_cloud")
add_test(test_point_cloud_sac "/home/maxime/libraries/opengv/build/bin/test_point_cloud_sac")
add_test(test_Sturm "/home/maxime/libraries/opengv/build/bin/test_Sturm")
subdirs(python)
