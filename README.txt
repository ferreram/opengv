library: OpenGV
pages:   http://laurentkneip.github.io/opengv
brief:   OpenGV is a collection of computer vision methods for solving
         geometric vision problems. It contains absolute-pose, relative-pose,
         triangulation, and point-cloud alignment methods for the calibrated
         case. All problems can be solved with central or non-central cameras,
         and embedded into a random sample consensus or nonlinear optimization
         context. Matlab and Python interfaces are implemented as well. The link
         to the above pages also shows links to precompiled Matlab mex-libraries.
         Please consult the documentation for more information.
author:  Laurent Kneip, The Australian National University
contact: kneip.laurent@gmail.com

============================================================================

Fork of the OpenGV library with a few updates on the Python wrapper.

* Modified version of relative_pose_ransac & absolute_pose_ransac:

- Both functions now return a tuple with the Pose and the list of Inliers
- Use: result = pyopengv.relative_pose_ransac(...) / result[0] = Pose &
  result[1] = inliers_list

- The Ransac threshold parameter to give is now a double which will be
  mutliplied to the default value: (1.0 - cos(atan(sqrt(2.0)*0.5/800.0)))

* Two new functions have been added: relative_pose_ransac_optimize(...) &
* absolute_pose_ransac_optimize(...)

-perform a nonlinear optimization to minimize the reprojection error from all the inliers and hence fine-tune the computed pose
