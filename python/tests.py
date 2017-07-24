import pyopengv
import numpy as np

from matplotlib import pyplot

import cv2


def normalized(x):
    return x / np.linalg.norm(x)

def generateRandomPoint( maximumDepth, minimumDepth ):
    cleanPoint = np.random.uniform(-1.0, 1.0, 3)
    direction = normalized(cleanPoint)
    return (maximumDepth - minimumDepth) * cleanPoint + minimumDepth * direction


def generateRandomTranslation( maximumParallax ):
    return np.random.uniform(-maximumParallax, maximumParallax, 3)


def generateRandomRotation( maxAngle ):
    rpy = np.random.uniform(-maxAngle, maxAngle, 3)

    R1 = np.array([[1.0,  0.0,  0.0],
                   [0.0,  np.cos(rpy[0]), -np.sin(rpy[0])],
                   [0.0,  np.sin(rpy[0]),  np.cos(rpy[0])]])

    R2 = np.array([[ np.cos(rpy[1]),  0.0,  np.sin(rpy[1])],
                   [0.0,  1.0,  0.0],
                   [-np.sin(rpy[1]),  0.0,  np.cos(rpy[1])]])

    R3 = np.array([[np.cos(rpy[2]), -np.sin(rpy[2]),  0.0],
                   [np.sin(rpy[2]),  np.cos(rpy[2]),  0.0],
                   [0.0,  0.0,  1.0]])

    return R3.dot(R2.dot(R1))


def addNoise( noiseLevel, cleanPoint ):
    noisyPoint = cleanPoint + np.random.uniform(-noiseLevel, noiseLevel, 3)
    return normalized(noisyPoint)


def extractRelativePose(position1, position2, rotation1, rotation2):
    relativeRotation = rotation1.T.dot(rotation2)
    relativePosition = rotation1.T.dot(position2 - position1)
    return relativePosition, relativeRotation


def essentialMatrix(position, rotation):
  # E transforms vectors from vp 2 to 1: x_1^T * E * x_2 = 0
  # and E = (t)_skew*R
  t_skew = np.zeros((3, 3))
  t_skew[0,1] = -position[2]
  t_skew[0,2] = position[1]
  t_skew[1,0] = position[2]
  t_skew[1,2] = -position[0]
  t_skew[2,0] = -position[1]
  t_skew[2,1] = position[0]

  E = t_skew.dot(rotation)
  return normalized(E)


def getPerturbedPose(position, rotation, amplitude):
    dp = generateRandomTranslation(amplitude)
    dR = generateRandomRotation(amplitude)
    return position + dp, rotation.dot(dR)


def proportional(x, y, tol=1e-2):
    xn = normalized(x)
    yn = normalized(y)
    return (np.allclose(xn, yn, rtol=1e20, atol=tol) or
            np.allclose(xn, -yn, rtol=1e20, atol=tol))


def matrix_in_list(a, l):
    for b in l:
        if proportional(a, b):
            return True
    return False


def same_transformation(position, rotation, transformation):
    R = transformation[:, :3]
    t = transformation[:, 3]
    return proportional(position, t) and proportional(rotation, R)



class RelativePoseDataset:
    def __init__(self, num_points, noise, outlier_fraction, rotation_only=False):
        # generate a random pose for viewpoint 1
        position1 = np.zeros(3)
        rotation1 = np.eye(3)

        # generate a random pose for viewpoint 2
        if rotation_only:
            position2 = np.zeros(3)
        else:
            position2 = generateRandomTranslation(2.0)
        rotation2 = generateRandomRotation(0.5)

        # derive correspondences based on random point-cloud
        self.generateCorrespondences(
            position1, rotation1, position2, rotation2,
            num_points, noise, outlier_fraction)

        # Extract the relative pose
        self.position, self.rotation = extractRelativePose(
            position1, position2, rotation1, rotation2)
        if not rotation_only:
            self.essential = essentialMatrix(self.position, self.rotation)


    def generateCorrespondences(self,
                                position1, rotation1,
                                position2, rotation2,
                                num_points,
                                noise, outlier_fraction):
        min_depth = 4.5
        max_depth = 5.5

        # initialize point-cloud
        self.points = np.empty((num_points, 3))
        for i in range(num_points):
            self.points[i] = generateRandomPoint(max_depth, min_depth)

        self.bearing_vectors1 = np.empty((num_points, 3))
        self.bearing_vectors2 = np.empty((num_points, 3))
        for i in range(num_points):
            # get the point in viewpoint 1
            body_point1 = rotation1.T.dot(self.points[i] - position1)

            # get the point in viewpoint 2
            body_point2 = rotation2.T.dot(self.points[i] - position2)

            self.bearing_vectors1[i] = normalized(body_point1)
            self.bearing_vectors2[i] = normalized(body_point2)

            # add noise
            if noise > 0.0:
                self.bearing_vectors1[i] = addNoise(noise, self.bearing_vectors1[i])
                self.bearing_vectors2[i] = addNoise(noise, self.bearing_vectors2[i])

        # add outliers
        num_outliers = int(outlier_fraction * num_points)
        for i in range(num_outliers):
            # create random point
            p = generateRandomPoint(max_depth, min_depth)

            # project this point into viewpoint 2
            body_point = rotation2.T.dot(p - position2)

            # normalize the bearing vector
            self.bearing_vectors2[i] = normalized(body_point)


def test_relative_pose():
    print "Testing relative pose"

    d = RelativePoseDataset(10, 0.0, 0.0)

    # running experiments
    twopt_translation = pyopengv.relative_pose_twopt(d.bearing_vectors1, d.bearing_vectors2, d.rotation)
    fivept_nister_essentials = pyopengv.relative_pose_fivept_nister(d.bearing_vectors1, d.bearing_vectors2)
    fivept_kneip_rotations = pyopengv.relative_pose_fivept_kneip(d.bearing_vectors1, d.bearing_vectors2)
    sevenpt_essentials = pyopengv.relative_pose_sevenpt(d.bearing_vectors1, d.bearing_vectors2)
    eightpt_essential = pyopengv.relative_pose_eightpt(d.bearing_vectors1, d.bearing_vectors2)
    t_perturbed, R_perturbed = getPerturbedPose( d.position, d.rotation, 0.01)
    eigensolver_rotation = pyopengv.relative_pose_eigensolver(d.bearing_vectors1, d.bearing_vectors2, R_perturbed)
    t_perturbed, R_perturbed = getPerturbedPose( d.position, d.rotation, 0.1)
    nonlinear_transformation = pyopengv.relative_pose_optimize_nonlinear(d.bearing_vectors1, d.bearing_vectors2, t_perturbed, R_perturbed)

    assert proportional(d.position, twopt_translation)
    assert matrix_in_list(d.essential, fivept_nister_essentials)
    assert matrix_in_list(d.rotation, fivept_kneip_rotations)
    assert matrix_in_list(d.essential, sevenpt_essentials)
    assert proportional(d.essential, eightpt_essential)
    assert proportional(d.rotation, eigensolver_rotation)
    assert same_transformation(d.position, d.rotation, nonlinear_transformation)

    print "Done testing relative pose"


def test_relative_pose_ransac():
    print "Testing relative pose ransac"

    d = RelativePoseDataset(100, 0.0, 0.3)

    ransac_transformation = pyopengv.relative_pose_ransac(
        d.bearing_vectors1, d.bearing_vectors2, "NISTER", 0.01, 1000)

    assert same_transformation(d.position, d.rotation, ransac_transformation[0])

    print("Test Relative Pose RANSAC \n")

    ransac_transformation = pyopengv.relative_pose_ransac(
        d.bearing_vectors1, d.bearing_vectors2, "STEWENIUS", 0.01, 1000)

    print("Result : \n %s \n" % ransac_transformation[0])
    print("Inliers : \n %s \n" % ransac_transformation[1])

    print "Done testing relative pose ransac"


def test_absolute_pose_ransac():
    outliers = 0.25
    noise = 0.01
    d = RelativePoseDataset(100, noise, outliers)

    iterations = 100
    th = 100000.

    ransac_transformation = pyopengv.absolute_pose_ransac(d.bearing_vectors2, d.points, "KNEIP", th, iterations)

    print("\n=========================================")
    print("\n=========================================")
    print("\nTest Absolute Pose RANSAC with :\n")
    print(" - %s points" % len(d.points))
    print(" - %s d'outliers" % (outliers * len(d.points)))
    print(" - Threshold : %s / Iterations : %s" % (th, iterations))

    print("\nResult / Truth: \n %s" % np.hstack((ransac_transformation[0][:,3].reshape(3,1),d.position.reshape(3,1))))
    print("Diff : %s \n" % abs(ransac_transformation[0][:,3].reshape(3,1) - d.position.reshape(3,1)).mean())
    print("Inliers : \n %s \n" % len(ransac_transformation[1]))

    ransac_transformation = pyopengv.absolute_pose_ransac_optimize(d.bearing_vectors2, d.points, "KNEIP", th, iterations)

    print("Optimized Result / Truth: \n %s" % np.hstack((ransac_transformation[0][:,3].reshape(3,1),d.position.reshape(3,1))))
    print("Diff : %s \n" % abs(ransac_transformation[0][:,3].reshape(3,1) - d.position.reshape(3,1)).mean())

    print "Done testing absolute pose ransac"


def test_relative_pose_ransac_rotation_only():
    print "Testing relative pose ransac rotation only"

    d = RelativePoseDataset(100, 0.0, 0.3, rotation_only=True)

    ransac_rotation = pyopengv.relative_pose_ransac_rotation_only(
        d.bearing_vectors1, d.bearing_vectors2, 0.01, 1000)

    assert proportional(d.rotation, ransac_rotation)

    print "Done testing relative pose ransac rotation only"


def test_triangulation():
    print "Testing triangulation"

    d = RelativePoseDataset(10, 0.0, 0.0)

    points1 = pyopengv.triangulation_triangulate(
        d.bearing_vectors1, d.bearing_vectors2, d.position, d.rotation)

    assert np.allclose(d.points, points1)

    points2 = pyopengv.triangulation_triangulate2(
        d.bearing_vectors1, d.bearing_vectors2, d.position, d.rotation)

    assert np.allclose(d.points, points2)

    print "Done testing triangulation \n \n"


def test_relative_pose_triangulated():

    # Creating Random Poses / Points
    d = RelativePoseDataset(500, 0.002, 0.15)

    result = pyopengv.relative_pose_ransac_optimize(d.bearing_vectors1, d.bearing_vectors2, "NISTER", 4., 1000)

    E = result[0]
    
    print("Testing mini Odom\n")
    print("Relative Pose Computed : \n")

    R = E[:,:3]
    t = normalized(E[:,3])
    
    print("Truth : \n %s \n Normalized Truth : %s \n" % (d.position, normalized(d.position)))
    print("Got : \n %s \n Normalized translation : \n %s \n" % (E, t))
    print("%s inliers / %s points \n" % (len(result[1]), len(d.points)))
    
    bearing_vectors_to_erase = []

    inliers = result[1]
    
    for i in xrange(len(d.bearing_vectors1)):
        if inliers != []:
            if inliers[0] == i:
                inliers.pop(0)
            else:
                bearing_vectors_to_erase.append(i)
        else:
            bearing_vectors_to_erase.append(i)

    bv_1 = np.delete(d.bearing_vectors1, bearing_vectors_to_erase, axis=0)
    bv_2 = np.delete(d.bearing_vectors2, bearing_vectors_to_erase, axis=0)

    pts_3d = pyopengv.triangulation_triangulate2(bv_1, bv_2, t, R)

    abs_pose = pyopengv.absolute_pose_ransac_optimize(bv_2, pts_3d, "KNEIP", 2., 500)

    print("Absolute Pose Computed : \n %s \n" % abs_pose[0])
    print("%s inliers / %s points \n" % (len(abs_pose[1]), len(pts_3d)))

    # for i in xrange(len(d.points)):
    #     d.bearing_vectors2[i] = R.T.dot(d.points[i] - t)

    result_ = pyopengv.absolute_pose_ransac(d.bearing_vectors2, d.points, "KNEIP", 0.001, 1000)

    print("\n----------------------\n")
    print("TEST \n")
    print("Truth : \n")
    print(np.hstack((d.rotation,d.position.reshape(3,1))))
    print("Essential M. : \n")
    print(E)
    print()
    print(np.hstack((R,t.reshape(3,1))))
    print()
    print(np.hstack((R,t.reshape(3,1))) / E)

    print("\n Truth : \n")
    print(d.points[:5])
    print()
    print(d.bearing_vectors1[:5])
    print()
    print((d.bearing_vectors1 / d.points)[:5])
    print()
    
    print("\nAbsolute Pose : \n")
    print(normalized(result_[0][:,3]))
    print()
    print(t)


if __name__ == "__main__":
    # test_relative_pose()
    # test_relative_pose_ransac()
    # test_absolute_pose_ransac()
    # test_relative_pose_ransac_rotation_only()
    # test_triangulation()
    # test_relative_pose_triangulated()
    test_absolute_pose_ransac()
    