
import SwiftFusion
import TensorFlow

/// Returns random pose within the given limits
func generatePose(minTheta: Double, maxTheta: Double, minT: Vector3, maxT: Vector3) -> Pose3 {
  let r = Rot3.fromAngleAxis(
    (Bool.random() ? 1.0 : -1.0) * Double.random(in: minTheta ... maxTheta),
    Vector3(
      Double.random(in: -1.0...1.0),
      Double.random(in: -1.0...1.0),
      Double.random(in: -1.0...1.0)
    )
  )

  let t = Vector3(
    (Bool.random() ? 1.0 : -1.0) * Double.random(in: minT.x ... maxT.x),
    (Bool.random() ? 1.0 : -1.0) * Double.random(in: minT.y ... maxT.y),
    (Bool.random() ? 1.0 : -1.0) * Double.random(in: minT.z ... maxT.z)
  )

  return Pose3(r, t)
}

/// Returns gripper2Base (`[Pose3]`), target2Cam ('[Pose3]'), cam2Gripper (`Pose3`), target2Base (`Pose3`)
public func simulateDataEyeInHand(nPoses: Int, addNoise: Bool) -> ([Pose3], [Pose3], Pose3, Pose3) {
  // Adapted from simulateDataEyeInHand()
  // https://github.com/opencv/opencv/blob/master/modules/calib3d/test/test_calibration_hand_eye.cpp#L63
  // Notation: cam2Gripper means the transformation to move a pose in camera frame to gripper frame (gTc)
  let cam2Gripper = generatePose(
    minTheta: 10.0 * .pi / 180.0,
    maxTheta: 50.0 * .pi / 180.0,
    minT: Vector3(0.05, 0.05, 0.05),
    maxT: Vector3(0.5, 0.5, 0.5)
  )

  let target2Base = generatePose(
    minTheta: 5.0 * .pi / 180.0,
    maxTheta: 85.0 * .pi / 180.0,
    minT: Vector3(0.5, 0.5, 0.5),
    maxT: Vector3(3.5, 3.5, 3.5)
  )

  var gripper2BaseClean: [Pose3] = []
  for _ in 0..<nPoses {
    gripper2BaseClean.append(generatePose(
      minTheta: 5.0 * .pi / 180.0,
      maxTheta: 45.0 * .pi / 180.0,
      minT: Vector3(0.5, 0.5, 0.5),
      maxT: Vector3(1.5, 1.5, 1.5)
    ))
  }

  var rng = SystemRandomNumberGenerator()

  let target2Cam = gripper2BaseClean.map { p -> Pose3 in
    let pose = cam2Gripper.inverse() * p.inverse() * target2Base

    if addNoise {
      let rnoise = NormalDistribution<Double>(mean: 0.0, standardDeviation: 0.002)
      let rvec = Rot3().localCoordinate(pose.rot) + Vector3(
        rnoise.next(using: &rng),
        rnoise.next(using: &rng),
        rnoise.next(using: &rng)
      )

      let tnoise = NormalDistribution<Double>(mean: 0.0, standardDeviation: 0.005)
      let tvec = pose.t + Vector3(
        tnoise.next(using: &rng),
        tnoise.next(using: &rng),
        tnoise.next(using: &rng)
      )

      return Pose3(Rot3.fromTangent(rvec), tvec)
    } else {
      return pose
    }
  }

  let gripper2Base = gripper2BaseClean.map { p -> Pose3 in 
    if addNoise {
      let rnoise = NormalDistribution<Double>(mean: 0.0, standardDeviation: 0.001)
      let rvec = Rot3().localCoordinate(p.rot) + Vector3(
        rnoise.next(using: &rng),
        rnoise.next(using: &rng),
        rnoise.next(using: &rng)
      )

      let tnoise = NormalDistribution<Double>(mean: 0.0, standardDeviation: 0.001)
      let tvec = p.t + Vector3(
        tnoise.next(using: &rng),
        tnoise.next(using: &rng),
        tnoise.next(using: &rng)
      )

      return Pose3(Rot3.fromTangent(rvec), tvec)
    } else {
      return p
    }
  }

  return (gripper2Base, target2Cam, cam2Gripper, target2Base)
}