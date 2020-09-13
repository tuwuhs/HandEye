
import SwiftFusion
import TensorFlow

// MARK: - Simulated dataset from OpenCV calib3d tests

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
public func simulatePoseEyeInHand(nPoses: Int, addNoise: Bool) -> ([Pose3], [Pose3], Pose3, Pose3) {
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

  let gripper2BaseClean = (0..<nPoses).map { _ in
    generatePose(
      minTheta: 5.0 * .pi / 180.0,
      maxTheta: 45.0 * .pi / 180.0,
      minT: Vector3(0.5, 0.5, 0.5),
      maxT: Vector3(1.5, 1.5, 1.5)
    )
  }

  var rng = SystemRandomNumberGenerator()

  let target2Cam = gripper2BaseClean.map { p -> Pose3 in
    let pose = cam2Gripper.inverse() * p.inverse() * target2Base

    if addNoise {
      let rnoise = NormalDistribution<Double>(mean: 0.0, standardDeviation: 0.002)
      let rvec = pose.rot.toRvec() + Vector3(
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

      return Pose3(Rot3.fromRvec(rvec), tvec)
    } else {
      return pose
    }
  }

  let gripper2Base = gripper2BaseClean.map { p -> Pose3 in 
    if addNoise {
      let rnoise = NormalDistribution<Double>(mean: 0.0, standardDeviation: 0.001)
      let rvec = p.rot.toRvec() + Vector3(
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

      return Pose3(Rot3.fromRvec(rvec), tvec)
    } else {
      return p
    }
  }

  return (gripper2Base, target2Cam, cam2Gripper, target2Base)
}

// MARK: - Simulated dataset from Koide

/// Returns gripper2Base (`[Pose3]`), target2Cam ('[Pose3]'), cam2Gripper (`Pose3`), target2Base (`Pose3`)
public func simulatePoseKoide() -> ([Pose3], [Pose3], Pose3, Pose3) {
  let xRange = 1
  let yRange = 1
  let zRange = 1

  let xStep = 0.5
  let yStep = 0.5
  let zStep = 0.25

  let xOffset = 1.0
  let yOffset = 0.0
  let zOffset = 0.5

  let handToEyeTrans = 0.3
  let handToEyeRotDeg = 90.0

  let wTo = Pose3(Rot3(), Vector3(1.0, 0.0, 0.0))
  
  var rng = SystemRandomNumberGenerator()
  let rotDistribution = NormalDistribution<Double>(mean: 0.0, standardDeviation: handToEyeRotDeg * .pi / 180.0)
  let transDistribution = NormalDistribution<Double>(mean: 0.0, standardDeviation: handToEyeTrans)
  let eTh = Pose3(Rot3.fromAngleAxis(
    rotDistribution.next(using: &rng), Vector3(
      Double.random(in: -1.0...1.0),
      Double.random(in: -1.0...1.0),
      Double.random(in: -1.0...1.0))), 
    transDistribution.next(using: &rng) * Vector3(
      Double.random(in: -1.0...1.0),
      Double.random(in: -1.0...1.0),
      Double.random(in: -1.0...1.0))
  )

  var wThList: [Pose3] = []
  var eToList: [Pose3] = []
  for z in 0...zRange {
    for y in -yRange...yRange {
      for x in -xRange...xRange {
        let wTe_t = Vector3(
          xOffset + xStep * Double(x),
          yOffset + yStep * Double(y),
          zOffset + zStep * Double(z)
        )

        let rotInit = Rot3.fromAngleAxis(.pi, Vector3(0.0, 1.0, 0.0))
        let zfrom = rotInit * Vector3(0.0, 0.0, 1.0)
        var zto = (wTo.t - wTe_t)
        zto = (1.0 / zto.norm) * zto
        
        let angle = acos(zfrom.dot(zto))
        var axis = zto.cross(zfrom)
        axis = (1.0 / axis.norm) * axis

        let wTe = Pose3(Rot3.fromAngleAxis(angle, -axis) * rotInit, wTe_t)
        let wTh = wTe * eTh
        wThList.append(wTh)

        let eTo = wTe.inverse() * wTo
        eToList.append(eTo)
      }
    }
  }

  return (wThList, eToList, eTh.inverse(), wTo)
}

public func applyNoise(_ poses: [Pose3], _ stdevTrans: Double, _ stdevRotDeg: Double) -> [Pose3] {
  var rng = SystemRandomNumberGenerator()
  let rnoise = NormalDistribution<Double>(mean: 0.0, standardDeviation: stdevRotDeg * .pi / 180.0)
  let tnoise = NormalDistribution<Double>(mean: 0.0, standardDeviation: stdevTrans)
  return poses.map { p -> Pose3 in
    let noise = Pose3(Rot3.fromAngleAxis(
      rnoise.next(using: &rng), Vector3(
        Double.random(in: -1.0...1.0),
        Double.random(in: -1.0...1.0),
        Double.random(in: -1.0...1.0))), 
      tnoise.next(using: &rng) * Vector3(
        Double.random(in: -1.0...1.0),
        Double.random(in: -1.0...1.0),
        Double.random(in: -1.0...1.0))
    )
    return noise * p
  }
}