
import HandEye
import SwiftFusion

func main() {
  let (wThList, eToList, hTe, wTo) = simulatePoseKoide(
    eTh: Pose3(
      Rot3(
        1, 0, 0,
        0, -1, 0,
        0, 0, -1), 
      Vector3(0.1, 0.1, 0.1)).inverse())
  let objectPoints = createTargetObject(rows: 3, cols: 2, dimension: 0.15)
  let cameraCalibration = CameraCalibration(fx: 300.0, fy: 300.0, s: 0.0, u0: 320.0, v0: 240.0)
  let imagePointsList = projectPoints(eToList: eToList, objectPoints: objectPoints, calibration: cameraCalibration)
  assert(imagePointsList.allSatisfy { $0.allSatisfy { $0.x >= 0 && $0.x < 640 && $0.y >= 0 && $0.y < 480 } },
    "Some image points fall outside the image boundary")

  for i in 0..<imagePointsList.count {
    let imagePoints = imagePointsList[i]
    let eTo = eToList[i]

    var x = VariableAssignments()
    let camPoseId = x.store(Pose3())
    // let camPoseId = x.store(Pose3(
      // Rot3(
      //   -1, 0, 0,
      //   0, 1, 0,
      //   0, 0, -1),
      // Vector3(-0.1, -0.1, 0.1)))

    for j in 0..<imagePoints.count {
      let crf = CameraResectioningFactor(camPoseId, objectPoints[j], imagePoints[j], cameraCalibration)

      // Move a bit from the true value
      var currPose = Pose3(
        Rot3(),
        Vector3(1.0, 0.0, 0.0)) * eTo
      
      // let currPose = Pose3(
      //   Rot3(
      //     -1, 0, 0,
      //     0, 1, 0,
      //     0, 0, -1),
      //   Vector3(0, 0, 1))

      // let currPose = Pose3(
      //   Rot3(
      //     -1, 0, 0,
      //     0, 1, 0,
      //     0, 0, -1),
      //   Vector3(-0.1, -0.1, 0.1))

      // let error = crf.errorVector(currPose)
      let (error, pb) = valueWithPullback(at: currPose, in: crf.errorVector)
      print(error)
      print(pb(Vector2(1, 0)))
      print(pb(Vector2(0, 1)))
      print()

      // Try gradient descent, doesn't really work: error decreases then oscillates
      for k in 0..<50 {
        print((currPose * eTo.inverse()).t)

        let (errorNorm, grad) = valueWithGradient(at: currPose, in: { p in crf.errorVector(p).norm })
        print(errorNorm)
        print(grad)

        currPose.move(along: -1e-4 * grad)
        print()
      }

      break
    }

    break
  }
}

main()