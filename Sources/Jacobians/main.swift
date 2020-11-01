
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

      // print(imagePoints[j])
      // print(objectPoints[j])

      // Move a bit from the true value
      let currPose = Pose3(
        Rot3(),
        Vector3(0.2, 0.4, 0.0)) * eTo

      // let error = crf.errorVector(currPose)
      let (error, pb) = valueWithPullback(at: currPose, in: crf.errorVector)
      print(error)
      print(pb(Vector2(1, 0)))
      print(pb(Vector2(0, 1)))
      print()

      // // Try gradient descent, doesn't really work: error decreases then oscillates
      // for k in 0..<50 {
      //   print((currPose * eTo.inverse()).t)

      //   let (errorNorm, grad) = valueWithGradient(at: currPose, in: { p in crf.errorVector(p).norm })
      //   print(errorNorm)
      //   print(grad)

      //   currPose.move(along: -1e-4 * grad)
      //   print()
      // }

      // break
    }

    break
  }

  print()

  for i in 0..<eToList.count {
    let eTo = eToList[i]
    let wTh = wThList[i]

    var x = VariableAssignments()
    let hTeID = x.store(Pose3())
    let wToID = x.store(Pose3())
    let eToID = x.store(Pose3())

    let hepf = HandEyePoseFactor(hTeID, wToID, eToID, wTh)

    let hTe_curr = Pose3() * hTe
      // Rot3(
      //   -1, 0, 0,
      //   0, 1, 0,
      //   0, 0, -1),
      // Vector3(0.1, 0.1, 0.1)) * hTe
    
    let eTo_curr = Pose3( // ) * eTo
      Rot3(),
      Vector3(0.2, 0.4, 0.0)) * eTo

    let wTo_curr = wTh * hTe * eTo

    let (error, pb) = valueWithPullback(at: hTe_curr, wTo_curr, eTo_curr, in: hepf.errorVector)
    print(error)
    print()
    print(pb(Vector6(1, 0, 0, 0, 0, 0)).0)
    print(pb(Vector6(0, 1, 0, 0, 0, 0)).0)
    print(pb(Vector6(0, 0, 1, 0, 0, 0)).0)
    print(pb(Vector6(0, 0, 0, 1, 0, 0)).0)
    print(pb(Vector6(0, 0, 0, 0, 1, 0)).0)
    print(pb(Vector6(0, 0, 0, 0, 0, 1)).0)
    print()
    print(pb(Vector6(1, 0, 0, 0, 0, 0)).1)
    print(pb(Vector6(0, 1, 0, 0, 0, 0)).1)
    print(pb(Vector6(0, 0, 1, 0, 0, 0)).1)
    print(pb(Vector6(0, 0, 0, 1, 0, 0)).1)
    print(pb(Vector6(0, 0, 0, 0, 1, 0)).1)
    print(pb(Vector6(0, 0, 0, 0, 0, 1)).1)
    print()
    print(pb(Vector6(1, 0, 0, 0, 0, 0)).2)
    print(pb(Vector6(0, 1, 0, 0, 0, 0)).2)
    print(pb(Vector6(0, 0, 1, 0, 0, 0)).2)
    print(pb(Vector6(0, 0, 0, 1, 0, 0)).2)
    print(pb(Vector6(0, 0, 0, 0, 1, 0)).2)
    print(pb(Vector6(0, 0, 0, 0, 0, 1)).2)

    break;
  }

}

main()