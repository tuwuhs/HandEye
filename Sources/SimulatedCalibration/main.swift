
import HandEye
import SwiftFusion

func main() {
  // let (wThList, eToList, hTe, wTo) = simulatePoseEyeInHand(nPoses: 10, addNoise: true)
  
  var (wThList, eToList, hTe, wTo) = simulatePoseKoide()

  let printError = { (handToEye: Pose3) in 
    print("Errors:")
    print("rvec: \(handToEye.rot.toRvec() - hTe.rot.toRvec())")
    print("tvec: \(handToEye.t - hTe.t)")
  }

  // Create target object
  let rows = 7
  let cols = 5
  let dimension = 0.1
  var objectPoints: [Vector3] = []
  for row in 0..<rows {
    for col in 0..<cols {
      objectPoints.append(dimension * Vector3(
        Double(row) - Double(rows - 1) / 2.0, 
        Double(col) - Double(cols - 1) / 2.0, 
        0.0))
    }
  }

  // Project points
  let cameraCalibration = CameraCalibration(fx: 300.0, fy: 300.0, s: 0.0, u0: 320.0, v0: 240.0)
  let imagePointsList = wThList.map { wTh -> [Vector2] in 
    let oTe = wTo.inverse() * wTh * hTe
    let cam = PinholeCamera(oTe, cameraCalibration)
    return objectPoints.map { op -> Vector2 in 
      cam.project(op)
    }
  }
  
  // Try camera resectioning
  for i in 0..<wThList.count {
    let imagePoints = imagePointsList[i]
    let wTh = wThList[i]

    var x = VariableAssignments()
    let camPoseId = x.store(Pose3(
      Rot3(
        -1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, -1.0), 
      Vector3(-0.1, -0.1, 0.1)))

    var graph = FactorGraph()

    for j in 0..<imagePoints.count {
      graph.store(CameraResectioningFactor(camPoseId, objectPoints[j], imagePoints[j], cameraCalibration))
    }

    var optimizer = LM(precision: 1e-6, max_iteration: 100)
    try? optimizer.optimize(graph: graph, initial: &x)

    print(x[camPoseId])
    print((wTo.inverse() * wTh * hTe))
    print()
  }

  // let (hTe_fgImagePoints, wTo_fgImagePoints) = calibrateHandEye_factorGraphImagePoints(
  //   worldToHand: wThList, 
  //   imagePointsList: imagePointsList, 
  //   objectPoints: objectPoints, 
  //   cameraCalibration: cameraCalibration,
  //   handToEyeEstimate: hTe,
  //   worldToObjectEstimate: wTo)

  // print("Factor graph, pose measurements")
  // print("Estimated hand-to-eye: \(hTe_fgImagePoints)")
  // print("Estimated world-to-object: \(wTo_fgImagePoints)")
  // printError(hTe_fgImagePoints)

  // print("Actual hand-to-eye: \(hTe)")
  // print("Actual world-to-object: \(wTo)")
  // print()

  // // Add pose noise
  // wThList = applyNoise(wThList, 0.05, 1.0)

  // let hTe_tsai = calibrateHandEye_tsai(worldToHand: wThList, eyeToObject: eToList)
  // print("Tsai's method")
  // print("Estimated hand-to-eye: \(hTe_tsai)")
  // printError(hTe_tsai)

  // print()

  // let (hTe_factorGraphPose, wTo_factorGraphPose) = calibrateHandEye_factorGraphPose(worldToHand: wThList, eyeToObject: eToList)
  // print("Factor graph, pose measurements")
  // print("Estimated hand-to-eye: \(hTe_factorGraphPose)")
  // printError(hTe_factorGraphPose)
}

main()
