
import Dispatch
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
  // for i in 0..<wThList.count {
  //   let imagePoints = imagePointsList[i]
  //   let wTh = wThList[i]

  //   var x = VariableAssignments()
  //   let camPoseId = x.store(Pose3(
  //     Rot3(
  //       -1.0, 0.0, 0.0,
  //       0.0, 1.0, 0.0,
  //       0.0, 0.0, -1.0), 
  //     Vector3(-0.1, -0.1, 0.1)))

  //   var graph = FactorGraph()

  //   for j in 0..<imagePoints.count {
  //     graph.store(CameraResectioningFactor(camPoseId, objectPoints[j], imagePoints[j], cameraCalibration))
  //   }

  //   // var timestamps: [DispatchTime] = []
  //   // let start = DispatchTime.now()
  //   // for _ in 0..<10 {
  //   //   timestamps.append(DispatchTime.now())
  //   //   let linearized = graph.linearized(at: x)
  //   //   timestamps.append(DispatchTime.now())
  //   //   var dx = x.tangentVectorZeros
  //   //   timestamps.append(DispatchTime.now())
  //   //   var optimizer = GenericCGLS(precision: 0, max_iteration: 6)
  //   //   timestamps.append(DispatchTime.now())
  //   //   optimizer.optimize(gfg: linearized, initial: &dx)
  //   //   timestamps.append(DispatchTime.now())
  //   //   x.move(along: dx)
  //   // }

  //   // for (ts1, ts2) in zip(timestamps[0..<timestamps.count-1], timestamps[1...]) {
  //   //   let timeInterval = ts2.uptimeNanoseconds - ts1.uptimeNanoseconds
  //   //   print(timeInterval)
  //   // }

  //   var optimizer = LM(precision: 1e-5, max_iteration: 100)
  //   try? optimizer.optimize(graph: graph, initial: &x)

  //   print(x[camPoseId])
  //   print((wTo.inverse() * wTh * hTe).inverse())
  //   print()

  //   // break;
  // }

  // Add pose noise
  wThList = applyNoise(wThList, 0.1, 1.0)

  print("Actual hand-to-eye: \(hTe)")
  print("Actual world-to-object: \(wTo)")
  print()

  let hTe_tsai = calibrateHandEye_tsai(worldToHand: wThList, eyeToObject: eToList)
  let wTo_tsai = wThList[0] * hTe_tsai * eToList[0]
  print("Tsai's method")
  print("Estimated hand-to-eye: \(hTe_tsai)")
  printError(hTe_tsai)
  print()

  let (hTe_factorGraphPose, wTo_factorGraphPose) = calibrateHandEye_factorGraphPose(worldToHand: wThList, eyeToObject: eToList)
  print("Factor graph, pose measurements")
  print("Estimated hand-to-eye: \(hTe_factorGraphPose)")
  print("Estimated world-to-object: \(wTo_factorGraphPose)")
  printError(hTe_factorGraphPose)
  print()

  let (hTe_fgImagePoints, wTo_fgImagePoints) = calibrateHandEye_factorGraphImagePoints(
    worldToHand: wThList, 
    imagePointsList: imagePointsList, 
    objectPoints: objectPoints, 
    cameraCalibration: cameraCalibration,
    handToEyeEstimate: hTe_tsai,
    worldToObjectEstimate: wTo_tsai)

  print("Factor graph, image point measurements")
  print("Estimated hand-to-eye: \(hTe_fgImagePoints)")
  print("Estimated world-to-object: \(wTo_fgImagePoints)")
  printError(hTe_fgImagePoints)
  print()
}

main()
