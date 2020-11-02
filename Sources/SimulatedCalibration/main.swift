
import Dispatch
import HandEye
import SwiftFusion

func main() {
  // let (wThList, eToList, hTe, wTo) = simulatePoseEyeInHand(nPoses: 10, addNoise: true)
  
  var (wThList, eToList, hTe, wTo) = simulatePoseKoide()
  // wThList = [wThList[1], wThList[5], wThList[12]]
  // eToList = [eToList[1], eToList[5], eToList[12]]
  
  let printError = { (handToEye: Pose3) in 
    print("Errors:")
    print("rvec: \(handToEye.rot.toRvec() - hTe.rot.toRvec())")
    print("tvec: \(handToEye.t - hTe.t)")
  }

  let printErrorMagnitude = { (handToEye: Pose3) in 
    print("rvec: \((handToEye.rot.toRvec() - hTe.rot.toRvec()).norm)")
    print("tvec: \((handToEye.t - hTe.t).norm)")
  }

  // Project points
  let objectPoints = createTargetObject(rows: 3, cols: 3, dimension: 0.25)
  let cameraCalibration = CameraCalibration(fx: 300.0, fy: 300.0, s: 0.0, u0: 320.0, v0: 240.0)
  let imagePointsList = projectPoints(eToList: eToList, objectPoints: objectPoints, calibration: cameraCalibration)
  assert(imagePointsList.allSatisfy { $0.allSatisfy { $0.x >= 0 && $0.x < 640 && $0.y >= 0 && $0.y < 480 } },
    "Some image points fall outside the image boundary")
  
  // Try camera resectioning
  // for i in 0..<wThList.count {
  //   let imagePoints = imagePointsList[i]
  //   let wTh = wThList[i]

  //   let oTe_estimate = performCameraResectioning(
  //     wTh: wTh, imagePoints: imagePoints, objectPoints: objectPoints, calibration: cameraCalibration)
    
  //   print(oTe_estimate)
  //   print((wTo.inverse() * wTh * hTe).inverse())
  //   print()

  //   // break
  // }

  // Add pose noise
  // wThList = applyNoise(wThList, 0.01, 0.1)

  print("Actual hand-to-eye: \(hTe)")
  print("Actual world-to-object: \(wTo)")
  print()

  // let hTe_tsai = calibrateHandEye_tsai(wThList: wThList, eToList: eToList)
  // let wTo_tsai = wThList[0] * hTe_tsai * eToList[0]
  // // print("Tsai's method")
  // // print("Estimated hand-to-eye: \(hTe_tsai)")
  // // printError(hTe_tsai)
  // printErrorMagnitude(hTe_tsai)
  // // print()

  // let (hTe_factorGraphPose, wTo_factorGraphPose) = calibrateHandEye_factorGraphPose(
  //   wThList: wThList, eToList: eToList)
  // // print("Factor graph, pose measurements")
  // // print("Estimated hand-to-eye: \(hTe_factorGraphPose)")
  // // print("Estimated world-to-object: \(wTo_factorGraphPose)")
  // // printError(hTe_factorGraphPose)
  // printErrorMagnitude(hTe_factorGraphPose)
  // // print()

  let (hTe_fgImagePoints, wTo_fgImagePoints) = calibrateHandEye_factorGraphImagePoints(
    wThList: wThList, 
    imagePointsList: imagePointsList, 
    objectPoints: objectPoints, 
    cameraCalibration: cameraCalibration,
    hTeEstimate: Pose3(),
    wToEstimate: Pose3())

  // print("Factor graph, image point measurements")
  print("Estimated hand-to-eye: \(hTe_fgImagePoints)")
  print("Estimated world-to-object: \(wTo_fgImagePoints)")
  // printError(hTe_fgImagePoints)
  printErrorMagnitude(hTe_fgImagePoints)
  // print()

  print()
}

main()
