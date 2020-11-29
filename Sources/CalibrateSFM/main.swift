
import ArgumentParser
import Dispatch
import Foundation
import HandEye
import SwiftFusion

func generateDataset() -> ([Pose3], [Pose3], Pose3, Pose3, [Vector3], [[Vector2]], Cal3_S2) {
  // let (wThList, eToList, hTe, wTo) = simulatePoseEyeInHand(nPoses: 10, addNoise: true)
  
  var (wThList, eToList, hTe, wTo) = simulatePoseKoide(eTh: Pose3(
    Rot3(), // Rot3.fromRvec(Vector3(0.1, -0.1, 0.1)),
    Vector3(0.1, -0.1, 0.05)))

  // Project points
  let objectPoints = createTargetObject(rows: 3, cols: 3, dimension: 0.25)
  let cameraCalibration = Cal3_S2(fx: 300.0, fy: 300.0, s: 0.0, u0: 320.0, v0: 240.0)
  let imagePointsList = projectPoints(eToList: eToList, objectPoints: objectPoints, calibration: cameraCalibration)
  assert(imagePointsList.allSatisfy { $0.allSatisfy { $0.x >= 0 && $0.x < 640 && $0.y >= 0 && $0.y < 480 } },
    "Some image points fall outside the image boundary")

  // Add pose noise
  // wThList = applyNoise(wThList, 0.01, 0.1)

  return (wThList, eToList, hTe, wTo, objectPoints, imagePointsList, cameraCalibration)
}

func main() {
  let (wThList, eToList, hTe, wTo, objectPoints, imagePointsList, cameraCalibration) = generateDataset()

  let printError = { (handToEye: Pose3) in 
    print("Errors:")
    print("rvec: \(handToEye.rot.toRvec() - hTe.rot.toRvec())")
    print("tvec: \(handToEye.t - hTe.t)")
  }

  let printErrorMagnitude = { (handToEye: Pose3) in 
    print("rvec: \((handToEye.rot.toRvec() - hTe.rot.toRvec()).norm)")
    print("tvec: \((handToEye.t - hTe.t).norm)")
  }

  // SFM optimization on the image points
  let (poses, landmarks) = performSFMOptimization(
    imagePointsList: imagePointsList,
    objectPointsEstimate: objectPoints.map { $0 + Vector3(0.01, 0, 0) },
    posesEstimate: eToList.map { $0.inverse() * Pose3(
      Rot3.fromRvec(Vector3(0, 0, 0)),
      Vector3(0, 0, 0))
    },
    cameraCalibration: cameraCalibration)
  for pose in poses {
    print(pose)
  }
  for landmark in landmarks {
    print(landmark)
  }
  print()
  
  print("Actual hand-to-eye: \(hTe)")
  print("Actual world-to-object: \(wTo)")
  print()

  let hTe_tsai = calibrateHandEye_tsai(wThList: wThList, eToList: eToList)
  let wTo_tsai = wThList[0] * hTe_tsai * eToList[0]
  print("Tsai's method")
  print("Estimated hand-to-eye: \(hTe_tsai)")
  print("Estimated world-to-object: \(wTo_tsai)")
  print("Estimated object-to-world: \(wTo_tsai.inverse())")
  // printError(hTe_tsai)
  printErrorMagnitude(hTe_tsai)
  print()

  // let (hTe_fgImagePoints, wTo_fgImagePoints) = calibrateHandEye_factorGraphImagePoints(
  //   wThList: wThList, 
  //   imagePointsList: imagePointsList, 
  //   objectPoints: objectPoints, 
  //   cameraCalibration: cameraCalibration,
  //   hTeEstimate: Pose3(),
  //   wToEstimate: Pose3())

  // print("Factor graph, image point measurements")
  // print("Estimated hand-to-eye: \(hTe_fgImagePoints)")
  // print("Estimated world-to-object: \(wTo_fgImagePoints)")
  // print("Estimated object-to-world: \(wTo_fgImagePoints.inverse())")
  // // printError(hTe_fgImagePoints)
  // printErrorMagnitude(hTe_fgImagePoints)
  // print()

  let (hTe_SFM, wTo_SFM) = calibrateHandEye_SFM(
    wThList: wThList, 
    imagePointsList: imagePointsList, 
    objectPointsEstimate: objectPoints.map { applyNoise($0, 0.2) },
    oTeListEstimate: eToList.map { applyNoise($0.inverse(), 0.4, 0.2) },
    cameraCalibration: cameraCalibration)

  print("Factor graph, SFM")
  print("Estimated hand-to-eye: \(hTe_SFM)")
  print("Estimated world-to-object: \(wTo_SFM)")
  print("Estimated object-to-world: \(wTo_SFM.inverse())")
  // printError(hTe_SFM)
  printErrorMagnitude(hTe_SFM)
  print()

  print()
}

main()
