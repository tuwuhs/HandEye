
import ArgumentParser
import Dispatch
import Foundation
import HandEye
import SwiftFusion
import Yams

struct Options: ParsableArguments {
  @Option(name: .shortAndLong, help: "Save generated dataset into filename (YAML)")
  var saveFilename: String?

  @Option(name: .shortAndLong, help: "Load dataset from filename (YAML), do not generate dataset")
  var loadFilename: String?
}

func generateDataset() -> (String, [Pose3], [Pose3], Pose3, Pose3) {
  // let (wThList, eToList, hTe, wTo) = simulatePoseEyeInHand(nPoses: 10, addNoise: true)
  
  var (wThList, eToList, hTe, wTo) = simulatePoseKoide(eTh: Pose3(
    Rot3.fromRvec(Vector3(0.1, -0.1, 0.1)),
    Vector3(0.1, -0.1, 0.05)))

  // Project points
  let objectPoints = createTargetObject(rows: 7, cols: 5, dimension: 0.15)
  let cameraCalibration = Cal3_S2(fx: 300.0, fy: 300.0, s: 0.0, u0: 320.0, v0: 240.0)
  let imagePointsList = projectPoints(eToList: eToList, objectPoints: objectPoints, calibration: cameraCalibration)
  assert(imagePointsList.allSatisfy { $0.allSatisfy { $0.x >= 0 && $0.x < 640 && $0.y >= 0 && $0.y < 480 } },
    "Some image points fall outside the image boundary")

  // Add pose noise
  wThList = applyNoise(wThList, 0.01, 0.1)

  var root: Yams.Node = [:]
  root["object_points"] = objectPoints.toYaml()
  root["camera_calibration"] = cameraCalibration.toYaml()

  var views: Yams.Node.Sequence = []
  for i in 0..<wThList.count {
    var viewMap: Yams.Node = [:]
    let imagePoints = imagePointsList[i]
    let wTh = wThList[i]

    viewMap["image_points"] = imagePoints.toYaml()
    viewMap["wTh"] = wTh.toYaml()
    views.append(viewMap)
  }
  root["views"] = Yams.Node.sequence(views)

  let yaml = try! Yams.serialize(node: root)

  return (yaml, wThList, eToList, hTe, wTo)
}

func readDataset(_ yaml: String) -> ([Vector3], [[Vector2]], [Pose3], [Pose3], Cal3_S2) {
  let root = try! Yams.compose(yaml: yaml)!

  var wThList: [Pose3] = []
  var eToList: [Pose3] = []
  var imagePointsList: [[Vector2]] = []
  for view in (root["views"]?.sequence)! {
    wThList.append(.fromYaml(view["wTh"]!))
    // TODO: Handle when no eTo in YAML!
    eToList.append(.fromYaml(view["eTo"]!))
    imagePointsList.append(.fromYaml(view["image_points"]!))
  }
  
  let objectPoints: [Vector3] = .fromYaml(root["object_points"]!)
  let cameraCalibration: Cal3_S2 = .fromYaml(root["camera_calibration"]!)

  return (objectPoints, imagePointsList, wThList, eToList, cameraCalibration)
}

func main() {
  let options = Options.parseOrExit()

  var eToList: [Pose3]?
  var hTe: Pose3?
  var wTo: Pose3?
  var yaml: String = ""
  if let loadFilename = options.loadFilename {
    let path = URL(fileURLWithPath: loadFilename)
    yaml = String(data: try! Data(contentsOf: path), encoding: .utf8)!
    print("Loaded dataset from \(loadFilename).")
  } else {
    (yaml, _, eToList, hTe, wTo) = generateDataset()
    if let saveFilename = options.saveFilename {
      let path = URL(fileURLWithPath: saveFilename)
      try! yaml.write(to: path, atomically: true, encoding: .utf8)
      print("Saved dataset to \(saveFilename).")
    }
  }

  let (objectPoints, imagePointsList, wThList, eToList_, cameraCalibration) = readDataset(yaml)
  if eToList == nil {
    eToList = eToList_
  }
  
  let printError = { (handToEye: Pose3) in 
    print("Errors:")
    if let hTe = hTe {
      print("rvec: \(handToEye.rot.toRvec() - hTe.rot.toRvec())")
      print("tvec: \(handToEye.t - hTe.t)")
    }
  }

  let printErrorMagnitude = { (handToEye: Pose3) in 
    if let hTe = hTe {
      print("rvec: \((handToEye.rot.toRvec() - hTe.rot.toRvec()).norm)")
      print("tvec: \((handToEye.t - hTe.t).norm)")
    }
  }

  // Try camera resectioning
  var eToList_estimate: [Pose3] = []
  for i in 0..<wThList.count {
    let imagePoints = imagePointsList[i]

    let eTo_estimate = performCameraResectioning(
      imagePoints: imagePoints, objectPoints: objectPoints, calibration: cameraCalibration)
    eToList_estimate.append(eTo_estimate)

    print(eTo_estimate)
    if let eToList = eToList {
      let eTo = eToList[i]
      print(eTo)
    }
    print()
    // break
  }
  
  print("Actual hand-to-eye: \(hTe)")
  print("Actual world-to-object: \(wTo)")
  print()

  let hTe_tsai = calibrateHandEye_tsai(wThList: wThList, eToList: eToList_estimate)
  let wTo_tsai = wThList[0] * hTe_tsai * eToList_estimate[0]
  print("Tsai's method")
  print("Estimated hand-to-eye: \(hTe_tsai)")
  print("Estimated world-to-object: \(wTo_tsai)")
  print("Estimated object-to-world: \(wTo_tsai.inverse())")
  // printError(hTe_tsai)
  printErrorMagnitude(hTe_tsai)
  print()

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

  print("Factor graph, image point measurements")
  print("Estimated hand-to-eye: \(hTe_fgImagePoints)")
  print("Estimated world-to-object: \(wTo_fgImagePoints)")
  print("Estimated object-to-world: \(wTo_fgImagePoints.inverse())")
  // printError(hTe_fgImagePoints)
  printErrorMagnitude(hTe_fgImagePoints)
  print()

  print()
}

main()
