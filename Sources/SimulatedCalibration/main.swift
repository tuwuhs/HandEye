
import Dispatch
import HandEye
import SwiftFusion
import Yams

extension Vector3 {
  func toYaml() -> Yams.Node {
    try! Yams.Node.sequence(
      Yams.Node.Sequence([x.represented(), y.represented(), z.represented()], .implicit, .flow))
  }

  static func fromYaml(_ node: Yams.Node) -> Self {
    let seq = node.sequence!
    return Vector3((seq[0].float)!, (seq[1].float)!, (seq[2].float)!)
  }
}

extension Vector2 {
  func toYaml() -> Yams.Node {
    try! Yams.Node.sequence(
      Yams.Node.Sequence([x.represented(), y.represented()], .implicit, .flow))
  }

  static func fromYaml(_ node: Yams.Node) -> Self {
    let seq = node.sequence!
    return Vector2((seq[0].float)!, (seq[1].float)!)
  }
}

extension Array where Element == Vector3 {
  func toYaml() -> Yams.Node {
    var seq: Yams.Node.Sequence = []
    for v in self {
      seq.append(v.toYaml())
    }
    return Yams.Node.sequence(seq)
  }

  static func fromYaml(_ node: Yams.Node) -> Self {
    node.sequence!.map { .fromYaml($0) }
  }
}

extension Array where Element == Vector2 {
  func toYaml() -> Yams.Node {
    var seq: Yams.Node.Sequence = []
    for v in self {
      seq.append(v.toYaml())
    }
    return Yams.Node.sequence(seq)
  }

  static func fromYaml(_ node: Yams.Node) -> Self {
    node.sequence!.map { .fromYaml($0) }
  }
}

extension Pose3 {
  func toYaml() -> Yams.Node {
    let rvec = self.rot.toRvec()
    let tvec = self.t
    let node: Yams.Node = [
      "rvec": rvec.toYaml(),
      "tvec": tvec.toYaml()]
    return node
  }

  static func fromYaml(_ node: Yams.Node) -> Self {
    let rvec: Vector3 = .fromYaml(node["rvec"]!)
    let tvec: Vector3 = .fromYaml(node["tvec"]!)
    return Pose3(Rot3.fromRvec(rvec), tvec)
  }
}

extension Cal3_S2 {
  func toYaml() -> Yams.Node {
    try! Yams.Node.mapping([
      "fx": coordinate.fx.represented(),
      "fy": coordinate.fy.represented(),
      "s": coordinate.s.represented(),
      "u0": coordinate.u0.represented(),
      "v0": coordinate.v0.represented(),
    ])
  }

  static func fromYaml(_ node: Yams.Node) -> Self {
    Cal3_S2(
      fx: (node["fx"]!.float)!,
      fy: (node["fy"]!.float)!,
      s: (node["s"]!.float)!,
      u0: (node["u0"]!.float)!,
      v0: (node["v0"]!.float)!
    )
  }
}

func generateDataset(_ wThList: [Pose3], _ eToList: [Pose3], _ hTe: Pose3, _ wTo: Pose3) -> String {
  // Project points
  let objectPoints = createTargetObject(rows: 3, cols: 3, dimension: 0.25)
  let cameraCalibration = Cal3_S2(fx: 300.0, fy: 300.0, s: 0.0, u0: 320.0, v0: 240.0)
  let imagePointsList = projectPoints(eToList: eToList, objectPoints: objectPoints, calibration: cameraCalibration)
  assert(imagePointsList.allSatisfy { $0.allSatisfy { $0.x >= 0 && $0.x < 640 && $0.y >= 0 && $0.y < 480 } },
    "Some image points fall outside the image boundary")

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
  print(yaml)

  return yaml
}

func processDataset(_ yaml: String) -> (Pose3, Pose3) {
  let root = try! Yams.compose(yaml: yaml)!

  var wThList: [Pose3] = []
  var imagePointsList: [[Vector2]] = []
  for view in (root["views"]?.sequence)! {
    wThList.append(.fromYaml(view["wTh"]!))
    imagePointsList.append(.fromYaml(view["image_points"]!))
  }
  
  let objectPoints: [Vector3] = .fromYaml(root["object_points"]!)
  let cameraCalibration: Cal3_S2 = .fromYaml(root["camera_calibration"]!)

  // Try camera resectioning
  for i in 0..<wThList.count {
    let imagePoints = imagePointsList[i]
    let wTh = wThList[i]

    let oTe_estimate = performCameraResectioning(
      imagePoints: imagePoints, objectPoints: objectPoints, calibration: cameraCalibration)
    
    print(oTe_estimate)
    print()

    // break
  }

  let (hTe_fgImagePoints, wTo_fgImagePoints) = calibrateHandEye_factorGraphImagePoints(
    wThList: wThList, 
    imagePointsList: imagePointsList, 
    objectPoints: objectPoints, 
    cameraCalibration: cameraCalibration,
    hTeEstimate: Pose3(),
    wToEstimate: Pose3())
  
  return (hTe_fgImagePoints, wTo_fgImagePoints)
}

func main() {
  // let (wThList, eToList, hTe, wTo) = simulatePoseEyeInHand(nPoses: 10, addNoise: true)
  
  var (wThList, eToList, hTe, wTo) = simulatePoseKoide(eTh: Pose3(
    Rot3(),
    Vector3(0.1, -0.1, 0.05)))
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

  // Add pose noise
  // wThList = applyNoise(wThList, 0.01, 0.1)

  let yaml = generateDataset(wThList, eToList, hTe, wTo)

  
  let (hTe_fgImagePoints, wTo_fgImagePoints) = processDataset(yaml)

  // Try camera resectioning
  // for i in 0..<wThList.count {
  //   let imagePoints = imagePointsList[i]
  //   let wTh = wThList[i]

  //   let oTe_estimate = performCameraResectioning(
  //     imagePoints: imagePoints, objectPoints: objectPoints, calibration: cameraCalibration)
    
  //   print(oTe_estimate)
  //   print((wTo.inverse() * wTh * hTe).inverse())
  //   print()

  //   // break
  // }

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

  // let (hTe_fgImagePoints, wTo_fgImagePoints) = calibrateHandEye_factorGraphImagePoints(
  //   wThList: wThList, 
  //   imagePointsList: imagePointsList, 
  //   objectPoints: objectPoints, 
  //   cameraCalibration: cameraCalibration,
  //   hTeEstimate: Pose3(),
  //   wToEstimate: Pose3())

  // print("Factor graph, image point measurements")
  print("Estimated hand-to-eye: \(hTe_fgImagePoints)")
  print("Estimated world-to-object: \(wTo_fgImagePoints)")
  // printError(hTe_fgImagePoints)
  printErrorMagnitude(hTe_fgImagePoints)
  // print()

  print()
}

main()

// let p = "abc"
// var map: Yams.Node = [
//   "hello": [try p.represented(), 2, 3],
//   "yeah": [
//     "hello": [1, 2, 3],
//     "ahyeah": ["a", "b"]]
// ]
// map["ah"] = ["abc": 259]

// var seq: Yams.Node = [1, 3, 5]
// map["oh"] = seq

// map["pose"] = Pose3().toYaml()

// let yaml = try Yams.serialize(node: map)
// print(yaml)

// print((map["hello"]?[0]?.any)!)
// print(type(of: (map["hello"]?[0]?.any)!))

