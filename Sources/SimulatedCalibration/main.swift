
import Dispatch
import HandEye
import SwiftFusion
import Yams

extension Vector3 {
  func toYaml() -> Yams.Node {
    Yams.Node.sequence(
      Yams.Node.Sequence([try! x.represented(), try! y.represented(), try! z.represented()], .implicit, .flow))
  }

  static func fromYaml(_ node: Yams.Node) -> Self {
    let seq = node.sequence!
    return Vector3((seq[0].float)!, (seq[1].float)!, (seq[2].float)!)
  }
}

extension Vector2 {
  func toYaml() -> Yams.Node {
    Yams.Node.sequence(
      Yams.Node.Sequence([try! x.represented(), try! y.represented()], .implicit, .flow))
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
    let R = self.rot.coordinate.R
    let t = self.t

    var node: Yams.Node.Mapping = [:]
    node.tag = Yams.Tag("tag:yaml.org,2002:opencv-matrix")
    node["rows"] = 4
    node["cols"] = 4
    node["dt"] = "d"
    node["data"] = Yams.Node.sequence(Yams.Node.Sequence([
      R[0,0], R[0,1], R[0,2], t.x,
      R[1,0], R[1,1], R[1,2], t.y,
      R[2,0], R[2,1], R[2,2], t.z,
           0,      0,      0,   1].map { try! $0.represented() }, .implicit, .flow))
    return Yams.Node.mapping(node)
  }
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

  // Project points
  let objectPoints = createTargetObject(rows: 3, cols: 3, dimension: 0.25)
  let cameraCalibration = Cal3_S2(fx: 300.0, fy: 300.0, s: 0.0, u0: 320.0, v0: 240.0)
  let imagePointsList = projectPoints(eToList: eToList, objectPoints: objectPoints, calibration: cameraCalibration)
  assert(imagePointsList.allSatisfy { $0.allSatisfy { $0.x >= 0 && $0.x < 640 && $0.y >= 0 && $0.y < 480 } },
    "Some image points fall outside the image boundary")

  var root: Yams.Node = [:]
  root["object_points"] = objectPoints.toYaml()

  var views: Yams.Node.Sequence = []
  for i in 0..<wThList.count {
    var viewMap: Yams.Node = [:]
    let imagePoints = imagePointsList[i]
    viewMap["image_points"] = imagePoints.toYaml()
    views.append(viewMap)
  }
  root["views"] = Yams.Node.sequence(views)

  let yaml = try! Yams.serialize(node: root)
  print(yaml)

  root = try! Yams.compose(yaml: yaml)!
  let yeah: [Vector3] = .fromYaml(root["object_points"]!)
  print(yeah)

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

  // let (hTe_fgImagePoints, wTo_fgImagePoints) = calibrateHandEye_factorGraphImagePoints(
  //   wThList: wThList, 
  //   imagePointsList: imagePointsList, 
  //   objectPoints: objectPoints, 
  //   cameraCalibration: cameraCalibration,
  //   hTeEstimate: Pose3(),
  //   wToEstimate: Pose3())

  // // print("Factor graph, image point measurements")
  // print("Estimated hand-to-eye: \(hTe_fgImagePoints)")
  // print("Estimated world-to-object: \(wTo_fgImagePoints)")
  // // printError(hTe_fgImagePoints)
  // printErrorMagnitude(hTe_fgImagePoints)
  // // print()

  print()
}

main()

let p = "abc"
var map: Yams.Node = [
  "hello": [try p.represented(), 2, 3],
  "yeah": [
    "hello": [1, 2, 3],
    "ahyeah": ["a", "b"]]
]
map["ah"] = ["abc": 259]

var seq: Yams.Node = [1, 3, 5]
map["oh"] = seq

map["pose"] = Pose3().toYaml()

let yaml = try Yams.serialize(node: map)
print(yaml)

print((map["hello"]?[0]?.any)!)
print(type(of: (map["hello"]?[0]?.any)!))

