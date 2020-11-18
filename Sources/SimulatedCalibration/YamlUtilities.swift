
import HandEye
import SwiftFusion
import Yams

public extension Vector3 {
  func toYaml() -> Yams.Node {
    try! Yams.Node.sequence(
      Yams.Node.Sequence([x.represented(), y.represented(), z.represented()], .implicit, .flow))
  }

  static func fromYaml(_ node: Yams.Node) -> Self {
    let seq = node.sequence!
    return Vector3((seq[0].float)!, (seq[1].float)!, (seq[2].float)!)
  }
}

public extension Vector2 {
  func toYaml() -> Yams.Node {
    try! Yams.Node.sequence(
      Yams.Node.Sequence([x.represented(), y.represented()], .implicit, .flow))
  }

  static func fromYaml(_ node: Yams.Node) -> Self {
    let seq = node.sequence!
    return Vector2((seq[0].float)!, (seq[1].float)!)
  }
}

public extension Array where Element == Vector3 {
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

public extension Array where Element == Vector2 {
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

public extension Pose3 {
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

public extension Cal3_S2 {
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
