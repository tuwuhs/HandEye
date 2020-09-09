
import HandEye
import PenguinStructures
import SwiftFusion

struct ResectioningFactor: LinearizableFactor1 {
  public let edges: Variables.Indices

  /// Camera calibration
  public let K: CameraCalibration

  /// 3D point on the target
  public let p: Vector3

  /// 2D point in the image
  public let q: Vector2

  public init(_ poseId: TypedID<Pose3>, _ p: Vector3, _ q: Vector2, _ K: CameraCalibration) {
    self.edges = Tuple1(poseId)
    self.p = p
    self.q = q
    self.K = K
  }

  @differentiable
  public func errorVector(_ pose: Pose3) -> Vector2 {
    let camera = PinholeCamera(pose, K)
    let reprojectionError = camera.project(p) - q
    return reprojectionError
  }
}

func main() {
  let K = CameraCalibration(fx: 1.0, fy: 1.0, s: 0.0, u0: 50.0, v0: 50.0)

  var x = VariableAssignments()
  let camPoseId = x.store(Pose3(
    Rot3(
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0), 
    Vector3(1.0, 1.0, 1.0)))

  var graph = FactorGraph()
  graph.store(ResectioningFactor(camPoseId, Vector3(10.0, 10.0, 0.0), Vector2(55, 45), K))
  graph.store(ResectioningFactor(camPoseId, Vector3(-10.0, 10.0, 0.0), Vector2(45, 45), K))
  graph.store(ResectioningFactor(camPoseId, Vector3(-10.0, -10.0, 0.0), Vector2(45, 55), K))
  graph.store(ResectioningFactor(camPoseId, Vector3(10.0, -10.0, 0.0), Vector2(55, 55), K))

  var optimizer = LM(precision: 1e-6, max_iteration: 500)
  try? optimizer.optimize(graph: graph, initial: &x)

  print(x[camPoseId])
}

main()
