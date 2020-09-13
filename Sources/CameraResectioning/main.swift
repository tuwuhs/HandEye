
import HandEye
import PenguinStructures
import SwiftFusion

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
  graph.store(CameraResectioningFactor(camPoseId, Vector3(10.0, 10.0, 0.0), Vector2(55, 45), K))
  graph.store(CameraResectioningFactor(camPoseId, Vector3(-10.0, 10.0, 0.0), Vector2(45, 45), K))
  graph.store(CameraResectioningFactor(camPoseId, Vector3(-10.0, -10.0, 0.0), Vector2(45, 55), K))
  graph.store(CameraResectioningFactor(camPoseId, Vector3(10.0, -10.0, 0.0), Vector2(55, 55), K))

  var optimizer = LM(precision: 1e-6, max_iteration: 500)
  try? optimizer.optimize(graph: graph, initial: &x)

  print(x[camPoseId])
}

main()
