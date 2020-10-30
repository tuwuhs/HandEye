
import HandEye
import PenguinStructures
import SwiftFusion

func main() {
  let K = Cal3_S2(fx: 1.0, fy: 1.0, s: 0.0, u0: 50.0, v0: 50.0)
  let camera = PinholeCamera(Pose3(
    Rot3(
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0), 
    Vector3(0.0, 0.0, 2.0)), K)
  // let (imagePoint, projectPullback) = valueWithPullback(at: Vector3(10.0, 10.0, 0.0), in: camera.project)
  let imagePoint = camera.project(Vector3(-10.0, 10.0, 0.0))
  let projectPullback = pullback(at: camera, Vector3(-10.0, 10.0, 0.0), in: { (c, p) in c.project(p) })
  print(imagePoint)
  print(projectPullback(Vector2(1.0, 0.0)))
  print(projectPullback(Vector2(0.0, 1.0)))

  // let (p, pb) = valueWithPullback(at: Vector2(5.0, -5.0), in: K.uncalibrate)
  // print(p)
  // print(pb(Vector2(1.0, 0.0)))

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

  for _ in 0..<5 {
    let gfg = graph.linearized(at: x)
    var dx = x.tangentVectorZeros
    var opt = GenericCGLS(precision: 0, max_iteration: 120)
    opt.optimize(gfg: gfg, initial: &dx)
    x.move(along: dx)
  }

  // var optimizer = LM(precision: 1e-6, max_iteration: 500)
  // try? optimizer.optimize(graph: graph, initial: &x)

  print(x[camPoseId])
}

main()
