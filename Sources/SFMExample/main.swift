/// Adapted from GTSAM SFMExample.cpp

import HandEye
import SwiftFusion

/// Creates the set of ground truth landmarks.
func createPoints() -> [Vector3] {
  [
    Vector3( 10.0,  10.0,  10.0),
    Vector3(-10.0,  10.0,  10.0),
    Vector3(-10.0, -10.0,  10.0),
    Vector3( 10.0, -10.0,  10.0),
    Vector3( 10.0,  10.0, -10.0),
    Vector3(-10.0,  10.0, -10.0),
    Vector3(-10.0, -10.0, -10.0),
    Vector3( 10.0, -10.0, -10.0),
  ]
}

/// Creates the set of ground truth poses.
func createPoses(initial: Pose3? = nil, increment: Pose3? = nil, steps: Int = 8) -> [Pose3] {
  let initial = initial ?? Pose3(
    Rot3(
      0, 0, -1,
      1, 0, 0,
      0, -1, 0),
    Vector3(30.0, 0.0, 0.0))
    
  let increment = increment ?? Pose3(
    Rot3.fromAngleAxis(-.pi / 4, Vector3(0.0, 1.0, 0.0)),
    Vector3(30.0 * .sin(.pi / 4), 0, 30.0 * (1.0 - .sin(.pi / 4))))
  
  var poses = [initial]
  for i in 1..<steps {
    poses.append(poses[i - 1] * increment)
  }

  return poses
}

func main() {
  let K = Cal3_S2(fx: 50.0, fy: 50.0, s: 0.0, u0: 50.0, v0: 50.0)
  let points = createPoints()
  let poses = createPoses()

  var values = VariableAssignments()
  var xIDList: [TypedID<Pose3>] = []
  var lIDList: [TypedID<Vector3>] = []

  var graph = FactorGraph()

  // Intentionally initialize the variables off from the ground truth
  for pose in poses {
    let corruptedPose = pose * Pose3(
      Rot3.fromRvec(Vector3(-0.1, 0.2, 0.25)),
      Vector3(0.05, -0.10, 0.20))
    xIDList.append(values.store(corruptedPose))
  }
  
  for point in points {
    let corruptedPoint = point + Vector3(-0.25, 0.20, 0.15)
    lIDList.append(values.store(corruptedPoint))
  }

  // Simulated measurements from each camera pose
  for i in 0..<poses.count {
    let camera = PinholeCamera(K, poses[i])

    for j in 0..<points.count {
      let measurement = camera.project(points[j])
      graph.store(ProjectionFactor(xIDList[i], lIDList[j], measurement, K))
    }
  }

  // Add a prior on the first pose x0, indirectly specifying the origin
  graph.store(PriorFactor(xIDList[0], poses[0]))

  // Add a prior on the first landmark, fixing the scale
  graph.store(VectorPriorFactor(lIDList[0], points[0]))

  var optimizer = LM(precision: 1e-6, max_iteration: 200)
  try? optimizer.optimize(graph: graph, initial: &values)

  for xID in xIDList {
    print(values[xID])
  }

  for lID in lIDList {
    print(values[lID])
  }
}

main()
