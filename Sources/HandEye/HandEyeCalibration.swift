
import PenguinStructures
import SwiftFusion
import TensorFlow

// MARK: - Tsai's method

/// Returns the estimated handToEye transformation
public func calibrateHandEye_tsai(worldToHand Hg: [Pose3], eyeToObject Hc: [Pose3]) -> Pose3 {
  // Solves AX = XB problem

  let K = Hg.count * (Hg.count - 1) / 2
  var A = Tensor(repeating: 0, shape: TensorShape(3*K, 3))
  var b = Tensor(repeating: 0, shape: TensorShape(3*K, 1))

  var idx = 0
  for i in 0 ..< Hg.count {
    for j in i+1 ..< Hg.count {
      let Hgij = Hg[j].inverse() * Hg[i]
      let Pgij = 2 * Hgij.rot.toQuatMinimal()

      let Hcij = Hc[j] * Hc[i].inverse()
      let Pcij = 2 * Hcij.rot.toQuatMinimal()

      let lhs = skew3(Pgij + Pcij)
      A[idx * 3 + 0] = Tensor([lhs[0, 0], lhs[0, 1], lhs[0, 2]])
      A[idx * 3 + 1] = Tensor([lhs[1, 0], lhs[1, 1], lhs[1, 2]])
      A[idx * 3 + 2] = Tensor([lhs[2, 0], lhs[2, 1], lhs[2, 2]])

      let rhs = Pcij - Pgij
      b[idx * 3 + 0] = Tensor([rhs.x])
      b[idx * 3 + 1] = Tensor([rhs.y])
      b[idx * 3 + 2] = Tensor([rhs.z])

      idx += 1
    }
  }

  let Pcg_ = solveWithSVD(A, b)
  let Pcg_norm = _Raw.matMul(Pcg_.transposed(), Pcg_)
  let Pcg = 2 * Pcg_ / .sqrt(1.0 + Pcg_norm[0, 0])

  let Rcg = Rot3.fromQuatMinimal(0.5 * Vector3.fromTensor(Pcg))

  idx = 0
  for i in 0 ..< Hg.count {
    for j in i+1 ..< Hg.count {
      let Hgij = Hg[j].inverse() * Hg[i]
      let Hcij = Hc[j] * Hc[i].inverse()

      let lhs = Hgij.rot.toTensor() - eye(rowCount: 3)
      A[idx * 3 + 0] = lhs[0]
      A[idx * 3 + 1] = lhs[1]
      A[idx * 3 + 2] = lhs[2]

      let rhs = _Raw.matMul(Rcg.toTensor(), Hcij.t.toTensor()) - Hgij.t.toTensor()
      b[idx * 3 + 0] = rhs[0]
      b[idx * 3 + 1] = rhs[1]
      b[idx * 3 + 2] = rhs[2]

      idx += 1
    }
  }

  let tcg = solveWithSVD(A, b)

  return Pose3(Rcg, Vector3.fromTensor(tcg))
}

// MARK: - Factor graph, pose measurements

public struct HandEyeMeasurementFactor<Pose: LieGroup>: LinearizableFactor2 {
  public let edges: Variables.Indices
  public let eye2Object: Pose
  public let world2Hand: Pose
  
  public init (_ hand2EyeID: TypedID<Pose>, _ world2ObjectID: TypedID<Pose>, _ eye2Object: Pose, _ world2Hand: Pose) {
    self.edges = Tuple2(hand2EyeID, world2ObjectID)
    self.eye2Object = eye2Object
    self.world2Hand = world2Hand
  }

  @differentiable
  public func errorVector(_ hand2Eye: Pose, _ world2Object: Pose) -> Pose.TangentVector {
    // Q: What is the difference? Which error vector is more correct?
    // return world2Hand.localCoordinate(world2Object * eye2Object.inverse() * hand2Eye.inverse())
    return eye2Object.localCoordinate(hand2Eye.inverse() * world2Hand.inverse() * world2Object)
  }
}

/// Returns the estimated handToEye transformation
public func calibrateHandEye_factorGraphPose(worldToHand: [Pose3], eyeToObject: [Pose3]) -> (Pose3, Pose3) {
  // Solves AX = ZB problem

  let nPoses = worldToHand.count

  var x = VariableAssignments()
  let handToEyeID = x.store(Pose3())
  let worldToObjectID = x.store(Pose3())

  var graph = FactorGraph()
  for i in 0..<nPoses {
    graph.store(HandEyeMeasurementFactor(handToEyeID, worldToObjectID, eyeToObject[i], worldToHand[i]))
  }

  // for _ in 0..<3 {
  //   let gfg = graph.linearized(at: x)
  //   var dx = x.tangentVectorZeros
  //   var opt = GenericCGLS(precision: 1e-6, max_iteration: 400)
  //   opt.optimize(gfg: gfg, initial: &dx)
  //   x.move(along: dx)
  // }

  var opt = LM(precision: 1e-6, max_iteration: 500)
  try? opt.optimize(graph: graph, initial: &x)

  // Error vectors
  // print(cam2Gripper.localCoordinate(x[handToEyeID]))
  // print(target2Base.localCoordinate(x[worldToObjectID]))

  return (x[handToEyeID], x[worldToObjectID])
}
