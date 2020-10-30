
import PenguinStructures
import SwiftFusion
import TensorFlow

// MARK: - Tsai's method

/// Returns the estimated handToEye transformation
public func calibrateHandEye_tsai(worldToHand Hg: [Pose3], eyeToObject Hc: [Pose3]) -> Pose3 {
  // Solves AX = XB problem

  assert(Hg.count == Hc.count)

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

public struct FixedHandEyePoseFactor<Pose: LieGroup>: LinearizableFactor2 {
  public let edges: Variables.Indices
  public let eyeToObject: Pose
  public let worldToHand: Pose
  
  public init (_ handToEyeID: TypedID<Pose>, _ worldToObjectID: TypedID<Pose>, _ eyeToObject: Pose, _ worldToHand: Pose) {
    self.edges = Tuple2(handToEyeID, worldToObjectID)
    self.eyeToObject = eyeToObject
    self.worldToHand = worldToHand
  }

  @differentiable
  public func errorVector(_ handToEye: Pose, _ worldToObject: Pose) -> Pose.TangentVector {
    // Q: What is the difference? Which error vector is more correct?
    // return worldToHand.localCoordinate(worldToObject * eyeToObject.inverse() * handToEye.inverse())
    return eyeToObject.localCoordinate(handToEye.inverse() * worldToHand.inverse() * worldToObject)
  }
}

/// Returns the estimated handToEye and worldToObject transformations
public func calibrateHandEye_factorGraphPose(worldToHand: [Pose3], eyeToObject: [Pose3]) -> (Pose3, Pose3) {
  // Solves AX = ZB problem

  assert(worldToHand.count == eyeToObject.count)
  let nPoses = worldToHand.count

  var x = VariableAssignments()
  let handToEyeID = x.store(Pose3())
  let worldToObjectID = x.store(Pose3())

  var graph = FactorGraph()
  for i in 0..<nPoses {
    graph.store(FixedHandEyePoseFactor(handToEyeID, worldToObjectID, eyeToObject[i], worldToHand[i]))
  }

  // for _ in 0..<3 {
  //   let gfg = graph.linearized(at: x)
  //   var dx = x.tangentVectorZeros
  //   var opt = GenericCGLS(precision: 1e-6, max_iteration: 400)
  //   opt.optimize(gfg: gfg, initial: &dx)
  //   x.move(along: dx)
  // }

  var opt = LM(precision: 1e-6, max_iteration: 400)
  // opt.max_inner_iteration = 40
  try? opt.optimize(graph: graph, initial: &x)

  // Error vectors
  // print(cam2Gripper.localCoordinate(x[handToEyeID]))
  // print(target2Base.localCoordinate(x[worldToObjectID]))

  return (x[handToEyeID], x[worldToObjectID])
}

// MARK: - Factor graph, image point measurements

public struct HandEyePoseFactor<Pose: LieGroup>: LinearizableFactor3 {
  public let edges: Variables.Indices
  public let worldToHand: Pose
  
  public init (_ handToEyeID: TypedID<Pose>, _ worldToObjectID: TypedID<Pose>, _ eyeToObjectID: TypedID<Pose>, _ worldToHand: Pose) {
    self.edges = Tuple3(handToEyeID, worldToObjectID, eyeToObjectID)
    self.worldToHand = worldToHand
  }

  @differentiable
  public func errorVector(_ handToEye: Pose, _ worldToObject: Pose, _ eyeToObject: Pose) -> Pose.TangentVector {
    let error = worldToHand.localCoordinate(worldToObject * eyeToObject.inverse() * handToEye.inverse())
    // print("HandEyePoseFactor", error)
    return error
  }
}

/// Returns the estimated handToEye and worldToObject transformations
public func calibrateHandEye_factorGraphImagePoints(
  worldToHand: [Pose3], 
  imagePointsList: [[Vector2]], 
  objectPoints: [Vector3], 
  cameraCalibration: Cal3_S2,
  handToEyeEstimate: Pose3,
  worldToObjectEstimate: Pose3) 
  -> (Pose3, Pose3) {
  assert(worldToHand.count == imagePointsList.count)
  
  let nPoses = worldToHand.count

  var x = VariableAssignments()
  let handToEyeID = x.store(handToEyeEstimate)
  let worldToObjectID = x.store(worldToObjectEstimate)

  var graph = FactorGraph()
  var eyeToObjectIDList: [TypedID<Pose3>] = []
  let eyeToObjectEstimates = worldToHand.map { wTh -> Pose3 in
    handToEyeEstimate.inverse() * wTh.inverse() * worldToObjectEstimate
  }
  for i in 0..<nPoses {
    let imagePoints = imagePointsList[i]
    assert(imagePoints.count == objectPoints.count)

    let eyeToObjectID = x.store(eyeToObjectEstimates[i])
    // let eyeToObjectID = x.store(Pose3(
    //   Rot3(
    //     -1.0, 0.0, 0.0,
    //     0.0, 1.0, 0.0,
    //     0.0, 0.0, -1.0), 
    //   Vector3(-0.1, -0.1, 0.1)))
    eyeToObjectIDList.append(eyeToObjectID)

    graph.store(HandEyePoseFactor(handToEyeID, worldToObjectID, eyeToObjectID, worldToHand[i]))
    for j in 0..<imagePoints.count {
      graph.store(CameraResectioningFactor(eyeToObjectID, objectPoints[j], imagePoints[j], cameraCalibration))
    }
  }

  // var opt = LM(precision: 1e-6, max_iteration: 200)
  // // opt.verbosity = .TRYLAMBDA
  // opt.max_inner_iteration = 120
  // try? opt.optimize(graph: graph, initial: &x)

  for _ in 0..<120 {
    let gfg = graph.linearized(at: x)
    var dx = x.tangentVectorZeros
    var opt = GenericCGLS(precision: 0, max_iteration: 120)
    opt.optimize(gfg: gfg, initial: &dx)
    x.move(along: dx)
  }

  // Error vectors
  // print(cam2Gripper.localCoordinate(x[handToEyeID]))
  // print(target2Base.localCoordinate(x[worldToObjectID]))

  return (x[handToEyeID], x[worldToObjectID])
}

// MARK: - 

// Assumes wTo = I
public struct HandEyePoseNoObjectFactor<Pose: LieGroup>: LinearizableFactor2 {
  public let edges: Variables.Indices
  public let worldToHand: Pose
  
  public init (_ handToEyeID: TypedID<Pose>, _ eyeToObjectID: TypedID<Pose>, _ worldToHand: Pose) {
    self.edges = Tuple2(handToEyeID, eyeToObjectID)
    self.worldToHand = worldToHand
  }

  @differentiable
  public func errorVector(_ handToEye: Pose, _ eyeToObject: Pose) -> Pose.TangentVector {
    let error = worldToHand.localCoordinate(eyeToObject.inverse() * handToEye.inverse())
    // print("HandEyePoseFactor", error)
    return error
  }
}

/// Returns the estimated handToEye and worldToObject transformations
public func calibrateHandEye_factorGraphImagePointsNoObject(
  worldToHand: [Pose3], 
  imagePointsList: [[Vector2]], 
  objectPoints: [Vector3], 
  cameraCalibration: Cal3_S2,
  handToEyeEstimate: Pose3,
  worldToObjectEstimate: Pose3) 
  -> (Pose3, Pose3) {
  assert(worldToHand.count == imagePointsList.count)
  
  let nPoses = worldToHand.count

  var x = VariableAssignments()
  let handToEyeID = x.store(handToEyeEstimate)
  let worldToObjectID = x.store(worldToObjectEstimate)

  var graph = FactorGraph()
  var eyeToObjectIDList: [TypedID<Pose3>] = []
  let eyeToObjectEstimates = worldToHand.map { wTh -> Pose3 in
    handToEyeEstimate.inverse() * wTh.inverse() * worldToObjectEstimate
  }
  for i in 0..<nPoses {
    let imagePoints = imagePointsList[i]
    assert(imagePoints.count == objectPoints.count)

    let eyeToObjectID = x.store(eyeToObjectEstimates[i])
    // let eyeToObjectID = x.store(Pose3(
    //   Rot3(
    //     -1.0, 0.0, 0.0,
    //     0.0, 1.0, 0.0,
    //     0.0, 0.0, -1.0), 
    //   Vector3(-0.1, -0.1, 0.1)))
    eyeToObjectIDList.append(eyeToObjectID)

    graph.store(HandEyePoseNoObjectFactor(handToEyeID, eyeToObjectID, worldToHand[i]))
    for j in 0..<imagePoints.count {
      graph.store(CameraResectioningFactor(eyeToObjectID, objectPoints[j], imagePoints[j], cameraCalibration))
    }
  }

  // var opt = LM(precision: 1e-6, max_iteration: 100)
  // opt.verbosity = .TRYLAMBDA
  // opt.max_inner_iteration = 120
  // try? opt.optimize(graph: graph, initial: &x)

  for _ in 0..<120 {
    let gfg = graph.linearized(at: x)
    var dx = x.tangentVectorZeros
    var opt = GenericCGLS(precision: 0 /*1e-16*/, max_iteration: 120)
    opt.optimize(gfg: gfg, initial: &dx)
    x.move(along: dx)
  }

  // Error vectors
  // print(cam2Gripper.localCoordinate(x[handToEyeID]))
  // print(target2Base.localCoordinate(x[worldToObjectID]))

  return (x[handToEyeID], x[worldToObjectID])
}