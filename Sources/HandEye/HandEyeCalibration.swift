
import PenguinStructures
import SwiftFusion
import TensorFlow

// MARK: - Camera resectioning

/// Returns the camera pose oTe
public func performCameraResectioning<Calibration: CameraCalibration>(
    imagePoints: [Vector2], objectPoints: [Vector3], calibration: Calibration) 
    -> Pose3 {
  var x = VariableAssignments()
  let camPoseId = x.store(Pose3(
    Rot3(
      -1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, -1.0), 
    Vector3(-0.1, -0.1, 0.1)))
  // let camPoseId = x.store(Pose3(
  //   Rot3(
  //     1.0, 0.0, 0.0,
  //     0.0, -1.0, 0.0,
  //     0.0, 0.0, -1.0), 
  //   Vector3(-0.1, -0.1, 0.1)))

  var graph = FactorGraph()

  for j in 0..<imagePoints.count {
    graph.store(ResectioningFactor(camPoseId, objectPoints[j], imagePoints[j], calibration))
  }

  // var timestamps: [DispatchTime] = []
  // let start = DispatchTime.now()
  // for _ in 0..<10 {
  //   timestamps.append(DispatchTime.now())
  //   let linearized = graph.linearized(at: x)
  //   timestamps.append(DispatchTime.now())
  //   var dx = x.tangentVectorZeros
  //   timestamps.append(DispatchTime.now())
  //   var optimizer = GenericCGLS(precision: 0, max_iteration: 6)
  //   timestamps.append(DispatchTime.now())
  //   optimizer.optimize(gfg: linearized, initial: &dx)
  //   timestamps.append(DispatchTime.now())
  //   x.move(along: dx)
  // }

  // for (ts1, ts2) in zip(timestamps[0..<timestamps.count-1], timestamps[1...]) {
  //   let timeInterval = ts2.uptimeNanoseconds - ts1.uptimeNanoseconds
  //   print(timeInterval)
  // }

  var optimizer = LM(precision: 1e-5, max_iteration: 100)
  try? optimizer.optimize(graph: graph, initial: &x)

  // print(x[camPoseId])

  return x[camPoseId]
}

// MARK: - Tsai's method

/// Returns the estimated hTe transformation
public func calibrateHandEye_tsai(wThList Hg: [Pose3], eToList Hc: [Pose3]) -> Pose3 {
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
  public let oTe: Pose
  public let wTh: Pose
  
  public init (_ hTeID: TypedID<Pose>, _ wToID: TypedID<Pose>, _ oTe: Pose, _ wTh: Pose) {
    self.edges = Tuple2(hTeID, wToID)
    self.oTe = oTe
    self.wTh = wTh
  }

  @differentiable
  public func errorVector(_ hTe: Pose, _ wTo: Pose) -> Pose.TangentVector {
    // Q: What is the difference? Which error vector is more correct?
    return wTh.localCoordinate(wTo * oTe * hTe.inverse())
    // return oTe.localCoordinate(wTo.inverse() * wTh * hTe)
  }
}

/// Returns the estimated hTe and wTo transformations
public func calibrateHandEye_factorGraphPose(wThList: [Pose3], eToList: [Pose3]) -> (Pose3, Pose3) {
  // Solves AX = ZB problem

  assert(wThList.count == eToList.count)
  let nPoses = wThList.count

  var x = VariableAssignments()
  let hTeID = x.store(Pose3())
  let wToID = x.store(Pose3())

  var graph = FactorGraph()
  for i in 0..<nPoses {
    graph.store(FixedHandEyePoseFactor(hTeID, wToID, eToList[i], wThList[i]))
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
  // print(cam2Gripper.localCoordinate(x[hTeID]))
  // print(target2Base.localCoordinate(x[wToID]))

  return (x[hTeID], x[wToID])
}

// MARK: - Factor graph, image point measurements

public struct HandEyePoseFactor<Pose: LieGroup>: LinearizableFactor3 {
  public let edges: Variables.Indices
  public let wTh: Pose
  
  public init (_ hTeID: TypedID<Pose>, _ wToID: TypedID<Pose>, _ oTeID: TypedID<Pose>, _ wTh: Pose) {
    self.edges = Tuple3(hTeID, wToID, oTeID)
    self.wTh = wTh
  }

  @differentiable
  public func errorVector(_ hTe: Pose, _ wTo: Pose, _ oTe: Pose) -> Pose.TangentVector {
    let error = wTh.localCoordinate(wTo * oTe * hTe.inverse())
    // print("HandEyePoseFactor", error)
    return error
  }
}

/// Returns the estimated hTe and wTo transformations
public func calibrateHandEye_factorGraphImagePoints<Calibration: CameraCalibration>(
    wThList: [Pose3], 
    imagePointsList: [[Vector2]], 
    objectPoints: [Vector3], 
    cameraCalibration: Calibration,
    hTeEstimate: Pose3,
    wToEstimate: Pose3) 
    -> (Pose3, Pose3) {
  assert(wThList.count == imagePointsList.count)
  
  let nPoses = wThList.count

  var x = VariableAssignments()
  let hTeID = x.store(hTeEstimate)
  let wToID = x.store(wToEstimate)

  var graph = FactorGraph()
  var oTeIDList: [TypedID<Pose3>] = []
  let eToEstimates = wThList.map { wTh -> Pose3 in
    hTeEstimate.inverse() * wTh.inverse() * wToEstimate
  }

  for i in 0..<nPoses {
    let imagePoints = imagePointsList[i]
    assert(imagePoints.count == objectPoints.count)

    // let oTeID = x.store(eToEstimates[i])
    let oTeID = x.store(Pose3(
      Rot3(
        -1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, -1.0), 
      Vector3(-0.1, -0.1, 0.1)))
    // let oTeID = x.store(Pose3(
    //   Rot3(
    //     1.0, 0.0, 0.0,
    //     0.0, -1.0, 0.0,
    //     0.0, 0.0, -1.0), 
    //   Vector3(-0.1, -0.1, 0.1)))
    oTeIDList.append(oTeID)

    graph.store(HandEyePoseFactor(hTeID, wToID, oTeID, wThList[i]))
    for j in 0..<imagePoints.count {
      graph.store(ResectioningFactor(oTeID, objectPoints[j], imagePoints[j], cameraCalibration))
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
  // print(cam2Gripper.localCoordinate(x[hTeID]))
  // print(target2Base.localCoordinate(x[wToID]))

  return (x[hTeID], x[wToID])
}


// MARK: - SFM

/// Returns list of poses and list of landmarks.
public func performSFMOptimization<Calibration: CameraCalibration & Equatable>(
    imagePointsList: [[Vector2]], 
    objectPointsEstimate: [Vector3], 
    posesEstimate: [Pose3], 
    cameraCalibration: Calibration) 
    -> ([Pose3], [Vector3]) {
  var values = VariableAssignments()
  var graph = FactorGraph()

  let xIDList = posesEstimate.map { values.store($0) }
  let lIDList = objectPointsEstimate.map { values.store($0) }

  // for xID in xIDList {
  //   print(values[xID])
  // }
  // for lID in lIDList {
  //   print(values[lID])
  // }
  
  for i in 0..<posesEstimate.count {
    for j in 0..<objectPointsEstimate.count {
      let measurement = imagePointsList[i][j]
      graph.store(ProjectionFactor(xIDList[i], lIDList[j], measurement, cameraCalibration))
    }
  }

  graph.store(PriorFactor(xIDList[0], posesEstimate[0]))
  graph.store(VectorPriorFactor(lIDList[0], objectPointsEstimate[0]))

  var optimizer = LM(precision: 1e-6, max_iteration: 200)
  try? optimizer.optimize(graph: graph, initial: &values)

  // for xID in xIDList {
  //   print(values[xID])
  // }
  // for lID in lIDList {
  //   print(values[lID])
  // }
  
  return (xIDList.map { values[$0] }, lIDList.map { values[$0] })
}

/// Returns the estimated hTe and wTo transformations.
public func calibrateHandEye_SFM<Calibration: CameraCalibration & Equatable>(
    wThList: [Pose3], 
    imagePointsList: [[Vector2]], 
    objectPointsEstimate: [Vector3], 
    oTeListEstimate: [Pose3],
    cameraCalibration: Calibration,
    hTeEstimate: Pose3 = Pose3(),
    wToEstimate: Pose3 = Pose3()) 
    -> (Pose3, Pose3) {
  assert(wThList.count == imagePointsList.count)
  assert(wThList.count == oTeListEstimate.count)
  
  let nPoses = wThList.count

  var values = VariableAssignments()
  var graph = FactorGraph()

  let hTeID = values.store(hTeEstimate)
  let wToID = values.store(wToEstimate)
  let xIDList = oTeListEstimate.map { values.store($0) }
  let lIDList = objectPointsEstimate.map { values.store($0) }

  for i in 0..<wThList.count {
    let imagePoints = imagePointsList[i]
    let wTh = wThList[i]

    graph.store(HandEyePoseFactor(hTeID, wToID, xIDList[i], wTh));

    for j in 0..<objectPointsEstimate.count {
      let measurement = imagePoints[j]
      graph.store(ProjectionFactor(xIDList[i], lIDList[j], measurement, cameraCalibration))
    }
  }

  graph.store(PriorFactor(xIDList[0], oTeListEstimate[0]))
  graph.store(VectorPriorFactor(lIDList[0], objectPointsEstimate[0]))

  // var optimizer = LM(precision: 1e-6, max_iteration: 200)
  // try? optimizer.optimize(graph: graph, initial: &values)

  for _ in 0..<120 {
    let gfg = graph.linearized(at: values)
    var dx = values.tangentVectorZeros
    var opt = GenericCGLS(precision: 0, max_iteration: (2 + xIDList.count) * 6 + lIDList.count * 3)
    opt.optimize(gfg: gfg, initial: &dx)
    values.move(along: dx)
  }

  // for xID in xIDList {
  //   print(values[xID])
  // }
  for lID in lIDList {
    print(values[lID])
  }
  
  return (values[hTeID], values[wToID])
}
