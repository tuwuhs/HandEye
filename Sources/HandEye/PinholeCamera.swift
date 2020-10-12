import SwiftFusion

public struct CameraCalibration: Equatable {
  public var fx: Double
  public var fy: Double
  public var s: Double
  public var u0: Double
  public var v0: Double

  public init(fx: Double, fy: Double, s: Double, u0: Double, v0: Double) {
    self.fx = fx
    self.fy = fy
    self.s = s
    self.u0 = u0
    self.v0 = v0
  }

  public init() {
    self.init(fx: 1.0, fy: 1.0, s: 0.0, u0: 0.0, v0: 0.0)
  }

  @differentiable
  public func uncalibrate(_ q: Vector2) -> Vector2 {
    Vector2(
      fx * q.x + s * q.y + u0,
      fy * q.y + v0)
  }
}

public struct PinholeCamera: Differentiable {
  public var pose: Pose3
  @noDerivative public var calibration: CameraCalibration

  @differentiable
  public init(_ pose: Pose3, _ calibration: CameraCalibration) {
    self.pose = pose
    self.calibration = calibration
  }

  public init(_ calibration: CameraCalibration) {
    self.pose = Pose3()
    self.calibration = calibration
  }

  public init() {
    self.init(Pose3(), CameraCalibration())
  }

  @differentiable
  public func project(_ point: Vector3) -> Vector2 {
    let intrinsicPoint: Vector2 = projectToIntrinsicCoordinates(point)
    let imagePoint = calibration.uncalibrate(intrinsicPoint)
    return imagePoint
  }

  @differentiable
  func projectToIntrinsicCoordinates(_ point: Vector3) -> Vector2 {
    projectToIntrinsicCoordinates(point).result
  }
  
  func projectToIntrinsicCoordinates(_ point: Vector3) -> (result: Vector2, Rt: Rot3, d: Double) {
    // Transform to camera coordinates
    let q = pose.inverse() * point

    // if q.z <= 0.0 {
    //   print("Chirality: z less than zero!")
    // }

    // Project to intrinsic coordinates
    let u = q.x / q.z
    let v = q.y / q.z

    return (
      result: Vector2(u, v),
      Rt: pose.rot.inverse(),
      d: 1.0 / q.z
    )
  }

  @usableFromInline
  @derivative(of: projectToIntrinsicCoordinates)
  func vjpProjectToIntrinsicCoordinates(_ point: Vector3) -> (
    value: Vector2, 
    pullback: (Vector2) -> (PinholeCamera.TangentVector, Vector3)
  ) {
    let (q, Rt, d) = projectToIntrinsicCoordinates(point)
    let Rt_ = Rt.coordinate.R
    let u = q.x
    let v = q.y
    return (
      value: q,
      pullback: { p in (
          PinholeCamera.TangentVector(pose: Vector6(
            p.x * (u*v) + p.y * (1 + v*v), 
            p.x * (-1 - u*u) + p.y * (-u*v), 
            p.x * (v) + p.y * (-u), 
            p.x * (-d), 
            p.y * (-d), 
            p.x * (d*u) + p.y * (d*v))), 
          d * Vector3(
            p.x * (Rt_[0, 0] - u * Rt_[2, 0]) + p.y * (Rt_[1, 0] - v * Rt_[2, 0]), 
            p.x * (Rt_[0, 1] - u * Rt_[2, 1]) + p.y * (Rt_[1, 1] - v * Rt_[2, 1]), 
            p.x * (Rt_[0, 2] - u * Rt_[2, 2]) + p.y * (Rt_[1, 2] - v * Rt_[2, 2]))
        ) 
      }
    )
  }
}
