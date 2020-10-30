
import PenguinStructures
import SwiftFusion

public struct CameraResectioningFactor: LinearizableFactor1 {
  public let edges: Variables.Indices

  /// Camera calibration
  @noDerivative public let calibration: Cal3_S2

  /// 3D point on the target
  @noDerivative public let objectPoint: Vector3

  /// 2D point in the image
  public let imagePoint: Vector2

  public init(_ poseId: TypedID<Pose3>, _ objectPoint: Vector3, _ imagePoint: Vector2, _ calibration: Cal3_S2) {
    self.edges = Tuple1(poseId)
    self.objectPoint = objectPoint
    self.imagePoint = imagePoint
    self.calibration = calibration
  }

  @differentiable
  public func errorVector(_ pose: Pose3) -> Vector2 {
    // pose is eTo, PinholeCamera takes oTe
    let camera = PinholeCamera(pose.inverse(), calibration)
    let projectedImagePoint = camera.project(objectPoint)
    let reprojectionError = projectedImagePoint - imagePoint
    // print("CameraResectioningFactor", objectPoint, projectedImagePoint, imagePoint, reprojectionError)
    return reprojectionError
  }
}

public struct CameraCalibrationFactor: LinearizableFactor2 {
  public let edges: Variables.Indices

  /// 3D point on the target
  @noDerivative public let objectPoint: Vector3

  /// 2D point in the image
  public let imagePoint: Vector2

  public init(_ poseId: TypedID<Pose3>, _ calibrationId: TypedID<Cal3_S2Manifold>, _ objectPoint: Vector3, _ imagePoint: Vector2) {
    self.edges = Tuple2(poseId, calibrationId)
    self.objectPoint = objectPoint
    self.imagePoint = imagePoint
  }

  @differentiable
  public func errorVector(_ pose: Pose3, _ calibration: Cal3_S2Manifold) -> Vector2 {
    // pose is eTo, PinholeCamera takes oTe
    let camera = PinholeCamera(pose.inverse(), calibration.coordinate)
    let projectedImagePoint = camera.project(objectPoint)
    let reprojectionError = projectedImagePoint - imagePoint
    // print("CameraResectioningFactor", objectPoint, projectedImagePoint, imagePoint, reprojectionError)
    return reprojectionError
  }
}
