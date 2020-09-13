
import HandEye
import PenguinStructures
import SwiftFusion

public struct CameraResectioningFactor: LinearizableFactor1 {
  public let edges: Variables.Indices

  /// Camera calibration
  public let calibration: CameraCalibration

  /// 3D point on the target
  public let objectPoint: Vector3

  /// 2D point in the image
  public let imagePoint: Vector2

  public init(_ poseId: TypedID<Pose3>, _ objectPoint: Vector3, _ imagePoint: Vector2, _ calibration: CameraCalibration) {
    self.edges = Tuple1(poseId)
    self.objectPoint = objectPoint
    self.imagePoint = imagePoint
    self.calibration = calibration
  }

  @differentiable
  public func errorVector(_ pose: Pose3) -> Vector2 {
    let camera = PinholeCamera(pose, calibration)
    let reprojectionError = camera.project(objectPoint) - imagePoint
    return reprojectionError
  }
}
