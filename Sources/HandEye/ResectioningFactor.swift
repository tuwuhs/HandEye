
import PenguinStructures
import SwiftFusion

public struct ResectioningFactor<Calibration: CameraCalibration>: LinearizableFactor1 {
  public let edges: Variables.Indices

  /// Camera calibration.
  @noDerivative public let calibration: Calibration

  /// 3D point on the target.
  @noDerivative public let objectPoint: Vector3

  /// 2D point in the image.
  public let imagePoint: Vector2

  public init(_ poseID: TypedID<Pose3>, _ objectPoint: Vector3, _ imagePoint: Vector2, _ calibration: Calibration) {
    self.edges = Tuple1(poseID)
    self.objectPoint = objectPoint
    self.imagePoint = imagePoint
    self.calibration = calibration
  }

  @differentiable
  public func errorVector(_ pose: Pose3) -> Vector2 {
    // pose is oTe
    let camera = PinholeCamera(calibration, pose)
    let reprojectionError = camera.project(objectPoint) - imagePoint
    // print("ResectioningFactor", reprojectionError)
    return reprojectionError
  }
}
