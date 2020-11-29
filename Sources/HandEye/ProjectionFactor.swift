
import PenguinStructures
import SwiftFusion

public struct ProjectionFactor<Calibration: CameraCalibration & Equatable>: LinearizableFactor2, Equatable {
  public let edges: Variables.Indices

  /// Camera calibration.
  @noDerivative public let calibration: Calibration

  /// 2D measurement on the image.
  public let measured: Vector2

  public init(_ poseID: TypedID<Pose3>, _ pointID: TypedID<Vector3>, _ measured: Vector2, _ calibration: Calibration) {
    self.edges = Tuple2(poseID, pointID)
    self.measured = measured
    self.calibration = calibration
  }

  @differentiable
  public func errorVector(_ pose: Pose3, _ point: Vector3) -> Vector2 {
    let camera = PinholeCamera(calibration, pose)
    return camera.project(point) - measured
  }
}
