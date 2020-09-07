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
}

public struct PinholeCamera {
    public var pose: Pose3
    public var calibration: CameraCalibration

    public init(_ pose: Pose3, _ calibration: CameraCalibration) {
        self.pose = pose
        self.calibration = calibration
    }

    public init() {
        self.init(Pose3(), CameraCalibration())
    }

    // @differentiable
    public func project(_ point: Vector3) -> Vector2 {
        // Transform to camera coordinates
        let q = pose.rot.inverse().Adjoint(point - pose.t)

        // Project
        let d = 1.0 / q.z
        let u = q.x * d
        let v = q.y * d

        // Run through the calibration
        let K = calibration
        return Vector2(
            K.u0 + K.fx * u + K.s * v,
            K.v0 + K.fy * v)
    }
}
