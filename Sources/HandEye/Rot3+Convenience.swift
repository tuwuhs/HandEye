
import SwiftFusion
import TensorFlow

// MARK: - AngleAxis

extension Rot3 {
  public func toAngleAxis() -> (Double, Vector3) {
    let rvec = self.toRvec()
    let angle = rvec.norm
    let axis = (1.0 / angle) * rvec

    return (angle, axis)
  }

  public static func fromAngleAxis(_ theta: Double, _ axis: Vector3) -> Self {
    Rot3.fromRvec(theta * axis)
  }

  /// Returns OpenCV-style rvec (Rodrigues)
  public func toRvec() -> Vector3 {
    Rot3().localCoordinate(self)
  }

  /// Returns Rot3 from OpenCV-style rvec (Rodrigues)
  public static func fromRvec(_ rvec: Vector3) -> Self {
    Rot3.fromTangent(rvec)
  }
}

// MARK: - QuatMinimal

extension Rot3 {
  // QuatMinimal is the 3 component quaternion representation used in Tsai
  // Reference: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
  public func toQuatMinimal() -> Vector3 {
    let M = coordinateStorage.R
    let trace = M[0, 0] + M[1, 1] + M[2, 2]
    let qx: Double
    let qy: Double
    let qz: Double

    if trace > 0 {
      let S = .sqrt(trace + 1.0) * 2
      qx = (M[2, 1] - M[1, 2]) / S
      qy = (M[0, 2] - M[2, 0]) / S
      qz = (M[1, 0] - M[0, 1]) / S
    } else if M[0, 0] > M[1, 1] && M[0, 0] > M[2, 2] {
      let S = .sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2
      qx = 0.25 * S
      qy = (M[0, 1] - M[1, 0]) / S
      qz = (M[0, 2] - M[2, 0]) / S
    } else if M[1, 1] > M[2, 2] {
      let S = .sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2
      qx = (M[0, 1] - M[1, 0]) / S
      qy = 0.25 * S
      qz = (M[1, 2] - M[2, 1]) / S
    } else {
      let S = .sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2
      qx = (M[0, 2] - M[2, 0]) / S
      qy = (M[1, 2] - M[2, 1]) / S
      qz = 0.25 * S
    }

    return Vector3(qx, qy, qz)
  }

  public static func fromQuatMinimal(_ q: Vector3) -> Rot3 {
    let qt = Tensor([q.x, q.y, q.z]).reshaped(to: TensorShape(-1, 1))
    let p = _Raw.matMul(qt.transposed(), qt)[0, 0].scalar!
    let w: Double = .sqrt(1.0 - p)
    let diag_p: Tensor<Double> = eye(rowCount: 3) * p

    let r1 = 2 * _Raw.matMul(qt, qt.transposed())
    let r2 = 2 * w * skew3(q) 
    let r3 = -2 * diag_p
    let r = r1 + r2 + r3 + eye(rowCount: 3)

    return Rot3(
      r[0, 0].scalar!, r[0, 1].scalar!, r[0, 2].scalar!,
      r[1, 0].scalar!, r[1, 1].scalar!, r[1, 2].scalar!,
      r[2, 0].scalar!, r[2, 1].scalar!, r[2, 2].scalar!
    )
  }
}

fileprivate func skew3(_ v: Vector3) -> Tensor<Double> {
  let result = Tensor([
    Tensor([0, -v.z, v.y]),
    Tensor([v.z, 0, -v.x]),
    Tensor([-v.y, v.x, 0])
  ])
  return result
}

// MARK: - Tensor

extension Rot3 {
  public func toTensor() -> Tensor<Double> {
    let R = coordinateStorage.R
    return Tensor([
      Tensor([R[0, 0], R[0, 1], R[0, 2]]),
      Tensor([R[1, 0], R[1, 1], R[1, 2]]),
      Tensor([R[2, 0], R[2, 1], R[2, 2]])
    ])
  }
}
