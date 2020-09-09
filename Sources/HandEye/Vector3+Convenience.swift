
import SwiftFusion
import TensorFlow

// MARK: - Tensor

extension Vector3 {
  public static func fromTensor(_ t: Tensor<Double>) -> Vector3 {
    // TODO: Ensure correct dimension, handle error (use optional?)
    return Vector3(
      t[0, 0].scalar!,
      t[1, 0].scalar!,
      t[2, 0].scalar!
    )
  }

  public func toTensor() -> Tensor<Double> {
    return Tensor([
        Tensor([x]),
        Tensor([y]),
        Tensor([z]),
    ])
  }
}