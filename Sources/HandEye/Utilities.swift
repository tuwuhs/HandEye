
import SwiftFusion
import TensorFlow

func solveWithSVD(_ A: Tensor<Double>, _ b: Tensor<Double>) -> Tensor<Double> {
  let (s, U, V) = A.svd()
  let W = s.diagonal()
  return _Raw.matMul(_Raw.matMul(_Raw.matMul(V!, _Raw.matrixInverse(W)), U!.transposed()), b)
}

func skew3(_ v: Vector3) -> Tensor<Double> {
  let result = Tensor([
    Tensor([0, -v.z, v.y]),
    Tensor([v.z, 0, -v.x]),
    Tensor([-v.y, v.x, 0])
  ])
  return result
}
