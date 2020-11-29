
import PenguinStructures
import SwiftFusion

/// A factor that specifies a prior on a vector.
public struct VectorPriorFactor<VectorType: Vector>: LinearizableFactor1 {
  public let edges: Variables.Indices
  public let prior: VectorType

  public init(_ id: TypedID<VectorType>, _ prior: VectorType) {
    self.edges = Tuple1(id)
    self.prior = prior
  }

  @differentiable
  public func errorVector(_ x: VectorType) -> VectorType.TangentVector {
    x - prior
  }
}
