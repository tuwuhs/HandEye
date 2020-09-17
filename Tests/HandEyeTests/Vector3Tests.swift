import XCTest
import SwiftFusion
@testable import HandEye

final class Vector3Tests: XCTestCase {
  /// Tests normalizing
  func testNormalize() {
    var p = Vector3(1.0, 2.0, 2.0)
    let expected = Vector3(1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0)

    assertAllKeyPathEqual(p.normalized(), expected, accuracy: 1e-9)
    p.normalize()
    assertAllKeyPathEqual(p, expected, accuracy: 1e-9)
  }
}
