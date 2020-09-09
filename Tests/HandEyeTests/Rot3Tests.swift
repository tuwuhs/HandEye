import XCTest
import SwiftFusion
@testable import HandEye

final class Rot3Tests: XCTestCase {
  /// Tests QuatMinimal identity
  func testQuatMinimal() {
    let R = Rot3.fromTangent(Vector3(0.3, 0.5, -0.8))

    assertAllKeyPathEqual(Rot3.fromQuatMinimal(R.toQuatMinimal()), R, accuracy: 1e-9)
  }

  /// Tests AngleAxis
  func testAngleAxis() {
    let angle = 0.6
    let axis = (1.0 / .sqrt(3.0)) * Vector3(1.0, -1.0, 1.0)
    
    let R = Rot3.fromAngleAxis(angle, axis)
    let (actualAngle, actualAxis) = R.toAngleAxis()

    let expected = Rot3.fromTangent(angle * axis)

    assertAllKeyPathEqual(R, expected, accuracy: 1e-9)
    XCTAssertEqual(actualAngle, angle, accuracy: 1e-9)
    assertAllKeyPathEqual(actualAxis, axis, accuracy: 1e-9)
  }

  
}
