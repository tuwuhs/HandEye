import XCTest
import SwiftFusion
@testable import HandEye

/// Test cases adapted from GTSAM.
final class ProjectionFactorTests: XCTestCase {
  let K = Cal3_S2(fov: 60.0, w: 640, h: 480)

  /// Tests equality.
  func testEquals() {
    var values = VariableAssignments()
    var xID = values.store(Pose3())
    var lID = values.store(Vector3(0.0, 0.0, 0.0))

    let measurement = Vector2(323.0, 240.0);
    
    let factor1 = ProjectionFactor(xID, lID, measurement, K)
    let factor2 = ProjectionFactor(xID, lID, measurement, K)

    XCTAssertEqual(factor1, factor2)
  }

  /// Tests error vector.
  func testError() {
    var values = VariableAssignments()
    var poseID = values.store(Pose3())
    var pointID = values.store(Vector3(0.0, 0.0, 0.0))
    let measurement = Vector2(323.0, 238.0);
    
    let factor = ProjectionFactor(poseID, pointID, measurement, K)

    // Set the linearization point
    let pose = Pose3(Rot3(), Vector3(0.0, 0.0, -10.0))
    let point = Vector3(0.0, 0.0, 0.0)

    // At this value, the point is projected to the center of the image (320, 240)
    // The projection is off by this amount compared to the measurement (323, 238)
    let expectedError = Vector2(-3.0, 2.0)

    let actualError = factor.errorVector(pose, point)

    XCTAssertEqual(actualError, expectedError)
  }
}
