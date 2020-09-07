import XCTest
import SwiftFusion
@testable import HandEye

final class PinholeCameraTests: XCTestCase {
    /// Tests default constructor
    func testConstructorDefault() {
        let camera = PinholeCamera()

        XCTAssertEqual(camera.pose, Pose3())
        XCTAssertEqual(camera.calibration, CameraCalibration(fx: 1, fy: 1, s: 0, u0: 0, v0: 0))
    }

    /// Tests constructor from pose and calibration
    func testConstructorEquality() {
        let pose = Pose3(
            Rot3(
                1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0), 
            Vector3(0.0, 0.0, 0.5))
        let K = CameraCalibration(fx: 625, fy: 625, s: 0, u0: 0, v0: 0)
        let camera = PinholeCamera(pose, K)

        XCTAssertEqual(camera.pose, pose)
        XCTAssertEqual(camera.calibration, K)
    }

    /// Tests projection, simple case
    func testProject() {
        let pose = Pose3(
            Rot3(
                1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0), 
            Vector3(0.0, 0.0, 0.5))
        let K = CameraCalibration(fx: 625, fy: 625, s: 0, u0: 0, v0: 0)
        let camera = PinholeCamera(pose, K)

        let p1 = Vector3(-0.08, -0.08, 0.0)
        let p2 = Vector3(-0.08,  0.08, 0.0)
        let p3 = Vector3( 0.08,  0.08, 0.0)
        let p4 = Vector3( 0.08, -0.08, 0.0)

        XCTAssertEqual(camera.project(p1), Vector2(-100,  100))
        XCTAssertEqual(camera.project(p2), Vector2(-100, -100))
        XCTAssertEqual(camera.project(p3), Vector2( 100, -100))
        XCTAssertEqual(camera.project(p4), Vector2( 100,  100))
    }
}

