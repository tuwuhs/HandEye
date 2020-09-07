import XCTest
@testable import HandEye

final class HandEyeTests: XCTestCase {
  func testExample() {
    // This is an example of a functional test case.
    // Use XCTAssert and related functions to verify your tests produce the correct
    // results.
    XCTAssertEqual(HandEye().text, "Hello, World!")
  }

  static var allTests = [
    ("testExample", testExample),
  ]
}
