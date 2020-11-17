// swift-tools-version:5.3
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
  name: "HandEye",
  products: [
    // Products define the executables and libraries a package produces, and make them visible to other packages.
    .library(name: "HandEye", targets: ["HandEye"]),
    .executable(name: "CameraResectioning", targets: ["CameraResectioning"]),
    .executable(name: "SimulatedCalibration", targets: ["SimulatedCalibration"]),
    .executable(name: "Jacobians", targets: ["Jacobians"]),
  ],
  dependencies: [
    // Dependencies declare other packages that this package depends on.
    // .package(url: /* package url */, from: "1.0.0"),
    // .package(name: "Benchmark", url: "https://github.com/google/swift-benchmark.git", .branch("master")),
    .package(url: "https://github.com/tuwuhs/SwiftFusion.git", .branch("master")),
  ],
  targets: [
    // Targets are the basic building blocks of a package. A target can define a module or a test suite.
    // Targets can depend on other targets in this package, and on products in packages this package depends on.
    .target(
      name: "HandEye",
      dependencies: [
        .product(name: "SwiftFusion", package: "SwiftFusion"),
      ]),
    .target(
      name: "CameraResectioning",
      dependencies: ["HandEye"]),
    .target(
      name: "SimulatedCalibration",
      dependencies: ["HandEye"]),
    .target(
      name: "Jacobians",
      dependencies: ["HandEye"]),
    .testTarget(
      name: "HandEyeTests",
      dependencies: ["HandEye"]),
  ]
)
