// swift-tools-version:5.3
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "HandEye",
    products: [
        // Products define the executables and libraries a package produces, and make them visible to other packages.
        .library(
            name: "HandEye",
            targets: ["HandEye"]),
    ],
    dependencies: [
        // Dependencies declare other packages that this package depends on.
        // .package(url: /* package url */, from: "1.0.0"),
        .package(url: "https://github.com/tuwuhs/SwiftFusion.git", .branch("dev-handeye")),
    ],
    targets: [
        // Targets are the basic building blocks of a package. A target can define a module or a test suite.
        // Targets can depend on other targets in this package, and on products in packages this package depends on.
        .target(
            name: "HandEye",
            dependencies: [
                .product(name: "SwiftFusion", package: "SwiftFusion"),
            ]),
        .testTarget(
            name: "HandEyeTests",
            dependencies: ["HandEye"]),
    ]
)
