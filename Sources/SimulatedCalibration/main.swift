
import HandEye
import SwiftFusion

func main() {
  let (gripper2Base, target2Cam, cam2Gripper, target2Base) = simulateDataEyeInHand(nPoses: 10, addNoise: true)

  let printError = { (handToEye: Pose3) in 
    print("Errors:")
    print("rvec: \(handToEye.rot.toRvec() - cam2Gripper.rot.toRvec())")
    print("tvec: \(handToEye.t - cam2Gripper.t)")
  }

  print("Actual hand-to-eye: \(cam2Gripper)")
  print("Actual world-to-object: \(target2Base)")

  print()

  let hTe_tsai = calibrateHandEye_tsai(worldToHand: gripper2Base, eyeToObject: target2Cam)
  print("Tsai's method")
  print("Estimated hand-to-eye: \(hTe_tsai)")
  printError(hTe_tsai)

  print()

  let (hTe_factorGraphPose, wTo_factorGraphPose) = calibrateHandEye_factorGraphPose(worldToHand: gripper2Base, eyeToObject: target2Cam)
  print("Factor graph, pose measurements")
  print("Estimated hand-to-eye: \(hTe_factorGraphPose)")
  printError(hTe_factorGraphPose)
}

main()