
import HandEye
import SwiftFusion

func main() {
  let (gripper2Base, target2Cam, cam2Gripper, target2Base) = simulateDataEyeInHand(nPoses: 10, addNoise: true)

  let printError = { (handToEye: Pose3) in 
    print("Errors:")
    print("rvec: \(handToEye.rot.toRvec() - cam2Gripper.rot.toRvec())")
    print("tvec: \(handToEye.t - cam2Gripper.t)")
  }

  let hTe_Tsai = calibrateHandEyeTsai(worldToHand: gripper2Base, eyeToObject: target2Cam)

  print("Tsai's method")
  print("Estimated hand-to-eye: \(hTe_Tsai)")
  printError(hTe_Tsai)
}

main()