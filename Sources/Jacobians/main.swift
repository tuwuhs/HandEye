
import HandEye
import SwiftFusion

func main() {
  let (wThList, eToList, hTe, wTo) = simulatePoseKoide()
  let objectPoints = createTargetObject(rows: 3, cols: 2, dimension: 0.15)
  let cameraCalibration = CameraCalibration(fx: 300.0, fy: 300.0, s: 0.0, u0: 320.0, v0: 240.0)
  let imagePointsList = projectPoints(eToList: eToList, objectPoints: objectPoints, calibration: cameraCalibration)
  assert(imagePointsList.allSatisfy { $0.allSatisfy { $0.x >= 0 && $0.x < 640 && $0.y >= 0 && $0.y < 480 } },
    "Some image points fall outside the image boundary")  
}