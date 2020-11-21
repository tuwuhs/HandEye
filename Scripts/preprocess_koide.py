
import argparse
import cv2
from preprocessor import *
import yaml

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Preprocess Hand-Eye dataset from Koide (st_handeye_eval)")
parser.add_argument('path', type=str,
  help="Path to the directory containing the image and pose files")
parser.add_argument('-p', '--pattern_size', type=int, nargs=2, metavar=("W", "H"), default=[4, 11],
  help="Target pattern size")
parser.add_argument('-t', '--pattern_type', type=int, metavar="TYPE", default=1,
  help="0: chessboard, 1: asymmetric circles grid, 2: symmetric circles grid")
parser.add_argument('-s', '--square_size', type=float, metavar="SIZE", default=0.016,
  help="Real world dimension of the target")
parser.add_argument('-o', '--out', type=str, metavar="FILENAME", default='out.yaml',
  help="Output filename (YAML)")

args = parser.parse_args()

flags = 0
is_circles_grid = False
if args.pattern_type == 0:
  pass
elif args.pattern_type == 1:
  flags |= cv2.CALIB_CB_ASYMMETRIC_GRID
  is_circles_grid = True
elif args.pattern_type == 2:
  flags |= cv2.CALIB_CB_SYMMETRIC_GRID
  is_circles_grid = True

print("Reading input files...")
image_points, image_size = read_and_detect_image_points(args.path, tuple(args.pattern_size), flags, is_circles_grid)
wTh_rvecs, wTh_tvecs = read_poses_koide(args.path)

print("Calibrating camera intrinsic parameters...")
target_points = create_target_points(args.pattern_size, args.square_size, (flags & cv2.CALIB_CB_ASYMMETRIC_GRID) != 0)
camera_info, eTo_rvecs, eTo_tvecs = calibrate_camera(target_points, image_points, image_size)

data_dict = {}
update_camera_info(data_dict, camera_info)
update_object_points(data_dict, target_points)
update_views(data_dict, image_points, wTh_rvecs, wTh_tvecs, eTo_rvecs, eTo_tvecs)

with open(args.out, 'w') as f:
  yaml.dump(data_dict, f)
  print("Output written to {}".format(args.out))