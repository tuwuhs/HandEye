
import argparse
import cv2
from preprocessor import *
import yaml

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Preprocess Hand-Eye dataset from Tabb (RWHEC)")
parser.add_argument('path', type=str,
  help="Path to the dataset directory (DS*)")
parser.add_argument('-i', '--camera_id', type=int, metavar="ID", default=0,
  help="Camera ID to use (this tool only supports single camera)")
parser.add_argument('-o', '--out', type=str, metavar="FILENAME", default='out.yaml',
  help="Output filename (YAML)")

args = parser.parse_args()

print("Reading calibration object metadata...")
with open(os.path.join(args.path, 'calibration_object.txt')) as f:
  object_params = {}
  for l in f:
    fields = l.split()
    try:
      object_params[fields[0]] = int(fields[1])
    except ValueError:
      object_params[fields[0]] = float(fields[1])
assert object_params['chess_mm_height'] == object_params['chess_mm_width']
pattern_size = (object_params['chess_width'], object_params['chess_height'])
square_size = object_params['chess_mm_width']

print("Reading images...")
images_path = os.path.join(args.path, 'images/camera{}'.format(args.camera_id))
image_points, image_size = read_and_detect_image_points(images_path, pattern_size)

# # Extra images not used for now
# print("Reading extra images for intrinsic calibration...")
# internal_images_path = os.path.join(args.path, 'internal_images/camera{}'.format(args.camera_id))
# internal_image_points, internal_image_size = read_and_detect_image_points(internal_images_path, pattern_size)
# if len(internal_image_points) > 0:
#   assert internal_image_size == image_size

print("Reading poses...")
robot_cali_file = os.path.join(args.path, 'robot_cali.txt')
wTh_rvecs, wTh_tvecs = read_poses_tabb(robot_cali_file)

print("Calibrating camera intrinsic parameters...")
target_points = create_target_points(pattern_size, square_size)
camera_info, eTo_rvecs, eTo_tvecs = calibrate_camera(target_points, image_points, image_size)

data_dict = {}
update_camera_info(data_dict, camera_info)
update_object_points(data_dict, target_points)
update_views(data_dict, image_points, wTh_rvecs, wTh_tvecs, eTo_rvecs, eTo_tvecs)

with open(args.out, 'w') as f:
  yaml.dump(data_dict, f)
  print("Output written to {}".format(args.out))
