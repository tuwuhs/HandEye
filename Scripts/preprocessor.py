
import csv
import cv2
from enum import Enum
import glob
import os
import numpy as np
import yaml

class CameraInfo(object):
  def __init__(self, filename=None):
    if filename is None:
      self.image_width = 0
      self.image_height = 0
      self.camera_name = ''
      self.camera_matrix = np.zeros((3, 3))
      self.distortion_model = ''
      self.distortion_coefficients = None
      self.rectification_matrix = np.eye(3)
      self.projection_matrix = np.zeros((4, 3))
      return
    
    with open(filename) as f:
      data = yaml.load(f)
      self.image_width = data['image_width']
      self.image_height = data['image_height']
      self.camera_name = str(data['camera_name'])
      self.camera_matrix = np.array(data['camera_matrix']['data']).reshape(
        (data['camera_matrix']['rows'], data['camera_matrix']['cols']))
      self.distortion_model = data['distortion_model']
      self.distortion_coefficients = np.array(data['distortion_coefficients']['data']).reshape(
        (data['distortion_coefficients']['rows'], data['distortion_coefficients']['cols']))
      self.rectification_matrix = np.array(data['rectification_matrix']['data']).reshape(
        (data['rectification_matrix']['rows'], data['rectification_matrix']['cols']))
      self.projection_matrix = np.array(data['projection_matrix']['data']).reshape(
        (data['projection_matrix']['rows'], data['projection_matrix']['cols']))


def create_target_points(pattern_size, square_size, is_asymmetric_grid=False):
  target_points = []
  if is_asymmetric_grid:
    for i in range(0, pattern_size[1]):
      for j in range(0, pattern_size[0]):
        target_points.append(np.array([(2*j + i%2)*square_size, i*square_size, 0]))
  else:
    for i in range(0, pattern_size[1]):
      for j in range(0, pattern_size[0]):
        target_points.append(np.array([j*square_size, i*square_size, 0]))
  return np.array(target_points, dtype=np.float32).reshape((1, -1, 3))


def read_and_detect_image_points(images_path, pattern_size, flags=None, is_circles_grid=False):
  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

  image_points = []
  image_size = None

  # Natural sort https://stackoverflow.com/questions/33159106/sort-filenames-in-directory-in-ascending-order
  filenames = glob.glob(os.path.join(images_path, '*'))
  filenames.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))

  for filename in filenames:
    # Let OpenCV decides if the file is an image
    image = cv2.imread(filename)
    print(filename, end="")
    if image is None:
      print(" skipped.")
      continue
    else:
      print(" OK.")
    
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if image_size is None:
      image_size = image_gray.shape
    elif image_gray.shape != image_size:
      raise RuntimeError("Image size in {} is different".format(filename))
    
    if is_circles_grid:
      ret, corners = cv2.findCirclesGrid(image_gray, pattern_size, flags=flags)
    else:
      ret, corners = cv2.findChessboardCorners(image_gray, pattern_size, flags=flags)
      if ret:
        corners = cv2.cornerSubPix(image_gray, corners, (11, 11), (-1, -1), criteria)

    if not ret:
      raise RuntimeError("No target detected in {}".format(filename))

    image_points.append(corners)
  
  return image_points, image_size


def calibrate_camera_from_path(images_path, pattern_size, square_size, flags=None, is_circles_grid=False):
  is_asymmetric_grid = (flags & cv2.CALIB_CB_ASYMMETRIC_GRID) != 0 if flags is not None else False
  target_points = create_target_points(pattern_size, square_size, is_asymmetric_grid)
  image_points, image_size = read_and_detect_image_points(images_path, pattern_size, flags, is_circles_grid)
  
  return calibrate_camera(target_points, image_points, image_size)


def calibrate_camera(target_points, image_points, image_size):
  object_points = [target_points for _ in image_points]

  ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, image_size, 
    None, None, flags=cv2.CALIB_FIX_K3)

  camera_info = CameraInfo()
  camera_info.image_width = image_size[0]
  camera_info.image_height = image_size[1]
  camera_info.camera_matrix = K
  camera_info.distortion_coefficients = dist
  return camera_info, rvecs, tvecs


def read_poses_koide(path):
  rvecs = []
  tvecs = []

  # Natural sort https://stackoverflow.com/questions/33159106/sort-filenames-in-directory-in-ascending-order
  filenames = glob.glob(os.path.join(path, '*.csv'))
  filenames.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))

  for filename in filenames:
    pose = []
    with open(filename) as f:
      reader = csv.reader(f, delimiter=' ')
      for row in reader:
        pose.extend(row)
    pose_homogeneous = np.array(pose).astype(np.float).reshape((4, 4))
    rvecs.append(cv2.Rodrigues(pose_homogeneous[:3,:3])[0].flatten())
    tvecs.append(pose_homogeneous[:3,3].flatten())

  return rvecs, tvecs


def read_poses_tabb(robot_cali_file):
  rvecs = []
  tvecs = []

  with open(robot_cali_file) as f:
    it = iter(f)
    num_poses = int(next(f))
    count = 0
    pose = []
    for l in f:
      val = l.split()
      if len(val) > 0:
        pose.extend(val)
      else:
        pose_homogeneous = np.array(pose).astype(np.float).reshape((4, 4))
        rvecs.append(cv2.Rodrigues(pose_homogeneous[:3,:3])[0].flatten())
        tvecs.append(pose_homogeneous[:3,3].flatten())
        count += 1
        pose = []
  
  return rvecs, tvecs


def update_camera_info(data_dict, camera_info):
  data_dict['camera_calibration'] = {
    'fx': camera_info.camera_matrix[0, 0].item(),
    'fy': camera_info.camera_matrix[1, 1].item(),
    's': camera_info.camera_matrix[0, 1].item(),
    'u0': camera_info.camera_matrix[0, 2].item(),
    'v0': camera_info.camera_matrix[1, 2].item(),
  }
  data_dict['distortion_coefficients'] = camera_info.distortion_coefficients.flatten().tolist()
  return data_dict


def update_object_points(data_dict, target_points):
  data_dict['object_points'] = target_points.reshape((-1, 3)).tolist()
  return data_dict


def update_views(data_dict, image_points, wTh_rvecs, wTh_tvecs, eTo_rvecs, eTo_tvecs):
  assert len(image_points) == len(wTh_rvecs)
  assert len(wTh_rvecs) == len(wTh_tvecs)
  assert len(wTh_tvecs) == len(eTo_rvecs)
  assert len(eTo_rvecs) == len(eTo_tvecs)

  views = [{
      'image_points': ip.reshape((-1, 2)).tolist(),
      'wTh': {
        'rvec': r1.flatten().tolist(),
        'tvec': t1.flatten().tolist()
      },
      'eTo': {
        'rvec': r2.flatten().tolist(),
        'tvec': t2.flatten().tolist()
      }
    } for ip, r1, t1, r2, t2 in zip(image_points, wTh_rvecs, wTh_tvecs, eTo_rvecs, eTo_tvecs)]
  data_dict['views'] = views

  return views

