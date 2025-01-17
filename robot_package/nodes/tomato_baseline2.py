#!/usr/bin/python3

from __future__ import print_function

import os
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from skimage import io

import glob
import time
import argparse
from filterpy.kalman import KalmanFilter

np.random.seed(0)


def linear_assignment(cost_matrix):
  try:
    import lap
    _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
    return np.array([[y[i],i] for i in x if i >= 0]) #
  except ImportError:
    from scipy.optimize import linear_sum_assignment
    x, y = linear_sum_assignment(cost_matrix)
    return np.array(list(zip(x, y)))


def iou_batch(bb_test, bb_gt):
  """
  From SORT: Computes IOU between two bboxes in the form [x1,y1,x2,y2]
  """
  bb_gt = np.expand_dims(bb_gt, 0)
  bb_test = np.expand_dims(bb_test, 1)
  
  xx1 = np.maximum(bb_test[..., 0], bb_gt[..., 0])
  yy1 = np.maximum(bb_test[..., 1], bb_gt[..., 1])
  xx2 = np.minimum(bb_test[..., 2], bb_gt[..., 2])
  yy2 = np.minimum(bb_test[..., 3], bb_gt[..., 3])
  w = np.maximum(0., xx2 - xx1)
  h = np.maximum(0., yy2 - yy1)
  wh = w * h
  o = wh / ((bb_test[..., 2] - bb_test[..., 0]) * (bb_test[..., 3] - bb_test[..., 1])                                      
    + (bb_gt[..., 2] - bb_gt[..., 0]) * (bb_gt[..., 3] - bb_gt[..., 1]) - wh)                                              
  return(o)  


def convert_bbox_to_z(bbox):
  """
  Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
    [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
    the aspect ratio
  """
  w = bbox[2] - bbox[0]
  h = bbox[3] - bbox[1]
  x = bbox[0] + w/2.
  y = bbox[1] + h/2.
  s = w * h    #scale is just area
  r = w / float(h)
  return np.array([x, y, s, r]).reshape((4, 1))


def convert_x_to_bbox(x,score=None):
  """
  Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
    [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
  """
  w = np.sqrt(x[2] * x[3])
  h = x[2] / w
  if(score==None):
    return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.]).reshape((1,4))
  else:
    return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.,score]).reshape((1,5))


class KalmanBoxTracker(object):
  """
  This class represents the internal state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self,bbox):
    """
    Initialises a tracker using initial bounding box.
    """
    #define constant velocity model
    self.kf = KalmanFilter(dim_x=7, dim_z=4) 
    self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],[0,0,0,1,0,0,0],  [0,0,0,0,1,0,0],[0,0,0,0,0,1,0],[0,0,0,0,0,0,1]])
    self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])

    self.kf.R[2:,2:] *= 10.
    self.kf.P[4:,4:] *= 1000. #give high uncertainty to the unobservable initial velocities
    self.kf.P *= 10.
    self.kf.Q[-1,-1] *= 0.01
    self.kf.Q[4:,4:] *= 0.01

    self.kf.x[:4] = convert_bbox_to_z(bbox)
    self.time_since_update = 0
    self.id = KalmanBoxTracker.count
    KalmanBoxTracker.count += 1
    self.history = []
    self.hits = 0
    self.hit_streak = 0
    self.age = 0

  def update(self,bbox):
    """
    Updates the state vector with observed bbox.
    """
    self.time_since_update = 0
    self.history = []
    self.hits += 1
    self.hit_streak += 1
    self.kf.update(convert_bbox_to_z(bbox))

  def predict(self):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    if((self.kf.x[6]+self.kf.x[2])<=0):
      self.kf.x[6] *= 0.0
    self.kf.predict()
    self.age += 1
    if(self.time_since_update>0):
      self.hit_streak = 0
    self.time_since_update += 1
    self.history.append(convert_x_to_bbox(self.kf.x))
    return self.history[-1]

  def get_state(self):
    """
    Returns the current bounding box estimate.
    """
    return convert_x_to_bbox(self.kf.x)


def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.3):
  """
  Assigns detections to tracked object (both represented as bounding boxes)

  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)

  iou_matrix = iou_batch(detections, trackers)

  if min(iou_matrix.shape) > 0:
    a = (iou_matrix > iou_threshold).astype(np.int32)
    if a.sum(1).max() == 1 and a.sum(0).max() == 1:
        matched_indices = np.stack(np.where(a), axis=1)
    else:
      matched_indices = linear_assignment(-iou_matrix)
  else:
    matched_indices = np.empty(shape=(0,2))

  unmatched_detections = []
  for d, det in enumerate(detections):
    if(d not in matched_indices[:,0]):
      unmatched_detections.append(d)
  unmatched_trackers = []
  for t, trk in enumerate(trackers):
    if(t not in matched_indices[:,1]):
      unmatched_trackers.append(t)

  #filter out matched with low IOU
  matches = []
  for m in matched_indices:
    if(iou_matrix[m[0], m[1]]<iou_threshold):
      unmatched_detections.append(m[0])
      unmatched_trackers.append(m[1])
    else:
      matches.append(m.reshape(1,2))
  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)

  return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class Sort(object):
  def __init__(self, max_age=1, min_hits=3, iou_threshold=0.3):
    """
    Sets key parameters for SORT
    """
    self.max_age = max_age
    self.min_hits = min_hits
    self.iou_threshold = iou_threshold
    self.trackers = []
    self.frame_count = 0

  def update(self, dets=np.empty((0, 5))):
    """
    Params:
      dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
    Requires: this method must be called once for each frame even with empty detections (use np.empty((0, 5)) for frames without detections).
    Returns the a similar array, where the last column is the object ID.

    NOTE: The number of objects returned may differ from the number of detections provided.
    """
    self.frame_count += 1
    # get predicted locations from existing trackers.
    trks = np.zeros((len(self.trackers), 5))
    to_del = []
    ret = []
    for t, trk in enumerate(trks):
      pos = self.trackers[t].predict()[0]
      trk[:] = [pos[0], pos[1], pos[2], pos[3], 0]
      if np.any(np.isnan(pos)):
        to_del.append(t)
    trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
    for t in reversed(to_del):
      self.trackers.pop(t)
    matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets,trks, self.iou_threshold)

    # update matched trackers with assigned detections
    for m in matched:
      self.trackers[m[1]].update(dets[m[0], :])

    # create and initialise new trackers for unmatched detections
    for i in unmatched_dets:
        trk = KalmanBoxTracker(dets[i,:])
        self.trackers.append(trk)
    i = len(self.trackers)
    for trk in reversed(self.trackers):
        d = trk.get_state()[0]
        if (trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
          ret.append(np.concatenate((d,[trk.id+1])).reshape(1,-1)) # +1 as MOT benchmark requires positive
        i -= 1
        # remove dead tracklet
        if(trk.time_since_update > self.max_age):
          self.trackers.pop(i)
    if(len(ret)>0):
      return np.concatenate(ret)
    return np.empty((0,5))

def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='SORT demo')
    parser.add_argument('--display', dest='display', help='Display online tracker output (slow) [False]',action='store_true')
    parser.add_argument("--seq_path", help="Path to detections.", type=str, default='data')
    parser.add_argument("--phase", help="Subdirectory in seq_path.", type=str, default='train')
    parser.add_argument("--max_age", 
                        help="Maximum number of frames to keep alive a track without associated detections.", 
                        type=int, default=1)
    parser.add_argument("--min_hits", 
                        help="Minimum number of associated detections before track is initialised.", 
                        type=int, default=3)
    parser.add_argument("--iou_threshold", help="Minimum IOU for match.", type=float, default=0.3)
    args = parser.parse_args()
    return args

import pyrealsense2.pyrealsense2 as rs
import time
import ros_numpy
import message_filters
import rospy
import numpy as np
import cv2
import torch
import PIL
from cv_bridge import CvBridge, CvBridgeError

from agro.msg import IMU, Gyro, Accel, Intrinsic, Extrinsic, CamParam
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, Int32


class TomatoModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        torch.cuda.empty_cache()
        torch.hub._validate_not_a_forked_repo=lambda a,b,c: True
        self._model = torch.hub.load('ultralytics/yolov5', 'custom', path="/home/aida/yolov5/runs/train/tom17/weights/best.pt",  force_reload=True)

    def forward(self, image):
        return self._model(image)

def get_center(a, b, llimit = 0, ulimit = 720):
    center = int(0.5 * (a + b))
    return max(llimit, min(ulimit - 1, center))
    
class image_processing():
    def __init__(self, model, param):
        self.model = model
        self.model.conf = param
        self.pub = rospy.Publisher('/tomato_count', Int32, queue_size=1)
        self.tomat_pub = rospy.Publisher('/tomat_img', Image, queue_size=1)
        self.depth_pub = rospy.Publisher('/test_depth', Image, queue_size=1)
        self.test_pub = rospy.Publisher('/test_', Image, queue_size=1)
        self.bridge = CvBridge()
        self.i = 0
        self.sort = Sort(max_age=5, min_hits=3, iou_threshold=0.1)
        self.count = 0
        

    def model_process(self, image_color, image_depth):
        start = time.time() 
        depth_scale = 0.0010000000474974513
        clipping_distance_in_meters = 3 # 1 meter
        clipping_distance = clipping_distance_in_meters / depth_scale
        
        color_image_ = self.bridge.imgmsg_to_cv2(image_color)
        depth_image_ = self.bridge.imgmsg_to_cv2(image_depth)

        self.i +=1
        #depth_image_ = cv2.cvtColor(depth_image_, cv2.COLOR_BGR2GRAY)
        #depth_image_ = cv2.convertScaleAbs(depth_image_, alpha=281)
        #depth_image_ = (depth_image_ - 15 )*301 
        #print("max depth_image_", depth_image_.max())
        #print("max depth_image_", depth_image_.min())
        #color_image_ = cv2.resize(color_image_, (640, 480))
        #color_image_ = color_image_[::2, ::2, :]

        #print(depth_image.max())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 0
        # depth_image_3d = np.dstack((depth_image_, depth_image_, depth_image_))
        # print("depth_image_3d", depth_image_3d)
        # depth image is 1 channel, color is 3 channels
        #bg_removed = np.where((depth_image_ > clipping_distance) | (depth_image_ <= 0), grey_color, color_image_)
        #print('bg_removed', bg_removed.shape)
        #print('color_image_', color_image_.shape)
        clr_im = color_image_

        
        img_depth = cv2.cvtColor(depth_image_, cv2.COLOR_BGR2GRAY)
        #crop_img = img_depth[205:205+905,112:112+490]
        crop_img = img_depth[112:112+490,175:175+905]
        #crop_img = cv2.rotate(crop_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        #crop_img = cv2.flip(crop_img, 0)
        #crop_img = cv2.flip(crop_img, 1)
        width = int(1280)
        height = int(720)
        dim = (width, height)
        resized = cv2.resize(crop_img, dim, interpolation = cv2.INTER_AREA)
        #resized = cv2.rotate(resized, cv2.ROTATE_90_CLOCKWISE)
        median = cv2.medianBlur(resized,7)
        test = (median - 15 )*301 
        grey_color = 0
        depth_image_3d = np.dstack((test, test, test))
        
        
        
        #self.img_test_pub.publish(self.bridge.cv2_to_imgmsg(depth_image_, "16UC1"))
        # print(type(bg_removed))
        #print('img shape', bg_removed.shape)
        #im = PIL.Image.fromarray(bg_removed)
        #clipping_distance = 3000
        im = cv2.cvtColor(clr_im, cv2.COLOR_BGR2RGB)
        #depth_image_3d  = cv2.flip(depth_image_3d, 0)
        #depth_image_3d  = cv2.flip(depth_image_3d, 1)
       # bg_ = np.where((depth_image_3d > clipping_distance)| (depth_image_3d <= 0), grey_color, clr_im)

       # kernel = np.ones((10,10),np.uint8)

       #bg_ = cv2.morphologyEx(bg_, cv2.MORPH_OPEN, kernel)
        #resized = cv2.resize(im, (640,640), interpolation = cv2.INTER_AREA)
        #results = self.model(resized)
        #print(results)
        
        
        results = self.model(im)

        #print(results)
        
        final_image = clr_im

        #bg_removed = np.where((depth_image_3d > 3000)| (depth_image_3d <= 0), grey_color, color_image_)
        #int_tensor = results.xyxy[0].int()
        int_tensor = results.pred[0].cpu().numpy() 
        det = results.pred[0].cpu().numpy()
        #print("OOO", int_tensor )
        #print(self.sort.trackers)
        #print(track_bb.id)
        

        import copy

        grey_color = 0
        depth_image_3d = np.dstack((test, test, test))

        depth_mask = np.where((depth_image_3d > 2000) | (depth_image_3d <= 0))

        mask_3ch = np.zeros_like(clr_im)
        mask_3ch[depth_mask] = 255
        mask = (255 - mask_3ch)[:, :, 0]

        #bg_removed = np.where(depth_mask, grey_color, img_color)
        kernel = np.ones((11,11),np.uint8)  

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask,kernel,iterations = 1)

        shift = 100
        mask[shift // 2 : -shift // 2, :] = cv2.bitwise_or(mask[shift:], mask[: -shift, :])

        bg_removed = copy.deepcopy(clr_im)
        bg_removed = cv2.bitwise_and(bg_removed, bg_removed, mask=mask)

        #bg_removed = cv2.morphologyEx(bg_removed, cv2.MORPH_CLOSE, kernel)

        output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
        num_labels = output[0]
        labels = output[1]
        stats = output[2]

        #print("asasd")

        for i in range(num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            
            #print(area)

            if (area < 7000):
                bg_removed[np.where(labels == i)] = 0

        tensor_deep = []
        g = np.array((0,0,0), dtype='uint8')
        #print("tensor shape", int_tensor.shape)
        """
        for box in int_tensor:
            #print("box shape:",box.shape)
            x_center = get_center(box[0], box[2], llimit=0, ulimit=720)
            y_center = get_center(box[1], box[3], llimit=0, ulimit=1280)
            
            #if test[x_center, y_center] > clipping_distance:
            if (bg_removed[x_center, y_center]==g).all():
                np.delete(int_tensor, np.where(int_tensor == box))
                continue
            else:
                continue
              #tensor_deep.append(box)"""
        
        #depth_res = torch.FloatTensor(tensor_deep)
        #depth_res = torch.cat(([torch.tensor(x) for x in tensor_deep]),dim=-1)
        depth_res = np.array(tensor_deep)
        #print("!!!",depth_res)
        #if len(det) == 0:
        #  print("!B!B!B!", type(det))
        #if len(depth_res) == 0:
        #  print("!A!A!A!", type(depth_res))
        #if len(tensor_deep)==0:
        #  tensor_deep.insert(np.empty([0, 5]))
        track_bb = self.sort.update(int_tensor)
        for j in range(len(track_bb.tolist())):
            
                coord = track_bb.tolist()[j]
                x1,y1,x2,y2 = int(coord[0]), int(coord[1]), int(coord[2]), int(coord[3])
                name_idx = int(coord[4])
                name = "ID: {}".format(str(coord[4]))
                
                final_image = cv2.rectangle(
                        final_image,
                        (x1, y1),
                        (x2, y2),
                        (255, 0, 0), 3
                    )
                
                
                
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(
                final_image,
                name,
                (x1, y1-10), font,
                fontScale=1,
                color=(0, 255, 0),
                thickness=2
                    )

        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_ , alpha=0.5), cv2.COLORMAP_JET)
        # выравнивание по прямой
        # images = np.hstack((bg_removed, depth_colormap))
        #cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        
        #final_rot = final_image.swapaxes(0,1)#[...,::-1,:]
        final_rot = np.ascontiguousarray(np.rot90(final_image,axes=(0, 1)), dtype=np.uint8)

        for b in (self.sort.trackers):
                #print(type(b))
                if b.id > self.count:
                    self.count = b.id
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(final_rot,
                    f"NUMBER: {int(self.count*0.7)}",
                    (50, 1200), font,
                    fontScale=1.5,   
                    color=(0, 255, 255),
                    thickness=3
                    )
        
        self.tomat_pub.publish(self.bridge.cv2_to_imgmsg(final_rot, "bgr8"))
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(bg_removed, "bgr8"))
        self.test_pub.publish(self.bridge.cv2_to_imgmsg(test, "16UC1"))
        end = time.time() - start 

        #print(" time for frame: ", end)
        print("fps: ", 1.0/end)
        #print(depth_image_)
        #self.pub.publish(int(len(int_tensor)))

    def start_sub_proc(self):
        image_color = message_filters.Subscriber("/rgbd_color_1", Image)
        image_depth = message_filters.Subscriber("/rgbd_depth_1", Image)
        ts = message_filters.TimeSynchronizer([image_color, image_depth], 1)
        ts.registerCallback(self.model_process) 
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('tomato', anonymous = True)
    ip = image_processing(TomatoModel(), 0.4)
    ip.start_sub_proc()
