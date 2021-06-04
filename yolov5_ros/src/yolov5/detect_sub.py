#!/usr/bin/env python
import realsense_subscriber as realsense
import socket
import pickle
import argparse
import os
import platform
import shutil
import time
from PIL import Image as PILImage
import numpy as np

import sys
# sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
# sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')

import rospy
from yolov5_ros.msg import yolo_object
# from msg import yolo_object

from pathlib import Path
import torch
import torch.backends.cudnn as cudnn
from numpy import random


from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages, letterbox
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, strip_optimizer, set_logging)
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized



def detect(save_img=False):
    out, source, weights, view_img, save_txt, imgsz = \
        opt.output, opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size
    webcam = source.isnumeric() or source.startswith('rtsp') or source.startswith('http') or source.endswith('.txt')

    # take a pic !!
    view_img=True
    img_arr, depth_arr = realsense.get_image(show=False) #array RGB
    img_shape = depth_arr.shape
    # print('shape=',depth_arr.shape,'max', np.max(depth_arr))  #(480, 640)
    
    x1,y1,x2,y2,depth_avg = 0,0,0,0,0
    xyz_obj = np.array([0,0,0])
    K = np.array([[609.674560546875, 0.0, 323.9862365722656], [0.0, 608.5648193359375, 227.5126495361328], [0.0, 0.0, 1.0]])  # intrinsic ?
    
    
    # convert
    img_pad = letterbox(img_arr[:, :, ::-1], new_shape=imgsz)[0] # first to BGR and padding
    img_pad = img_pad[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img_pad = np.ascontiguousarray(img_pad)  # 将一个内存不连续存储的数组转换为内存连续存储的数组
    # cv2.imwrite('letterbox.jpg', 255 * img.transpose((1, 2, 0))[:, :, ::-1])  # save letterbox image

    realsense_once = 1
    # 1: take an image from realsense and infer without imwrite locally; 
    # 0: infer all images from the path, including one taken from realsense
    if realsense_once == 0:
        cv2.imwrite('./inference/images/snap.jpg', cv2.cvtColor(img_arr, cv2.COLOR_RGB2BGR)) #opencv assume BGR

    # Initialize
    set_logging()
    device = select_device(opt.device)
    if os.path.exists(out):
        shutil.rmtree(out)  # delete output folder
    os.makedirs(out)  # make new output folder
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    imgsz = check_img_size(imgsz, s=model.stride.max())  # check img_size
    if half:
        model.half()  # to FP16

    # Second-stage classifier
    classify = False
    if classify:
        modelc = load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model'])  # load weights
        modelc.to(device).eval()

    # Set Dataloader
    vid_path, vid_writer = None, None
    if webcam:
        view_img = True
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz)
    else:
        #save_img = True
        dataset = LoadImages(source, img_size=imgsz)

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

    # Run inference !!
    t0 = time.time()
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once

    # infer one image from realsense
    if realsense_once:
        img = img_pad
        im0s = img_arr[:, :, ::-1]  # BGR
        path= './data/images/viz.png'
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        
        # Inference
        t1 = time_synchronized()
        pred = model(img, augment=opt.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t2 = time_synchronized()

        # Apply Classifier
        if classify:
            pred = apply_classifier(pred, modelc, img, im0s)

        # Process detections
        how_far_to_center = []
        xyz_objs = []
        cls_names = []

        for i, det in enumerate(pred):  # detections per image
            if webcam:  # batch_size >= 1
                p, s, im0 = path[i], '%g: ' % i, im0s[i].copy()
            else:
                p, s, im0 = path, '', im0s

            save_path = str(Path(out) / Path(p).name)
            txt_path = str(Path(out) / Path(p).stem) + ('_%g' % dataset.frame if dataset.mode == 'video' else '')
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, names[int(c)])  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    # average depth
                    x1,y1,x2,y2 = xyxy[0].cpu().numpy(),xyxy[1].cpu().numpy(),xyxy[2].cpu().numpy(),xyxy[3].cpu().numpy()
                    '''x: horizontal axis; y: vertical axis; origin: upper left corner'''
                    
                    #print('x1,y1,x2,y2 = ',x1,y1,x2,y2)
                    xc,yc = (x1+x2)/2, (y1+y2)/2
                    
                    #dist_to_phoelectric = abs(xc-img_shape[1]/2) 
                    dist_to_phoelectric = abs(xc-(img_shape[1]/2-210))  #center on topleft corner
                    how_far_to_center.append(dist_to_phoelectric)

                    # x1c, x2c, y1c, y2c = int((x1+xc)/2), int((x2+xc)/2), int((y1+yc)/2), int((y2+yc)/2)



                    depth_nsamples = depth_arr[int(y1):int(y2), int(x1):int(x2)]  #a patch
                    depth_nsamples = depth_nsamples.flatten()
                    depth_nsamples = depth_nsamples[depth_nsamples!=0]
                    depth_nsamples = depth_nsamples[depth_nsamples>200]
                    # print(depth_nsamples)
                    if depth_nsamples!=[]:
                        depth_nsamples = depth_nsamples[ (depth_nsamples>depth_nsamples.mean()-2*depth_nsamples.std()) & (depth_nsamples<depth_nsamples.mean()+2*depth_nsamples.std())]
                        # print(depth_nsamples)
                        if depth_nsamples!=[]:
                            depth_avg = depth_nsamples.mean() # depth option one
                            print('xc=',xc,'yc=',yc, 'depth=',depth_avg)
                            depth_min = depth_nsamples.min()
                            print('min depth:', depth_min)  # depth option two

                    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_arr, alpha=0.03), cv2.COLORMAP_JET)
                    # cv2.imwrite('./inference/output/depth.png', depth_colormap)
                    # cv2.imwrite('./inference/output/rgb.png', im0)

                    # depth_samples = im0[int(0):int(480), int(0):int(640),:]
                    # print('my box=',int(x1), int(x2), int(y1), int(y2))

                    # cv2.imshow('x',depth_colormap)
                    # if cv2.waitKey(1) == ord('q'):  # q to quit
                    #     raise StopIteration

                    # depth_4samples = [depth_arr[int(y1c),int(x1c)], depth_arr[int(y1c),int(x2c)],depth_arr[int(y2c),int(x1c)],depth_arr[int(y2c),int(x2c)]]
                    # print('depth_4samples:',depth_4samples)
                    # depth_validsamples = [b for b in depth_4samples if b>0]  
                    # depth_avg = np.mean(depth_validsamples)
                    #print('depth for grasping =', depth_avg)

                    xyz_obj = np.dot(np.linalg.inv(K), depth_avg * np.array([xc,yc,1]).transpose())  # estimated object center-XYZ (mm)
                    xyz_obj = xyz_obj.transpose()
                    xyz_objs.append(xyz_obj)

                    # cls_name = '%s' % (names[int(cls)])
                    cls_names.append(int(cls))  # object classes
                    # print(names)  # ['moon cake', 'mango', 'durian', 'pineapple']


                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * 5 + '\n') % (cls, *xywh))  # label format

                    if save_img or view_img:  # Add bbox to image
                        label = '%s %.2f' % (names[int(cls)], conf)
                        im0 = np.array(im0) # this fix the error https://github.com/opencv/opencv/issues/18120
                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)  # error

            # Print time (inference + NMS)
            print('%sDone. (%.3fs)' % (s, t2 - t1)) #done with one det

            # Stream results
            if view_img:
                cv2.imshow(p, im0)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'images':
                    cv2.imwrite(save_path, im0)  # release previous video writer
                else:
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer
                        fourcc = 'mp4v'  # output video codec
                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*fourcc), fps, (w, h))
                    vid_writer.write(im0)

    print('All Done. (%.3fs)' % (time.time() - t0)) #done with all imgs

    # output one det closest to the image center
    cls_obj = 0
    if len(how_far_to_center) != 0:        
        output_index = np.argmin(how_far_to_center)
        xyz_obj = xyz_objs[output_index] #XYZ in camera frame
        #cls_obj = cls_names[output_index] #class  ['moon cake', 'mango', 'durian', 'pineapple']
        cls_obj = cls_names[output_index] #class  ['moon cake', 'mango', 'durian', 'pineapple', 'apple', 'pear', 'orange', 'lemon']
    obj_cls =['PET', 'metal']
    cls_obj = obj_cls[cls_obj]
    return 0,0,0, cls_obj, xyz_obj[0], xyz_obj[1], xyz_obj[2], depth_avg


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='last.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='data/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='run/output', help='output folder')  # output folder
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.4, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    opt = parser.parse_args()
    print(opt)


    # # Define publishers
    pub_pos = rospy.Publisher("/detected_objects_in_image", yolo_object, queue_size=1)
    rospy.loginfo("Launched node for object detection")

    #delay = sys.argv[1:]
    delay = 0.001
    # UDP send out of python3.8 env
    client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)            
    ip_port = ('127.0.0.1', 9999)


    while True:
    
        with torch.no_grad():
            if opt.update:  # update all models (to fix SourceChangeWarning)
                for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt']:
                    detect()
                    strip_optimizer(opt.weights)
            else:
                '''img_arr = realsense.get_image(show=False)
                im_pil = PILImage.fromarray(img_arr, 'RGB')
                im_pil.show()'''
                yolo_results = detect()
                print('yolo_results =', yolo_results[3:7])

                # create msg
                msg_yolo = yolo_object()
                msg_yolo.Class, msg_yolo.x, msg_yolo.y, msg_yolo.z = yolo_results[3:7]
                pub_pos.publish(msg_yolo)
                client.sendto(pickle.dumps(yolo_results,protocol=2),ip_port) #send                                 
            
        # time.sleep(delay)
        
    client.close()    


