 #encoding=utf-8 
import cv2
import argparse
import numpy as np
from utils.anchor_generator import generate_anchors
from utils.anchor_decode import decode_bbox
from utils.nms import single_class_non_max_suppression
from PIL import Image, ImageDraw, ImageFont
import time
import face_recognition
from utils import udp_send

# anchor configuration
feature_map_sizes = [[33, 33], [17, 17], [9, 9], [5, 5], [3, 3]]
anchor_sizes = [[0.04, 0.056], [0.08, 0.11], [0.16, 0.22], [0.32, 0.45], [0.64, 0.72]]
anchor_ratios = [[1, 0.62, 0.42]] * 5

admin_img = face_recognition.load_image_file("faces/admin.jpg")
admin_face_encoding = face_recognition.face_encodings(admin_img)[0]


# generate anchors
anchors = generate_anchors(feature_map_sizes, anchor_sizes, anchor_ratios)

# for inference , the batch size is 1, the model output shape is [1, N, 4],
# so we expand dim for anchors to [1, anchor_num, 4]
anchors_exp = np.expand_dims(anchors, axis=0)

id2class = {0: 'Mask', 1: 'NoMask'}
id2chiclass = {0: '您戴了口罩', 1: '您没有戴口罩'}
colors = ((0, 255, 0), (255, 0 , 0))
frame_cnt = 0


def puttext_chinese(img, text, point, color):
    pilimg = Image.fromarray(img)
    draw = ImageDraw.Draw(pilimg)  # 图片上打印汉字
    fontsize = int(min(img.shape[:2])*0.04)
    font = ImageFont.truetype("simhei.ttf", fontsize, encoding="utf-8")
    y = point[1]-font.getsize(text)[1]
    if y <= font.getsize(text)[1]:
        y = point[1]+font.getsize(text)[1]
    draw.text((point[0], y), text, color, font=font)
    img = np.asarray(pilimg)
    return img

def getOutputsNames(net):
    # Get the names of all the layers in the network
    layersNames = net.getLayerNames()
    # Get the names of the output layers, i.e. the layers with unconnected outputs
    return [layersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]

def recognition(image,face_locations):
    ## do recognition when only one face show.
    face_encoding = face_recognition.face_encodings(image, face_locations)[0]
    return face_recognition.compare_faces([admin_face_encoding], face_encoding,tolerance=0.4)

def inference(net, image, conf_thresh=0.5, iou_thresh=0.4, target_shape=(160, 160), draw_result=True, chinese=False):
    global frame_cnt
    height, width, _ = image.shape
    blob = cv2.dnn.blobFromImage(image, scalefactor=1/255.0, size=target_shape)
    net.setInput(blob)
    y_bboxes_output, y_cls_output = net.forward(getOutputsNames(net))
    # remove the batch dimension, for batch is always 1 for inference.
    y_bboxes = decode_bbox(anchors_exp, y_bboxes_output)[0]
    y_cls = y_cls_output[0]
    # To speed up, do single class NMS, not multiple classes NMS.
    bbox_max_scores = np.max(y_cls, axis=1)
    bbox_max_score_classes = np.argmax(y_cls, axis=1)

    # keep_idx is the alive bounding box after nms.
    keep_idxs = single_class_non_max_suppression(y_bboxes, bbox_max_scores, conf_thresh=conf_thresh, iou_thresh=iou_thresh)
    # keep_idxs  = cv2.dnn.NMSBoxes(y_bboxes.tolist(), bbox_max_scores.tolist(), conf_thresh, iou_thresh)[:,0]
    tl = round(0.002 * (height + width) * 0.5) + 1  # line thickness

    sending_data = ''
    for idx in keep_idxs:
        conf = float(bbox_max_scores[idx])
        class_id = bbox_max_score_classes[idx]
        bbox = y_bboxes[idx]
        # clip the coordinate, avoid the value exceed the image boundary.
        xmin = max(0, int(bbox[0] * width))
        ymin = max(0, int(bbox[1] * height))
        xmax = min(int(bbox[2] * width), width)
        ymax = min(int(bbox[3] * height), height)

        label,color = id2class[class_id],colors[class_id]
        if len(keep_idxs) == 1:
            match = recognition(image,[(ymin,xmax,ymax,xmin)])[0]
            print('recog:',match)
        if match:
            label = 'admin'
            color = colors[0]
            sending_data = 'admin:1'
        else:
            sending_data = 'mask:0'

        # print('label:',label)
        if frame_cnt % 16 == 0:
            udp_send.send_data(sending_data)

        # (left, top), (right, bottom)
        if draw_result:
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color) #, thickness=tl
            if chinese:
                image = puttext_chinese(image,label , (xmin, ymin),color)  ###puttext_chinese
            else:
                cv2.putText(image, "%s" % (label), (xmin + 2, ymin - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color)
    return image

def run_on_video(Net, video_path, conf_thresh=0.5):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError("Video open failed.")
    
    startTime = time.time()
    global frame_cnt
    frame_cnt = 0
    status = True

    while status:
        status, img_raw = cap.read()
        if not status:
            print("Done processing !!!")
            break
        
        frame_cnt+=1
        print(frame_cnt/(time.time()-startTime))
        
        img_raw = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)
        img_raw = inference(Net, img_raw, target_shape=(260, 260), conf_thresh=conf_thresh)
        cv2.imshow('image', img_raw[:,:,::-1])
        cv2.waitKey(1)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Face Mask Detection")
    parser.add_argument('--proto', type=str, default='models/face_mask_detection.prototxt', help='prototxt path')
    parser.add_argument('--model', type=str, default='models/face_mask_detection.caffemodel', help='model path')
    parser.add_argument('--img-mode', type=int, default=0, help='set 1 to run on image, 0 to run on video.')
    parser.add_argument('--img-path', type=str, default='img/demo2.jpg', help='path to your image.')
    parser.add_argument('--video-path', type=str, default='0', help='path to your video, `0` means to use camera.')
    # parser.add_argument('--hdf5', type=str, help='keras hdf5 file')
    args = parser.parse_args()

    Net = cv2.dnn.readNet(args.model, args.proto)
    if args.img_mode:
        img = cv2.imread(args.img_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        result = inference(Net, img, target_shape=(260, 260))
        cv2.namedWindow('detect', cv2.WINDOW_NORMAL)
        cv2.imshow('detect', result[:,:,::-1])
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        video_path = args.video_path
        if args.video_path == '0':
            video_path = 0
        run_on_video(Net, video_path, conf_thresh=0.5)