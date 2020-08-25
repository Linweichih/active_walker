#!/usr/bin/env python3
import torch
import torchvision.transforms as transforms
import time
import segmentation_models_pytorch as smp
from lib_walker.track import *

parent_dir = os.path.dirname(os.path.abspath(__file__))
config.read(parent_dir + '/device.cfg')
INPUT_IMG_SIZE = (int(config.get('usb_cam', 'image_height')), int(config.get('usb_cam', 'image_width')))
MODELS_ROOT = parent_dir + "/model/"
BACKBONE = config.get('AI_CONFIG', 'BACKBONE')
CLASSES = ["background", "right_shoe", "left_shoe"]
NUM_CLASSES = len(CLASSES)


class ShoeDetection:
    def __init__(self):

        self.model_name = config.get('AI_CONFIG', 'model_name')
        self.model_path = os.path.join(MODELS_ROOT, self.model_name)
        self.model = self.load_model()
        if torch.cuda.is_available():
            self.model.cuda()

        self.data_transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize(INPUT_IMG_SIZE),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        # track class init
        self.right_shoe_track = ObjectTrack()
        self.left_shoe_track = ObjectTrack()
        self.human_position = [0, 0]
        self.human_angle = 0.0

    def load_model(self):
        print("Start loading model")
        t_s = time.time()
        model = smp.Unet(BACKBONE, classes=NUM_CLASSES, activation='softmax', encoder_weights='imagenet')
        print("successfully load the model in {} sec".format(time.time() - t_s))
        state_dict = torch.load(self.model_path)
        model.load_state_dict(state_dict)
        model.eval()

        print("successfully load the hole model in {} sec".format(time.time() - t_s))
        return model

    def detect(self, image):
        no_foot_flag = False
        # get the image of camera
        resized_image = cv2.resize(image, (int(config.get('usb_cam', 'image_width')),
                                           int(config.get('usb_cam', 'image_height'))))
        # NHWC -> NCHW
        input_img = self.data_transform(image)
        input_img = torch.unsqueeze(input_img, 0)
        # print("Size of image tensor:{}".format(input_img.size()))
        # put the tensor in to model
        # print("start process a image ")
        if torch.cuda.is_available():
            input_img = input_img.cuda()
        t_s = time.time()
        with torch.no_grad():
            # outputs = self.model_trt(input_img)
            outputs = self.model(input_img)
        # outputs = outputs.data.cpu().numpy()
        outputs = outputs.clone()
        outputs = outputs.mul(255).byte()
        outputs = outputs.cpu().numpy().squeeze(0).transpose((1, 2, 0))

        # mask_gray = cv2.cvtColor(outputs, cv2.COLOR_BGR2GRAY)
        right_mask = outputs[:, :, 2]
        left_mask = outputs[:, :, 1]
        right_pos, right_angle, right_bbox = self.right_shoe_track.update(right_mask)
        left_pos, left_angle, left_bbox = self.left_shoe_track.update(left_mask)

        if left_angle > 1.7 and right_angle > 1.7:
            no_foot_flag = True
        if not no_foot_flag:
            self.human_position[0] = right_pos[0] / 2 + left_pos[0] / 2
            self.human_position[1] = right_pos[1] / 2 + left_pos[1] / 2
            self.human_angle = right_angle + (left_angle - right_angle) / 2

            # print("human_position:", self.human_position, "    human_angle:", self.human_angle)
            # image = self.bridge.cv2_to_imgmsg(processed_image,"bgr8")

            # put the human pos and angle onto image

            processed_image = cv2.line(resized_image,
                                       (int(self.human_position[0]), int(self.human_position[1])),
                                       (int(self.human_position[0] + 60 * math.sin(self.human_angle)),
                                        int(self.human_position[1] + 60 * math.cos(self.human_angle))),
                                       (0, 0, 200), 8)
            # print("right box:", right_bbox)
            if right_bbox.all() != 0:
                processed_image = cv2.drawContours(processed_image, [right_bbox], -1, (120, 120, 0), 3)
            # print("left box:", left_bbox)
            if left_bbox.all() != 0:
                processed_image = cv2.drawContours(processed_image, [left_bbox], -1, (0, 120, 120), 3)
            # print("Use {} sec to process a image".format(time.time() - t_s))
            return processed_image, outputs, self.human_position, self.human_angle
        else:
            return resized_image


if __name__ == '__main__':
    detector = ShoeDetection()
