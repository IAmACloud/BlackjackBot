#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, String, UInt8, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from beginner_tutorial.msg import RpsResult, CardResult, VisionControl
from ultralytics import YOLO
import random
import rospkg

class ModelManager:
    def __init__(self, simulation_mode=False):
        self.simulation_mode = simulation_mode
        if not self.simulation_mode:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('beginner_tutorial')
            
            self.rps_model = YOLO(f"{pkg_path}/weights/yolo11-rps-detection.pt")
            self.rps_classnames = ["paper","rock","scissors"]
            self.card_model = YOLO(f"{pkg_path}/weights/yolov8s_playing_cards.pt")
            self.card_classnames = [
                'AC','2C','3C','4C','5C','6C','7C','8C','9C','10C','JC','QC','KC',
                'AD','2D','3D','4D','5D','6D','7D','8D','9D','10D','JD','QD','KD',
                'AH','2H','3H','4H','5H','6H','7H','8H','9H','10H','JH','QH','KH',
                'AS','2S','3S','4S','5S','6S','7S','8S','9S','10S','JS','QS','KS'
            ]
        self._rps_cycle = ['R','P','S']
        self._rps_idx = 0
        self._cards_cycle = ['AC','2C','3D','KH','QS','10H','JC']
        self._cards_idx = 0

    def detect_rps(self, img):
        if self.simulation_mode:
            label = self._rps_cycle[self._rps_idx % 3]
            self._rps_idx += 1
            conf = 0.75 + random.random()*0.25
            return label, conf
        else:
            results = self.rps_model(img, stream=True)
            best_conf = 0.0
            best_cls = None
            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])
                    cls_idx = int(box.cls[0])
                    if conf > best_conf:
                        best_conf = conf
                        best_cls = cls_idx
            if best_cls is None:
                return 'U',0.0
            mapping = {"rock":"R","paper":"P","scissors":"S"}
            return mapping.get(self.rps_classnames[best_cls],'U'), best_conf

    def detect_card(self,img):
        if self.simulation_mode:
            card = self._cards_cycle[self._cards_idx % len(self._cards_cycle)]
            self._cards_idx += 1
            conf = 0.7 + random.random()*0.3
            return card, conf
        else:
            results = self.card_model(img, stream=True)
            best_conf = 0.0
            best_card = None
            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])
                    cls_idx = int(box.cls[0])
                    if conf > best_conf:
                        best_conf = conf
                        best_card = self.card_classnames[cls_idx]
            if best_card is None:
                return "",0.0
            return best_card,best_conf

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node', anonymous=False)
        p = rospy.get_param('vision_node',{})
        self.camera_topic = p.get('camera_topic','/camera/image_raw')
        self.frame_rate = float(p.get('frame_rate',10))
        self.rps_conf = float(p.get('rps_confidence',0.8))
        self.card_conf = float(p.get('card_confidence',0.5))
        self.compile_frames = int(p.get('compile_frames',8))
        self.percent_required = float(p.get('percent_compiled_required',0.1))
        self.simulation_mode = bool(p.get('simulation_mode',False))

        rospy.loginfo(f"[vision_node] Initialized with camera_topic: {self.camera_topic}, frame_rate: {self.frame_rate}, simulation_mode: {self.simulation_mode}")

        # Initialize flags and buffers first
        self.collect_rps = True
        self.collect_cards = False
        self.rps_buffer = []
        self.card_buffer = []
        self.current_img = None

        # Model
        self.model_mgr = ModelManager(simulation_mode=self.simulation_mode)

        # Publishers
        self.pub_rps = rospy.Publisher('/vision/rps_result', RpsResult, queue_size=10)
        self.pub_card = rospy.Publisher('/vision/card_result', CardResult, queue_size=10)
        self.pub_status = rospy.Publisher('/vision/ready', Bool, queue_size=10)

        # Subscribers
        self.bridge = CvBridge()
        self.sub_ctrl = rospy.Subscriber('/vision/control', VisionControl, self.control_cb, queue_size=5)
        self.sub_image = rospy.Subscriber(self.camera_topic,Image,self.image_cb,queue_size=1)

    # ---------------- control callback ----------------
    def control_cb(self, msg: VisionControl):
        mode = msg.mode.strip().upper()
        rospy.loginfo(f"[vision_node] Control received: {mode}")
        if mode == "RPS_START":
            self.collect_rps = True
        elif mode == "RPS_STOP":
            self.collect_rps = False
            self.rps_buffer.clear()
        elif mode=="CARDS_START":
            self.collect_cards = True
        elif mode == "CARDS_STOP":
            self.collect_cards = False
            self.card_buffer.clear()
        elif mode=="BOTH_START":
            self.collect_rps = True
            self.collect_cards = True
        elif mode=="BOTH_STOP":
            self.collect_rps = False
            self.collect_cards = False
            self.rps_buffer.clear()
            self.card_buffer.clear()

    # ---------------- image callback ----------------
    def image_cb(self,img_msg:Image):
        try:
            self.current_img = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")

    # ---------------- main loop ----------------
    def spin(self):
        rate = rospy.Rate(self.frame_rate)
        rospy.sleep(0.5)
        self.pub_status.publish(Bool(data=True))
        rospy.loginfo("📢 VisionNode is active.")

        while not rospy.is_shutdown():
            if self.current_img is not None:
                # RPS detection
                if self.collect_rps:
                    label, conf = self.model_mgr.detect_rps(self.current_img)
                    if label != 'U':
                        self.rps_buffer.append({'label':label,'conf':conf})
                        if len(self.rps_buffer) > self.compile_frames:
                            self.rps_buffer = self.rps_buffer[-self.compile_frames:]
                        self.vote_rps()

                # Card detection
                if self.collect_cards:
                    card, conf = self.model_mgr.detect_card(self.current_img)
                    if card != "":
                        self.card_buffer.append({'card':card,'conf':conf})
                        if len(self.card_buffer) > self.compile_frames:
                            self.card_buffer = self.card_buffer[-self.compile_frames:]
                        self.vote_cards()
            rate.sleep()

    # ---------------- voting ----------------
    def vote_rps(self):
        buf = self.rps_buffer
        if not buf: return
        counts = {'R':0,'P':0,'S':0}
        confs = {'R':[],'P':[],'S':[]}
        for d in buf:
            if d['conf'] >= self.rps_conf:
                counts[d['label']] += 1
                confs[d['label']].append(d['conf'])
        top_label = max(counts,key=counts.get)
        ratio = counts[top_label]/len(buf)
        if ratio >= self.percent_required:
            avg_conf = sum(confs[top_label])/len(confs[top_label])
            msg = RpsResult()
            msg.header.stamp = rospy.Time.now()
            msg.result = {'R':1,'P':2,'S':3}[top_label]
            msg.confidence = avg_conf
            self.pub_rps.publish(msg)
            rospy.loginfo(f"[vision_node] RPS result: {top_label} (ratio: {ratio:.2f}, conf: {avg_conf:.2f})")

    def vote_cards(self):
        buf = self.card_buffer
        if not buf: return
        counts = {}
        confs = {}
        for d in buf:
            if d['conf'] >= self.card_conf:
                counts[d['card']] = counts.get(d['card'],0)+1
                confs.setdefault(d['card'],[]).append(d['conf'])
        for card,cnt in counts.items():
            if cnt/len(buf) >= self.percent_required:
                avg_conf = sum(confs[card])/len(confs[card])
                msg = CardResult()
                msg.header.stamp = rospy.Time.now()
                msg.card = card
                msg.confidence = avg_conf
                self.pub_card.publish(msg)
                rospy.loginfo(f"[vision_node] Card result: {card} (ratio: {cnt/len(buf):.2f}, conf: {avg_conf:.2f})")

if __name__=='__main__':
    try:
        node = VisionNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
