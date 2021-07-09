from styx_msgs.msg import TrafficLight
import tensorflow as tf
# from PIL import Image
import numpy as np
import rospy

tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # Load saved model and build the detection function
        self.model = tf.saved_model.load('../models/ssd-mobilenet-v2-sim/saved_model')
        self.detect_fn = self.model.signatures['serving_default']
        self.label_map = {1: 'green', 2: 'red', 3: 'yellow', 4: 'off'}
        rospy.logwarn('Traffic Light Detection Model Loaded')

    def run_detection(self, image):
        image_np = np.asarray(image, dtype=np.uint8)

        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image_np)
        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]
        
        detections = self.detect_fn(input_tensor)
        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                    for key, value in detections.items()}
        detections['num_detections'] = num_detections

        # detection_classes should be ints.
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
        
        return detections

    def filter_boxes(self, min_score, detections):
        """Return boxes with a confidence >= `min_score`"""
        mask = detections['detection_scores'] >= min_score
        filtered_detections =  (detections['detection_boxes'][mask], 
                                detections['detection_scores'][mask], 
                                detections['detection_classes'][mask])
        return filtered_detections
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        detections = self.run_detection(image)
        boxes, scores, classes = self.filter_boxes(0.05, detections)
        # Scores are ordered highest -> lowest
        if len(classes) > 0:
            if self.label_map[classes[0]] == 'red':
                # rospy.logwarn('Red Light Detected: {}'.format(scores[0]))
                return TrafficLight.RED
        # if len(scores) > 0:
        #     rospy.logwarn('Resuming: {}'.format(scores[0]))
        # else:
        #     rospy.logwarn('Resuming')
        # for s, c in zip(scores, classes):
            # rospy.logwarn('Detected: {} with Score: {}'.format(self.label_map[c], s))
            # if self.label_map[c] == 'red':
                # rospy.logwarn('Red Light Detected: {}'.format(s))
                # return TrafficLight.RED
        # rospy.logwarn('---------------------------------------------------')

            # if(c == 1):
            #     rospy.logwarn('Red Light Detected')
            #     return TrafficLight.RED
        
        return TrafficLight.UNKNOWN