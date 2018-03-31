#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar  8 12:37:13 2018

@author: simonchu
"""

"""
How to use the file?
python test_classifier.py --image <your image path>
"""
from light_classification.tl_classifier import TLClassifier
import cv2
import tensorflow as tf
import glob
import xml.etree.ElementTree as ET

flags = tf.app.flags
FLAGS = flags.FLAGS

# Command line flags
flags.DEFINE_string('image', '', "The image path")
flags.DEFINE_string('dir_images', '', "The image path")

if FLAGS.image is not '':
    img_path = FLAGS.image

    image = cv2.imread(img_path)
    light_classifier = TLClassifier()
    state = light_classifier.get_classification(image)
    if state == 0:
        print("It's RED Light")
    elif state == 1:
        print("It's YELLOW Light")
    elif state == 2:
        print("It's GREEN Light")
    elif state == 4:
        print("It's Unknown")

if FLAGS.dir_images is not '':
    path = FLAGS.dir_images
    right_count = 0
    wrong_count = 0
    total_test = 0
    tag_map = {"Red": 0,
               "Yellow": 1,
               "Green": 2}
    for pic_file in glob.glob(path + '/*.jpg'):
        print("==================")
        image = cv2.imread(pic_file)
        total_test += 1
        light_classifier = TLClassifier()
        state = light_classifier.get_classification(image)
        file_name = pic_file.split(".")[0].split("/")[-1]
        xml_file = path + "/annotations/" + file_name + ".xml"
        tree = ET.parse(xml_file)
        root = tree.getroot()
        first_member = root.findall('object')[0]
        tag = first_member[0].text
        print("state is {}".format(state))
        print("tag is {}".format(tag))
        mapped_state = tag_map[tag]
        """
        if tag is "Red":
            tag_map = 0
        elif tag is "Yellow":
            tag_map = 1
        elif tag is "Green":
            tag_map = 2
        else:
            tag_map = 4
        """
        print("mapped_state is {}".format(mapped_state))
        if state == mapped_state:
            right_count += 1
        else:
            wrong_count += 1
            
        print("right count is {}".format(right_count))
        print("wrong count is {}".format(wrong_count))
    print("right count is {}".format(right_count))
    print("wrong count is {}".format(wrong_count))
    print("total count is {}".format(total_test))