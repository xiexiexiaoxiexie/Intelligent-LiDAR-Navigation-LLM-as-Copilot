#!/usr/bin/env python
import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA

# for rviz overlay text visualization
class OverlayTextManager:
    def __init__(self):
        self.publishers = []
        self.texts = []
    def add_text_box(self, topic, text, width, height, left, top, font_size, line_width, fg_color, bg_color):
        pub = rospy.Publisher(topic, OverlayText, queue_size=1)
        self.publishers.append(pub)
        overlay = OverlayText()
        overlay.width = width
        overlay.height = height
        overlay.left = left
        overlay.top = top
        overlay.text_size = font_size
        overlay.text = text
        overlay.line_width = line_width
        overlay.font = "DejaVu Sans Mono"
        overlay.fg_color = fg_color
        overlay.bg_color = bg_color
        self.texts.append(overlay)
    def set_text(self, text, i=0):
        self.texts[i].text = text
    def publish_texts(self):
        for pub, text in zip(self.publishers, self.texts):
            pub.publish(text)
    def publish_text(self,i=0):
        self.publishers[i].publish(self.texts[i])
