#!/usr/bin/env python3
"""Модуль содержит логику клиента действий по созданию отложенного снимка."""
import actionlib
import cv2
from cv_bridge import CvBridge
import rospy
import simpleaudio

from hw4_actions.msg import DelayedShotAction, DelayedShotFeedback, DelayedShotGoal, DelayedShotResult

AUDIO_RATE = 44100

cv_bridge = CvBridge()


def play_audio(feedback: DelayedShotFeedback):
    """Функция проигрывания аудио."""
    audio_msg = feedback.beep
    audio_content = audio_msg.audio.data
    simpleaudio.play_buffer(
        audio_content,
        num_channels=1,
        bytes_per_sample=1,
        sample_rate=AUDIO_RATE,
    )


def show_image(result: DelayedShotResult):
    """Функция отображения изображения."""
    image_msg = result.shot
    frame = cv_bridge.imgmsg_to_cv2(image_msg)
    cv2.imshow('image_from_cam', frame)
    cv2.waitKey(50000)


def action_client():
    """Реализация логики клиента действий по созданию отложенного снимка."""
    client = actionlib.SimpleActionClient('delay_shot', DelayedShotAction)
    client.wait_for_server()

    goal = DelayedShotGoal(count=5)

    client.send_goal(goal, feedback_cb=play_audio)
    client.wait_for_result()

    result = client.get_result()
    show_image(result)


if __name__ == '__main__':
    rospy.init_node('delayed_shot_client')
    action_client()
