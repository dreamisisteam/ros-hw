#!/usr/bin/env python3
"""Модуль содержит реализацию клиента сервиса."""
import simpleaudio
from audio_common_msgs.msg import AudioDataStamped
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image

from hw4_service.srv import BeepAndCapture, BeepAndCaptureResponse

AUDIO_RATE = 44100

cv_bridge = CvBridge()


def show_image(image_msg: Image):
    """Функция отображения изображения."""
    frame = cv_bridge.imgmsg_to_cv2(image_msg)
    cv2.imshow('image_from_cam', frame)
    cv2.waitKey(50000)


def play_audio(audio_msg: AudioDataStamped):
    """Функция проигрывания аудио."""
    audio_content = audio_msg.audio.data
    simpleaudio.play_buffer(
        audio_content,
        num_channels=1,
        bytes_per_sample=1,
        sample_rate=AUDIO_RATE,
    )


def callback(image_msg: Image, audio_msg: AudioDataStamped):
    """Функция обработки синхронных сообщений.

    :param image: Сообщение-репрезентация изображения.
    :param audio: Сообщение-репрезентация аудио.
    """
    rospy.loginfo('Получены сообщения для чтения.')

    show_image(image_msg)
    play_audio(audio_msg)


def send_request():
    rospy.wait_for_service('beep_and_capture')

    try:
        beep_and_capture = rospy.ServiceProxy(
            'beep_and_capture',
            BeepAndCapture,
        )
        response: BeepAndCaptureResponse = beep_and_capture()
    except rospy.ServiceException as ex:
        rospy.logerr('Ошибка: %s', ex)

    callback(response.shot, response.beep)


if __name__ == '__main__':
    send_request()
