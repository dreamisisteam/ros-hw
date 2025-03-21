#!/usr/bin/env python3
"""Модуль содержит реализацию синхронного потребления видео- и аудиосиганала."""
from audio_common_msgs.msg import AudioDataStamped
import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
import simpleaudio
import rospy

AUDIO_RATE = 44100

bridge = CvBridge()


def show_image(image_msg: Image):
    """Функция отображения изображения."""
    frame = bridge.imgmsg_to_cv2(image_msg)
    cv2.imshow('image_from_cam', frame)
    cv2.waitKey(1)


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


if __name__ == '__main__':
    rospy.init_node('hw4_topic_consumer')

    image_sub = Subscriber(
        '/image/camera',
        Image,
    )
    audio_sub = Subscriber(
        '/audio/mock_files',
        AudioDataStamped,
    )

    sync = ApproximateTimeSynchronizer(
        [image_sub, audio_sub],
        queue_size=1,
        slop=1,
    )
    sync.registerCallback(callback)

    rospy.spin()
