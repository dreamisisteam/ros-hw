#!/usr/bin/env python3
"""Модуль содержит реализацию публикатора аудиосигнала."""
import sys
import pathlib

from audio_common_msgs.msg import AudioData, AudioDataStamped
import rospy
import wave

from base_publisher import BasePublisherRunner

TOPIC_NAME = '/audio/mock_files'
AUDIO_DIR = pathlib.Path(__file__).parent / 'audio'
FILENAME = 'coin.wav'


class AudioPublisherRunner(BasePublisherRunner):
    """Публикатор аудиосигнала."""

    def __call__(self):
        """Отправка событий."""
        sound_path = AUDIO_DIR / FILENAME
        with wave.open(str(sound_path)) as sound_file:
            sound_file: wave.Wave_read

            frames = sound_file.getnframes()
            sound_content = sound_file.readframes(frames)

        while not rospy.is_shutdown():
            rospy.loginfo('Формирование сообщения для отправки.')
            audio_msg = AudioDataStamped(
                audio=AudioData(data=sound_content)
            )

            self._publisher.publish(audio_msg)
            self._rospy_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('audio_publisher')

    publisher = rospy.Publisher(
        TOPIC_NAME,
        AudioDataStamped,
        queue_size=1,
    )
    publisher_runner = AudioPublisherRunner(publisher)

    try:
        publisher_runner()
    except rospy.ROSInterruptException:
        rospy.loginfo('Неожиданное исключение.')
        sys.exit(1)
