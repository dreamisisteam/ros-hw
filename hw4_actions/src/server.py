#!/usr/bin/env python3
"""Модуль содержит логику сервера действий по созданию отложенного снимка."""
import pathlib
from typing import Generator, Tuple, Union
import wave

import actionlib
from audio_common_msgs.msg import AudioData, AudioDataStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy

from hw4_actions.msg import DelayedShotAction, DelayedShotFeedback, DelayedShotGoal, DelayedShotResult


AUDIO_DIR = pathlib.Path(__file__).parent / 'audio'
FILENAME = 'coin.wav'

cv_bridge = CvBridge()


class DelayedShotServer:
    """Класс содержит логику оп работе сервера отправки отложенного снимка."""

    _camera_stream: 'CameraStream'

    def __init__(self):
        """Конструктор класса."""
        self._server = actionlib.SimpleActionServer('delay_shot', DelayedShotAction, self.process, False)

    def __call__(self, camera_stream: 'CameraStream'):
        """Старт сервера."""
        self._camera_stream = camera_stream
        self._server.start()

    def process(self, goal: DelayedShotGoal):
        """Обработка запроса."""
        rate = rospy.Rate(1)

        for _ in range(goal.count):
            beep = _get_sound_content()
            self._server.publish_feedback(
                DelayedShotFeedback(
                    beep=AudioDataStamped(
                        audio=AudioData(data=beep),
                    ),
                ),
            )

            rate.sleep()

        _, frame = next(self._camera_stream())
        shot = cv_bridge.cv2_to_imgmsg(frame)

        self._server.set_succeeded(
            DelayedShotResult(shot=shot),
        )


class CameraStream:
    """Класс реализует логику по генерации стрима с камеры."""

    def __init__(
        self,
        device_path: Union[str, int] = 0,
        *,
        rate: int = 15,
    ) -> None:
        """Конструктор класса.

        :param device_path: Путь до камеры.
        :param rate: FPS.
        """
        self.device_path = device_path
        self.rate = rate

    def __enter__(self):
        """Запуск видеопотока.

        :raises ValueError: Ошибка, которая вызывается, если было передано неверный путь до устройства.
        :return: Экземпляр класса-генератора стрима.
        """
        self.__capture = cv2.VideoCapture(self.device_path)

        self.__capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.__capture.set(cv2.CAP_PROP_FPS, self.rate)

        if not self.__capture.isOpened():
            err_content = f'Видеопоток не может быть начат. Проверьте подключение видеокамеры: {self.device_path}.'
            rospy.logerr(err_content)
            raise ValueError(err_content)

        rospy.loginfo('Настройки для потока были применены.')
        return self

    def __call__(self) -> Generator[Tuple[bool, np.ndarray], None, None]:
        """Запуск стрима."""
        rospy.loginfo('Запуск чтения фреймов с камеры.')

        while self.__capture.isOpened() and not rospy.is_shutdown():
            yield self.__capture.read()

        return

    def __exit__(self, *_, **__) -> None:
        """Остановка видеопотока."""
        rospy.loginfo('Остановка видеопотока.')
        self.__capture.release()



def _get_sound_content() -> bytes:
    sound_path = AUDIO_DIR / FILENAME

    with wave.open(str(sound_path)) as sound_file:
        sound_file: wave.Wave_read

        frames = sound_file.getnframes()
        sound_content = sound_file.readframes(frames)

    return sound_content


if __name__ == '__main__':
    rospy.init_node('delayed_shot_server')
    server = DelayedShotServer()
    with (
        CameraStream(
            device_path='/dev/video0',
            rate=1,
        )
    ) as camera_stream:
        server(camera_stream)
        rospy.spin()
