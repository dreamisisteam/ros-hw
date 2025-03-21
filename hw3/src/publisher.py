#!/usr/bin/env python3
"""Модуль содержит имплементацию отправителя видео-фреймов."""
import sys
from typing import Generator, Tuple, Union

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image

TOPIC_NAME: str = 'camera_frames'


class CameraFramesPublisher:
    """Класс реализует функциональность отправителя видео-фреймов."""

    def __init__(self, rate: int = 15) -> None:
        """Конструктор класса.

        :param rate: Периодичность отправки.
        """
        self.rate = rate

        self.__rospy_rate = rospy.Rate(rate)
        self.__publisher = rospy.Publisher(
            TOPIC_NAME,
            Image,
            queue_size=rate,
        )

    def __call__(self) -> None:
        """Запуск отправки видео-фреймов с камеры."""
        camera_path = rospy.get_param('camera_path', '/dev/video0')

        camera_stream_factory = CameraStream(
            camera_path,
            rate=self.rate,
        )
        bridge = CvBridge()

        rospy.loginfo('Подготовка к чтению фреймов с камеры.')

        with camera_stream_factory as camera_stream:
            for idx, (status, frame) in enumerate(camera_stream()):
                if not status:
                    rospy.logwarn(f'Пропуск фрейма: {idx}.')
                    continue

                image = bridge.cv2_to_imgmsg(frame)

                self.__publisher.publish(image)
                self.__rospy_rate.sleep()


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
        self.__capture.release()


if __name__ == '__main__':
    rospy.init_node(f'{TOPIC_NAME}_pub')

    try:
        camera_frames_publisher = CameraFramesPublisher()
        camera_frames_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo('Неожиданное исключение.')
        sys.exit(1)
