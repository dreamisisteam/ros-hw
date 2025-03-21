#!/usr/bin/env python3
"""Модуль содержит имплементацию публикатора видеосигнала."""
import sys
from typing import Generator, Tuple, Union

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image

from base_publisher import BasePublisherRunner

TOPIC_NAME = '/image/camera'


class CameraPublisherRunner(BasePublisherRunner):
    """Публикатор видеосигнала."""

    def __call__(self) -> None:
        """Запуск отправки видео-фреймов с камеры."""
        camera_path = rospy.get_param('camera_path', '/dev/video0')

        camera_stream_factory = CameraStream(
            camera_path,
            rate=self._rate,
        )
        bridge = CvBridge()

        rospy.loginfo('Подготовка к чтению фреймов с камеры.')

        with camera_stream_factory as camera_stream:
            for idx, (status, frame) in enumerate(camera_stream()):
                if not status:
                    rospy.logwarn(f'Пропуск фрейма: {idx}.')
                    continue

                image = bridge.cv2_to_imgmsg(frame)

                rospy.loginfo('Кадр будет отправлен в топик.')
                self._publisher.publish(image)
                self._rospy_rate.sleep()


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
    rospy.init_node('camera_publisher')

    publisher = rospy.Publisher(
        TOPIC_NAME,
        Image,
        queue_size=1,
    )
    publisher_runner = CameraPublisherRunner(publisher)

    try:
        publisher_runner()
    except rospy.ROSInterruptException:
        rospy.loginfo('Неожиданное исключение.')
        sys.exit(1)
