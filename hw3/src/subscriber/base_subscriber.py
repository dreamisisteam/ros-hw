#!/usr/bin/env python3
"""Модуль содержит имплементацию потребителя видео-фреймов."""
import abc

import cv2
import numpy as np
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image


class BaseCameraFramesSubscriber(abc.ABC):
    """Класс реализует функциональность потребителя видео-фреймов."""

    def __init__(
        self,
        topic_name: str,
        window_layout_title: str,
    ) -> None:
        """Конструктор класса."""
        self.window_layout_title = window_layout_title

        self.__bridge = CvBridge()
        rospy.Subscriber(
            topic_name,
            Image,
            self.callback,
        )

    def callback(self, content: Image) -> None:
        """Метод, вызываемый при получении сообщения из топика.

        :param content: Содержимое сообщения.
        """
        frame = self.__bridge.imgmsg_to_cv2(content)

        frame = self._post_process_frame(frame)

        cv2.imshow(self.window_layout_title, frame)
        cv2.waitKey(1)

    @abc.abstractmethod
    def _post_process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Метод вызывает пост-обработку кадра.

        :param frame: Кадр.
        :return: Кадр после пост-обработки.
        """
        raise NotImplementedError

    def __call__(self) -> None:
        """Запуск потребителя."""
        rospy.loginfo('Запуск потребления.')
        rospy.spin()
