#!/usr/bin/env python3
"""Модуль содержит имплементацию потребителя видео-фреймов."""
import sys

import cv2
import rospy

from base_subscriber import BaseCameraFramesSubscriber

TOPIC_NAME: str = 'camera_frames'


class MirrorCameraFramesSubscriber(BaseCameraFramesSubscriber):
    """Класс реализует функциональность потребителя видео-фреймов."""

    def _post_process_frame(self, frame):
        """Возвращает фрейм."""
        return cv2.flip(frame, 1)


if __name__ == '__main__':
    rospy.init_node(f'{TOPIC_NAME}_mirror_sub')

    try:
        camera_frames_subscriber = MirrorCameraFramesSubscriber(
            TOPIC_NAME,
            window_layout_title=f'MIRROR {TOPIC_NAME}',
        )
        camera_frames_subscriber()
    except rospy.ROSInterruptException:
        rospy.loginfo('Неожиданное исключение.')
        sys.exit(1)
