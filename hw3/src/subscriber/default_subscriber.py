#!/usr/bin/env python3
"""Модуль содержит имплементацию потребителя видео-фреймов."""
import sys

import rospy

from base_subscriber import BaseCameraFramesSubscriber

TOPIC_NAME: str = 'camera_frames'


class DefaultCameraFramesSubscriber(BaseCameraFramesSubscriber):
    """Класс реализует функциональность потребителя видео-фреймов."""

    def _post_process_frame(self, frame):
        """Возвращает фрейм."""
        return frame


if __name__ == '__main__':
    rospy.init_node(f'{TOPIC_NAME}_sub')

    try:
        camera_frames_subscriber = DefaultCameraFramesSubscriber(
            TOPIC_NAME,
            window_layout_title=f'DEFAULT {TOPIC_NAME}',
        )
        camera_frames_subscriber()
    except rospy.ROSInterruptException:
        rospy.loginfo('Неожиданное исключение.')
        sys.exit(1)
