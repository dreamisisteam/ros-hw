"""Модуль содержит имплементацию базового публикатора."""
import abc
import rospy


class BasePublisherRunner(abc.ABC):
    """Базовый раннер публикатора."""

    def __init__(
        self,
        publisher: rospy.Publisher,
        rate: int = 1,
    ):
        """Конструктор класса.

        :param topic_name: Наименование топика.
        :param rate: Частота отправки сообщений.
        """
        self._publisher = publisher
        self._rate = rate
        self._rospy_rate = rospy.Rate(self._rate)

    @abc.abstractmethod
    def __call__(self):
        """Отправка событий."""
        raise NotImplementedError
