import typing as t


class _Summable(t.Protocol):
    def __add__(self, other: object): ...
    def __sub__(self, other: object): ...
    def __iadd__(self, other: object): ...
    def __isub__(self, other: object): ...
    def __truediv__(self, other: object): ...


T = t.TypeVar("T", bound=_Summable)


class CircularBuffer(t.Generic[T]):
    _buffer: list[T]
    _buffer_size: int
    _front: int
    _back: int
    _full: bool
    _sum: T

    def __init__(self, buffer_size: int):
        self._buffer_size = buffer_size
        self._buffer = [0] * buffer_size
        self._front = 0
        self._back = 0
        self._sum = 0
        self._full = False

    @classmethod
    def from_buffer(buff: list[T]) -> "CircularBuffer":
        self = CircularBuffer[T](len(buff))
        self._buffer = buff
        self._front = 0
        self._back = len(buff) - 1
        self._sum = sum(buff)
        self._full = True

        return self

    def append(self, value: T):
        self._incrememnt_index()

        self._sum -= self._buffer[self._front]
        self._buffer[self._front] = value
        self._sum += value

    def mean(self) -> T:
        if self._full:
            count = self._buffer_size
        else:
            count = self._front
        return self._sum / count

    def _incrememnt_index(self):
        self._front = (self._front + 1) % self._buffer_size

        if self._front == self._back:
            self._full = True
            self._back = (self._back + 1) % self._buffer_size

    def full(self) -> bool:
        return self._full
