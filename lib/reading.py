import typing as t
import datetime

T = t.TypeVar("T")


class Reading(t.Generic[T]):
    start_ns: int
    stop_ns: int
    value: T

    def __init__(self, start: datetime.datetime, stop: datetime.datetime, value: T):
        self.start_ns = start
        self.stop_ns = stop
        self.value = value

    @classmethod
    def csv_header(cls, value_title: str, units: str) -> str:
        return f"sample start (ns), sample end (ns), {value_title} ({units})"

    def __str__(self) -> str:
        return f"{self.start_ns}, {self.stop_ns}, {self.value}"
