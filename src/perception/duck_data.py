import datetime


class DuckData:
    def __init__(self, object_id: int, velocity_ms: float, time_per_lap: datetime.timedelta, radius: float):
        self.object_id = object_id
        self.velocity_ms = velocity_ms
        self.time_per_lap = time_per_lap
        self.radius = radius
