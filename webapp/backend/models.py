from sqlalchemy import Column, Integer, String, DateTime, JSON, Float
from sqlalchemy.sql import func
from database import Base


class FlightRecording(Base):
    """Flight recording model storing BlueBox flight data."""

    __tablename__ = "flight_recordings"

    id = Column(Integer, primary_key=True, index=True)
    device_id = Column(String, index=True, nullable=True)  # Device identifier (if provided)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), index=True)

    # Flight metadata (extracted from samples)
    duration_seconds = Column(Float, nullable=True)
    sample_count = Column(Integer, nullable=True)
    max_gyro_magnitude = Column(Float, nullable=True)
    max_accel_magnitude = Column(Float, nullable=True)

    # Raw JSON data from BlueBox
    data = Column(JSON, nullable=False)

    def __repr__(self):
        return f"<FlightRecording(id={self.id}, device={self.device_id}, samples={self.sample_count})>"
