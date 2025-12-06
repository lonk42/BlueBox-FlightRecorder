from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
import math

from database import engine, get_db, Base
from models import FlightRecording

# Create database tables
Base.metadata.create_all(bind=engine)

app = FastAPI(
    title="BlueBox Flight Recorder API",
    description="API for collecting and serving flight data from BlueBox devices",
    version="1.0.0"
)

# CORS middleware to allow frontend access
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict to your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Pydantic models for request/response validation
class GPSData(BaseModel):
    lat: float
    lon: float
    alt: float
    spd: float
    hdg: float
    sat: int


class FlightSample(BaseModel):
    t: int = Field(..., description="Timestamp in microseconds")
    gx: float = Field(..., description="Gyro X (deg/s)")
    gy: float = Field(..., description="Gyro Y (deg/s)")
    gz: float = Field(..., description="Gyro Z (deg/s)")
    ax: float = Field(..., description="Accel X (g)")
    ay: float = Field(..., description="Accel Y (g)")
    az: float = Field(..., description="Accel Z (g)")
    p: float = Field(..., description="Pressure (Pa)")
    temp: float = Field(..., description="Temperature (°C)")
    gps: Optional[GPSData] = Field(None, description="GPS data (if available)")


class FlightData(BaseModel):
    samples: List[FlightSample]
    device_id: Optional[str] = Field(None, description="Device identifier")


class FlightRecordingResponse(BaseModel):
    id: int
    device_id: Optional[str]
    created_at: datetime
    duration_seconds: Optional[float]
    sample_count: Optional[int]
    max_gyro_magnitude: Optional[float]
    max_accel_magnitude: Optional[float]

    class Config:
        from_attributes = True


@app.get("/")
def read_root():
    """API health check endpoint."""
    return {
        "status": "ok",
        "service": "BlueBox Flight Recorder API",
        "version": "1.0.0"
    }


@app.post("/api/flights", response_model=FlightRecordingResponse)
def upload_flight(flight_data: FlightData, db: Session = Depends(get_db)):
    """
    Upload flight data from a BlueBox device.

    This endpoint receives the JSON data dump from a BlueBox device
    after landing and stores it in the database with computed metadata.
    """
    if not flight_data.samples:
        raise HTTPException(status_code=400, detail="No samples provided")

    # Calculate metadata from samples
    sample_count = len(flight_data.samples)

    # Calculate duration (first to last sample timestamp)
    duration_seconds = None
    if sample_count > 1:
        duration_us = flight_data.samples[-1].t - flight_data.samples[0].t
        duration_seconds = duration_us / 1_000_000.0

    # Calculate max gyro magnitude
    max_gyro_mag = 0.0
    for sample in flight_data.samples:
        mag = math.sqrt(sample.gx**2 + sample.gy**2 + sample.gz**2)
        max_gyro_mag = max(max_gyro_mag, mag)

    # Calculate max accel magnitude
    max_accel_mag = 0.0
    for sample in flight_data.samples:
        mag = math.sqrt(sample.ax**2 + sample.ay**2 + sample.az**2)
        max_accel_mag = max(max_accel_mag, mag)

    # Create database record
    recording = FlightRecording(
        device_id=flight_data.device_id,
        duration_seconds=duration_seconds,
        sample_count=sample_count,
        max_gyro_magnitude=max_gyro_mag,
        max_accel_magnitude=max_accel_mag,
        data=flight_data.model_dump()  # Store full JSON
    )

    db.add(recording)
    db.commit()
    db.refresh(recording)

    return recording


@app.get("/api/flights", response_model=List[FlightRecordingResponse])
def list_flights(
    skip: int = 0,
    limit: int = 100,
    device_id: Optional[str] = None,
    db: Session = Depends(get_db)
):
    """
    List all flight recordings with optional filtering.

    Query parameters:
    - skip: Number of records to skip (pagination)
    - limit: Maximum number of records to return
    - device_id: Filter by specific device
    """
    query = db.query(FlightRecording)

    if device_id:
        query = query.filter(FlightRecording.device_id == device_id)

    recordings = query.order_by(
        FlightRecording.created_at.desc()
    ).offset(skip).limit(limit).all()

    return recordings


@app.get("/api/flights/{flight_id}")
def get_flight(flight_id: int, db: Session = Depends(get_db)):
    """
    Get full flight data including all samples for a specific recording.
    """
    recording = db.query(FlightRecording).filter(
        FlightRecording.id == flight_id
    ).first()

    if not recording:
        raise HTTPException(status_code=404, detail="Flight not found")

    return {
        "id": recording.id,
        "device_id": recording.device_id,
        "created_at": recording.created_at,
        "duration_seconds": recording.duration_seconds,
        "sample_count": recording.sample_count,
        "max_gyro_magnitude": recording.max_gyro_magnitude,
        "max_accel_magnitude": recording.max_accel_magnitude,
        "data": recording.data
    }


@app.delete("/api/flights/{flight_id}")
def delete_flight(flight_id: int, db: Session = Depends(get_db)):
    """
    Delete a specific flight recording.
    """
    recording = db.query(FlightRecording).filter(
        FlightRecording.id == flight_id
    ).first()

    if not recording:
        raise HTTPException(status_code=404, detail="Flight not found")

    db.delete(recording)
    db.commit()

    return {"status": "deleted", "id": flight_id}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
