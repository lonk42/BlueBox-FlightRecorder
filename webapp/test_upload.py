#!/usr/bin/env python3
"""
Test script to upload realistic flight data to the BlueBox API.
Generates sample data with proper microsecond timestamps.
"""

import requests
import json
import math
import random

API_URL = "http://localhost:8049"

def generate_flight_data(duration_seconds=5.0, sample_rate_hz=1000):
    """
    Generate realistic flight data.

    Args:
        duration_seconds: Flight duration in seconds
        sample_rate_hz: Sampling rate in Hz (samples per second)
    """
    samples = []
    num_samples = int(duration_seconds * sample_rate_hz)

    # Start timestamp (arbitrary starting point in microseconds)
    start_time_us = 1000000  # 1 second

    for i in range(num_samples):
        # Calculate timestamp in microseconds
        t = start_time_us + int(i * (1_000_000 / sample_rate_hz))

        # Simulate flight phases
        time_s = i / sample_rate_hz

        # Launch phase (first 0.5s): high rotation and acceleration
        if time_s < 0.5:
            gx = random.uniform(-50, 50) + 200 * math.sin(time_s * 10)
            gy = random.uniform(-50, 50) + 180 * math.cos(time_s * 12)
            gz = random.uniform(-50, 50) + 150 * math.sin(time_s * 8)

            ax = random.uniform(-0.5, 0.5) + 8.0 * math.sin(time_s * 15)
            ay = random.uniform(-0.5, 0.5) + 6.0 * math.cos(time_s * 10)
            az = random.uniform(-0.5, 0.5) + 12.0

        # Flight phase: moderate rotation, decreasing acceleration
        elif time_s < duration_seconds - 1.0:
            gx = random.uniform(-20, 20) + 30 * math.sin(time_s * 3)
            gy = random.uniform(-20, 20) + 25 * math.cos(time_s * 2.5)
            gz = random.uniform(-10, 10)

            ax = random.uniform(-1, 1) + 0.5 * math.sin(time_s)
            ay = random.uniform(-1, 1) + 0.3 * math.cos(time_s)
            az = random.uniform(0.8, 1.2) - (time_s / duration_seconds) * 0.2

        # Landing phase: stabilizing
        else:
            gx = random.uniform(-5, 5) * (1 - (time_s - (duration_seconds - 1)))
            gy = random.uniform(-5, 5) * (1 - (time_s - (duration_seconds - 1)))
            gz = random.uniform(-3, 3) * (1 - (time_s - (duration_seconds - 1)))

            ax = random.uniform(-0.2, 0.2)
            ay = random.uniform(-0.2, 0.2)
            az = random.uniform(0.95, 1.05)

        # Environmental data
        # Pressure decreases with altitude (simplified)
        altitude_m = 100 * math.sin(time_s * 0.5) if time_s > 0.5 else 0
        p = 101325 - (altitude_m * 12)  # ~12 Pa per meter

        # Temperature decreases slightly with altitude
        temp = 22.0 - (altitude_m * 0.0065)  # Standard lapse rate

        sample = {
            "t": t,
            "gx": round(gx, 2),
            "gy": round(gy, 2),
            "gz": round(gz, 2),
            "ax": round(ax, 2),
            "ay": round(ay, 2),
            "az": round(az, 2),
            "p": round(p, 2),
            "temp": round(temp, 2)
        }

        # Add GPS data at 5Hz (every 200 samples at 1kHz)
        if i % 200 == 0 and time_s > 0.5:
            sample["gps"] = {
                "lat": 37.7749 + (time_s * 0.0001) * math.cos(time_s),
                "lon": -122.4194 + (time_s * 0.0001) * math.sin(time_s),
                "alt": round(altitude_m, 1),
                "spd": round(50 + 30 * math.sin(time_s), 1),
                "hdg": round((time_s * 10) % 360, 1),
                "sat": random.randint(6, 12)
            }

        samples.append(sample)

    return {
        "device_id": f"bluebox-test-{random.randint(100, 999)}",
        "samples": samples
    }

def upload_flight(flight_data):
    """Upload flight data to the API."""
    try:
        response = requests.post(f"{API_URL}/api/flights", json=flight_data)
        response.raise_for_status()
        result = response.json()
        print(f"✓ Flight uploaded successfully!")
        print(f"  ID: {result['id']}")
        print(f"  Device: {result['device_id']}")
        print(f"  Samples: {result['sample_count']}")
        print(f"  Duration: {result['duration_seconds']:.3f} seconds")
        print(f"  Max Gyro: {result['max_gyro_magnitude']:.1f} deg/s")
        print(f"  Max Accel: {result['max_accel_magnitude']:.1f} g")
        return result
    except requests.exceptions.RequestException as e:
        print(f"✗ Error uploading flight: {e}")
        return None

if __name__ == "__main__":
    print("BlueBox Test Data Generator\n")

    # Generate a few different flight profiles
    flights = [
        {"name": "Short test flight", "duration": 2.0, "rate": 1000},
        {"name": "Medium flight", "duration": 5.0, "rate": 1000},
        {"name": "Long flight", "duration": 10.0, "rate": 1000},
    ]

    for flight_spec in flights:
        print(f"\nGenerating: {flight_spec['name']}")
        print(f"  Duration: {flight_spec['duration']}s at {flight_spec['rate']}Hz")

        flight_data = generate_flight_data(
            duration_seconds=flight_spec['duration'],
            sample_rate_hz=flight_spec['rate']
        )

        print(f"  Generated {len(flight_data['samples'])} samples")
        upload_flight(flight_data)

    print("\n✓ All test flights uploaded!")
    print(f"\nView dashboard at: http://localhost:8050")
