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
    Generate realistic flight data with pre-flight and post-flight buffers.

    Args:
        duration_seconds: Main flight duration in seconds (excluding buffers)
        sample_rate_hz: Sampling rate in Hz (samples per second)
    """
    samples = []

    # Add 1 second pre-flight buffer and 1 second post-flight buffer
    prelaunch_duration = 1.0
    postlanding_duration = 1.0
    total_duration = prelaunch_duration + duration_seconds + postlanding_duration
    num_samples = int(total_duration * sample_rate_hz)

    # Start timestamp (arbitrary starting point in microseconds)
    start_time_us = 1000000  # 1 second

    # Calculate phase transition timestamps
    launch_time_us = start_time_us + int(prelaunch_duration * 1_000_000)
    landing_time_us = start_time_us + int((prelaunch_duration + duration_seconds) * 1_000_000)

    for i in range(num_samples):
        # Calculate timestamp in microseconds
        t = start_time_us + int(i * (1_000_000 / sample_rate_hz))

        # Simulate flight phases
        time_s = i / sample_rate_hz

        # PRE-FLIGHT PHASE (first 1 second): stable, waiting on ground
        if time_s < prelaunch_duration:
            gx = random.uniform(-2, 2)
            gy = random.uniform(-2, 2)
            gz = random.uniform(-2, 2)

            ax = random.uniform(-0.1, 0.1)
            ay = random.uniform(-0.1, 0.1)
            az = random.uniform(0.98, 1.02)  # ~1g gravity

            altitude_m = 0
            p = 101325  # Sea level pressure
            temp = 22.0

        # LAUNCH PHASE (first 0.5s after launch): high rotation and acceleration
        elif time_s < prelaunch_duration + 0.5:
            flight_time = time_s - prelaunch_duration
            gx = random.uniform(-50, 50) + 200 * math.sin(flight_time * 10)
            gy = random.uniform(-50, 50) + 180 * math.cos(flight_time * 12)
            gz = random.uniform(-50, 50) + 150 * math.sin(flight_time * 8)

            ax = random.uniform(-0.5, 0.5) + 8.0 * math.sin(flight_time * 15)
            ay = random.uniform(-0.5, 0.5) + 6.0 * math.cos(flight_time * 10)
            az = random.uniform(-0.5, 0.5) + 12.0

            altitude_m = 50 * flight_time
            p = 101325 - (altitude_m * 12)
            temp = 22.0 - (altitude_m * 0.0065)

        # FLIGHT PHASE: moderate rotation, decreasing acceleration
        elif time_s < prelaunch_duration + duration_seconds - 0.5:
            flight_time = time_s - prelaunch_duration
            gx = random.uniform(-20, 20) + 30 * math.sin(flight_time * 3)
            gy = random.uniform(-20, 20) + 25 * math.cos(flight_time * 2.5)
            gz = random.uniform(-10, 10)

            ax = random.uniform(-1, 1) + 0.5 * math.sin(flight_time)
            ay = random.uniform(-1, 1) + 0.3 * math.cos(flight_time)
            az = random.uniform(0.8, 1.2) - (flight_time / duration_seconds) * 0.2

            altitude_m = 100 * math.sin(flight_time * 0.5)
            p = 101325 - (altitude_m * 12)
            temp = 22.0 - (altitude_m * 0.0065)

        # LANDING PHASE (last 0.5s of flight): stabilizing
        elif time_s < prelaunch_duration + duration_seconds:
            landing_progress = (time_s - (prelaunch_duration + duration_seconds - 0.5)) / 0.5
            gx = random.uniform(-15, 15) * (1 - landing_progress)
            gy = random.uniform(-15, 15) * (1 - landing_progress)
            gz = random.uniform(-10, 10) * (1 - landing_progress)

            ax = random.uniform(-0.5, 0.5) * (1 - landing_progress)
            ay = random.uniform(-0.5, 0.5) * (1 - landing_progress)
            az = 0.95 + landing_progress * 0.07  # Transitioning back to 1g

            altitude_m = 20 * (1 - landing_progress)
            p = 101325 - (altitude_m * 12)
            temp = 22.0 - (altitude_m * 0.0065)

        # POST-FLIGHT PHASE (last 1 second): landed, stable on ground
        else:
            gx = random.uniform(-1, 1)
            gy = random.uniform(-1, 1)
            gz = random.uniform(-1, 1)

            ax = random.uniform(-0.05, 0.05)
            ay = random.uniform(-0.05, 0.05)
            az = random.uniform(0.99, 1.01)  # ~1g gravity

            altitude_m = 0
            p = 101325
            temp = 22.0

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
        # GPS only available during and after launch (not in pre-flight)
        if i % 200 == 0 and time_s >= prelaunch_duration:
            flight_time = time_s - prelaunch_duration
            sample["gps"] = {
                "lat": 37.7749 + (flight_time * 0.0001) * math.cos(flight_time),
                "lon": -122.4194 + (flight_time * 0.0001) * math.sin(flight_time),
                "alt": round(altitude_m, 1),
                "spd": round(max(0, 50 + 30 * math.sin(flight_time) - flight_time * 3), 1),
                "hdg": round((flight_time * 10) % 360, 1),
                "sat": random.randint(6, 12)
            }

        samples.append(sample)

    return {
        "device_id": f"bluebox-test-{random.randint(100, 999)}",
        "phase_transitions": {
            "launch": launch_time_us,
            "landing": landing_time_us
        },
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

        # Show phase transitions if available
        if result.get('launch_time_us') is not None and result.get('landing_time_us') is not None:
            launch_s = result['launch_time_us'] / 1_000_000.0
            landing_s = result['landing_time_us'] / 1_000_000.0
            print(f"  Phase Transitions:")
            print(f"    Launch: {launch_s:.3f}s")
            print(f"    Landing: {landing_s:.3f}s")

        return result
    except requests.exceptions.RequestException as e:
        print(f"✗ Error uploading flight: {e}")
        return None

if __name__ == "__main__":
    print("BlueBox Test Data Generator\n")
    print("Generates realistic flight data with:")
    print("  • 1 second pre-flight buffer (waiting on ground)")
    print("  • Main flight data (specified duration)")
    print("  • 1 second post-flight buffer (after landing)")
    print("  • Phase transition timestamps\n")

    # Generate a few different flight profiles
    flights = [
        {"name": "Short test flight", "duration": 2.0, "rate": 1000},
        {"name": "Medium flight", "duration": 5.0, "rate": 1000},
        {"name": "Long flight", "duration": 10.0, "rate": 1000},
    ]

    for flight_spec in flights:
        print(f"\nGenerating: {flight_spec['name']}")
        total_duration = flight_spec['duration'] + 2.0  # +1s pre-flight, +1s post-flight
        print(f"  Flight duration: {flight_spec['duration']}s")
        print(f"  Total duration (with buffers): {total_duration}s at {flight_spec['rate']}Hz")

        flight_data = generate_flight_data(
            duration_seconds=flight_spec['duration'],
            sample_rate_hz=flight_spec['rate']
        )

        print(f"  Generated {len(flight_data['samples'])} samples")
        print(f"  Phase transitions: Launch at {flight_data['phase_transitions']['launch']/1e6:.3f}s, "
              f"Landing at {flight_data['phase_transitions']['landing']/1e6:.3f}s")
        upload_flight(flight_data)

    print("\n✓ All test flights uploaded!")
    print(f"\nView dashboard at: http://localhost:8050")
