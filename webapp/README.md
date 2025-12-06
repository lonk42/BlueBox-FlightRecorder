# BlueBox Web Application

Web-based dashboard for collecting, storing, and visualizing flight data from BlueBox devices.

## Architecture

The web application consists of three components:

1. **Backend (FastAPI):** REST API for receiving and serving flight data
2. **Frontend (Dash):** Interactive dashboard with Plotly graphs
3. **Database (PostgreSQL):** Persistent storage for flight recordings

## Local Development

### Prerequisites

- Python 3.11+
- Docker & Docker Compose
- PostgreSQL 15 (or use Docker)

### Running with Docker Compose

The easiest way to run the entire stack locally:

```bash
docker-compose up --build
```

## Helm Deployment

The Docker images are not uploaded anywhere, you will need a registry to build and push them.


### Build and Push Images

```bash
# Build backend
cd backend
docker build -t your-registry/bluebox-backend:latest .
docker push your-registry/bluebox-backend:latest

# Build frontend
cd ../frontend
docker build -t your-registry/bluebox-frontend:latest .
docker push your-registry/bluebox-frontend:latest
```

### Helm install

```bash
cd helm

# Install with values
helm show values ./bluebox/ > values.yaml
helm upgrade -i bluebox ./bluebox -n bluebox --create-namespace --values values.yaml
```

## Testing the Application

### Test data

There is a python script `test_upload.py` which will populate three sets of test data.

### Uploading Test Flight Data (manually)

Use curl or Python to send test data:

```bash
curl -X POST http://localhost:8000/api/flights \
  -H "Content-Type: application/json" \
  -d '{
    "device_id": "bluebox-test",
    "samples": [
      {
        "t": 0,
        "gx": 0.1, "gy": -0.2, "gz": 0.3,
        "ax": 0.01, "ay": 0.02, "az": 1.00,
        "p": 101325.0,
        "temp": 22.5
      },
      {
        "t": 1000000,
        "gx": 150.5, "gy": -20.2, "gz": 30.3,
        "ax": 2.5, "ay": 0.8, "az": 1.20,
        "p": 100000.0,
        "temp": 20.0
      }
    ]
  }'
```

## Development

### Backend Development

The backend uses FastAPI with SQLAlchemy ORM:

- `main.py` - API endpoints and Pydantic models
- `database.py` - Database connection and session management
- `models.py` - SQLAlchemy database models

To add new endpoints, edit `main.py` and define Pydantic request/response models.

### Frontend Development

The frontend uses Plotly Dash with Bootstrap styling:

- `app.py` - Dash layout and callbacks

The app uses Dash callbacks for interactivity. To add new visualizations:

1. Add UI components to the layout
2. Create callback functions with `@callback` decorator
3. Update graphs in `create_flight_graphs()`

### Database Migrations

For schema changes, consider using Alembic:

```bash
cd backend
pip install alembic

# Initialize Alembic
alembic init alembic

# Create migration
alembic revision --autogenerate -m "Add new column"

# Apply migration
alembic upgrade head
```

## API Endpoints

### Health Check
```
GET /
Returns API status and version
```

### Upload Flight
```
POST /api/flights
Body: { "device_id": "...", "samples": [...] }
Returns: Flight metadata with ID
```

### List Flights
```
GET /api/flights?skip=0&limit=100&device_id=bluebox-001
Returns: Array of flight metadata
```

### Get Flight Data
```
GET /api/flights/{flight_id}
Returns: Full flight data with all samples
```

### Delete Flight
```
DELETE /api/flights/{flight_id}
Returns: Deletion confirmation
```

Interactive API documentation available at: http://localhost:8000/docs
