import os
import requests
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import dash
from dash import Dash, html, dcc, Input, Output, State, callback, ALL
import dash_bootstrap_components as dbc
from dotenv import load_dotenv
from datetime import datetime

load_dotenv()

API_URL = os.getenv("API_URL", "http://localhost:8000")

# Initialize Dash app with Dark Bootstrap theme
app = Dash(
    __name__,
    external_stylesheets=[dbc.themes.DARKLY],
    title="BlueBox Flight Recorder"
)
app.config.suppress_callback_exceptions = True

# App layout
app.layout = dbc.Container([
    dbc.Row([
        dbc.Col([
            html.H1("BlueBox Flight Recorder", className="text-center my-4 text-light"),
            html.Hr(),
        ])
    ]),

    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardHeader(html.H4("Flight Recordings")),
                dbc.CardBody([
                    dbc.Button(
                        "Refresh Flights",
                        id="refresh-button",
                        color="primary",
                        className="mb-3"
                    ),
                    html.Div(id="flight-list")
                ])
            ])
        ], width=12, lg=3),

        dbc.Col([
            dbc.Card([
                dbc.CardHeader(html.H4("Flight Details", id="flight-title")),
                dbc.CardBody([
                    html.Div(id="flight-stats", className="mb-3"),
                    dcc.Loading(
                        id="loading",
                        type="default",
                        children=html.Div(id="graphs-container")
                    )
                ])
            ])
        ], width=12, lg=9)
    ]),

    # Confirmation modal for delete
    dbc.Modal([
        dbc.ModalHeader("Confirm Deletion"),
        dbc.ModalBody("Are you sure you want to delete this flight recording? This action cannot be undone."),
        dbc.ModalFooter([
            dbc.Button("Cancel", id="delete-cancel", color="secondary", className="me-2"),
            dbc.Button("Delete", id="delete-confirm", color="danger")
        ])
    ], id="delete-modal", is_open=False),

    # Rename modal
    dbc.Modal([
        dbc.ModalHeader("Rename Flight"),
        dbc.ModalBody([
            dbc.Label("Flight Name"),
            dbc.Input(id="rename-input", type="text", placeholder="Enter flight name", maxLength=100)
        ]),
        dbc.ModalFooter([
            dbc.Button("Cancel", id="rename-cancel", color="secondary", className="me-2"),
            dbc.Button("Save", id="rename-confirm", color="primary")
        ])
    ], id="rename-modal", is_open=False),

    # Store for selected flight data
    dcc.Store(id="selected-flight-data"),
    dcc.Store(id="selected-flight-id"),
    dcc.Store(id="delete-flight-id"),
    dcc.Store(id="rename-flight-id"),

], fluid=True, className="p-4", style={"backgroundColor": "#222"})


def fetch_flights():
    """Fetch list of flights from API."""
    try:
        response = requests.get(f"{API_URL}/api/flights")
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"Error fetching flights: {e}")
        return []


def fetch_flight_data(flight_id):
    """Fetch full flight data including samples."""
    try:
        response = requests.get(f"{API_URL}/api/flights/{flight_id}")
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"Error fetching flight data: {e}")
        return None


def delete_flight_api(flight_id):
    """Delete a flight via API."""
    try:
        response = requests.delete(f"{API_URL}/api/flights/{flight_id}")
        response.raise_for_status()
        return True
    except Exception as e:
        print(f"Error deleting flight: {e}")
        return False


def rename_flight_api(flight_id, new_name):
    """Rename a flight via API."""
    try:
        response = requests.patch(
            f"{API_URL}/api/flights/{flight_id}",
            json={"flight_name": new_name}
        )
        response.raise_for_status()
        return True
    except Exception as e:
        print(f"Error renaming flight: {e}")
        return False


@callback(
    Output("flight-list", "children"),
    Input("refresh-button", "n_clicks")
)
def update_flight_list(n_clicks):
    """Update the list of available flights."""
    flights = fetch_flights()

    if not flights:
        return dbc.Alert("No flights recorded yet.", color="info")

    flight_cards = []
    for flight in flights:
        created = datetime.fromisoformat(flight["created_at"].replace("Z", "+00:00"))

        # Display flight name with pencil icon, or default to "Flight #id"
        flight_name = flight.get('flight_name') or f"Flight #{flight['id']}"

        card = dbc.Card([
            dbc.CardBody([
                html.Div([
                    html.H6(flight_name, className="card-title d-inline me-2", style={"marginBottom": "0"}),
                    dbc.Button(
                        "✎",  # Pencil icon
                        id={"type": "rename-button", "index": flight["id"]},
                        color="link",
                        size="sm",
                        style={"padding": "0", "fontSize": "16px", "verticalAlign": "baseline"}
                    )
                ], style={"display": "flex", "alignItems": "center"}),
                html.P([
                    html.Small(f"Device: {flight['device_id'] or 'Unknown'}"),
                    html.Br(),
                    html.Small(f"Samples: {flight['sample_count'] or 'N/A'}"),
                    html.Br(),
                    html.Small(f"Duration: {flight['duration_seconds']:.2f}s" if flight['duration_seconds'] else "N/A"),
                    html.Br(),
                    html.Small(created.strftime("%Y-%m-%d %H:%M:%S")),
                ], className="card-text"),
                html.Div([
                    dbc.Button(
                        "View",
                        id={"type": "flight-button", "index": flight["id"]},
                        color="success",
                        size="sm",
                        style={"flex": "0 0 60%"}
                    ),
                    dbc.Button(
                        "×",
                        id={"type": "delete-button", "index": flight["id"]},
                        color="danger",
                        size="sm",
                        style={"flex": "0 0 15%", "padding": "0"}
                    )
                ], className="mt-2", style={"display": "flex", "width": "100%", "justifyContent": "space-between"})
            ])
        ], className="mb-3")
        flight_cards.append(card)

    return flight_cards


@callback(
    [Output("selected-flight-data", "data"),
     Output("selected-flight-id", "data")],
    Input({"type": "flight-button", "index": ALL}, "n_clicks"),
    State({"type": "flight-button", "index": ALL}, "id"),
    prevent_initial_call=True
)
def select_flight(n_clicks_list, button_ids):
    """Handle flight selection."""
    ctx = dash.callback_context

    if not ctx.triggered:
        return None, None

    # Find which button was clicked
    button_id = ctx.triggered[0]["prop_id"].split(".")[0]
    if button_id:
        import json
        button_data = json.loads(button_id)
        flight_id = button_data["index"]
        return fetch_flight_data(flight_id), flight_id

    return None, None


@callback(
    [Output("flight-title", "children"),
     Output("flight-stats", "children"),
     Output("graphs-container", "children")],
    Input("selected-flight-data", "data"),
    prevent_initial_call=False
)
def display_flight(flight_data):
    """Display flight statistics and graphs."""
    if not flight_data:
        return (
            "Flight Details",
            dbc.Alert("Select a flight to view details.", color="secondary"),
            None
        )

    # Extract metadata
    flight_id = flight_data["id"]
    flight_name = flight_data.get("flight_name") or f"Flight #{flight_id}"
    device_id = flight_data["device_id"] or "Unknown"
    sample_count = flight_data["sample_count"]
    duration = flight_data["duration_seconds"]
    max_gyro = flight_data["max_gyro_magnitude"]
    max_accel = flight_data["max_accel_magnitude"]

    # Flight statistics
    stats = dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    html.H6("Device", className="text-muted"),
                    html.H4(device_id)
                ])
            ])
        ], width=3),
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    html.H6("Samples", className="text-muted"),
                    html.H4(f"{sample_count:,}")
                ])
            ])
        ], width=3),
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    html.H6("Duration", className="text-muted"),
                    html.H4(f"{duration:.2f}s" if duration else "N/A")
                ])
            ])
        ], width=3),
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    html.H6("Max Gyro", className="text-muted"),
                    html.H4(f"{max_gyro:.1f}°/s" if max_gyro else "N/A")
                ])
            ])
        ], width=3),
    ], className="g-2")

    # Parse samples data
    samples = flight_data["data"]["samples"]
    df = pd.DataFrame(samples)

    # Convert timestamp from microseconds to seconds (relative to start)
    if len(df) > 0:
        df["time_s"] = (df["t"] - df["t"].iloc[0]) / 1_000_000.0

    # Create graphs
    graphs = create_flight_graphs(df)

    return flight_name, stats, graphs


def create_flight_graphs(df):
    """Create interactive Plotly graphs for flight data."""
    if df.empty:
        return dbc.Alert("No sample data available.", color="warning")

    graphs = []

    # 1. Gyroscope data
    fig_gyro = go.Figure()
    fig_gyro.add_trace(go.Scatter(x=df["time_s"], y=df["gx"], name="Gyro X", line=dict(color="#EF553B")))
    fig_gyro.add_trace(go.Scatter(x=df["time_s"], y=df["gy"], name="Gyro Y", line=dict(color="#00CC96")))
    fig_gyro.add_trace(go.Scatter(x=df["time_s"], y=df["gz"], name="Gyro Z", line=dict(color="#636EFA")))
    fig_gyro.update_layout(
        title="Gyroscope (deg/s)",
        xaxis_title="Time (s)",
        yaxis_title="Angular Velocity (deg/s)",
        hovermode="x unified",
        height=400,
        template="plotly_dark"
    )
    graphs.append(dcc.Graph(figure=fig_gyro, config={"displayModeBar": False}))

    # 2. Accelerometer data
    fig_accel = go.Figure()
    fig_accel.add_trace(go.Scatter(x=df["time_s"], y=df["ax"], name="Accel X", line=dict(color="#EF553B")))
    fig_accel.add_trace(go.Scatter(x=df["time_s"], y=df["ay"], name="Accel Y", line=dict(color="#00CC96")))
    fig_accel.add_trace(go.Scatter(x=df["time_s"], y=df["az"], name="Accel Z", line=dict(color="#636EFA")))
    fig_accel.update_layout(
        title="Accelerometer (g)",
        xaxis_title="Time (s)",
        yaxis_title="Acceleration (g)",
        hovermode="x unified",
        height=400,
        template="plotly_dark"
    )
    graphs.append(dcc.Graph(figure=fig_accel, config={"displayModeBar": False}))

    # 3. Pressure and Temperature
    fig_env = make_subplots(
        rows=2, cols=1,
        subplot_titles=("Barometric Pressure", "Temperature"),
        vertical_spacing=0.15
    )
    fig_env.add_trace(
        go.Scatter(x=df["time_s"], y=df["p"]/1000, name="Pressure", line=dict(color="#AB63FA")),
        row=1, col=1
    )
    fig_env.add_trace(
        go.Scatter(x=df["time_s"], y=df["temp"], name="Temperature", line=dict(color="#FFA15A")),
        row=2, col=1
    )
    fig_env.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig_env.update_yaxes(title_text="Pressure (kPa)", row=1, col=1)
    fig_env.update_yaxes(title_text="Temperature (°C)", row=2, col=1)
    fig_env.update_layout(height=500, hovermode="x unified", showlegend=False, template="plotly_dark")
    graphs.append(dcc.Graph(figure=fig_env, config={"displayModeBar": False}))

    # 4. GPS trajectory (if GPS data available)
    if "gps" in df.columns and df["gps"].notna().any():
        # Extract GPS coordinates
        gps_data = []
        for idx, row in df.iterrows():
            if pd.notna(row["gps"]) and row["gps"]:
                gps_data.append({
                    "lat": row["gps"]["lat"],
                    "lon": row["gps"]["lon"],
                    "alt": row["gps"]["alt"],
                    "spd": row["gps"]["spd"],
                    "time_s": row["time_s"]
                })

        if gps_data:
            gps_df = pd.DataFrame(gps_data)

            fig_gps = go.Figure()

            # GPS trajectory on map
            fig_gps.add_trace(go.Scattermapbox(
                lat=gps_df["lat"],
                lon=gps_df["lon"],
                mode="markers+lines",
                marker=dict(size=8, color=gps_df["time_s"], colorscale="Viridis", showscale=True),
                text=[f"Time: {t:.2f}s<br>Alt: {a:.1f}m<br>Speed: {s:.1f}km/h"
                      for t, a, s in zip(gps_df["time_s"], gps_df["alt"], gps_df["spd"])],
                hoverinfo="text",
                name="Flight Path"
            ))

            # Calculate map center
            center_lat = gps_df["lat"].mean()
            center_lon = gps_df["lon"].mean()

            fig_gps.update_layout(
                title="GPS Trajectory",
                mapbox=dict(
                    style="carto-darkmatter",
                    center=dict(lat=center_lat, lon=center_lon),
                    zoom=13
                ),
                height=500,
                margin=dict(l=0, r=0, t=40, b=0),
                template="plotly_dark",
                paper_bgcolor="#222",
                plot_bgcolor="#222"
            )
            graphs.append(dcc.Graph(figure=fig_gps, config={"displayModeBar": False}))

            # Altitude profile
            fig_alt = go.Figure()
            fig_alt.add_trace(go.Scatter(
                x=gps_df["time_s"],
                y=gps_df["alt"],
                mode="lines+markers",
                line=dict(color="#19D3F3"),
                marker=dict(size=6),
                name="Altitude"
            ))
            fig_alt.update_layout(
                title="GPS Altitude Profile",
                xaxis_title="Time (s)",
                yaxis_title="Altitude (m)",
                hovermode="x unified",
                height=300,
                template="plotly_dark"
            )
            graphs.append(dcc.Graph(figure=fig_alt, config={"displayModeBar": False}))

    return html.Div(graphs)


@callback(
    [Output("delete-modal", "is_open"),
     Output("delete-flight-id", "data")],
    [Input({"type": "delete-button", "index": ALL}, "n_clicks"),
     Input("delete-cancel", "n_clicks"),
     Input("delete-confirm", "n_clicks")],
    [State({"type": "delete-button", "index": ALL}, "id"),
     State("delete-flight-id", "data"),
     State("delete-modal", "is_open")],
    prevent_initial_call=True
)
def handle_delete_modal(delete_clicks, cancel_clicks, confirm_clicks, button_ids, stored_flight_id, is_open):
    """Handle opening/closing the delete confirmation modal."""
    ctx = dash.callback_context

    if not ctx.triggered:
        return is_open, stored_flight_id

    trigger_id = ctx.triggered[0]["prop_id"].split(".")[0]

    # Delete button clicked - open modal and store flight ID
    if "delete-button" in trigger_id:
        # Check if this is an actual click (not None)
        trigger_value = ctx.triggered[0]["value"]
        if trigger_value is None or trigger_value == 0:
            return is_open, stored_flight_id

        import json
        button_data = json.loads(trigger_id)
        flight_id = button_data["index"]
        return True, flight_id

    # Cancel or confirm clicked - close modal
    if trigger_id in ["delete-cancel", "delete-confirm"]:
        return False, stored_flight_id

    return is_open, stored_flight_id


@callback(
    Output("refresh-button", "n_clicks"),
    Input("delete-confirm", "n_clicks"),
    State("delete-flight-id", "data"),
    State("refresh-button", "n_clicks"),
    prevent_initial_call=True
)
def delete_flight_confirmed(confirm_clicks, flight_id, current_clicks):
    """Delete the flight when user confirms and trigger refresh."""
    if flight_id is not None:
        success = delete_flight_api(flight_id)
        if success:
            print(f"Successfully deleted flight {flight_id}")
            # Trigger refresh by incrementing the refresh button clicks
            return (current_clicks or 0) + 1
        else:
            print(f"Failed to delete flight {flight_id}")

    return current_clicks or 0


@callback(
    [Output("rename-modal", "is_open"),
     Output("rename-flight-id", "data"),
     Output("rename-input", "value")],
    [Input({"type": "rename-button", "index": ALL}, "n_clicks"),
     Input("rename-cancel", "n_clicks"),
     Input("rename-confirm", "n_clicks")],
    [State({"type": "rename-button", "index": ALL}, "id"),
     State("rename-flight-id", "data"),
     State("rename-modal", "is_open"),
     State("rename-input", "value")],
    prevent_initial_call=True
)
def handle_rename_modal(rename_clicks, cancel_clicks, confirm_clicks, button_ids, stored_flight_id, is_open, input_value):
    """Handle opening/closing the rename modal."""
    ctx = dash.callback_context

    if not ctx.triggered:
        return is_open, stored_flight_id, input_value

    trigger_id = ctx.triggered[0]["prop_id"].split(".")[0]

    # Rename button clicked - open modal and store flight ID
    if "rename-button" in trigger_id:
        # Check if this is an actual click (not None)
        trigger_value = ctx.triggered[0]["value"]
        if trigger_value is None or trigger_value == 0:
            return is_open, stored_flight_id, input_value

        import json
        button_data = json.loads(trigger_id)
        flight_id = button_data["index"]

        # Fetch current flight name
        flight_data = fetch_flight_data(flight_id)
        current_name = ""
        if flight_data:
            current_name = flight_data.get("flight_name") or ""

        return True, flight_id, current_name

    # Cancel or confirm clicked - close modal
    if trigger_id in ["rename-cancel", "rename-confirm"]:
        return False, stored_flight_id, ""

    return is_open, stored_flight_id, input_value


@callback(
    Output("refresh-button", "n_clicks", allow_duplicate=True),
    Input("rename-confirm", "n_clicks"),
    State("rename-flight-id", "data"),
    State("rename-input", "value"),
    State("refresh-button", "n_clicks"),
    prevent_initial_call=True
)
def rename_flight_confirmed(confirm_clicks, flight_id, new_name, current_clicks):
    """Rename the flight when user confirms and trigger refresh."""
    if flight_id is not None and new_name:
        success = rename_flight_api(flight_id, new_name)
        if success:
            print(f"Successfully renamed flight {flight_id} to '{new_name}'")
            # Trigger refresh by incrementing the refresh button clicks
            return (current_clicks or 0) + 1
        else:
            print(f"Failed to rename flight {flight_id}")

    return current_clicks or 0


if __name__ == "__main__":
    host = os.getenv("FRONTEND_HOST", "0.0.0.0")
    port = int(os.getenv("FRONTEND_PORT", 8050))
    app.run_server(debug=True, host=host, port=port)
