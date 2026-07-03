// -- Cesium Ion ----------------------------------------------------------------
export const CESIUM_ION_TOKEN = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI4MDc5ZTc3My03NTBjLTRhNDAtODFhNy0wMmY2MmI5NmJhYmQiLCJpZCI6MjYwMTU5LCJpYXQiOjE3Mzg4NzQ0NjN9.OKY5cXo8TzgE6H_tMB84ggCprbZD91TEuBaebhl08wA';

// -- MQTT ----------------------------------------------------------------------
// The Rust backend connects to a plain-TCP MQTT broker.
// Override via env vars: ROCKETRY_GS_MQTT_HOST (default: localhost)
//                        ROCKETRY_GS_MQTT_PORT (default: 1883)
//                        ROCKETRY_GS_MQTT_USER
//                        ROCKETRY_GS_MQTT_PASSWORD

export const TOPICS = {
  ROCKET_TELEMETRY:  'rocket/telemetry',
  ROCKET_LORA0:      'rocket/lora0',
  ROCKET_LORA1:      'rocket/lora1',
  ROCKET_LORA1_RF69: 'rocket/lora1/rf69',
  ROCKET_INTER_PICO: 'rocket/inter_pico',
  RADIO_STATUS:      'gs/radio/status',
  ANTENNA_STATE:     'antenna/state',
  GROUND_IMU:        'gs/pico/primary/imu',
  AHRS_STATUS:       'gs/pico/primary/ahrs/status',
  RAW_IMU:           'gs/pico/primary/raw/imu',
  RAW_MAG:           'gs/pico/primary/raw/mag',
  RAW_YAW_IMU:       'gs/pico/primary/raw/yaw_imu',
  NODES_WILDCARD:    'nodes/+/position',
  GS_DEMO:           'gs/demo',
  GS_LOG:            'gs/log',
  STEPPER_AZ_CMD:    'gs/cmd/az',
  // Backward-compatible topic name; payload is elevation degrees.
  STEPPER_EL_CMD:    'gs/cmd/zen',
  STEPPER_JOG_CMD:   'gs/cmd/jog',
  CALIBRATION_CMD:   'gs/cmd/calibration',
  CALIBRATION_EVENT: 'antenna/calibration/event',
  RAW_SENSORS_CMD:   'gs/cmd/raw_sensors',
  DECLINATION_CMD:   'gs/cmd/declination',
  MAG_CAL_CMD:       'gs/cmd/mag_cal',
  TRACKER_MODE_CMD:  'gs/cmd/tracker/mode',
  TRACKER_ARM_CMD:   'gs/cmd/tracker/arm',
  TRACKER_CONFIG_CMD:'gs/cmd/tracker/config',
  GS_LOCATION:        'gs/location',
  ROCKET_LOCATION:    'rocket/location',
  DEBUG_AZ_OSC:      'gs/cmd/debug/az',
  DEBUG_EL_OSC:      'gs/cmd/debug/zen',
} as const;

export const KNOWN_MQTT_TOPICS = Object.values(TOPICS);

// -- History API (Python logger) -----------------------------------------------
export const API_BASE_URL = 'http://localhost:8000';

// -- Starlink proxy (starlink_proxy.py, protobuf response, polls dish at 192.168.100.1:9200)
export const STARLINK_PROXY_URL = 'http://localhost:8001/starlink';

// How many live telemetry samples to keep in memory per streamed MQTT history.
export const MAX_HISTORY = 500;

// -- Antenna beam visualization ------------------------------------------------
// Adjust BEAM_HALF_ANGLE_DEG to match your actual antenna's half-power beamwidth.
export const BEAM_HALF_ANGLE_DEG = 15;   // degrees — half of total beam width
export const BEAM_RANGE_M        = 4000; // meters  — how far the cone extends
