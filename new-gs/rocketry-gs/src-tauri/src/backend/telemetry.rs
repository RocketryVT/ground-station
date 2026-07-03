use serde::{Deserialize, Serialize};
use serde_json::{json, Map, Value};
use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};

pub const EVENT_NAME: &str = "telemetry://event";
const MAX_HISTORY: usize = 500;
const MAX_LOG: usize = 500;
const MAX_RAW_PER_TOPIC: usize = 500;
const MAX_SENSOR_RAW: usize = 500;
const MAX_AHRS: usize = 500;
const MAX_CALIBRATION_EVENTS: usize = 25;

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LogLine {
    pub id: u64,
    pub ts: i64,
    pub text: String,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RawMessage {
    pub id: u64,
    pub ts: i64,
    pub topic: String,
    pub payload: String,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TelemetrySnapshot {
    pub latest: Option<Value>,
    pub history: Vec<Value>,
    pub antenna: Option<Value>,
    pub ground_imu: Option<Value>,
    pub ahrs_status: Option<Value>,
    pub nodes: HashMap<String, Value>,
    pub connected: bool,
    pub flight_start: Option<i64>,
    pub log_lines: Vec<LogLine>,
    pub raw_messages: Vec<RawMessage>,
    pub raw_imu: Vec<Value>,
    pub raw_mag: Vec<Value>,
    pub raw_yaw_imu: Vec<Value>,
    pub ahrs_history: Vec<Value>,
    pub calibration_events: Vec<Value>,
    pub radio_statuses: HashMap<String, Value>,
}

#[derive(Clone, Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum UiEvent {
    Connected { connected: bool },
    LogLine { line: LogLine },
    Telemetry { telemetry: Value },
    Antenna { antenna: Value },
    GroundImu { imu: Value },
    AhrsStatus { status: Value },
    CalibrationEvent { event: Value },
    RadioStatus { status: Value },
    Node { node: Value },
    RawImu { sample: Value },
    RawMag { sample: Value },
    RawYawImu { sample: Value },
    ActiveDrag { data: Value },
}

#[derive(Default)]
struct RuntimeState {
    latest: Option<Value>,
    history: VecDeque<Value>,
    antenna: Option<Value>,
    ground_imu: Option<Value>,
    ahrs_status: Option<Value>,
    nodes: HashMap<String, Value>,
    connected: bool,
    flight_start: Option<i64>,
    log_lines: VecDeque<LogLine>,
    raw_messages: VecDeque<RawMessage>,
    raw_imu: VecDeque<Value>,
    raw_mag: VecDeque<Value>,
    raw_yaw_imu: VecDeque<Value>,
    ahrs_history: VecDeque<Value>,
    calibration_events: VecDeque<Value>,
    radio_statuses: HashMap<String, Value>,
    seq: u64,
}

#[derive(Clone, Default)]
pub struct TelemetryState {
    inner: Arc<Mutex<RuntimeState>>,
}

impl TelemetryState {
    pub fn snapshot(&self) -> TelemetrySnapshot {
        let state = self.inner.lock().expect("telemetry state");
        TelemetrySnapshot {
            latest: state.latest.clone(),
            history: state.history.iter().cloned().collect(),
            antenna: state.antenna.clone(),
            ground_imu: state.ground_imu.clone(),
            ahrs_status: state.ahrs_status.clone(),
            nodes: state.nodes.clone(),
            connected: state.connected,
            flight_start: state.flight_start,
            log_lines: state.log_lines.iter().cloned().collect(),
            raw_messages: state.raw_messages.iter().cloned().collect(),
            raw_imu: state.raw_imu.iter().cloned().collect(),
            raw_mag: state.raw_mag.iter().cloned().collect(),
            raw_yaw_imu: state.raw_yaw_imu.iter().cloned().collect(),
            ahrs_history: state.ahrs_history.iter().cloned().collect(),
            calibration_events: state.calibration_events.iter().cloned().collect(),
            radio_statuses: state.radio_statuses.clone(),
        }
    }

    pub fn set_connected(&self, connected: bool) -> bool {
        let mut state = self.inner.lock().expect("telemetry state");
        if state.connected == connected {
            return false;
        }
        state.connected = connected;
        true
    }

    pub fn add_log_line(&self, text: impl Into<String>) -> LogLine {
        let mut state = self.inner.lock().expect("telemetry state");
        let line = LogLine {
            id: state.seq,
            ts: now_ms(),
            text: text.into(),
        };
        state.seq += 1;
        push_capped(&mut state.log_lines, line.clone(), MAX_LOG);
        line
    }

    pub fn add_raw_message(&self, topic: &str, payload: String) -> RawMessage {
        let mut state = self.inner.lock().expect("telemetry state");
        let message = RawMessage {
            id: state.seq,
            ts: now_ms(),
            topic: topic.to_string(),
            payload,
        };
        state.seq += 1;
        push_topic_capped(&mut state.raw_messages, message.clone(), MAX_RAW_PER_TOPIC);
        message
    }

    pub fn add_telemetry(&self, telemetry: Value) {
        let mut state = self.inner.lock().expect("telemetry state");
        if state.flight_start.is_none() {
            state.flight_start = telemetry.get("timestamp").and_then(Value::as_i64);
        }
        state.latest = Some(telemetry.clone());
        push_capped(&mut state.history, telemetry, MAX_HISTORY);
    }

    pub fn set_antenna(&self, mut antenna: Value) -> Value {
        stamp_received_time(&mut antenna);
        self.inner.lock().expect("telemetry state").antenna = Some(antenna.clone());
        antenna
    }

    pub fn set_ground_imu(&self, mut imu: Value) -> Value {
        stamp_received_time(&mut imu);
        let mut state = self.inner.lock().expect("telemetry state");
        state.ground_imu = Some(imu.clone());
        push_capped(&mut state.ahrs_history, imu.clone(), MAX_AHRS);
        imu
    }

    pub fn set_ahrs_status(&self, status: Value) -> Value {
        let mut status = status;
        stamp_received_time(&mut status);
        self.inner.lock().expect("telemetry state").ahrs_status = Some(status.clone());
        status
    }

    pub fn add_calibration_event(&self, mut event: Value) -> Value {
        stamp_received_time(&mut event);
        push_capped(
            &mut self
                .inner
                .lock()
                .expect("telemetry state")
                .calibration_events,
            event.clone(),
            MAX_CALIBRATION_EVENTS,
        );
        event
    }

    pub fn set_radio_status(&self, mut status: Value) -> Value {
        let id = status
            .get("id")
            .and_then(Value::as_str)
            .unwrap_or("unknown")
            .to_string();
        stamp_received_time(&mut status);
        self.inner
            .lock()
            .expect("telemetry state")
            .radio_statuses
            .insert(id, status.clone());
        status
    }

    pub fn update_node(&self, id: &str, mut node: Value) {
        if let Value::Object(fields) = &mut node {
            fields.insert("id".to_string(), Value::String(id.to_string()));
        }
        self.inner
            .lock()
            .expect("telemetry state")
            .nodes
            .insert(id.to_string(), node);
    }

    pub fn add_raw_imu(&self, sample: Value) {
        push_capped(
            &mut self.inner.lock().expect("telemetry state").raw_imu,
            sample,
            MAX_SENSOR_RAW,
        );
    }

    pub fn add_raw_mag(&self, sample: Value) {
        push_capped(
            &mut self.inner.lock().expect("telemetry state").raw_mag,
            sample,
            MAX_SENSOR_RAW,
        );
    }

    pub fn add_raw_yaw_imu(&self, sample: Value) {
        push_capped(
            &mut self.inner.lock().expect("telemetry state").raw_yaw_imu,
            sample,
            MAX_SENSOR_RAW,
        );
    }

    pub fn set_active_drag(&self, data: &Value) {
        let mut state = self.inner.lock().expect("telemetry state");
        let Some(Value::Object(latest)) = state.latest.as_mut() else {
            return;
        };
        let Some(fields) = data.as_object() else {
            return;
        };
        for (key, value) in fields {
            latest.insert(key.clone(), value.clone());
        }
    }

    pub fn clear_flight(&self) {
        let mut state = self.inner.lock().expect("telemetry state");
        state.latest = None;
        state.history.clear();
        state.flight_start = None;
    }

    pub fn clear_debug(&self) {
        let mut state = self.inner.lock().expect("telemetry state");
        state.log_lines.clear();
        state.raw_messages.clear();
        state.calibration_events.clear();
    }

    pub fn clear_raw_sensors(&self) {
        let mut state = self.inner.lock().expect("telemetry state");
        state.raw_imu.clear();
        state.raw_mag.clear();
        state.raw_yaw_imu.clear();
    }

    pub fn clear_ahrs_history(&self) {
        self.inner
            .lock()
            .expect("telemetry state")
            .ahrs_history
            .clear();
    }
}

pub fn now_ms() -> i64 {
    chrono::Utc::now().timestamp_millis()
}

fn push_capped<T>(items: &mut VecDeque<T>, value: T, limit: usize) {
    while items.len() >= limit {
        items.pop_front();
    }
    items.push_back(value);
}

fn push_topic_capped(items: &mut VecDeque<RawMessage>, value: RawMessage, limit: usize) {
    let topic_count = items
        .iter()
        .filter(|message| message.topic == value.topic)
        .count();
    let mut to_remove = topic_count.saturating_sub(limit - 1);
    if to_remove > 0 {
        items.retain(|message| {
            if message.topic == value.topic && to_remove > 0 {
                to_remove -= 1;
                false
            } else {
                true
            }
        });
    }
    items.push_back(value);
}

fn stamp_received_time(value: &mut Value) {
    if let Value::Object(fields) = value {
        if let Some(timestamp) = fields.get("timestamp").cloned() {
            fields.insert("fw_timestamp".to_string(), timestamp);
        }
        fields.insert("timestamp".to_string(), json!(now_ms()));
    }
}

pub fn object(entries: impl IntoIterator<Item = (&'static str, Value)>) -> Value {
    Value::Object(Map::from_iter(
        entries.into_iter().map(|(k, v)| (k.to_string(), v)),
    ))
}
