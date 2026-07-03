use super::codec;
use super::logger::{DiagnosticCsv, PacketLogger};
use super::telemetry::{TelemetryState, UiEvent, EVENT_NAME};
use rumqttc::{AsyncClient, Event, EventLoop, Incoming, MqttOptions, QoS};
use serde_json::{json, Value};
use std::time::Duration;
use tauri::{AppHandle, Emitter, Manager};
use tokio::sync::mpsc;

#[derive(Debug)]
pub struct PublishRequest {
    pub topic: String,
    pub payload: String,
}

pub fn start(
    app: AppHandle,
    state: TelemetryState,
    logger: PacketLogger,
    diagnostic_csv: DiagnosticCsv,
) {
    let host = std::env::var("ROCKETRY_GS_MQTT_HOST").unwrap_or_else(|_| "localhost".to_string());
    let port = std::env::var("ROCKETRY_GS_MQTT_PORT")
        .ok()
        .and_then(|value| value.parse::<u16>().ok())
        .unwrap_or(1883);
    let username = std::env::var("ROCKETRY_GS_MQTT_USER").ok();
    let password = std::env::var("ROCKETRY_GS_MQTT_PASSWORD").ok();

    let (publish_tx, publish_rx) = mpsc::channel::<PublishRequest>(128);
    app.manage(publish_tx);

    tauri::async_runtime::spawn(async move {
        run_mqtt(
            app,
            state,
            logger,
            diagnostic_csv,
            host,
            port,
            username,
            password,
            publish_rx,
        )
        .await;
    });
}

async fn run_mqtt(
    app: AppHandle,
    state: TelemetryState,
    logger: PacketLogger,
    diagnostic_csv: DiagnosticCsv,
    host: String,
    port: u16,
    username: Option<String>,
    password: Option<String>,
    mut publish_rx: mpsc::Receiver<PublishRequest>,
) {
    let client_id = format!("rocketry-gs-{}", std::process::id());
    let mut options = MqttOptions::new(client_id, host.clone(), port);
    options.set_keep_alive(Duration::from_secs(30));
    options.set_clean_session(true);
    if let Some(username) = username {
        options.set_credentials(username, password.unwrap_or_default());
    }

    let (client, mut eventloop) = AsyncClient::new(options, 64);
    if let Err(error) = client.subscribe("#", QoS::AtMostOnce).await {
        emit_log(&app, &state, format!("[mqtt] subscribe failed: {error}"));
    }

    emit_log(&app, &state, format!("[mqtt] connecting to {host}:{port}"));

    loop {
        tokio::select! {
            request = publish_rx.recv() => {
                if let Some(request) = request {
                    publish(&app, &state, &logger, &diagnostic_csv, &client, request).await;
                } else {
                    break;
                }
            }
            event = poll_event(&mut eventloop) => {
                match event {
                    Ok(Event::Incoming(Incoming::ConnAck(_))) => {
                        if state.set_connected(true) {
                            emit(&app, &UiEvent::Connected { connected: true });
                        }
                        emit_log(&app, &state, "[mqtt] connected");
                        diagnostic_csv.log_event("mqtt_connected", "event", "mqtt", "", &state);
                    }
                    Ok(Event::Incoming(Incoming::Publish(packet))) => {
                        handle_publish(&app, &state, &logger, &diagnostic_csv, &packet.topic, packet.payload.as_ref());
                    }
                    Ok(_) => {}
                    Err(error) => {
                        if state.set_connected(false) {
                            emit(&app, &UiEvent::Connected { connected: false });
                        }
                        emit_log(&app, &state, format!("[mqtt] {error}"));
                        diagnostic_csv.log_event(
                            "mqtt_error",
                            "event",
                            "mqtt",
                            &error.to_string(),
                            &state,
                        );
                        tokio::time::sleep(Duration::from_secs(2)).await;
                    }
                }
            }
        }
    }
}

async fn poll_event(eventloop: &mut EventLoop) -> Result<Event, rumqttc::ConnectionError> {
    eventloop.poll().await
}

async fn publish(
    app: &AppHandle,
    state: &TelemetryState,
    logger: &PacketLogger,
    diagnostic_csv: &DiagnosticCsv,
    client: &AsyncClient,
    request: PublishRequest,
) {
    let encoded = codec::encode_command_payload(&request.topic, &request.payload);
    logger.log_packet("tx", &request.topic, &encoded, Some(&request.payload));
    diagnostic_csv.log_event(
        "mqtt_tx_attempt",
        "tx",
        &request.topic,
        &request.payload,
        state,
    );

    match client
        .publish(&request.topic, QoS::AtMostOnce, false, encoded)
        .await
    {
        Ok(()) => {
            diagnostic_csv.log_event("mqtt_tx", "tx", &request.topic, "", state);
        }
        Err(error) => {
            let text = format!("[mqtt] publish failed on {}: {error}", request.topic);
            emit_log(app, state, &text);
            diagnostic_csv.log_event("mqtt_tx_failed", "tx", &request.topic, &text, state);
        }
    }
}

fn handle_publish(
    app: &AppHandle,
    state: &TelemetryState,
    logger: &PacketLogger,
    diagnostic_csv: &DiagnosticCsv,
    topic: &str,
    payload: &[u8],
) {
    let (raw_text, decoded) = codec::decode_to_raw_text(topic, payload);
    logger.log_packet("rx", topic, payload, Some(&raw_text));

    state.add_raw_message(topic, raw_text.clone());

    match topic {
        codec::ROCKET_TELEMETRY => {
            if let Some(Value::Object(_)) = decoded.as_ref() {
                let telemetry = decoded.expect("checked decoded");
                state.add_telemetry(telemetry.clone());
                emit(
                    app,
                    &UiEvent::Telemetry {
                        telemetry: telemetry.clone(),
                    },
                );
                if let Some(status) = radio_status_from_packet(topic, &telemetry) {
                    let status = state.set_radio_status(status);
                    emit(app, &UiEvent::RadioStatus { status });
                }
                maybe_emit_active_drag(app, state, &telemetry);
            }
        }
        codec::ANTENNA_STATE => {
            if let Some(antenna) = decoded {
                let antenna = state.set_antenna(antenna);
                emit(app, &UiEvent::Antenna { antenna });
            }
        }
        codec::GROUND_IMU => {
            if let Some(imu) = decoded {
                let imu = state.set_ground_imu(imu);
                emit(app, &UiEvent::GroundImu { imu });
            }
        }
        codec::AHRS_STATUS => {
            if let Some(status) = decoded {
                let status = state.set_ahrs_status(status);
                emit(app, &UiEvent::AhrsStatus { status });
            }
        }
        codec::CALIBRATION_EVENT => {
            if let Some(event) = decoded {
                let event = state.add_calibration_event(event);
                emit(app, &UiEvent::CalibrationEvent { event });
            }
        }
        codec::RADIO_STATUS => {
            if let Some(status) = decoded {
                let status = state.set_radio_status(status);
                emit(app, &UiEvent::RadioStatus { status });
            }
        }
        codec::RAW_IMU => {
            if let Some(sample) = decoded {
                state.add_raw_imu(sample.clone());
                emit(app, &UiEvent::RawImu { sample });
            }
        }
        codec::RAW_MAG => {
            if let Some(sample) = decoded {
                state.add_raw_mag(sample.clone());
                emit(app, &UiEvent::RawMag { sample });
            }
        }
        codec::RAW_YAW_IMU => {
            if let Some(sample) = decoded {
                state.add_raw_yaw_imu(sample.clone());
                emit(app, &UiEvent::RawYawImu { sample });
            }
        }
        codec::ROCKET_LORA0
        | codec::ROCKET_LORA1
        | codec::ROCKET_LORA1_RF69
        | codec::ROCKET_INTER_PICO => {
            if let Some(data) = decoded.as_ref() {
                if let Some(status) = radio_status_from_packet(topic, data) {
                    let status = state.set_radio_status(status);
                    emit(app, &UiEvent::RadioStatus { status });
                }
                maybe_emit_active_drag(app, state, data);
            }
        }
        codec::GS_LOG => emit_log(app, state, &raw_text),
        topic if topic.starts_with("nodes/") => {
            let id = topic.split('/').nth(1).unwrap_or("unknown");
            if let Some(node) = decoded {
                state.update_node(id, node.clone());
                emit(app, &UiEvent::Node { node });
            }
        }
        _ => {}
    }

    diagnostic_csv.log_event("mqtt_rx", "rx", topic, &raw_text, state);
}

fn radio_status_from_packet(topic: &str, data: &Value) -> Option<Value> {
    let (id, label, board, radio, freq_mhz) = match topic {
        codec::ROCKET_LORA0 => ("primary-915", "Primary 915", "primary", "SX1276", 915.0),
        codec::ROCKET_LORA1 => ("primary-433", "Primary 433", "primary", "RF69", 424.5),
        codec::ROCKET_LORA1_RF69 => ("primary-433", "Primary 433", "primary", "RF69", 424.5),
        codec::ROCKET_INTER_PICO => ("secondary-relay", "Secondary relay", "secondary", "USB relay", 0.0),
        codec::ROCKET_TELEMETRY => (
            "secondary-usb-altitude",
            "USB altitude",
            "secondary",
            "USB relay",
            0.0,
        ),
        _ => return None,
    };

    let lat = data.get("lat").and_then(Value::as_f64);
    let lon = data.get("lon").and_then(Value::as_f64);
    let state = data.get("state").and_then(Value::as_str);
    let has_gps = match (lat, lon) {
        (Some(lat), Some(lon)) => {
            state != Some("BARO_ONLY") && (lat.abs() > 0.000001 || lon.abs() > 0.000001)
        }
        _ => false,
    };
    let has_baro = data.get("alt_baro_m").and_then(Value::as_f64).is_some();
    let has_gps_alt = data.get("alt_gps_m").and_then(Value::as_f64).is_some();

    Some(json!({
        "id": id,
        "label": label,
        "board": board,
        "radio": radio,
        "freq_mhz": freq_mhz,
        "source_topic": topic,
        "state": "rx",
        "packet_count": 1,
        "has_gps": has_gps,
        "has_baro": has_baro,
        "has_gps_alt": has_gps_alt,
        "lat": data.get("lat").cloned(),
        "lon": data.get("lon").cloned(),
        "alt_baro_m": data.get("alt_baro_m").cloned(),
        "alt_gps_m": data.get("alt_gps_m").cloned(),
        "rssi": data.get("rssi").cloned(),
        "snr": data.get("snr").cloned(),
        "len": data.get("len").cloned(),
    }))
}

fn maybe_emit_active_drag(app: &AppHandle, state: &TelemetryState, data: &Value) {
    if let Some(active_drag) = codec::active_drag_from_payload(data) {
        state.set_active_drag(&active_drag);
        emit(app, &UiEvent::ActiveDrag { data: active_drag });
    }
}

fn emit_log(app: &AppHandle, state: &TelemetryState, text: impl Into<String>) {
    let line = state.add_log_line(text.into());
    emit(app, &UiEvent::LogLine { line });
}

fn emit(app: &AppHandle, event: &UiEvent) {
    if let Err(error) = app.emit(EVENT_NAME, event) {
        eprintln!("[tauri-event] {error}");
    }
}
