import { useEffect, useRef } from 'react';
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { useTelemetryStore, type LogLine, type RawMessage, type PacketLogEntry, type TelemetryState } from '../store/telemetryStore';
import type {
  AntennaState,
  AhrsStatus,
  CalibrationEvent,
  GroundImuState,
  MobileNode,
  RawImuSample,
  RawMagSample,
  RawYawImuSample,
  RadioStatus,
  RocketTelemetry,
} from '../types/telemetry';

export interface MQTTHandle {
  publish: (topic: string, payload: string | Uint8Array) => void;
}

interface TelemetrySnapshot {
  latest: RocketTelemetry | null;
  history: RocketTelemetry[];
  antenna: AntennaState | null;
  groundImu: GroundImuState | null;
  ahrsStatus: AhrsStatus | null;
  nodes: Record<string, MobileNode>;
  connected: boolean;
  flightStart: number | null;
  logLines: LogLine[];
  rawMessages: RawMessage[];
  rawImu: RawImuSample[];
  rawMag: RawMagSample[];
  rawYawImu: RawYawImuSample[];
  ahrsHistory: GroundImuState[];
  calibrationEvents: CalibrationEvent[];
  radioStatuses: Record<string, RadioStatus>;
}

type TelemetryEvent =
  | { kind: 'connected'; connected: boolean }
  | { kind: 'log_line'; line: LogLine }
  | { kind: 'telemetry'; telemetry: RocketTelemetry }
  | { kind: 'antenna'; antenna: AntennaState }
  | { kind: 'ground_imu'; imu: GroundImuState }
  | { kind: 'ahrs_status'; status: AhrsStatus }
  | { kind: 'calibration_event'; event: CalibrationEvent }
  | { kind: 'radio_status'; status: RadioStatus }
  | { kind: 'node'; node: MobileNode }
  | { kind: 'raw_imu'; sample: RawImuSample }
  | { kind: 'raw_mag'; sample: RawMagSample }
  | { kind: 'raw_yaw_imu'; sample: RawYawImuSample }
  | { kind: 'active_drag'; data: Partial<RocketTelemetry> };

const EVENT_NAME = 'telemetry://event';
const UI_FLUSH_INTERVAL_MS = 50;
const MAX_LOG = 500;
const MAX_SENSOR_RAW = 500;
const MAX_HISTORY = 500;
const MAX_AHRS = 500;
const MAX_CALIBRATION_EVENTS = 25;
const MAX_PACKET_LOG = 250;

function appendCapped<T>(items: T[], next: T, limit: number): T[] {
  return items.length >= limit
    ? [...items.slice(items.length - limit + 1), next]
    : [...items, next];
}

// Single source of unique ids for packet-log rows (used by both the single
// and batched event paths so React keys never collide).
let _packetSeq = 0;
function makePacketEntry(status: PacketLogEntry['status']): PacketLogEntry {
  return { id: _packetSeq++, ts: Date.now(), status };
}

function applySnapshot(snapshot: TelemetrySnapshot) {
  useTelemetryStore.setState({
    latest: snapshot.latest,
    history: snapshot.history ?? [],
    antenna: snapshot.antenna,
    groundImu: snapshot.groundImu,
    ahrsStatus: snapshot.ahrsStatus,
    nodes: snapshot.nodes ?? {},
    connected: snapshot.connected,
    flightStart: snapshot.flightStart,
    logLines: snapshot.logLines ?? [],
    rawMessages: [],
    rawImu: snapshot.rawImu ?? [],
    rawMag: snapshot.rawMag ?? [],
    rawYawImu: snapshot.rawYawImu ?? [],
    ahrsHistory: snapshot.ahrsHistory ?? [],
    calibrationEvents: snapshot.calibrationEvents ?? [],
    radioStatuses: snapshot.radioStatuses ?? {},
    packetLog: [],
  });
}

function applyEvent(event: TelemetryEvent) {
  const store = useTelemetryStore.getState();

  switch (event.kind) {
    case 'connected':
      store.setConnected(event.connected);
      break;
    case 'log_line':
      useTelemetryStore.setState((state) => ({
        logLines: [...state.logLines.slice(-499), event.line],
      }));
      break;
    case 'telemetry':
      store.addTelemetry(event.telemetry);
      break;
    case 'antenna':
      store.setAntenna(event.antenna);
      break;
    case 'ground_imu':
      store.setGroundImu(event.imu);
      break;
    case 'ahrs_status':
      store.setAhrsStatus(event.status);
      break;
    case 'calibration_event':
      store.addCalibrationEvent(event.event);
      break;
    case 'radio_status': {
      store.setRadioStatus(event.status);
      const entry = makePacketEntry(event.status);
      useTelemetryStore.setState((s) => ({
        packetLog: appendCapped(s.packetLog, entry, MAX_PACKET_LOG),
      }));
      break;
    }
    case 'node':
      store.updateNode(event.node);
      break;
    case 'raw_imu':
      store.addRawImu(event.sample);
      break;
    case 'raw_mag':
      store.addRawMag(event.sample);
      break;
    case 'raw_yaw_imu':
      store.addRawYawImu(event.sample);
      break;
    case 'active_drag':
      store.setActiveDrag(event.data);
      break;
  }
}

function applyEvents(events: TelemetryEvent[]) {
  if (events.length === 0) return;
  if (events.length === 1) {
    applyEvent(events[0]);
    return;
  }

  useTelemetryStore.setState((state) => {
    let latest = state.latest;
    let history = state.history;
    let antenna = state.antenna;
    let groundImu = state.groundImu;
    let ahrsStatus = state.ahrsStatus;
    let nodes = state.nodes;
    let connected = state.connected;
    let flightStart = state.flightStart;
    let logLines = state.logLines;
    let rawMessages = state.rawMessages;
    let rawImu = state.rawImu;
    let rawMag = state.rawMag;
    let rawYawImu = state.rawYawImu;
    let ahrsHistory = state.ahrsHistory;
    let calibrationEvents = state.calibrationEvents;
    let radioStatuses = state.radioStatuses;
    let packetLog = state.packetLog;

    for (const event of events) {
      switch (event.kind) {
        case 'connected':
          connected = event.connected;
          break;
        case 'log_line':
          logLines = appendCapped(logLines, event.line, MAX_LOG);
          break;
        case 'telemetry':
          latest = event.telemetry;
          history = appendCapped(history, event.telemetry, MAX_HISTORY);
          flightStart = flightStart ?? event.telemetry.timestamp;
          break;
        case 'antenna':
          antenna = event.antenna;
          break;
        case 'ground_imu':
          groundImu = event.imu;
          ahrsHistory = appendCapped(ahrsHistory, event.imu, MAX_AHRS);
          break;
        case 'ahrs_status':
          ahrsStatus = event.status;
          break;
        case 'calibration_event':
          calibrationEvents = appendCapped(calibrationEvents, event.event, MAX_CALIBRATION_EVENTS);
          break;
        case 'radio_status':
          if (radioStatuses === state.radioStatuses) radioStatuses = { ...radioStatuses };
          radioStatuses[event.status.id] = event.status;
          packetLog = appendCapped(packetLog, makePacketEntry(event.status), MAX_PACKET_LOG);
          break;
        case 'node':
          if (nodes === state.nodes) nodes = { ...nodes };
          nodes[event.node.id] = event.node;
          break;
        case 'raw_imu':
          rawImu = appendCapped(rawImu, event.sample, MAX_SENSOR_RAW);
          break;
        case 'raw_mag':
          rawMag = appendCapped(rawMag, event.sample, MAX_SENSOR_RAW);
          break;
        case 'raw_yaw_imu':
          rawYawImu = appendCapped(rawYawImu, event.sample, MAX_SENSOR_RAW);
          break;
        case 'active_drag':
          if (latest) latest = { ...latest, ...event.data };
          break;
      }
    }

    return {
      latest,
      history,
      antenna,
      groundImu,
      ahrsStatus,
      nodes,
      connected,
      flightStart,
      logLines,
      rawMessages,
      rawImu,
      rawMag,
      rawYawImu,
      ahrsHistory,
      calibrationEvents,
      radioStatuses,
      packetLog,
    } satisfies Partial<TelemetryState>;
  });
}

function payloadToString(payload: string | Uint8Array): string {
  if (typeof payload === 'string') return payload;
  return Array.from(payload, (byte) => String.fromCharCode(byte)).join('');
}

export function useMQTT(enabled = true): MQTTHandle {
  const enabledRef = useRef(enabled);
  enabledRef.current = enabled;

  useEffect(() => {
    let unlisten: UnlistenFn | null = null;
    let cancelled = false;
    let flushTimer: number | null = null;
    const pendingEvents: TelemetryEvent[] = [];

    const flushEvents = () => {
      flushTimer = null;
      if (pendingEvents.length === 0) return;
      const events = pendingEvents.splice(0, pendingEvents.length);
      applyEvents(events);
    };

    const enqueueEvent = (event: TelemetryEvent) => {
      pendingEvents.push(event);
      if (flushTimer == null) {
        flushTimer = window.setTimeout(flushEvents, UI_FLUSH_INTERVAL_MS);
      }
    };

    invoke<TelemetrySnapshot>('get_telemetry_snapshot')
      .then((snapshot) => {
        if (!cancelled && enabledRef.current) applySnapshot(snapshot);
      })
      .catch((error) => {
        useTelemetryStore.getState().addLogLine(`[tauri] snapshot failed: ${String(error)}`);
      });

    listen<TelemetryEvent>(EVENT_NAME, ({ payload }) => {
      if (enabledRef.current) enqueueEvent(payload);
    })
      .then((fn) => {
        if (cancelled) fn();
        else unlisten = fn;
      })
      .catch((error) => {
        useTelemetryStore.getState().addLogLine(`[tauri] event bridge failed: ${String(error)}`);
      });

    return () => {
      cancelled = true;
      if (flushTimer != null) window.clearTimeout(flushTimer);
      pendingEvents.length = 0;
      unlisten?.();
    };
  }, []);

  return {
    publish: (topic, payload) => {
      invoke('publish_mqtt', { topic, payload: payloadToString(payload) }).catch((error) => {
        useTelemetryStore.getState().addLogLine(`[mqtt] publish failed: ${String(error)}`);
      });
    },
  };
}
