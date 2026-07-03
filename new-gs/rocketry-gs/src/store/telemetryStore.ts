import { create } from 'zustand';
import type {
  RocketTelemetry, AntennaState, MobileNode, GroundImuState, AhrsStatus, CalibrationEvent,
  RawImuSample, RawMagSample, RawYawImuSample, RadioStatus, TxId, TxTelemetry,
} from '../types/telemetry';
import { MAX_HISTORY } from '../config';

const MAX_LOG = 500;
const MAX_RAW = 500;
const MAX_SENSOR_RAW = 500;
const MAX_AHRS = 500;
const MAX_CALIBRATION_EVENTS = 25;
const MAX_PACKET_LOG = 250;

let _seq = 0;

export interface LogLine    { id: number; ts: number; text: string; }
export interface RawMessage { id: number; ts: number; topic: string; payload: string; }
// One row per received & decoded radio packet.
export interface PacketLogEntry { id: number; ts: number; status: RadioStatus; }

function appendCapped<T>(items: T[], next: T, limit: number): T[] {
  return items.length >= limit
    ? [...items.slice(items.length - limit + 1), next]
    : [...items, next];
}

function appendTopicCapped<T extends { topic: string }>(items: T[], next: T, limit: number): T[] {
  const topicCount = items.reduce((count, item) => count + (item.topic === next.topic ? 1 : 0), 0);
  const removeCount = Math.max(0, topicCount - limit + 1);

  if (removeCount === 0) return [...items, next];

  let removed = 0;
  return [
    ...items.filter((item) => {
      if (item.topic !== next.topic || removed >= removeCount) return true;
      removed += 1;
      return false;
    }),
    next,
  ];
}

function capTail<T>(items: T[], limit: number): T[] {
  return items.length > limit ? items.slice(-limit) : items;
}

export interface TelemetryState {
  latest:      RocketTelemetry | null;
  history:     RocketTelemetry[];
  tx:          Record<TxId, TxTelemetry | null>;
  antenna:     AntennaState | null;
  groundImu:   GroundImuState | null;
  ahrsStatus:  AhrsStatus | null;
  nodes:       Record<string, MobileNode>;
  connected:   boolean;
  flightStart: number | null;
  logLines:    LogLine[];
  rawMessages: RawMessage[];
  rawImu:      RawImuSample[];
  rawMag:      RawMagSample[];
  rawYawImu:   RawYawImuSample[];
  ahrsHistory: GroundImuState[];
  calibrationEvents: CalibrationEvent[];
  radioStatuses: Record<string, RadioStatus>;
  packetLog:     PacketLogEntry[];

  addTelemetry: (t: RocketTelemetry) => void;
  setTx:        (t: TxTelemetry) => void;
  setAntenna:   (a: AntennaState) => void;
  setGroundImu: (i: GroundImuState) => void;
  setAhrsStatus: (s: AhrsStatus) => void;
  addCalibrationEvent: (e: CalibrationEvent) => void;
  setRadioStatus: (s: RadioStatus) => void;
  addPacket:    (s: RadioStatus) => void;
  clearPackets: () => void;
  updateNode:   (n: MobileNode) => void;
  setConnected: (v: boolean) => void;
  clearFlight:  () => void;
  loadHistory:  (rows: RocketTelemetry[]) => void;
  addLogLine:   (text: string) => void;
  addRawMessage:(topic: string, payload: string) => void;
  addRawImu:    (s: RawImuSample) => void;
  addRawMag:    (s: RawMagSample) => void;
  addRawYawImu: (s: RawYawImuSample) => void;
  clearRawSensors: () => void;
  clearAhrsHistory: () => void;
  clearDebug:   () => void;
  setActiveDrag: (d: Partial<Pick<RocketTelemetry,
    'flap_angle_deg' | 'flap_deployment_percent' | 'target_apogee_m' | 'predicted_apogee_m'
  >>) => void;
}

export const useTelemetryStore = create<TelemetryState>((set) => ({
  latest:      null,
  history:     [],
  tx:          { nose: null, ads: null },
  antenna:     null,
  groundImu:   null,
  ahrsStatus:  null,
  nodes:       {},
  connected:   false,
  flightStart: null,
  logLines:    [],
  rawMessages: [],
  rawImu:      [],
  rawMag:      [],
  rawYawImu:   [],
  ahrsHistory: [],
  calibrationEvents: [],
  radioStatuses: {},
  packetLog:   [],

  addTelemetry: (t) =>
    set((s) => ({
      latest:      t,
      history:     appendCapped(s.history, t, MAX_HISTORY),
      flightStart: s.flightStart ?? t.timestamp,
    })),

  setTx: (t) => set((s) => ({ tx: { ...s.tx, [t.id]: t } })),

  setAntenna: (a) => set({ antenna: a }),

  setGroundImu: (i) =>
    set((s) => ({
      groundImu: i,
      ahrsHistory: appendCapped(s.ahrsHistory, i, MAX_AHRS),
    })),

  setAhrsStatus: (status) => set({ ahrsStatus: status }),

  addCalibrationEvent: (event) =>
    set((s) => ({
      calibrationEvents: appendCapped(s.calibrationEvents, event, MAX_CALIBRATION_EVENTS),
    })),

  setRadioStatus: (status) =>
    set((s) => ({
      radioStatuses: { ...s.radioStatuses, [status.id]: status },
    })),

  addPacket: (status) =>
    set((s) => ({
      packetLog: appendCapped(
        s.packetLog,
        { id: _seq++, ts: Date.now(), status },
        MAX_PACKET_LOG,
      ),
    })),

  clearPackets: () => set({ packetLog: [] }),

  updateNode: (n) =>
    set((s) => ({ nodes: { ...s.nodes, [n.id]: n } })),

  setConnected: (v) => set({ connected: v }),

  clearFlight: () =>
    set({ history: [], latest: null, flightStart: null, tx: { nose: null, ads: null }, packetLog: [] }),

  loadHistory: (rows) =>
    set(() => {
      const history = capTail(rows, MAX_HISTORY);
      return { history, latest: history.at(-1) ?? null };
    }),

  addLogLine: (text) =>
    set((s) => ({
      logLines: appendCapped(s.logLines, { id: _seq++, ts: Date.now(), text }, MAX_LOG),
    })),

  addRawMessage: (topic, payload) =>
    set((s) => ({
      rawMessages: appendTopicCapped(
        s.rawMessages,
        { id: _seq++, ts: Date.now(), topic, payload },
        MAX_RAW,
      ),
    })),

  addRawImu: (sample) =>
    set((s) => ({ rawImu: appendCapped(s.rawImu, sample, MAX_SENSOR_RAW) })),

  addRawMag: (sample) =>
    set((s) => ({ rawMag: appendCapped(s.rawMag, sample, MAX_SENSOR_RAW) })),

  addRawYawImu: (sample) =>
    set((s) => ({ rawYawImu: appendCapped(s.rawYawImu, sample, MAX_SENSOR_RAW) })),

  clearRawSensors: () => set({ rawImu: [], rawMag: [], rawYawImu: [] }),

  clearAhrsHistory: () => set({ ahrsHistory: [] }),

  clearDebug: () => set({ logLines: [], rawMessages: [], calibrationEvents: [] }),

  setActiveDrag: (d) =>
    set((s) => (s.latest ? { latest: { ...s.latest, ...d } } : {})),
}));
