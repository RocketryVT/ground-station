import { useEffect, useState, useCallback } from 'react';
import { useTelemetryStore } from '../../store/telemetryStore';
import type { AppTab } from '../../App';
import type { RadioStatus } from '../../types/telemetry';
import { formatFeet } from '../../utils/units';
import { phaseFromState, PHASE_LABEL, PHASE_COLOR } from '../../utils/flightPhase';
import styles from './StatusBar.module.css';

function formatElapsed(startMs: number): string {
  const s = Math.floor((Date.now() - startMs) / 1000);
  const m = Math.floor(s / 60);
  const ss = s % 60;
  return `T+${String(m).padStart(2, '0')}:${String(ss).padStart(2, '0')}`;
}

interface Props {
  demo:         boolean;
  tab:          AppTab;
  onToggleDemo: () => void;
  onSetTab:     (t: AppTab) => void;
}

const RADIO_ORDER = [
  { id: 'primary-915', label: 'P915' },
  { id: 'primary-433', label: 'P433' },
  { id: 'secondary-915', label: 'S915' },
  { id: 'secondary-433', label: 'S433' },
] as const;

function radioKind(status?: RadioStatus): 'off' | 'active' | 'ready' | 'stale' | 'fault' {
  if (!status) return 'off';
  const ageMs = Date.now() - status.timestamp;
  if (status.state === 'init_failed' || status.state === 'bad_frame') return 'fault';
  if (status.state === 'ready') return ageMs < 15000 ? 'ready' : 'stale';
  if (status.state === 'rx') return ageMs < 5000 ? 'active' : 'stale';
  return ageMs < 10000 ? 'ready' : 'stale';
}

function radioDataLabel(status?: RadioStatus): string {
  if (!status) return 'off';
  if (status.state === 'init_failed') return 'fail';
  if (status.state === 'bad_frame') return 'bad';
  if (status.state === 'ready') return 'ready';
  if (status.has_gps) return status.has_baro ? 'gps+baro' : 'gps';
  if (status.has_baro) return 'baro';
  if (status.len != null) return `${status.len} B`;
  return status.state ?? 'seen';
}

function radioTitle(status?: RadioStatus): string {
  if (!status) return 'No packets seen';
  const parts = [
    status.label,
    status.radio,
    status.freq_mhz ? `${status.freq_mhz} MHz` : undefined,
    status.rssi != null ? `RSSI ${status.rssi} dBm` : undefined,
    status.snr != null ? `SNR ${status.snr} dB` : undefined,
    status.alt_baro_m != null ? `baro ${status.alt_baro_m.toFixed(1)} m` : undefined,
    status.alt_gps_m != null ? `gps alt ${status.alt_gps_m.toFixed(1)} m` : undefined,
    status.lat != null && status.lon != null ? `${status.lat.toFixed(5)}, ${status.lon.toFixed(5)}` : undefined,
    status.message,
  ].filter(Boolean);
  return parts.join(' | ');
}

export function StatusBar({ demo, tab, onToggleDemo, onSetTab }: Props) {
  const latest = useTelemetryStore((s) => s.latest);
  const antenna = useTelemetryStore((s) => s.antenna);
  const radioStatuses = useTelemetryStore((s) => s.radioStatuses);
  const connected = useTelemetryStore((s) => s.connected);
  const flightStart = useTelemetryStore((s) => s.flightStart);
  const clearFlight = useTelemetryStore((s) => s.clearFlight);
  const [elapsed, setElapsed] = useState('T+00:00');
  const [, setNow] = useState(Date.now());
  const [fullscreen, setFullscreen] = useState(false);

  useEffect(() => {
    const onFsChange = () => setFullscreen(!!document.fullscreenElement);
    document.addEventListener('fullscreenchange', onFsChange);
    return () => document.removeEventListener('fullscreenchange', onFsChange);
  }, []);

  const toggleFullscreen = useCallback(() => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen();
    } else {
      document.exitFullscreen();
    }
  }, []);

  useEffect(() => {
    const id = setInterval(() => {
      setNow(Date.now());
      if (flightStart) setElapsed(formatElapsed(flightStart));
    }, 1000);
    return () => clearInterval(id);
  }, [flightStart]);

  const speed = latest
    ? Math.sqrt(latest.vel_n ** 2 + latest.vel_e ** 2 + latest.vel_d ** 2).toFixed(1)
    : '--';

  const phase = phaseFromState(latest?.state);

  return (
    <div className={styles.bar}>
      <div className={styles.brand}>
        <span className={styles.title}>Ground Station</span>
        <span className={styles.subtitle}>{flightStart ? elapsed : 'Standby'}</span>
      </div>

      <div
        className={styles.phaseField}
        style={{ color: PHASE_COLOR[phase], borderColor: PHASE_COLOR[phase] }}
      >
        <span className={styles.phaseEyebrow}>PHASE</span>
        <strong className={styles.phaseValue}>{PHASE_LABEL[phase]}</strong>
      </div>

      <div className={`${styles.linkPill} ${connected ? styles.linkPillConnected : styles.linkPillOffline}`}>
        <span className={styles.dot} />
        {connected ? 'Connected' : 'No link'}
      </div>

      <div className={styles.radioGroup} aria-label="Radio receiver status">
        {RADIO_ORDER.map((radio) => {
          const status = radioStatuses[radio.id];
          const kind = radioKind(status);
          return (
            <div
              key={radio.id}
              className={`${styles.radioPill} ${styles[`radio_${kind}`]}`}
              title={radioTitle(status)}
            >
              <span>{radio.label}</span>
              <strong>{radioDataLabel(status)}</strong>
            </div>
          );
        })}
      </div>

      <div className={styles.metrics}>
        <div className={styles.metricBlock}>
          <span className={styles.metric}>Altitude</span>
          <strong className={styles.value}>{formatFeet(latest?.alt_m)}</strong>
        </div>
        <div className={styles.metricBlock}>
          <span className={styles.metric}>Speed</span>
          <strong className={styles.value}>{speed !== '--' ? `${speed} m/s` : '--'}</strong>
        </div>
        <div className={styles.metricBlock}>
          <span className={styles.metric}>RSSI</span>
          <strong className={styles.value}>{latest ? `${latest.rssi} dBm` : '--'}</strong>
        </div>
        <div className={styles.metricBlock}>
          <span className={styles.metric}>Antenna</span>
          <strong className={styles.value}>
            {antenna ? `${antenna.actual_az.toFixed(1)}° / ${antenna.actual_el.toFixed(1)}°` : '--'}
          </strong>
        </div>
      </div>

      <span className={styles.spacer} />

      <div className={styles.navGroup}>
        <button
          className={`${styles.tabBtn} ${tab === 'flight' ? styles.tabActive : ''}`}
          onClick={() => onSetTab('flight')}
        >
          Mission
        </button>
        <button
          className={`${styles.tabBtn} ${tab === 'debug' ? styles.tabActive : ''}`}
          onClick={() => onSetTab('debug')}
        >
          Systems
        </button>
      </div>

      <button
        className={`${styles.clearBtn} ${demo ? styles.demoActive : ''}`}
        onClick={onToggleDemo}
      >
        {demo ? 'Demo on' : 'Demo'}
      </button>
      <button className={styles.clearBtn} onClick={clearFlight}>Clear</button>
      <button className={styles.clearBtn} onClick={toggleFullscreen} title="Toggle fullscreen">
        {fullscreen ? 'Exit' : 'Full'}
      </button>
    </div>
  );
}
