import { useMemo } from 'react';
import { useTelemetryStore } from '../../store/telemetryStore';
import type { RadioStatus } from '../../types/telemetry';
import styles from './PacketLog.module.css';

function formatTime(ts: number): string {
  return new Date(ts).toISOString().slice(11, 23); // HH:MM:SS.mmm
}

function fmtNum(v: number | undefined, decimals: number, suffix = ''): string {
  return v == null || !Number.isFinite(v) ? '--' : `${v.toFixed(decimals)}${suffix}`;
}

function radioName(s: RadioStatus): string {
  if (s.label) return s.label;
  const freq = s.freq_mhz ? ` ${s.freq_mhz}` : '';
  return `${s.radio ?? s.id}${freq}`;
}

function fixText(s: RadioStatus): string {
  if (s.has_gps && s.lat != null && s.lon != null) {
    return `${s.lat.toFixed(5)}, ${s.lon.toFixed(5)}`;
  }
  if (s.has_baro) return 'baro only';
  return 'no fix';
}

export function PacketLog() {
  const packetLog    = useTelemetryStore((s) => s.packetLog);
  const clearPackets = useTelemetryStore((s) => s.clearPackets);

  // Newest first.
  const rows = useMemo(() => [...packetLog].reverse(), [packetLog]);

  return (
    <div className={styles.wrapper}>
      <div className={styles.header}>
        <span className={styles.title}>PACKET LOG</span>
        <span className={styles.count}>{packetLog.length}</span>
        <button className={styles.clearBtn} onClick={clearPackets}>Clear</button>
      </div>

      <div className={styles.headRow}>
        <span>TIME</span>
        <span>RADIO</span>
        <span>STATE</span>
        <span className={styles.num}>RSSI</span>
        <span className={styles.num}>SNR</span>
        <span>FIX</span>
      </div>

      <div className={styles.body}>
        {rows.length === 0 ? (
          <div className={styles.empty}>No packets received yet</div>
        ) : (
          rows.map((entry) => (
            <div key={entry.id} className={styles.row}>
              <span className={styles.ts}>{formatTime(entry.ts)}</span>
              <span className={styles.radio}>{radioName(entry.status)}</span>
              <span className={styles.state}>{entry.status.state ?? '--'}</span>
              <span className={styles.num}>{fmtNum(entry.status.rssi, 0)}</span>
              <span className={styles.num}>{fmtNum(entry.status.snr, 1)}</span>
              <span className={styles.fix}>{fixText(entry.status)}</span>
            </div>
          ))
        )}
      </div>
    </div>
  );
}
