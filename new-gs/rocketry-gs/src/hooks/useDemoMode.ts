/**
 * useDemoMode — feeds synthetic telemetry into the store at 10 Hz.
 *
 * Flight profile (repeats every CYCLE_S seconds):
 *   0 –  3 s  motor burn   fast climb, high acceleration
 *   3 – 12 s  coast        deceleration, rotation slows
 *  12 – 14 s  apogee       10,000 ft above pad, near-zero velocity
 *  14 – 28 s  descent      drogue -> main chute, slow fall
 *  28 – 30 s  landed       stationary on ground
 */

import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { useTelemetryStore } from '../store/telemetryStore';
import { displayToNedQuat } from '../utils/attitude';

// -- Launch-pad coordinates (White Sands, NM) --------------------------------
const LAT0 =  32.9503;
const LON0 = -106.9750;
const ALT0 =  1220;   // m MSL (pad elevation)

// Apogee = pad + 10,000 ft (3048 m), so the ground-relative altitude chart
// climbs from 0 to exactly 10,000 ft.
const MAX_ALT   = ALT0 + 3048;   // m MSL at apogee (10,000 ft above the pad)
const CYCLE_S   = 30;     // seconds per simulated flight
const HZ        = 10;
const DT        = 1 / HZ;
const BURN_END_ALT_FRAC = 0.7; // fraction of apogee reached at t=3 s

// Altitude profile — returns MSL altitude for t ∈ [0, CYCLE_S]
function altProfile(t: number): number {
  const deltaAlt = MAX_ALT - ALT0;
  const burnEndAlt = ALT0 + deltaAlt * BURN_END_ALT_FRAC;

  if (t < 3) {
    // Powered ascent: accelerating climb.
    return ALT0 + (burnEndAlt - ALT0) * (t / 3) ** 2;
  }

  if (t < 14) {
    // Coast to apogee: decelerating climb, continuous at t=3 and t=14.
    const u = (t - 3) / 11; // 0 -> 1
    return burnEndAlt + (MAX_ALT - burnEndAlt) * (1 - (1 - u) ** 2);
  }

  if (t < 28)      return ALT0 + (MAX_ALT - ALT0) * Math.max(0, 1 - ((t - 14) / 14) ** 0.6); // descent
  return ALT0;
}

// -- Chase-car offset from pad (degrees) ------------------------------------
const CHASE_DLAT =  0.0012;
const CHASE_DLON = -0.0008;

// Firmware-style flight state across the demo timeline (matches codec
// flight_state_name, mapped to display phases by flightPhase.ts).
function demoState(t: number): string {
  if (t < 0.4)  return 'GROUND_IDLE';     // PAD (brief, at the start of each cycle)
  if (t < 3)    return 'POWERED_ASCENT';  // BOOST
  if (t < 12.5) return 'COAST_ASCENT';    // COAST
  if (t < 14.5) return 'APOGEE';
  if (t < 21)   return 'DESCENT_DROGUE';  // drogue out, body separated
  if (t < 28)   return 'DESCENT_MAIN';    // main out
  return 'LANDED';
}

// Display-frame attitude of one separated half hanging under canopy: a gentle
// pendulum swing + slow spin, optionally flipped nose-down. Each half is given a
// different `seed` so the two transmitters report visibly independent motion.
function hangDisplayQuat(noseDown: boolean, t: number, seed: number): THREE.Quaternion {
  const amp = 0.16;
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(
    Math.sin(t * 1.1 + seed) * amp,   // swing about X
    (t * 0.5 + seed) % (Math.PI * 2), // slow spin about vertical
    Math.cos(t * 0.9 + seed) * amp,   // swing about Z
    'YXZ',
  ));
  if (noseDown) {
    q.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI));
  }
  return q;
}

export function useDemoMode(enabled: boolean) {
  const { addTelemetry, setTx, setAntenna, updateNode, setConnected, clearFlight, setRadioStatus, addPacket } =
    useTelemetryStore();

  const tRef       = useRef(0);
  // Simulate motor lag: actual position lags behind setpoint with a ~1.5 s time constant.
  const actualAzRef = useRef(0);
  const actualElRef = useRef(0);

  useEffect(() => {
    if (!enabled) return;

    clearFlight();
    setConnected(true);
    tRef.current = 0;
    actualAzRef.current = 0;
    actualElRef.current = 0;

    // Seeded horizontal drift direction so the trajectory looks intentional
    const driftBearing = 42; // degrees from north

    const id = setInterval(() => {
      const t = tRef.current % CYCLE_S;
      tRef.current += DT;

      // -- Position -------------------------------------------------------
      const alt  = altProfile(t);
      const frac = Math.min(t, 28) / 28;            // 0->1 over the flight

      // Horizontal drift — monotonic so the trajectory is a true arc, not a loop.
      // Rocket drifts ~700 m downrange by landing.
      const downrange = 700 * frac;
      const bearRad   = (driftBearing * Math.PI) / 180;
      const dlat = (downrange * Math.cos(bearRad)) / 111_320;
      const dlon = (downrange * Math.sin(bearRad)) / (111_320 * Math.cos(LAT0 * Math.PI / 180));
      const lat  = LAT0 + dlat;
      const lon  = LON0 + dlon;

      // -- Velocity (numerical derivative of alt + drift) --------------
      const altNext = altProfile(Math.min(t + DT, 28));
      const vel_d   = -(altNext - alt) / DT;                    // positive = falling

      const fracNext = Math.min(t + DT, 28) / 28;
      const downNext = 700 * fracNext;
      const vel_horiz   = (downNext - downrange) / DT;
      const vel_n       = vel_horiz * Math.cos(bearRad);
      const vel_e       = vel_horiz * Math.sin(bearRad);

      // -- Attitude ----------------------------------------------------
      // During burn: pitched slightly toward drift; during descent: stable
      const pitch = t < 3
        ? 5 + 3 * Math.sin(t * 2)
        : t < 14
          ? 2 + 4 * Math.sin(t * 0.4)
          : 1 * Math.sin(t * 0.2);

      const roll = t < 14
        ? 360 * (t / 14) * 0.5 + 8 * Math.sin(t * 1.3)  // slow spin
        : 5 * Math.sin(t * 0.3);

      const yaw = (driftBearing + 5 * Math.sin(t * 0.7)) % 360;

      // -- Signal ------------------------------------------------------
      const slantRange = Math.sqrt(
        ((lat - LAT0) * 111_320) ** 2 +
        ((lon - LON0) * 111_320 * Math.cos(LAT0 * Math.PI / 180)) ** 2 +
        (alt - ALT0) ** 2,
      );
      const rssi = Math.round(-60 - 20 * Math.log10(Math.max(1, slantRange / 100)));
      const snr  = parseFloat((15 - slantRange / 500).toFixed(1));
      const targetApogee = 3000;
      const predictedApogee = t < 3
        ? 3250
        : t < 14
          ? 3180 - 160 * Math.min(1, (t - 3) / 11)
          : MAX_ALT;
      const deploymentPercent = t < 3
        ? 0
        : t < 12
          ? Math.max(0, Math.min(70, ((predictedApogee - targetApogee) / 220) * 70))
          : 0;
      const flapAngleDeg = deploymentPercent * 0.6;

      addTelemetry({
        timestamp: Date.now(),
        lat, lon, alt_m: alt,
        vel_n, vel_e, vel_d,
        roll, pitch, yaw,
        rssi, snr,
        flap_angle_deg: flapAngleDeg,
        flap_deployment_percent: deploymentPercent,
        predicted_apogee_m: predictedApogee,
        target_apogee_m: targetApogee,
        state: demoState(t),
      });

      // -- Onboard transmitters (915 on nose, 433 on ADS) -----------------
      // On ascent both radios ride the same airframe, so they report the same
      // attitude. After separation each half swings under its own canopy: the
      // nose section falls nose-down, the aft/ADS section motor-down (nose-up).
      const state = demoState(t);
      const separated = state === 'DESCENT_DROGUE' || state === 'DESCENT_MAIN' || state === 'LANDED';
      const rocketQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(
        (pitch * Math.PI) / 180, (yaw * Math.PI) / 180, (roll * Math.PI) / 180, 'XYZ',
      ));
      const noseDisp = separated ? hangDisplayQuat(true,  t, 0.0) : rocketQuat;
      const adsDisp  = separated ? hangDisplayQuat(false, t, 2.3) : rocketQuat;
      const split = separated ? 0.00003 : 0; // ~3 m between the two halves once apart

      setTx({
        id: 'nose', freq_mhz: 915, timestamp: Date.now(),
        quat: displayToNedQuat(noseDisp), have_gps: true,
        lat: lat + split, lon: lon + split, alt_m: alt + (separated ? 6 : 0),
        vel_n, vel_e, vel_d: vel_d + (separated ? -0.6 : 0),
        rssi: rssi + 3, snr,
      });
      setTx({
        id: 'ads', freq_mhz: 433, timestamp: Date.now(),
        quat: displayToNedQuat(adsDisp), have_gps: true,
        lat: lat - split, lon: lon - split, alt_m: alt - (separated ? 6 : 0),
        vel_n, vel_e, vel_d: vel_d + (separated ? 0.6 : 0),
        rssi: rssi - 2, snr: snr - 1,
      });

      // -- Decoded-packet log (one row per radio per tick) ----------------
      const noseStatus = {
        id: 'primary-915', label: 'Primary 915', board: 'primary', radio: 'SX1276',
        freq_mhz: 915, source_topic: 'rocket/lora0', state, timestamp: Date.now(),
        has_gps: true, lat: lat + split, lon: lon + split,
        rssi: rssi + 3, snr,
      };
      const adsStatus = {
        id: 'primary-433', label: 'Primary 433', board: 'primary', radio: 'RF69',
        freq_mhz: 424.5, source_topic: 'rocket/lora1', state, timestamp: Date.now(),
        has_gps: true, lat: lat - split, lon: lon - split,
        rssi: rssi - 2, snr: snr - 1,
      };
      setRadioStatus(noseStatus);  addPacket(noseStatus);
      setRadioStatus(adsStatus);   addPacket(adsStatus);

      // -- Antenna tracking (az/el from GS node position, not pad) ---------
      // The beam is drawn from the GS node, so angles must be referenced there.
      const gsLat = LAT0 + CHASE_DLAT;
      const gsLon = LON0 + CHASE_DLON;
      const dx_m = (lat - gsLat) * 111_320;
      const dy_m = (lon - gsLon) * 111_320 * Math.cos(gsLat * Math.PI / 180);
      const horiz = Math.sqrt(dx_m ** 2 + dy_m ** 2);
      const el    = Math.atan2(alt - ALT0, Math.max(1, horiz)) * (180 / Math.PI);
      const az    = ((Math.atan2(dy_m, dx_m) * (180 / Math.PI)) + 360) % 360;

      // Motor lag: exponential smoothing toward setpoint (~1.5 s time constant)
      const ALPHA = 1 - Math.exp(-DT / 1.5);
      actualAzRef.current += (az - actualAzRef.current) * ALPHA;
      actualElRef.current += (el - actualElRef.current) * ALPHA;

      setAntenna({
        timestamp: Date.now(),
        actual_az: actualAzRef.current,
        actual_el: actualElRef.current,
        target_az: az,
        target_el: el,
      });

      // -- Ground nodes -------------------------------------------------
      updateNode({
        id:        'gs',
        name:      'GS',
        lat:       LAT0 + CHASE_DLAT,
        lon:       LON0 + CHASE_DLON,
        timestamp: Date.now(),
      });
    }, 1000 / HZ);

    return () => {
      clearInterval(id);
      setConnected(false);
    };
  }, [enabled, addTelemetry, setTx, setAntenna, updateNode, setConnected, clearFlight, setRadioStatus, addPacket]);
}
