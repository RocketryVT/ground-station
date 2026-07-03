#include "fusion_task.hpp"
#include "shared.hpp"
#include "Tasks/MQTT/mqtt_task.hpp"
#include "Tasks/Stepper/stepper_task.hpp"
#include "Proto/mqtt_proto.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pico/time.h"

#include "Fusion.h"

#include <math.h>
#include <stdio.h>

// -----------------------------------------------------------------------------
// Sensor calibration stubs
// Replace with values from your calibration procedure.
// -----------------------------------------------------------------------------

static const FusionMatrix k_gyro_misalign  = {1,0,0, 0,1,0, 0,0,1};
static const FusionVector k_gyro_sens      = {1.0f, 1.0f, 1.0f};
static const FusionVector k_gyro_offset    = {0.0f, 0.0f, 0.0f};

static const FusionMatrix k_accel_misalign = {1,0,0, 0,1,0, 0,0,1};
static const FusionVector k_accel_sens     = {1.0f, 1.0f, 1.0f};
static const FusionVector k_accel_offset   = {0.0f, 0.0f, 0.0f};

// Mag hard/soft-iron calibration.  Mutable at runtime via the mag calibration
// wizard (gs/cmd/mag_cal → fusion_set_mag_calibration).  Guarded by a critical
// section because the fusion task reads them at 100 Hz while MQTT may rewrite them.
//
// Zenith/bar LIS3MDL — default identity until calibrated.
static FusionMatrix k_mag_soft_iron  = {1,0,0, 0,1,0, 0,0,1};
static FusionVector k_mag_hard_iron  = {0.0f, 0.0f, 0.0f};

// Yaw platform LIS3MDL (µT, sensor native frame — applied before the sensor remap).
// Seed values from the OLD mounting (PYNXPZ); stale after the +Y-fwd/+X-right
// remount and meant to be overwritten by a fresh wizard calibration.
static FusionMatrix k_yaw_mag_soft_iron  = {1,0,0, 0,1,0, 0,0,1};
static FusionVector k_yaw_mag_hard_iron  = {0, 0, 0};
static bool         s_yaw_heading_aligned = false;

void fusion_set_mag_calibration( bool yaw, const float hard_iron[3], const float soft_iron[9] )
{
    taskENTER_CRITICAL();
    FusionMatrix* soft = yaw ? &k_yaw_mag_soft_iron : &k_mag_soft_iron;
    FusionVector* hard = yaw ? &k_yaw_mag_hard_iron : &k_mag_hard_iron;
    for ( int i = 0; i < 3; ++i ) hard->array[i] = hard_iron[i];
    for ( int i = 0; i < 9; ++i ) soft->array[i] = soft_iron[i];
    if ( yaw ) s_yaw_heading_aligned = false;
    taskEXIT_CRITICAL();
}

// -----------------------------------------------------------------------------
// Coordinate frames
// -----------------------------------------------------------------------------
//
// E: Earth / fixed pole frame using FusionConventionNed.
// Y: yaw rotating frame after the azimuth servo.  The flat ICM-20948 estimates
//    q_EY.
// B: zenith/elevation bar and antenna frame.  The ISM330DLC + LIS3MDL estimate
//    q_EB.
//
// The AHRS quaternions are local, tilt-stabilised orientation estimates.  They
// intentionally do not take direct magnetometer feedback because nearby metal
// and energized motors can corrupt yaw.  Magnetic data is treated only as a
// gated, slow correction to the scalar true-heading offset below.
//
// q_EY: yaw platform orientation in the local AHRS frame.
// q_EB: bar/zenith orientation in the local AHRS frame.
// q_YB: relative bar orientation, inverse(q_EY) * q_EB.
//
// Scalar yaw fields on gs/pico/primary/imu apply declination + heading offset;
// quaternion fields remain the local diagnostic frames for now.
//
// Change these remaps to match the physical mounting of each sensor.
//
// Fusion remap names list the sensor axes which become body [X,Y,Z].
//
// Yaw board (LSM6DSOX + LIS3MDL): sensor +Y faces body-forward, +X faces
// body-right → body X(fwd)=+sensorY, Y(right)=+sensorX, Z(down)=-sensorZ = PYPXNZ.
//
// Bar/zenith board (ISM330DLC + LIS3MDL), reoriented 2026-06-06: sensor +X faces
// body-forward, +Y faces body-left → body X(fwd)=+sensorX, Y(right)=-sensorY,
// Z(down)=-sensorZ = PXNYNZ. Applies to the ISM accel/gyro and the LIS3MDL mag.

#define BAR_SENSOR_REMAP  FusionRemapAlignmentPXNYNZ  // body [X,Y,Z] = sensor [+X,-Y,-Z]
#define YAW_SENSOR_REMAP  FusionRemapAlignmentPYPXNZ  // body [X,Y,Z] = sensor [+Y,+X,-Z]

// Magnetic declination: true_heading = magnetic_heading + declination_deg.
// Negative = west (Blacksburg VA ≈ -8.53°).  Updated at runtime via gs/cmd/declination.
static volatile float s_declination_deg = 5.46882f;
static volatile float s_heading_offset_deg = 0.0f;

void fusion_set_declination( float deg ) { s_declination_deg = deg; }
float fusion_get_declination()           { return s_declination_deg; }

// -----------------------------------------------------------------------------
// Fusion AHRS settings
// -----------------------------------------------------------------------------

#define FUSION_SAMPLE_RATE_HZ   100
#define FUSION_PERIOD_MS        ( 1000 / FUSION_SAMPLE_RATE_HZ )
#define FUSION_MQTT_INTERVAL_MS 50
#define FUSION_STATUS_INTERVAL_MS 1000
#define FUSION_DEBUG_LOGS       0
#define FUSION_DEBUG_INTERVAL_MS 1000
#define FUSION_RECOVERY_SAMPLES ( 5 * FUSION_SAMPLE_RATE_HZ )

// Magnetometers are useful as a north reference but are not allowed to directly
// pull the local AHRS quaternions.  Nearby steel and energized servos can create
// plausible-looking but wrong fields, so mag data is only used as a slow yaw
// offset trim when the tracker is stationary and the heading residual is small.
#define FUSION_MAG_SETTLE_MS       750
#define FUSION_MAG_MAX_GYRO_DPS    5.0f
#define FUSION_MAG_MIN_FIELD_UT    20.0f
#define FUSION_MAG_MAX_FIELD_UT    75.0f
#define FUSION_MAG_MAX_HEADING_ERR 20.0f
#define FUSION_MAG_OFFSET_STEP_DEG 0.02f
// Scalar compass correction for the yaw-board mounting/body convention.  Keep
// this at zero unless a bench compass check proves the remapped FusionCompass
// heading is consistently offset.
#define FUSION_YAW_COMPASS_OFFSET_DEG 0.0f

static StackType_t  s_stack[ 4096 ];
static StaticTask_t s_tcb;
static FusionAhrs   s_bar_ahrs;
static FusionBias   s_bar_bias;
static FusionAhrs   s_yaw_ahrs;
static FusionBias   s_yaw_bias;

static float wrap_360( float deg )
{
    while ( deg < 0.0f )    deg += 360.0f;
    while ( deg >= 360.0f ) deg -= 360.0f;
    return deg;
}

static float wrap_180( float deg )
{
    while ( deg > 180.0f )   deg -= 360.0f;
    while ( deg <= -180.0f ) deg += 360.0f;
    return deg;
}

static float clamp_float( float v, float min_v, float max_v )
{
    if ( v < min_v ) return min_v;
    if ( v > max_v ) return max_v;
    return v;
}

static float vector_norm( FusionVector v )
{
    return sqrtf( v.axis.x * v.axis.x
                  + v.axis.y * v.axis.y
                  + v.axis.z * v.axis.z );
}

static float gyro_norm_dps( FusionVector gyro )
{
    return vector_norm( gyro );
}

static bool tracker_motion_quiet( uint64_t now_us )
{
    static uint64_t s_last_motion_us = 0;

    StepperStatus az = {};
    StepperStatus el = {};
    const bool have_az = g_stepper_az_status_q &&
        xQueuePeek( g_stepper_az_status_q, &az, 0 ) == pdTRUE;
    const bool have_el = g_stepper_zen_status_q &&
        xQueuePeek( g_stepper_zen_status_q, &el, 0 ) == pdTRUE;

    if ( ( have_az && az.moving ) || ( have_el && el.moving ) ) {
        s_last_motion_us = now_us;
        return false;
    }

    if ( s_last_motion_us == 0 ) return true;
    return ( now_us - s_last_motion_us ) >=
        (uint64_t)FUSION_MAG_SETTLE_MS * 1000ull;
}

void fusion_adjust_heading_offset( float delta_deg )
{
    s_heading_offset_deg = wrap_180( s_heading_offset_deg + delta_deg );
}

float fusion_get_heading_offset() { return s_heading_offset_deg; }

void fusion_reset_heading_alignment()
{
    taskENTER_CRITICAL();
    s_yaw_heading_aligned = false;
    taskEXIT_CRITICAL();
}

static FusionQuaternion quat_conjugate( FusionQuaternion q )
{
    q.element.x = -q.element.x;
    q.element.y = -q.element.y;
    q.element.z = -q.element.z;
    return q;
}

static FusionVector quat_rotate_body_to_reference( FusionQuaternion q, FusionVector body )
{
    return FusionMatrixMultiply( FusionQuaternionToMatrix( q ), body );
}

static FusionVector quat_forward_in_reference( FusionQuaternion q )
{
    return quat_rotate_body_to_reference( q, (FusionVector){ .axis = {1.0f, 0.0f, 0.0f} } );
}

static float heading_from_forward_ned( FusionVector forward )
{
    return FusionRadiansToDegrees( atan2f( forward.axis.y, forward.axis.x ) );
}

static float elevation_from_forward_ned( FusionVector forward )
{
    const float horizontal = sqrtf( forward.axis.x * forward.axis.x
                                   + forward.axis.y * forward.axis.y );
    return FusionRadiansToDegrees( atan2f( -forward.axis.z, horizontal ) );
}

static void fill_quat4( float out[4], FusionQuaternion q )
{
    out[0] = q.element.w;
    out[1] = q.element.x;
    out[2] = q.element.y;
    out[3] = q.element.z;
}

static float sane_delta_t( uint64_t now_us, uint64_t last_us )
{
    float delta_t = (float)( now_us - last_us ) * 1e-6f;
    if ( delta_t < 0.001f || delta_t > 0.1f )
        delta_t = FUSION_PERIOD_MS / 1000.0f;
    return delta_t;
}

static void fusion_publish_status( bool have_bar_imu,
                                   bool have_bar_mag,
                                   bool have_yaw_imu,
                                   bool have_yaw_mag,
                                   uint32_t bar_updates,
                                   uint32_t yaw_updates )
{
    MqttMessage m = {};
    groundstation_AhrsStatus pb = groundstation_AhrsStatus_init_zero;
    pb.has_timestamp = true;    pb.timestamp = time_us_64() / 1000u;
    pb.has_running = true;      pb.running = true;
    pb.has_have_bar_imu = true; pb.have_bar_imu = have_bar_imu;
    pb.has_have_bar_mag = true; pb.have_bar_mag = have_bar_mag;
    pb.has_have_yaw_imu = true; pb.have_yaw_imu = have_yaw_imu;
    pb.has_have_yaw_mag = true; pb.have_yaw_mag = have_yaw_mag;
    pb.has_bar_updates = true;  pb.bar_updates = bar_updates;
    pb.has_yaw_updates = true;  pb.yaw_updates = yaw_updates;

    if ( mqtt_encode_proto( m, "gs/pico/primary/ahrs/status",
                            groundstation_AhrsStatus_fields, &pb ) )
        xQueueSend( g_mqtt_queue, &m, 0 );
}

static void configure_ahrs( FusionAhrs* ahrs, FusionBias* bias )
{
    FusionAhrsInitialise( ahrs );
    FusionBiasInitialise( bias );

    const FusionAhrsSettings ahrs_cfg = {
        .convention            = FusionConventionNed,
        .gain                  = 0.5f,
        .gyroscopeRange        = 2000.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection     = 10.0f,
        .recoveryTriggerPeriod = (unsigned int)FUSION_RECOVERY_SAMPLES,
    };
    FusionAhrsSetSettings( ahrs, &ahrs_cfg );

    const FusionBiasSettings bias_cfg = {
        .sampleRate          = (float)FUSION_SAMPLE_RATE_HZ,
        .stationaryThreshold = 3.0f,
        .stationaryPeriod    = 3.0f,
    };
    FusionBiasSetSettings( bias, &bias_cfg );
}

static bool update_yaw_frame( const YawImuMsg& yaw_imu,
                              bool have_yaw_mag,
                              bool mag_motion_quiet,
                              float delta_t )
{
    FusionVector gyro  = {yaw_imu.gyro[0],  yaw_imu.gyro[1],  yaw_imu.gyro[2]};
    FusionVector accel = {yaw_imu.accel[0], yaw_imu.accel[1], yaw_imu.accel[2]};
    FusionVector mag   = {yaw_imu.mag_ut[0], yaw_imu.mag_ut[1], yaw_imu.mag_ut[2]};

    taskENTER_CRITICAL();
    const FusionMatrix yaw_soft = k_yaw_mag_soft_iron;
    const FusionVector yaw_hard = k_yaw_mag_hard_iron;
    taskEXIT_CRITICAL();
    mag   = FusionModelMagnetic( mag, yaw_soft, yaw_hard );

    gyro  = FusionRemap( gyro,  YAW_SENSOR_REMAP );
    accel = FusionRemap( accel, YAW_SENSOR_REMAP );
    mag   = FusionRemap( mag,   YAW_SENSOR_REMAP );

    gyro = FusionBiasUpdate( &s_yaw_bias, gyro );

    FusionAhrsUpdateNoMagnetometer( &s_yaw_ahrs, gyro, accel, delta_t );

    const float mag_norm_ut = vector_norm( mag );
    const bool mag_field_ok =
        mag_norm_ut >= FUSION_MAG_MIN_FIELD_UT &&
        mag_norm_ut <= FUSION_MAG_MAX_FIELD_UT;
    const bool gyro_quiet = gyro_norm_dps( gyro ) <= FUSION_MAG_MAX_GYRO_DPS;
    const FusionAhrsFlags yaw_flags = FusionAhrsGetFlags( &s_yaw_ahrs );

    if ( !have_yaw_mag || FusionVectorIsZero( mag ) || yaw_flags.startup ||
         !mag_motion_quiet || !gyro_quiet || !mag_field_ok ) {
        return false;
    }

    const FusionVector yaw_forward =
        quat_forward_in_reference( FusionAhrsGetQuaternion( &s_yaw_ahrs ) );
    const float estimated_true_heading =
        wrap_180( heading_from_forward_ned( yaw_forward )
                  + s_declination_deg + s_heading_offset_deg );
    const float mag_true_heading =
        wrap_180( FusionCompass( accel, mag, FusionConventionNed )
                  + FUSION_YAW_COMPASS_OFFSET_DEG
                  + s_declination_deg );
    const float heading_error =
        wrap_180( mag_true_heading - estimated_true_heading );

    if ( !s_yaw_heading_aligned ) {
        fusion_adjust_heading_offset( heading_error );
        s_yaw_heading_aligned = true;
        return true;
    }

    if ( fabsf( heading_error ) > FUSION_MAG_MAX_HEADING_ERR ) {
        return false;
    }

    fusion_adjust_heading_offset(
        clamp_float( heading_error,
                     -FUSION_MAG_OFFSET_STEP_DEG,
                     FUSION_MAG_OFFSET_STEP_DEG ) );
    return true;
}

static void update_bar_frame( const IcmMsg& bar_imu,
                              const MagMsg& bar_mag,
                              bool have_bar_mag,
                              float delta_t )
{
    FusionVector gyro  = {bar_imu.gyro[0],  bar_imu.gyro[1],  bar_imu.gyro[2]};
    FusionVector accel = {bar_imu.accel[0], bar_imu.accel[1], bar_imu.accel[2]};
    FusionVector mag   = {bar_mag.mag[0],   bar_mag.mag[1],   bar_mag.mag[2]};

    gyro  = FusionModelInertial( gyro,  k_gyro_misalign,  k_gyro_sens,  k_gyro_offset  );
    accel = FusionModelInertial( accel, k_accel_misalign, k_accel_sens, k_accel_offset );
    taskENTER_CRITICAL();
    const FusionMatrix bar_soft = k_mag_soft_iron;
    const FusionVector bar_hard = k_mag_hard_iron;
    taskEXIT_CRITICAL();
    mag   = FusionModelMagnetic( mag, bar_soft, bar_hard );

    gyro  = FusionRemap( gyro,  BAR_SENSOR_REMAP );
    accel = FusionRemap( accel, BAR_SENSOR_REMAP );
    mag   = FusionRemap( mag,   BAR_SENSOR_REMAP );

    gyro = FusionBiasUpdate( &s_bar_bias, gyro );

    (void)have_bar_mag;
    (void)mag;
    FusionAhrsUpdateNoMagnetometer( &s_bar_ahrs, gyro, accel, delta_t );
}

static void fusion_task( void* )
{
    vTaskDelay( pdMS_TO_TICKS(500) );

    configure_ahrs( &s_bar_ahrs, &s_bar_bias );
    configure_ahrs( &s_yaw_ahrs, &s_yaw_bias );

    log_print( "[fusion] ground-station AHRS starting at %d Hz\n", FUSION_SAMPLE_RATE_HZ );

    uint64_t   last_bar_ts = 0;
    uint64_t   last_yaw_ts = 0;
    uint64_t   last_bar_update_us = time_us_64();
    uint64_t   last_yaw_update_us = time_us_64();
    TickType_t last_mqtt = xTaskGetTickCount();
    TickType_t last_status = xTaskGetTickCount();
    TickType_t last_debug = xTaskGetTickCount();
    TickType_t last_tick = xTaskGetTickCount();
    uint32_t   bar_update_count = 0;
    uint32_t   yaw_update_count = 0;
    bool       yaw_mag_heading_used = false;

    for ( ;; ) {
        IcmMsg    bar_imu = {};
        MagMsg    bar_mag = {};
        YawImuMsg yaw_imu = {};
        BaroMsg   baro    = {};

        const bool have_bar_imu = ( xQueuePeek( g_icm_q, &bar_imu, 0 ) == pdTRUE ) && bar_imu.valid;
        const bool have_bar_mag = ( xQueuePeek( g_mag_q, &bar_mag, 0 ) == pdTRUE ) && bar_mag.valid;
        const bool have_yaw_imu = ( xQueuePeek( g_yaw_imu_q, &yaw_imu, 0 ) == pdTRUE ) && yaw_imu.valid;
        const bool have_yaw_mag = have_yaw_imu && yaw_imu.mag_valid && !yaw_imu.mag_overflow;
        const bool have_baro    = ( xQueuePeek( g_baro_q, &baro, 0 ) == pdTRUE ) && baro.valid;
        const bool mag_motion_quiet = tracker_motion_quiet( time_us_64() );

        if ( have_yaw_imu && yaw_imu.timestamp_us != last_yaw_ts ) {
            const uint64_t now_us = time_us_64();
            yaw_mag_heading_used =
                update_yaw_frame( yaw_imu, have_yaw_mag, mag_motion_quiet,
                                  sane_delta_t( now_us, last_yaw_update_us ) );
            last_yaw_update_us = now_us;
            last_yaw_ts = yaw_imu.timestamp_us;
            yaw_update_count++;
        }

        if ( have_bar_imu && bar_imu.timestamp_us != last_bar_ts ) {
            const uint64_t now_us = time_us_64();
            update_bar_frame( bar_imu, bar_mag, have_bar_mag, sane_delta_t( now_us, last_bar_update_us ) );
            last_bar_update_us = now_us;
            last_bar_ts = bar_imu.timestamp_us;
            bar_update_count++;
        }

        const FusionQuaternion q_bar_sensor = FusionAhrsGetQuaternion( &s_bar_ahrs ); // q_EB measurement
        const FusionQuaternion q_yaw = FusionAhrsGetQuaternion( &s_yaw_ahrs );        // q_EY
        FusionQuaternion q_yaw_to_bar = FusionQuaternionProduct( quat_conjugate( q_yaw ), q_bar_sensor );
        q_yaw_to_bar = FusionQuaternionNormalise( q_yaw_to_bar );
        FusionQuaternion q_truth = FusionQuaternionProduct( q_yaw, q_yaw_to_bar );    // q_EB
        q_truth = FusionQuaternionNormalise( q_truth );

        const FusionEuler truth_euler = FusionQuaternionToEuler( q_truth );
        const FusionEuler relative_euler = FusionQuaternionToEuler( q_yaw_to_bar );
        const FusionVector truth_forward = quat_forward_in_reference( q_truth );
        const FusionVector yaw_forward = quat_forward_in_reference( q_yaw );
        const FusionVector relative_forward = quat_forward_in_reference( q_yaw_to_bar );
        const float heading_correction =
            s_declination_deg + s_heading_offset_deg;
        const float truth_heading_signed =
            wrap_180( heading_from_forward_ned( truth_forward ) + heading_correction );
        const float yaw_heading_signed =
            wrap_180( heading_from_forward_ned( yaw_forward ) + heading_correction );
        const float relative_heading_signed =
            wrap_180( heading_from_forward_ned( relative_forward ) );
        const float truth_pitch = elevation_from_forward_ned( truth_forward );
        const float relative_pitch = elevation_from_forward_ned( relative_forward );
        const FusionVector earth_accel = FusionAhrsGetEarthAcceleration( &s_bar_ahrs );
        const FusionAhrsFlags bar_flags = FusionAhrsGetFlags( &s_bar_ahrs );
        const FusionAhrsFlags yaw_flags = FusionAhrsGetFlags( &s_yaw_ahrs );

        ImuMsg out = {};
        out.timestamp_us    = time_us_64();
        out.q[0]           = q_truth.element.w;
        out.q[1]           = q_truth.element.x;
        out.q[2]           = q_truth.element.y;
        out.q[3]           = q_truth.element.z;
        out.euler[0]       = truth_euler.angle.roll;
        out.euler[1]       = truth_pitch;
        out.euler[2]       = truth_heading_signed;
        out.earth_accel[0] = earth_accel.axis.x;
        out.earth_accel[1] = earth_accel.axis.y;
        out.earth_accel[2] = earth_accel.axis.z;
        out.baro_alt_m     = have_baro ? baro.alt_m : 0.0f;
        out.temp_c         = have_baro ? baro.temp_c
                                       : ( have_bar_imu ? bar_imu.temp_c : 25.0f );
        out.valid          = have_bar_imu && !bar_flags.startup;
        out.have_yaw_frame = have_yaw_imu && !yaw_flags.startup;
        out.yaw_frame_yaw360 = wrap_360( yaw_heading_signed );
        out.bar_rel_pitch  = relative_pitch;

        xQueueOverwrite( g_imu_q, &out );

        const TickType_t now_ticks = xTaskGetTickCount();
        if ( ( now_ticks - last_status ) >= pdMS_TO_TICKS(FUSION_STATUS_INTERVAL_MS) ) {
            last_status = now_ticks;
            fusion_publish_status( have_bar_imu, have_bar_mag,
                                   have_yaw_imu, have_yaw_mag,
                                   bar_update_count, yaw_update_count );
        }

        if ( FUSION_DEBUG_LOGS &&
             ( now_ticks - last_debug ) >= pdMS_TO_TICKS(FUSION_DEBUG_INTERVAL_MS) ) {
            last_debug = now_ticks;
            log_print( "[ahrs_dbg] imu_pitch=%.2f rel_pitch=%.2f yaw_platform_heading=%.2f yaw_signed=%.2f bar_imu=%u yaw_imu=%u yaw_mag=%u yaw_mag_trim=%u\n",
                       (double)truth_pitch,
                       (double)relative_pitch,
                       (double)wrap_360( yaw_heading_signed ),
                       (double)yaw_heading_signed,
                       have_bar_imu ? 1u : 0u,
                       have_yaw_imu ? 1u : 0u,
                       have_yaw_mag ? 1u : 0u,
                       yaw_mag_heading_used ? 1u : 0u );
            log_print( "[ahrs_q] bar_q=[%.4f,%.4f,%.4f,%.4f] yaw_q=[%.4f,%.4f,%.4f,%.4f] rel_q=[%.4f,%.4f,%.4f,%.4f]\n",
                       (double)q_bar_sensor.element.w,
                       (double)q_bar_sensor.element.x,
                       (double)q_bar_sensor.element.y,
                       (double)q_bar_sensor.element.z,
                       (double)q_yaw.element.w,
                       (double)q_yaw.element.x,
                       (double)q_yaw.element.y,
                       (double)q_yaw.element.z,
                       (double)q_yaw_to_bar.element.w,
                       (double)q_yaw_to_bar.element.x,
                       (double)q_yaw_to_bar.element.y,
                       (double)q_yaw_to_bar.element.z );
        }

        if ( ( now_ticks - last_mqtt ) >= pdMS_TO_TICKS(FUSION_MQTT_INTERVAL_MS) )
        {
            last_mqtt = now_ticks;

            MqttMessage m = {};
            groundstation_GroundImu pb = groundstation_GroundImu_init_zero;
            pb.has_timestamp = true; pb.timestamp = time_us_64() / 1000u;
            pb.has_roll = true;      pb.roll = truth_euler.angle.roll;
            pb.has_pitch = true;     pb.pitch = truth_pitch;
            pb.has_yaw = true;       pb.yaw = truth_heading_signed;
            pb.has_yaw360 = true;    pb.yaw360 = wrap_360( truth_heading_signed );
            pb.q_count = 4;
            fill_quat4( pb.q, q_truth );
            pb.bar_q_count = 4;
            fill_quat4( pb.bar_q, q_bar_sensor );
            pb.yaw_q_count = 4;
            fill_quat4( pb.yaw_q, q_yaw );
            pb.bar_rel_q_count = 4;
            fill_quat4( pb.bar_rel_q, q_yaw_to_bar );
            pb.a_count = 3;
            pb.a[0] = bar_imu.accel[0]; pb.a[1] = bar_imu.accel[1];
            pb.a[2] = bar_imu.accel[2];
            pb.m_count = 3;
            pb.m[0] = bar_mag.mag[0]; pb.m[1] = bar_mag.mag[1];
            pb.m[2] = bar_mag.mag[2];
            pb.has_have_mag = true;       pb.have_mag = have_bar_mag;
            pb.has_have_yaw_frame = true; pb.have_yaw_frame = have_yaw_imu;
            pb.has_yaw_frame_yaw = true;
            pb.yaw_frame_yaw = yaw_heading_signed;
            pb.has_yaw_frame_yaw360 = true;
            pb.yaw_frame_yaw360 = wrap_360( yaw_heading_signed );
            pb.has_bar_rel_roll = true;  pb.bar_rel_roll = relative_euler.angle.roll;
            pb.has_bar_rel_pitch = true; pb.bar_rel_pitch = relative_pitch;
            pb.has_bar_rel_yaw = true;   pb.bar_rel_yaw = relative_heading_signed;
            pb.has_startup = true;       pb.startup = bar_flags.startup;
            pb.has_mag_rec = true;       pb.mag_rec = bar_flags.magneticRecovery;
            pb.has_acc_rec = true;       pb.acc_rec = bar_flags.accelerationRecovery;
            pb.has_yaw_startup = true;   pb.yaw_startup = yaw_flags.startup;
            pb.has_alt_baro = true;      pb.alt_baro = out.baro_alt_m;
            pb.has_temp = true;          pb.temp = out.temp_c;
            pb.has_valid = true;         pb.valid = out.valid;

            if ( mqtt_encode_proto( m, "gs/pico/primary/imu",
                                    groundstation_GroundImu_fields, &pb ) )
                xQueueSend( g_mqtt_queue, &m, 0 );
        }

        vTaskDelayUntil( &last_tick, pdMS_TO_TICKS(FUSION_PERIOD_MS) );
    }
}

void fusion_task_init()
{
    task_create( fusion_task, "fusion", 4096, nullptr, tskIDLE_PRIORITY + 3,
                 s_stack, &s_tcb );
}
