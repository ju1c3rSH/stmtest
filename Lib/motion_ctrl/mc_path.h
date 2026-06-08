/**
 * @file    mc_path.h
 * @brief   Path segment descriptors and declarative path executor
 *
 * A path is a const array of mc_seg_t ending with MC_SEG_END.
 * The executor runs segments sequentially, checking completion each tick.
 *
 * Usage:
 *   const mc_seg_t my_path[] = {
 *       { MC_SEG_LINE, MC_CMP_DIST, 0.5f, 0,1,0,0, 0.1f,
 *         {.line = {0.157f, 0.5f, 0.0f}} },
 *       { MC_SEG_ARC, MC_CMP_YAW, 90.0f, 1,0,0,0, 0,
 *         {.arc = {0.157f, 0.2f, 90.0f}} },
 *       { MC_SEG_END }
 *   };
 */

#ifndef __MC_PATH_H
#define __MC_PATH_H

#include <stdint.h>
#include "mc_config.h"

/* ========== Segment Types ========== */

typedef enum
{
    MC_SEG_LINE,            /* straight line */
    MC_SEG_ARC,             /* circular arc */
    MC_SEG_LINE_FOLLOW,     /* follow line sensor */
    MC_SEG_BEZIER,          /* cubic Bezier curve */
    MC_SEG_SPLINE,          /* Catmull-Rom spline */
    MC_SEG_SPIN,            /* rotate in place */
    MC_SEG_WAIT,            /* hold position */
    MC_SEG_END              /* sentinel: end of path */
} mc_seg_type_t;

/* ========== Completion Modes ========== */

typedef enum
{
    MC_CMP_DIST,            /* distance traveled >= value (meters) */
    MC_CMP_YAW,             /* yaw delta >= value (degrees) */
    MC_CMP_TIME,            /* elapsed time >= value (ms) */
    MC_CMP_NONE             /* never auto-complete (external stop only) */
} mc_cmp_mode_t;

/* ========== Segment Parameter Structs ========== */

/** MC_SEG_LINE: drive straight at constant heading */
typedef struct
{
    float speed;            /* forward speed (m/s) */
    float distance;         /* distance to travel (m) */
    float heading;          /* absolute yaw to hold (degrees) */
} mc_seg_line_t;

/** MC_SEG_ARC: circular arc at constant radius */
typedef struct
{
    float speed;            /* tangential speed (m/s) */
    float radius;           /* arc radius (m), positive=left, negative=right */
    float delta_yaw;        /* total yaw change over arc (degrees) */
} mc_seg_arc_t;

/** MC_SEG_LINE_FOLLOW: follow line sensor */
typedef struct
{
    float speed;            /* forward speed (m/s) */
    float distance;         /* distance to follow (m), 0 = use timeout */
    uint32_t timeout_ms;    /* max time (ms), 0 = use distance */
    float yaw_offset;       /* base heading bias (degrees) */
} mc_seg_line_follow_t;

/** MC_SEG_BEZIER: cubic Bezier, P0=current, P1-P3 relative to P0 */
typedef struct
{
    float speed;            /* approximate speed (m/s) */
    float p1x, p1y;        /* control point 1 (m, relative) */
    float p2x, p2y;        /* control point 2 (m, relative) */
    float p3x, p3y;        /* end point (m, relative) */
} mc_seg_bezier_t;

/** MC_SEG_SPLINE: Catmull-Rom through waypoints (world coords) */
typedef struct
{
    float speed;            /* approximate speed (m/s) */
    uint8_t n_pts;          /* number of waypoints */
    float pts[MC_SPLINE_MAX_PTS][2];  /* [x,y] in meters, world frame */
} mc_seg_spline_t;

/** MC_SEG_SPIN: rotate in place to target heading */
typedef struct
{
    float target_heading;   /* absolute heading (degrees) */
    float spin_speed;       /* angular rate (deg/s) */
} mc_seg_spin_t;

/** MC_SEG_WAIT: hold position for duration */
typedef struct
{
    uint32_t duration_ms;   /* hold time (ms) */
} mc_seg_wait_t;

/* ========== Unified Segment Descriptor ========== */

/**
 * @brief Path segment descriptor
 *
 * Declare as const array in flash. ~80 bytes per segment.
 * Use designated initializers for clarity:
 *
 *   { MC_SEG_ARC, MC_CMP_YAW, 90.0f, 1,0,0,0, 0.0f,
 *     {.arc = {0.157f, 0.2f, 90.0f}} }
 */
typedef struct
{
    mc_seg_type_t type;     /* segment type */
    mc_cmp_mode_t cmp;      /* completion mode */
    float cmp_value;        /* completion threshold */

    /* Transition flags (bitfield) */
    uint8_t reset_i   : 1;  /* reset PID integrators on entry */
    uint8_t ramp      : 1;  /* ramp speed from previous segment */
    uint8_t use_line  : 1;  /* enable line-follow PID overlay */
    uint8_t _rsv      : 5;

    float ramp_rate;        /* speed ramp rate (m/s^2), used if ramp=1 */

    /* Type-specific parameters */
    union
    {
        mc_seg_line_t        line;
        mc_seg_arc_t         arc;
        mc_seg_line_follow_t line_follow;
        mc_seg_bezier_t      bezier;
        mc_seg_spline_t      spline;
        mc_seg_spin_t        spin;
        mc_seg_wait_t        wait;
    } p;

} mc_seg_t;

/* ========== Path Executor Context ========== */

typedef enum
{
    MC_PATH_IDLE,           /* not running */
    MC_PATH_TRANSITION,     /* entering new segment */
    MC_PATH_RAMP,           /* speed ramping */
    MC_PATH_RUN,            /* executing segment */
    MC_PATH_DONE            /* all segments complete */
} mc_path_state_t;

typedef struct
{
    const mc_seg_t *path;   /* pointer to const path array */
    uint16_t seg_idx;       /* current segment index */
    mc_path_state_t state;  /* executor state */

    /* Per-segment accumulators (reset on entry) */
    float dist_accum;       /* meters traveled */
    float yaw_accum;        /* degrees of yaw change */
    uint32_t time_accum;    /* ms elapsed */

    /* Segment-entry snapshots */
    float entry_yaw;        /* m_yaw at segment start */
    float entry_pos;        /* m_pos at segment start */

    /* Speed ramp state */
    float current_speed;    /* commanded speed (m/s) */
    float target_speed;     /* segment's target speed (m/s) */

    /* Bezier/spline curve state */
    float t_param;          /* parametric t in [0,1] */
    float curve_len;        /* precomputed arc length */

    /* Yaw blend state */
    float yaw_blend_err;    /* yaw error to blend out */
    int16_t yaw_blend_cnt;  /* blend countdown ticks */

} mc_path_ctx_t;

/* ========== Control Mode ========== */

typedef enum
{
    MC_CTRL_NORMAL = 0,
    MC_CTRL_SPIN,
} mc_ctrl_mode_t;

/* ========== Path Executor Output ========== */

/**
 * @brief Setpoints computed by the path executor each tick
 */
typedef struct
{
    mc_ctrl_mode_t mode;
    float speed_rpm;        /* forward speed in RPM */
    float yaw_set;          /* yaw setpoint in degrees */
    float spin_rpm;         /* direct spin diff RPM (MC_CTRL_SPIN only) */
    uint8_t line_pid_en;    /* 1 = apply line-follow PID overlay */
} mc_path_output_t;

/* ========== API ========== */

/**
 * @brief Initialize path context.
 */
void mc_path_init(mc_path_ctx_t *ctx);

/**
 * @brief Start executing a path from the beginning.
 * @param ctx   Path context
 * @param path  Pointer to const path array (must end with MC_SEG_END)
 */
void mc_path_start(mc_path_ctx_t *ctx, const mc_seg_t *path);

/**
 * @brief Stop path execution (goes to IDLE).
 */
void mc_path_stop(mc_path_ctx_t *ctx);

/**
 * @brief Tick the path executor. Call once per control loop iteration.
 *
 * @param ctx       Path context
 * @param m_yaw     Current multi-turn yaw (degrees)
 * @param m_pos     Current cumulative position (meters)
 * @param wheel_d   Wheel diameter (meters) for speed conversion
 * @param out       Output: setpoints for this tick
 */
void mc_path_tick(mc_path_ctx_t *ctx, float m_yaw, float m_pos,
                  float wheel_d, mc_path_output_t *out);

/**
 * @brief Get current path state.
 */
mc_path_state_t mc_path_get_state(const mc_path_ctx_t *ctx);

/**
 * @brief Get current segment index.
 */
uint16_t mc_path_get_seg_index(const mc_path_ctx_t *ctx);

#endif /* __MC_PATH_H */
