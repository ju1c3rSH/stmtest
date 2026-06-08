/**
 * @file    mc_path.c
 * @brief   Path executor state machine implementation
 */

#include "mc_path.h"
#include "mc_math.h"

/* Forward declarations for curve modules */
extern float mc_bezier_arclen(float p1x, float p1y, float p2x, float p2y,
                               float p3x, float p3y, int n_segments);
extern float mc_bezier_heading(float p1x, float p1y, float p2x, float p2y,
                                float p3x, float p3y, float t);
extern float mc_spline_arclen(const float pts[][2], int n_pts, int n_chords);
extern float mc_spline_heading(const float pts[][2], int n_pts, int i, float t);
extern void  mc_spline_eval(const float pts[][2], int n_pts, int i, float t,
                            float *out_x, float *out_y);

/* ========== Internal helpers ========== */

static float get_seg_speed(const mc_seg_t *seg)
{
    switch (seg->type)
    {
        case MC_SEG_LINE:        return seg->p.line.speed;
        case MC_SEG_ARC:         return seg->p.arc.speed;
        case MC_SEG_LINE_FOLLOW: return seg->p.line_follow.speed;
        case MC_SEG_BEZIER:      return seg->p.bezier.speed;
        case MC_SEG_SPLINE:      return seg->p.spline.speed;
        case MC_SEG_SPIN:        return 0.0f;
        case MC_SEG_WAIT:        return 0.0f;
        default:                 return 0.0f;
    }
}

static uint8_t seg_complete(const mc_path_ctx_t *ctx, const mc_seg_t *seg)
{
    switch (seg->cmp)
    {
        case MC_CMP_DIST:
            return (mc_abs(ctx->dist_accum) >= seg->cmp_value) ? 1 : 0;
        case MC_CMP_YAW:
            return (mc_abs(ctx->yaw_accum) >= seg->cmp_value) ? 1 : 0;
        case MC_CMP_TIME:
            return (ctx->time_accum >= (uint32_t)seg->cmp_value) ? 1 : 0;
        case MC_CMP_NONE:
        default:
            return 0;
    }
}

static void advance_segment(mc_path_ctx_t *ctx)
{
    ctx->seg_idx++;
    if (ctx->path[ctx->seg_idx].type == MC_SEG_END)
    {
        ctx->state = MC_PATH_DONE;
        ctx->current_speed = 0.0f;
    }
    else
    {
        ctx->state = MC_PATH_TRANSITION;
    }
}

/**
 * @brief Precompute curve arc length for Bezier/spline segments.
 */
static void precompute_curve(mc_path_ctx_t *ctx, const mc_seg_t *seg)
{
    if (seg->type == MC_SEG_BEZIER)
    {
        const mc_seg_bezier_t *b = &seg->p.bezier;
        ctx->curve_len = mc_bezier_arclen(b->p1x, b->p1y, b->p2x, b->p2y,
                                           b->p3x, b->p3y, 20);
    }
    else if (seg->type == MC_SEG_SPLINE)
    {
        const mc_seg_spline_t *s = &seg->p.spline;
        ctx->curve_len = mc_spline_arclen(s->pts, s->n_pts, 10);
    }
    else
    {
        ctx->curve_len = 0.0f;
    }
}

/**
 * @brief Compute yaw setpoint from Bezier curve heading.
 */
static void bezier_yaw_setpoint(const mc_seg_t *seg, mc_path_ctx_t *ctx,
                                float *yaw_out)
{
    const mc_seg_bezier_t *b = &seg->p.bezier;
    float t = ctx->t_param;
    if (t > 1.0f) t = 1.0f;

    /* Heading from tangent, relative to entry_yaw */
    float heading = mc_bezier_heading(b->p1x, b->p1y, b->p2x, b->p2y,
                                       b->p3x, b->p3y, t);
    *yaw_out = ctx->entry_yaw + heading;
}

/**
 * @brief Compute yaw setpoint from spline curve heading.
 */
static void spline_yaw_setpoint(const mc_seg_t *seg, mc_path_ctx_t *ctx,
                                float *yaw_out)
{
    const mc_seg_spline_t *s = &seg->p.spline;
    int n_segs = (int)s->n_pts - 1;
    if (n_segs < 1) { *yaw_out = ctx->entry_yaw; return; }

    /* Map t_param to segment index + local t */
    float total_t = ctx->t_param * (float)n_segs;
    int seg_i = (int)total_t;
    float local_t = total_t - (float)seg_i;
    if (seg_i >= n_segs) { seg_i = n_segs - 1; local_t = 1.0f; }

    float heading = mc_spline_heading(s->pts, (int)s->n_pts, seg_i, local_t);
    *yaw_out = ctx->entry_yaw + heading;
}

/* ========== Public API ========== */

void mc_path_init(mc_path_ctx_t *ctx)
{
    ctx->path = (void *)0;
    ctx->seg_idx = 0;
    ctx->state = MC_PATH_IDLE;
    ctx->dist_accum = 0.0f;
    ctx->yaw_accum = 0.0f;
    ctx->time_accum = 0;
    ctx->entry_yaw = 0.0f;
    ctx->entry_pos = 0.0f;
    ctx->current_speed = 0.0f;
    ctx->target_speed = 0.0f;
    ctx->t_param = 0.0f;
    ctx->curve_len = 0.0f;
    ctx->yaw_blend_err = 0.0f;
    ctx->yaw_blend_cnt = 0;
}

void mc_path_start(mc_path_ctx_t *ctx, const mc_seg_t *path)
{
    ctx->path = path;
    ctx->seg_idx = 0;
    ctx->state = MC_PATH_TRANSITION;
    ctx->dist_accum = 0.0f;
    ctx->yaw_accum = 0.0f;
    ctx->time_accum = 0;
    ctx->current_speed = 0.0f;
    ctx->yaw_blend_err = 0.0f;
    ctx->yaw_blend_cnt = 0;
}

void mc_path_stop(mc_path_ctx_t *ctx)
{
    ctx->state = MC_PATH_IDLE;
    ctx->current_speed = 0.0f;
}

mc_path_state_t mc_path_get_state(const mc_path_ctx_t *ctx)
{
    return ctx->state;
}

uint16_t mc_path_get_seg_index(const mc_path_ctx_t *ctx)
{
    return ctx->seg_idx;
}

void mc_path_tick(mc_path_ctx_t *ctx, float m_yaw, float m_pos,
                  float wheel_d, mc_path_output_t *out)
{
    /* Default: zero output */
    out->mode = MC_CTRL_NORMAL;
    out->speed_rpm = 0.0f;
    out->yaw_set = m_yaw;
    out->spin_rpm = 0.0f;
    out->line_pid_en = 0;

    if (ctx->state == MC_PATH_IDLE || ctx->state == MC_PATH_DONE)
        return;

    if (!ctx->path)
        return;

    const mc_seg_t *seg = &ctx->path[ctx->seg_idx];

    switch (ctx->state)
    {
    /* ---- TRANSITION: snapshot entry state, prepare segment ---- */
    case MC_PATH_TRANSITION:
    {
        ctx->entry_yaw = m_yaw;
        ctx->entry_pos = m_pos;
        ctx->dist_accum = 0.0f;
        ctx->yaw_accum = 0.0f;
        ctx->time_accum = 0;
        ctx->t_param = 0.0f;
        ctx->yaw_blend_err = 0.0f;
        ctx->yaw_blend_cnt = 0;

        ctx->target_speed = get_seg_speed(seg);

        /* Precompute curve length for Bezier/spline */
        precompute_curve(ctx, seg);

        if (seg->ramp)
        {
            ctx->state = MC_PATH_RAMP;
        }
        else
        {
            ctx->current_speed = ctx->target_speed;
            ctx->state = MC_PATH_RUN;
        }
        break;
    }

    /* ---- RAMP: linearly interpolate speed ---- */
    case MC_PATH_RAMP:
    {
        float dt = MC_CONTROL_PERIOD_S;
        float rate = (seg->ramp_rate > 0.0f) ? seg->ramp_rate : 1.0f;

        if (ctx->current_speed < ctx->target_speed)
        {
            ctx->current_speed += rate * dt;
            if (ctx->current_speed >= ctx->target_speed)
                ctx->current_speed = ctx->target_speed;
        }
        else if (ctx->current_speed > ctx->target_speed)
        {
            ctx->current_speed -= rate * dt;
            if (ctx->current_speed <= ctx->target_speed)
                ctx->current_speed = ctx->target_speed;
        }

        ctx->state = MC_PATH_RUN;
        /* fall through to RUN */
    }

    /* ---- RUN: compute setpoints, check completion ---- */
    case MC_PATH_RUN:
    {
        /* Update accumulators */
        ctx->dist_accum = m_pos - ctx->entry_pos;
        ctx->yaw_accum = m_yaw - ctx->entry_yaw;
        ctx->time_accum++;

        /* Convert current speed to RPM */
        float speed_rpm = mc_ms_to_rpm(ctx->current_speed, wheel_d);
        out->speed_rpm = speed_rpm;

        /* Compute setpoints per segment type */
        switch (seg->type)
        {
        case MC_SEG_LINE:
            out->yaw_set = seg->p.line.heading;
            break;

        case MC_SEG_ARC:
        {
            float r = seg->p.arc.radius;
            if (mc_abs(r) > 0.001f)
            {
                float yaw_from_dist = (ctx->dist_accum / r) * MC_RAD2DEG;
                out->yaw_set = ctx->entry_yaw + yaw_from_dist;
            }
            break;
        }

        case MC_SEG_LINE_FOLLOW:
            /* Yaw setpoint stays at entry; line PID overlays externally */
            out->yaw_set = ctx->entry_yaw;
            out->line_pid_en = 1;
            break;

        case MC_SEG_BEZIER:
        {
            if (ctx->curve_len > 0.001f)
            {
                ctx->t_param = mc_abs(ctx->dist_accum) / ctx->curve_len;
                if (ctx->t_param > 1.0f) ctx->t_param = 1.0f;
            }
            bezier_yaw_setpoint(seg, ctx, &out->yaw_set);
            break;
        }

        case MC_SEG_SPLINE:
        {
            if (ctx->curve_len > 0.001f)
            {
                ctx->t_param = mc_abs(ctx->dist_accum) / ctx->curve_len;
                if (ctx->t_param > 1.0f) ctx->t_param = 1.0f;
            }
            spline_yaw_setpoint(seg, ctx, &out->yaw_set);
            break;
        }

        case MC_SEG_SPIN:
        {
            out->mode = MC_CTRL_SPIN;
            float remaining = seg->p.spin.target_heading - m_yaw;
            float step = seg->p.spin.spin_speed * MC_CONTROL_PERIOD_S;
            if (mc_abs(remaining) < step)
            {
                out->spin_rpm = 0.0f;
                out->yaw_set = seg->p.spin.target_heading;
                advance_segment(ctx);
            }
            else
            {
                float base_rpm = seg->p.spin.spin_speed * MC_WHEEL_BASE / (6.0f * wheel_d);
                out->spin_rpm = base_rpm * mc_sign(remaining);
                out->yaw_set = seg->p.spin.target_heading;
            }
            break;
        }

        case MC_SEG_WAIT:
            out->speed_rpm = 0.0f;
            out->yaw_set = ctx->entry_yaw;
            break;

        default:
            break;
        }

        /* Yaw blend for smooth transitions */
        if (ctx->yaw_blend_cnt > 0)
        {
            float frac = (float)ctx->yaw_blend_cnt / (float)MC_YAW_BLEND_MS;
            out->yaw_set -= ctx->yaw_blend_err * frac;
            ctx->yaw_blend_cnt--;
        }

        /* Check completion */
        if (seg_complete(ctx, seg))
        {
            advance_segment(ctx);
        }
        break;
    }

    default:
        break;
    }
}
