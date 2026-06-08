/**
 * @file    mc_bezier.c
 * @brief   Cubic Bezier curve evaluation for path planning
 */

#include "mc_math.h"

/**
 * @brief Evaluate cubic Bezier position at parameter t.
 *
 * B(t) = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3
 *
 * P0 is assumed to be origin (0,0). P1, P2, P3 are relative offsets.
 *
 * @param p1x,p1y  Control point 1 (relative to P0)
 * @param p2x,p2y  Control point 2 (relative to P0)
 * @param p3x,p3y  End point (relative to P0)
 * @param t        Parameter in [0, 1]
 * @param out_x    Output: x position
 * @param out_y    Output: y position
 */
void mc_bezier_eval(float p1x, float p1y,
                    float p2x, float p2y,
                    float p3x, float p3y,
                    float t,
                    float *out_x, float *out_y)
{
    float u  = 1.0f - t;
    float u2 = u * u;
    float u3 = u2 * u;
    float t2 = t * t;
    float t3 = t2 * t;
    float c1 = 3.0f * u2 * t;
    float c2 = 3.0f * u * t2;

    /* P0 = (0,0) so that term drops out */
    *out_x = c1 * p1x + c2 * p2x + t3 * p3x;
    *out_y = c1 * p1y + c2 * p2y + t3 * p3y;
}

/**
 * @brief Evaluate cubic Bezier tangent (derivative) at parameter t.
 *
 * B'(t) = 3*(1-t)^2*(P1-P0) + 6*(1-t)*t*(P2-P1) + 3*t^2*(P3-P2)
 *
 * @return Heading angle in degrees (atan2 of tangent)
 */
float mc_bezier_heading(float p1x, float p1y,
                         float p2x, float p2y,
                         float p3x, float p3y,
                         float t)
{
    float u  = 1.0f - t;
    float u2 = u * u;
    float t2 = t * t;

    /* Derivative components (P0 = origin) */
    float dx = 3.0f * u2 * p1x + 6.0f * u * t * (p2x - p1x) + 3.0f * t2 * (p3x - p2x);
    float dy = 3.0f * u2 * p1y + 6.0f * u * t * (p2y - p1y) + 3.0f * t2 * (p3y - p2y);

    return atan2f(dy, dx) * MC_RAD2DEG;
}

/**
 * @brief Approximate arc length of cubic Bezier by chord summation.
 *
 * @param n_segments  Number of chord segments (10-20 is usually enough)
 * @return Approximate total arc length in meters
 */
float mc_bezier_arclen(float p1x, float p1y,
                       float p2x, float p2y,
                       float p3x, float p3y,
                       int n_segments)
{
    float len = 0.0f;
    float prev_x = 0.0f, prev_y = 0.0f;  /* P0 = origin */
    int i;

    for (i = 1; i <= n_segments; i++)
    {
        float t = (float)i / (float)n_segments;
        float x, y;
        mc_bezier_eval(p1x, p1y, p2x, p2y, p3x, p3y, t, &x, &y);
        float dx = x - prev_x;
        float dy = y - prev_y;
        len += sqrtf(dx * dx + dy * dy);
        prev_x = x;
        prev_y = y;
    }
    return len;
}
