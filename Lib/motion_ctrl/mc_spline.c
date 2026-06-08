/**
 * @file    mc_spline.c
 * @brief   Catmull-Rom spline evaluation for path planning
 */

#include "mc_math.h"
#include "mc_config.h"

/**
 * @brief Evaluate Catmull-Rom spline at parameter t between points i and i+1.
 *
 * P(t) = 0.5 * [(2*P1) + (-P0+P2)*t + (2*P0-5*P1+4*P2-P3)*t^2
 *              + (-P0+3*P1-3*P2+P3)*t^3]
 *
 * @param pts    Array of [x,y] pairs
 * @param n_pts  Number of points (>= 4)
 * @param i      Index of segment start (0-based, valid: 0..n_pts-4)
 * @param t      Parameter in [0, 1]
 * @param out_x  Output x
 * @param out_y  Output y
 */
void mc_spline_eval(const float pts[][2], int n_pts, int i, float t,
                    float *out_x, float *out_y)
{
    /* Clamp indices to valid range with endpoint extrapolation */
    int i0 = (i > 0) ? i - 1 : 0;
    int i1 = i;
    int i2 = (i + 1 < n_pts) ? i + 1 : n_pts - 1;
    int i3 = (i + 2 < n_pts) ? i + 2 : n_pts - 1;

    float p0x = pts[i0][0], p0y = pts[i0][1];
    float p1x = pts[i1][0], p1y = pts[i1][1];
    float p2x = pts[i2][0], p2y = pts[i2][1];
    float p3x = pts[i3][0], p3y = pts[i3][1];

    float t2 = t * t;
    float t3 = t2 * t;

    *out_x = 0.5f * ((2.0f * p1x)
                    + (-p0x + p2x) * t
                    + (2.0f * p0x - 5.0f * p1x + 4.0f * p2x - p3x) * t2
                    + (-p0x + 3.0f * p1x - 3.0f * p2x + p3x) * t3);

    *out_y = 0.5f * ((2.0f * p1y)
                    + (-p0y + p2y) * t
                    + (2.0f * p0y - 5.0f * p1y + 4.0f * p2y - p3y) * t2
                    + (-p0y + 3.0f * p1y - 3.0f * p2y + p3y) * t3);
}

/**
 * @brief Evaluate Catmull-Rom tangent (derivative) heading at parameter t.
 *
 * @return Heading in degrees
 */
float mc_spline_heading(const float pts[][2], int n_pts, int i, float t)
{
    int i0 = (i > 0) ? i - 1 : 0;
    int i1 = i;
    int i2 = (i + 1 < n_pts) ? i + 1 : n_pts - 1;
    int i3 = (i + 2 < n_pts) ? i + 2 : n_pts - 1;

    float p0x = pts[i0][0], p0y = pts[i0][1];
    float p1x = pts[i1][0], p1y = pts[i1][1];
    float p2x = pts[i2][0], p2y = pts[i2][1];
    float p3x = pts[i3][0], p3y = pts[i3][1];

    float t2 = t * t;

    /* dP/dt = 0.5 * [(-P0+P2) + 2*(2P0-5P1+4P2-P3)*t + 3*(-P0+3P1-3P2+P3)*t^2] */
    float dx = 0.5f * ((-p0x + p2x)
                      + 2.0f * (2.0f * p0x - 5.0f * p1x + 4.0f * p2x - p3x) * t
                      + 3.0f * (-p0x + 3.0f * p1x - 3.0f * p2x + p3x) * t2);

    float dy = 0.5f * ((-p0y + p2y)
                      + 2.0f * (2.0f * p0y - 5.0f * p1y + 4.0f * p2y - p3y) * t
                      + 3.0f * (-p0y + 3.0f * p1y - 3.0f * p2y + p3y) * t2);

    return atan2f(dy, dx) * MC_RAD2DEG;
}

/**
 * @brief Compute approximate arc length of Catmull-Rom spline through all segments.
 *
 * @param pts      Array of [x,y] waypoints
 * @param n_pts    Number of waypoints (>= 2)
 * @param n_chords Chords per segment for approximation
 * @return Total arc length in meters
 */
float mc_spline_arclen(const float pts[][2], int n_pts, int n_chords)
{
    float total = 0.0f;
    int seg;
    int n_segs = n_pts - 1;
    if (n_segs < 1) return 0.0f;

    for (seg = 0; seg < n_segs; seg++)
    {
        float prev_x, prev_y;
        mc_spline_eval(pts, n_pts, seg, 0.0f, &prev_x, &prev_y);

        int j;
        for (j = 1; j <= n_chords; j++)
        {
            float t = (float)j / (float)n_chords;
            float x, y;
            mc_spline_eval(pts, n_pts, seg, t, &x, &y);
            float dx = x - prev_x;
            float dy = y - prev_y;
            total += sqrtf(dx * dx + dy * dy);
            prev_x = x;
            prev_y = y;
        }
    }
    return total;
}
