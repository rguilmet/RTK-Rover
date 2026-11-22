#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Solve an N x N linear system (N <= 9) via Gaussian elimination with partial pivoting.
static inline bool matrix_solve_nxn(int n, double M[9][10], const double b[9], double x[9])
{
    if (n <= 0 || n > 9) {
        return false;
    }
    for (int i = 0; i < n; i++) {
        M[i][n] = b[i];
    }

    for (int i = 0; i < n; i++) {
        int max_row = i;
        double max_val = fabs(M[i][i]);
        for (int k = i + 1; k < n; k++) {
            double v = fabs(M[k][i]);
            if (v > max_val) {
                max_val = v;
                max_row = k;
            }
        }

        if (max_val < 1e-9) {
            M[i][i] += (M[i][i] >= 0.0) ? 1e-6 : -1e-6;
        }

        if (max_row != i) {
            for (int k = i; k <= n; k++) {
                double tmp = M[max_row][k];
                M[max_row][k] = M[i][k];
                M[i][k] = tmp;
            }
        }

        if (fabs(M[i][i]) < 1e-12) {
            return false;
        }

        for (int k = i + 1; k < n; k++) {
            double c = -M[k][i] / M[i][i];
            M[k][i] = 0.0;
            for (int j = i + 1; j <= n; j++) {
                M[k][j] += c * M[i][j];
            }
        }
    }

    for (int i = n - 1; i >= 0; i--) {
        double acc = M[i][n];
        for (int j = i + 1; j < n; j++) {
            acc -= M[i][j] * x[j];
        }
        if (fabs(M[i][i]) < 1e-12) {
            return false;
        }
        x[i] = acc / M[i][i];
        if (!isfinite(x[i])) {
            return false;
        }
    }
    return true;
}

// Helper to solve 9x9 linear system (ellipsoid fitting)
static inline bool solve_9x9(double M[9][10], const double b[9], double x[9])
{
    return matrix_solve_nxn(9, M, b, x);
}

static inline double mat_det_3x3(const double m[3][3])
{
    return m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

static inline bool invert_3x3(const double m[3][3], double inv[3][3])
{
    double det = mat_det_3x3(m);
    if (fabs(det) < 1e-9 || !isfinite(det)) {
        return false;
    }

    double invdet = 1.0 / det;
    inv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
    inv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
    inv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
    inv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
    inv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
    return true;
}

static inline void mat_mult_3x3(const double A[3][3], const double B[3][3], double C[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double s = 0.0;
            for (int k = 0; k < 3; k++) {
                s += A[i][k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
}

static inline void mat_mult_3x3_float(const float A[3][3], const float B[3][3], float C[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += A[i][k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
}

static inline void mat_trans_3x3(const double A[3][3], double B[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            B[i][j] = A[j][i];
        }
    }
}

// Jacobi eigenvalue algorithm for 3x3 symmetric matrix
static inline void jacobi_eigen_3x3(const double A[3][3], double V[3][3], double d[3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            V[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    double B[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            B[i][j] = A[i][j];
        }
    }

    for (int step = 0; step < 50; step++) {
        int p = 0, q = 1;
        double max_val = fabs(B[0][1]);
        if (fabs(B[0][2]) > max_val) { max_val = fabs(B[0][2]); p = 0; q = 2; }
        if (fabs(B[1][2]) > max_val) { max_val = fabs(B[1][2]); p = 1; q = 2; }

        if (max_val < 1e-9) {
            break;
        }

        double theta = 0.5 * atan2(2.0 * B[p][q], B[q][q] - B[p][p]);
        double c = cos(theta);
        double s = sin(theta);

        double tempB_pp = c * c * B[p][p] - 2.0 * s * c * B[p][q] + s * s * B[q][q];
        double tempB_qq = s * s * B[p][p] + 2.0 * s * c * B[p][q] + c * c * B[q][q];

        B[p][q] = B[q][p] = 0.0;
        B[p][p] = tempB_pp;
        B[q][q] = tempB_qq;

        int r = 3 - p - q;
        double tempB_pr = c * B[p][r] - s * B[q][r];
        double tempB_qr = s * B[p][r] + c * B[q][r];
        B[p][r] = B[r][p] = tempB_pr;
        B[q][r] = B[r][q] = tempB_qr;

        for (int i = 0; i < 3; i++) {
            double tempV_ip = c * V[i][p] - s * V[i][q];
            double tempV_iq = s * V[i][p] + c * V[i][q];
            V[i][p] = tempV_ip;
            V[i][q] = tempV_iq;
        }
    }
    for (int i = 0; i < 3; i++) {
        d[i] = B[i][i];
    }
}

static inline float matrix_compute_cal_rms_pct(const float *x,
                                               const float *y,
                                               const float *z,
                                               int count,
                                               const float center[3],
                                               const float soft_iron[3][3])
{
    if (!x || !y || !z || count <= 0) {
        return 100.0f;
    }

    double sum_r = 0.0;
    double sum_r2 = 0.0;
    for (int i = 0; i < count; i++) {
        double dx = (double)x[i] - (double)center[0];
        double dy = (double)y[i] - (double)center[1];
        double dz = (double)z[i] - (double)center[2];
        double cx = dx * soft_iron[0][0] + dy * soft_iron[0][1] + dz * soft_iron[0][2];
        double cy = dx * soft_iron[1][0] + dy * soft_iron[1][1] + dz * soft_iron[1][2];
        double cz = dx * soft_iron[2][0] + dy * soft_iron[2][1] + dz * soft_iron[2][2];
        double r2 = cx * cx + cy * cy + cz * cz;
        double r = sqrt(r2);
        sum_r += r;
        sum_r2 += r2;
    }
    double mean_r = sum_r / (double)count;
    if (mean_r <= 1e-3 || !isfinite(mean_r)) {
        return 100.0f;
    }
    double var_r = (sum_r2 / (double)count) - (mean_r * mean_r);
    if (var_r < 0.0) {
        var_r = 0.0;
    }
    return (float)(100.0 * sqrt(var_r) / mean_r);
}

// 4-parameter hard-iron sphere fit (or 3-parameter horizontal circle fit when Z span is small).
static inline bool fit_sphere_hard_iron_f32(const float *x,
                                            const float *y,
                                            const float *z,
                                            int count,
                                            double mx,
                                            double my,
                                            double mz,
                                            double s_xy,
                                            bool fit_z,
                                            float center[3],
                                            float soft_iron[3][3],
                                            float *rms_pct_out)
{
    if (count < 12 || s_xy < 1.5) {
        return false;
    }

    const int n = fit_z ? 4 : 3;
    double M[9][10] = {0};
    double rhs[9] = {0};
    double sol[9] = {0};

    for (int i = 0; i < count; i++) {
        double u = ((double)x[i] - mx) / s_xy;
        double v = ((double)y[i] - my) / s_xy;
        double w = ((double)z[i] - mz) / s_xy;
        double yi = u * u + v * v + w * w;
        double row[4];
        if (fit_z) {
            row[0] = u;
            row[1] = v;
            row[2] = w;
            row[3] = 1.0;
        } else {
            row[0] = u;
            row[1] = v;
            row[2] = 1.0;
            row[3] = 0.0;
        }
        for (int r = 0; r < n; r++) {
            rhs[r] += row[r] * yi;
            for (int c = 0; c < n; c++) {
                M[r][c] += row[r] * row[c];
            }
        }
    }

    for (int i = 0; i < n; i++) {
        M[i][i] += 1e-5;
    }

    if (!matrix_solve_nxn(n, M, rhs, sol)) {
        return false;
    }

    double cu = 0.5 * sol[0];
    double cv = 0.5 * sol[1];
    double cw = fit_z ? (0.5 * sol[2]) : 0.0;
    double d_term = fit_z ? sol[3] : sol[2];
    double rad2 = d_term + cu * cu + cv * cv + cw * cw;
    if (rad2 <= 1e-4 || !isfinite(rad2)) {
        return false;
    }

    float c_out[3] = {
        (float)(cu * s_xy + mx),
        (float)(cv * s_xy + my),
        fit_z ? (float)(cw * s_xy + mz) : (float)mz
    };
    if (!isfinite(c_out[0]) || !isfinite(c_out[1]) || !isfinite(c_out[2]) ||
        fabsf(c_out[0]) >= 500.0f || fabsf(c_out[1]) >= 500.0f || fabsf(c_out[2]) >= 500.0f) {
        return false;
    }

    float ident[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    float rms_pct = matrix_compute_cal_rms_pct(x, y, z, count, c_out, ident);
    if (!isfinite(rms_pct) || rms_pct > 25.0f) {
        return false;
    }

    center[0] = c_out[0];
    center[1] = c_out[1];
    center[2] = c_out[2];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            soft_iron[i][j] = ident[i][j];
        }
    }
    if (rms_pct_out) {
        *rms_pct_out = rms_pct;
    }
    return true;
}

// Generalized float32 9-parameter 3D ellipsoid fit with flat-spin 4-param sphere fallback.
static inline bool fit_ellipsoid_3d_f32(const float *x,
                                        const float *y,
                                        const float *z,
                                        int count,
                                        float center[3],
                                        float soft_iron[3][3],
                                        float *rms_pct_out)
{
    if (!x || !y || !z || !center || !soft_iron || count < 24) {
        return false;
    }

    double mx = 0.0, my = 0.0, mz = 0.0;
    for (int i = 0; i < count; i++) {
        if (!isfinite(x[i]) || !isfinite(y[i]) || !isfinite(z[i])) {
            return false;
        }
        mx += (double)x[i];
        my += (double)y[i];
        mz += (double)z[i];
    }
    mx /= (double)count;
    my /= (double)count;
    mz /= (double)count;

    double vx = 0.0, vy = 0.0, vz = 0.0;
    for (int i = 0; i < count; i++) {
        double dx = (double)x[i] - mx;
        double dy = (double)y[i] - my;
        double dz = (double)z[i] - mz;
        vx += dx * dx;
        vy += dy * dy;
        vz += dz * dz;
    }
    double sx = sqrt(vx / (double)count);
    double sy = sqrt(vy / (double)count);
    double sz = sqrt(vz / (double)count);

    if (sx < 1.5 || sy < 1.5) {
        return false;
    }

    double s_xy = (sx > sy) ? sx : sy;

    if (sz >= 2.5) {
        double DtD[9][10] = {0};
        double Dt1[9] = {0};

        for (int i = 0; i < count; i++) {
            double nx = ((double)x[i] - mx) / sx;
            double ny = ((double)y[i] - my) / sy;
            double nz = ((double)z[i] - mz) / sz;

            double row[9] = {
                nx * nx, ny * ny, nz * nz,
                2.0 * nx * ny, 2.0 * nx * nz, 2.0 * ny * nz,
                2.0 * nx, 2.0 * ny, 2.0 * nz
            };
            for (int r = 0; r < 9; r++) {
                Dt1[r] += row[r];
                for (int c = 0; c < 9; c++) {
                    DtD[r][c] += row[r] * row[c];
                }
            }
        }

        for (int i = 0; i < 9; i++) {
            DtD[i][i] += 1e-4;
        }

        double v[9] = {0};
        if (solve_9x9(DtD, Dt1, v)) {
            double A[3][3] = {
                {v[0], v[3], v[4]},
                {v[3], v[1], v[5]},
                {v[4], v[5], v[2]}
            };
            double b[3] = {v[6], v[7], v[8]};
            double A_inv[3][3];
            if (invert_3x3(A, A_inv)) {
                double n_center[3];
                for (int i = 0; i < 3; i++) {
                    n_center[i] = -(A_inv[i][0] * b[0] + A_inv[i][1] * b[1] + A_inv[i][2] * b[2]);
                }
                double offset = n_center[0] * b[0] + n_center[1] * b[1] + n_center[2] * b[2];
                double denom = 1.0 - offset;
                if (denom > 1e-6 && isfinite(denom)) {
                    double R[3][3];
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            R[i][j] = A[i][j] / denom;
                        }
                    }

                    double V[3][3], evals[3];
                    jacobi_eigen_3x3(R, V, evals);

                    bool eigs_ok = true;
                    double radii[3];
                    double sum_radii = 0.0;
                    double min_r = 1e9, max_r = 0.0;
                    for (int i = 0; i < 3; i++) {
                        if (!isfinite(evals[i]) || evals[i] <= 1e-6) {
                            eigs_ok = false;
                            break;
                        }
                        radii[i] = 1.0 / sqrt(evals[i]);
                        if (radii[i] < min_r) min_r = radii[i];
                        if (radii[i] > max_r) max_r = radii[i];
                        sum_radii += radii[i];
                    }

                    if (eigs_ok && min_r > 1e-6 && (max_r / min_r) < 3.0) {
                        double target_radius = sum_radii / 3.0;
                        double D_diag[3][3] = {0};
                        for (int i = 0; i < 3; i++) {
                            D_diag[i][i] = target_radius / radii[i];
                        }

                        double temp[3][3], Vt[3][3], nW[3][3], W_raw[3][3];
                        mat_trans_3x3(V, Vt);
                        mat_mult_3x3(V, D_diag, temp);
                        mat_mult_3x3(temp, Vt, nW);

                        double S_inv[3][3] = {
                            {1.0 / sx, 0.0, 0.0},
                            {0.0, 1.0 / sy, 0.0},
                            {0.0, 0.0, 1.0 / sz}
                        };
                        mat_mult_3x3(nW, S_inv, W_raw);

                        double scale = (W_raw[0][0] + W_raw[1][1] + W_raw[2][2]) / 3.0;
                        if (fabs(scale) > 1e-9 && isfinite(scale)) {
                            float c_cand[3] = {
                                (float)(n_center[0] * sx + mx),
                                (float)(n_center[1] * sy + my),
                                (float)(n_center[2] * sz + mz)
                            };
                            double S_dbl[3][3];
                            float s_cand[3][3];
                            bool mat_ok = true;

                            for (int i = 0; i < 3; i++) {
                                if (!isfinite(c_cand[i]) || fabsf(c_cand[i]) >= 500.0f) {
                                    mat_ok = false;
                                }
                                for (int j = 0; j < 3; j++) {
                                    S_dbl[i][j] = W_raw[i][j] / scale;
                                    s_cand[i][j] = (float)S_dbl[i][j];
                                    if (!isfinite(s_cand[i][j])) {
                                        mat_ok = false;
                                    }
                                    if (i == j && (s_cand[i][j] < 0.5f || s_cand[i][j] > 2.0f)) {
                                        mat_ok = false;
                                    }
                                    if (i != j && fabsf(s_cand[i][j]) >= 0.5f) {
                                        mat_ok = false;
                                    }
                                }
                            }

                            double s_det = mat_det_3x3(S_dbl);
                            if (mat_ok && isfinite(s_det) && s_det >= 0.3 && s_det <= 3.0) {
                                float rms_pct = matrix_compute_cal_rms_pct(x, y, z, count, c_cand, s_cand);
                                if (isfinite(rms_pct) && rms_pct <= 25.0f) {
                                    for (int i = 0; i < 3; i++) {
                                        center[i] = c_cand[i];
                                        for (int j = 0; j < 3; j++) {
                                            soft_iron[i][j] = s_cand[i][j];
                                        }
                                    }
                                    if (rms_pct_out) {
                                        *rms_pct_out = rms_pct;
                                    }
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Fallback: flat-spin or ill-conditioned 9-param fit -> 4-param / 3-param hard-iron sphere fit
    return fit_sphere_hard_iron_f32(x, y, z, count, mx, my, mz, s_xy, (sz >= 1.5), center, soft_iron, rms_pct_out);
}

// Compatibility wrapper matching Magnetic_Field_Scanner int32 raw signature
static inline bool get_calibration_matrices(const int32_t *x,
                                            const int32_t *y,
                                            const int32_t *z,
                                            int count,
                                            float center[3],
                                            float soft_iron[3][3])
{
    if (!x || !y || !z || !center || !soft_iron || count < 10) {
        return false;
    }

    double mx = 0.0, my = 0.0, mz = 0.0;
    for (int i = 0; i < count; i++) {
        mx += (double)x[i];
        my += (double)y[i];
        mz += (double)z[i];
    }
    mx /= (double)count;
    my /= (double)count;
    mz /= (double)count;

    double vx = 0.0, vy = 0.0, vz = 0.0;
    for (int i = 0; i < count; i++) {
        double dx = (double)x[i] - mx;
        double dy = (double)y[i] - my;
        double dz = (double)z[i] - mz;
        vx += dx * dx;
        vy += dy * dy;
        vz += dz * dz;
    }
    double sx = sqrt(vx / (double)count);
    double sy = sqrt(vy / (double)count);
    double sz = sqrt(vz / (double)count);
    if (sx < 1.0 || sy < 1.0 || sz < 1.0) {
        return false;
    }

    double DtD[9][10] = {0};
    double Dt1[9] = {0};
    for (int i = 0; i < count; i++) {
        double nx = ((double)x[i] - mx) / sx;
        double ny = ((double)y[i] - my) / sy;
        double nz = ((double)z[i] - mz) / sz;
        double row[9] = {
            nx * nx, ny * ny, nz * nz,
            2.0 * nx * ny, 2.0 * nx * nz, 2.0 * ny * nz,
            2.0 * nx, 2.0 * ny, 2.0 * nz
        };
        for (int r = 0; r < 9; r++) {
            Dt1[r] += row[r];
            for (int c = 0; c < 9; c++) {
                DtD[r][c] += row[r] * row[c];
            }
        }
    }
    for (int i = 0; i < 9; i++) {
        DtD[i][i] += 1e-4;
    }

    double v[9] = {0};
    if (!solve_9x9(DtD, Dt1, v)) {
        return false;
    }

    double A[3][3] = {
        {v[0], v[3], v[4]},
        {v[3], v[1], v[5]},
        {v[4], v[5], v[2]}
    };
    double b[3] = {v[6], v[7], v[8]};
    double A_inv[3][3];
    if (!invert_3x3(A, A_inv)) {
        return false;
    }

    double n_center[3];
    for (int i = 0; i < 3; i++) {
        n_center[i] = -(A_inv[i][0] * b[0] + A_inv[i][1] * b[1] + A_inv[i][2] * b[2]);
    }
    double offset = n_center[0] * b[0] + n_center[1] * b[1] + n_center[2] * b[2];
    double denom = 1.0 - offset;
    if (fabs(denom) < 1e-9) {
        return false;
    }

    double R[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = A[i][j] / denom;
        }
    }

    double V[3][3], evals[3];
    jacobi_eigen_3x3(R, V, evals);

    double radii[3];
    double sum_radii = 0.0;
    for (int i = 0; i < 3; i++) {
        if (fabs(evals[i]) < 1e-9) {
            return false;
        }
        radii[i] = 1.0 / sqrt(fabs(evals[i]));
        sum_radii += radii[i];
    }
    double target_radius = sum_radii / 3.0;

    double D_diag[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        D_diag[i][i] = target_radius / radii[i];
    }

    double temp[3][3], Vt[3][3], nW[3][3], W_raw[3][3];
    mat_trans_3x3(V, Vt);
    mat_mult_3x3(V, D_diag, temp);
    mat_mult_3x3(temp, Vt, nW);

    double S_inv[3][3] = {
        {1.0 / sx, 0.0, 0.0},
        {0.0, 1.0 / sy, 0.0},
        {0.0, 0.0, 1.0 / sz}
    };
    mat_mult_3x3(nW, S_inv, W_raw);

    double scale = (W_raw[0][0] + W_raw[1][1] + W_raw[2][2]) / 3.0;
    if (fabs(scale) < 1e-9) {
        return false;
    }
    center[0] = (float)(n_center[0] * sx + mx);
    center[1] = (float)(n_center[1] * sy + my);
    center[2] = (float)(n_center[2] * sz + mz);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            soft_iron[i][j] = (float)(W_raw[i][j] / scale);
        }
    }
    return true;
}

// Generalized Kabsch 3D rotation alignment operating on pre-calibrated physical unit/uT vectors.
static inline bool kabsch_align_3x3_f32(const float *ref_x,
                                        const float *ref_y,
                                        const float *ref_z,
                                        const float *tip_x,
                                        const float *tip_y,
                                        const float *tip_z,
                                        int count,
                                        const float ref_center[3],
                                        const float ref_soft[3][3],
                                        const float tip_center[3],
                                        const float tip_soft[3][3],
                                        double norm_scale,
                                        float R_out[3][3])
{
    if (!ref_x || !ref_y || !ref_z || !tip_x || !tip_y || !tip_z || count < 3) {
        return false;
    }
    double inv_scale = (fabs(norm_scale) > 1e-9) ? (1.0 / norm_scale) : 1.0;

    double H[3][3] = {0};
    for (int i = 0; i < count; i++) {
        double rx = (double)ref_x[i] - ref_center[0];
        double ry = (double)ref_y[i] - ref_center[1];
        double rz = (double)ref_z[i] - ref_center[2];
        double px = (rx * ref_soft[0][0] + ry * ref_soft[0][1] + rz * ref_soft[0][2]) * inv_scale;
        double py = (rx * ref_soft[1][0] + ry * ref_soft[1][1] + rz * ref_soft[1][2]) * inv_scale;
        double pz = (rx * ref_soft[2][0] + ry * ref_soft[2][1] + rz * ref_soft[2][2]) * inv_scale;

        double tx = (double)tip_x[i] - tip_center[0];
        double ty = (double)tip_y[i] - tip_center[1];
        double tz = (double)tip_z[i] - tip_center[2];
        double qx = (tx * tip_soft[0][0] + ty * tip_soft[0][1] + tz * tip_soft[0][2]) * inv_scale;
        double qy = (tx * tip_soft[1][0] + ty * tip_soft[1][1] + tz * tip_soft[1][2]) * inv_scale;
        double qz = (tx * tip_soft[2][0] + ty * tip_soft[2][1] + tz * tip_soft[2][2]) * inv_scale;

        H[0][0] += px * qx; H[0][1] += px * qy; H[0][2] += px * qz;
        H[1][0] += py * qx; H[1][1] += py * qy; H[1][2] += py * qz;
        H[2][0] += pz * qx; H[2][1] += pz * qy; H[2][2] += pz * qz;
    }

    double Ht[3][3], HtH[3][3];
    mat_trans_3x3(H, Ht);
    mat_mult_3x3(Ht, H, HtH);

    double V[3][3], S2[3];
    jacobi_eigen_3x3(HtH, V, S2);

    double S_inv[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        if (S2[i] > 1e-9) {
            S_inv[i][i] = 1.0 / sqrt(S2[i]);
        }
    }

    double temp[3][3], U[3][3];
    mat_mult_3x3(H, V, temp);
    mat_mult_3x3(temp, S_inv, U);

    double Ut[3][3], R[3][3];
    mat_trans_3x3(U, Ut);
    mat_mult_3x3(V, Ut, R);

    double det = mat_det_3x3(R);
    if (det < 0.0) {
        for (int i = 0; i < 3; i++) {
            V[i][2] = -V[i][2];
        }
        mat_mult_3x3(H, V, temp);
        mat_mult_3x3(temp, S_inv, U);
        mat_trans_3x3(U, Ut);
        mat_mult_3x3(V, Ut, R);
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_out[i][j] = (float)R[i][j];
        }
    }
    return true;
}

#ifdef __cplusplus
}
#endif

#endif // MATRIX_MATH_H
