#ifndef SETTINGS_H
#define SETTINGS_H

/** @brief Number of planned waypoints. */
#define N_PLAN 50

/** @brief Number of planned waypoints to keep between updates. */
#define N_KEEP 10

/** @brief Time gap between waypoints. */
#define T_PLAN 0.02

/** @brief Optimal speed in m/s. */
#define V_PLAN 22.0

/** @brief Order of the polynomial used to fit waypoints. */
#define N_FIT 3

/** @brief Number of sample waypoints extracted from the lane map. */
#define N_SAMPLES 2

#endif
