#ifndef PDL_NAV_KF_H
#define PDL_NAV_KF_H

/** @page nav_kf Navigation Kalman Filter
 *
 *  PDL implements linear Kalman Filter to estimate velocity, acceleration and position
 *  along one axis. This way allows to keep memory resources. If to construct Kalman filter
 *  to estimate state along 3 axis there to be redundant nulls in covariance matrices
 *  because we assume there is no correlation between x and z movements for example.
 *  This filter uses constant acceleration process model to predict next state.
 *  It allows to correct state by accelerometer, gps, baro, optical flow sensor measurements
 *  and other sensors
 *
 *  For more information about Kalman Filter visit https://www.kalmanfilter.net
 */

///! Size of state vector (number of estimated parameters by Kalman Filter)
#define PDL_NAV_KF_STATE_SIZE   3

typedef struct
{
  ///! Position in meters
  float pos;
  ///! Velocity in m/s
  float vel;
  ///! Acceleration in m/s^2
  float acc;
  /// Constant accel offset. This value subtracted from each accelerometer measure. You can measure it before start process
  float accBias;
} pdlNavState;

typedef struct
{
  ///! The estimate uncertainty (covariance) matrix of the current state
  float P[PDL_NAV_KF_STATE_SIZE][PDL_NAV_KF_STATE_SIZE];
  ///! Random variance in process model
  float Q_sigma;
} pdlNavKF;

/** Inits Kalman filter
 *
 * @param pKf             Pointer to Kalman filter object
 * @param pState          Pointer to last state
 * @param processSigma    Random variance in process model. It is a magic parameter and
 *                        should to be determined by experiments.
 *                        It is recommended to start with 0.005
 *                        If this value is too large, the Kalman Filter will
 *                        follow the measurements and produce noisy estimations.
 *                        If this value to small there be lag between new measurements and estimated state
 */
void pdlNavKF_Init(pdlNavKF *pKf, pdlNavState *pState, float processSigma);

/** Predicts new state with const acceleration model, invoke this function with constant period.
 *  It uses discrete noise model for process model uncertainty matrix Q
 *
 * @param pKf     Pointer to Kalman filter object
 * @param pState  Pointer to last state
 * @param dt      Delta time in seconds
 */
void pdlNavKF_Predict_ConstAcc(pdlNavKF *pKf, pdlNavState *pState, float dt);

/** Predicts new state with known control input, invoke this function with constant period.
 *  It uses discrete noise model for process model uncertainty matrix Q
 *
 * @param pKf     Pointer to Kalman filter object
 * @param pState  Pointer to last state
 * @param accel   Acceleration in m/s^2
 * @param dt      Delta time in seconds
 */
void pdlNavKF_Predict_ControlInput(pdlNavKF *pKf, pdlNavState *pState, float accel, float dt);

/** Predicts new state with const velocity model, invoke this function with constant period.
 *  It uses discrete noise model for process model uncertainty matrix Q
 *
 * @param pKf     Pointer to Kalman filter object
 * @param pState  Pointer to last state
 * @param dt      Delta time in seconds
 */
void pdlNavKF_Predict_ConstVelo(pdlNavKF *pKf, pdlNavState *pState, float dt);

/** Correct state with new acceleration measurement
 *
 *  @param pKf      Pointer to Kalman filter object
 *  @param pState   Pointer to last state
 *  @param a        Acceleration measurement in m/s
 *  @param R        Measurement Uncertainty.
 *                  This param means how much we trust this measurement.
 *                  It can be obtained from sensor datasheet, see RMS.
 *                  Put RMS*RMS for this param
 */
void pdlNavKF_CorrectByAccMeasurement(pdlNavKF *pKf, pdlNavState *pState, float a, float R);

/** Correct state with new position measurement
 *
 * @param pKf       Pointer to Kalman filter object
 * @param pState    Pointer to last state
 * @param p         Position measurement in m
 * @param R         Measurement Uncertainty.
 *                  This param means how much we trust this measurement.
 *                  It can be obtained from sensor datasheet, see RMS.
 *                  Put RMS*RMS for this param
 */
void pdlNavKF_CorrectByPosMeasurement(pdlNavKF *pKf, pdlNavState *pState, float p, float R);

/** Correct state with new velocity measurement
 *
 * @param pKf       Pointer to Kalman filter object
 * @param pState    Pointer to last state
 * @param v         Velocity measurement in m/s
 * @param R         Measurement Uncertainty
 *                  This param means how much we trust this measurement.
 *                  It can be obtained from sensor datasheet, see RMS.
 *                  Put RMS*RMS for this param
 */
void pdlNavKF_CorrectByVelMeasurement(pdlNavKF *pKf, pdlNavState *pState, float v, float R);

#endif
