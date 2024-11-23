#include "pdl_nav_kf.h"

void pdlNavKF_Init(pdlNavKF *pKf, pdlNavState *pState, float processSigma)
{
  int i,j;

  if(!pKf || !pState)
    return;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    for(j = 0; j < PDL_NAV_KF_STATE_SIZE; j++)
    {
      pKf->P[i][j] = 0;
      if(i == j)
        pKf->P[i][j] = 10.f; // We assume that first state is accurate
    }
  }

  pKf->Q_sigma = processSigma;

  pState->pos = 0;
  pState->vel = 0;
  pState->acc = 0;
  pState->accBias = 0;
}

void pdlNavKF_Predict_ConstAcc(pdlNavKF *pKf, pdlNavState *pState, float dt)
{
  int i, j;

  float _dt2;
  float _dt2_div2;
  float _dt3_div2;
  float N[PDL_NAV_KF_STATE_SIZE][PDL_NAV_KF_STATE_SIZE];
  float *P[PDL_NAV_KF_STATE_SIZE];
  float q = pKf->Q_sigma;

  if(!pKf || !pState)
    return;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    P[i] = &pKf->P[0][0] + i * PDL_NAV_KF_STATE_SIZE;
  }

  _dt2 = dt*dt;
  _dt2_div2 = 0.5f*_dt2;
  _dt3_div2 = 0.5f*_dt2*dt;

  pState->pos += pState->vel*dt + pState->acc*_dt2_div2;
  pState->vel += pState->acc*dt;

  // P_n+1,n = F * P_n,n * F' + Q

  //            |1   dt   0.5*dt*dt|           |    1       0   0|   |dt^4/4    dt^3/2    dt^2/2|
  // P_n+1,n =  |0   1        dt   | * P_n,n * |    dt      1   0| + |dt^3/2    dt^2      dt    | * Q_sigma
  //            |0   0        1    |           |0.5*dt*dt   dt  1|   |dt^2/2    dt        1     |

  N[0][0] = P[0][0]+P[1][0]*dt+_dt2_div2*P[2][0]+dt*(P[0][1]+P[1][1]*dt+_dt2_div2*P[2][1])+_dt2_div2*(P[0][2]+dt*P[1][2]+P[2][2]*_dt2_div2)+0.25f*_dt2*_dt2*q;
  N[0][1] = P[1][1]*dt+dt*(P[0][2]+P[2][2]*_dt2_div2+dt*P[1][2])+P[0][1]+_dt2_div2*P[2][1]+_dt3_div2*q;
  N[0][2] = P[0][2]+P[2][2]*_dt2_div2+dt*P[1][2]+_dt2_div2*q;
  N[1][0] = P[1][0]+dt*P[2][0]+dt*(dt*P[2][1]+P[1][1])+_dt2_div2*(P[2][2]*dt+P[1][2])+_dt3_div2*q;
  N[1][1] = dt*P[2][1]+dt*(P[2][2]*dt+P[1][2])+P[1][1]+_dt2*q;
  N[1][2] = P[2][2]*dt+P[1][2]+dt*q;
  N[2][0] = P[2][0]+dt*P[2][1]+P[2][2]*_dt2_div2+_dt2_div2*q;
  N[2][1] = P[2][2]*dt+P[2][1]+dt*q;
  N[2][2] = P[2][2]+q;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    for(j = 0; j < PDL_NAV_KF_STATE_SIZE; j++)
    {
      pKf->P[i][j] = N[i][j];
    }
  }
}

void pdlNavKF_Predict_ControlInput(pdlNavKF *pKf, pdlNavState *pState, float accel, float dt)
{
  int i, j;

  float _dt2;
  float _dt2_div2;
  float _dt3_div2;
  float N[PDL_NAV_KF_STATE_SIZE][PDL_NAV_KF_STATE_SIZE];
  float *P[PDL_NAV_KF_STATE_SIZE];
  float q = pKf->Q_sigma;

  if(!pKf || !pState)
    return;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    P[i] = &pKf->P[0][0] + i * PDL_NAV_KF_STATE_SIZE;
  }

  _dt2 = dt*dt;
  _dt2_div2 = 0.5f*_dt2;
  _dt3_div2 = 0.5f*_dt2*dt;

  accel = accel - pState->accBias;

  pState->acc = accel;
  pState->pos += pState->vel*dt + accel*_dt2_div2;
  pState->vel += accel*dt;

  // P_n+1,n = F * P_n,n * F' + Q

  //            |1   dt   0.5*dt*dt|           |    1       0   0|   |dt^4/4    dt^3/2    dt^2/2|
  // P_n+1,n =  |0   1        dt   | * P_n,n * |    dt      1   0| + |dt^3/2    dt^2      dt    | * Q_sigma
  //            |0   0        1    |           |0.5*dt*dt   dt  1|   |dt^2/2    dt        1     |

  N[0][0] = P[0][0]+P[1][0]*dt+_dt2_div2*P[2][0]+dt*(P[0][1]+P[1][1]*dt+_dt2_div2*P[2][1])+_dt2_div2*(P[0][2]+dt*P[1][2]+P[2][2]*_dt2_div2)+0.25f*_dt2*_dt2*q;
  N[0][1] = P[1][1]*dt+dt*(P[0][2]+P[2][2]*_dt2_div2+dt*P[1][2])+P[0][1]+_dt2_div2*P[2][1]+_dt3_div2*q;
  N[0][2] = P[0][2]+P[2][2]*_dt2_div2+dt*P[1][2]+_dt2_div2*q;
  N[1][0] = P[1][0]+dt*P[2][0]+dt*(dt*P[2][1]+P[1][1])+_dt2_div2*(P[2][2]*dt+P[1][2])+_dt3_div2*q;
  N[1][1] = dt*P[2][1]+dt*(P[2][2]*dt+P[1][2])+P[1][1]+_dt2*q;
  N[1][2] = P[2][2]*dt+P[1][2]+dt*q;
  N[2][0] = P[2][0]+dt*P[2][1]+P[2][2]*_dt2_div2+_dt2_div2*q;
  N[2][1] = P[2][2]*dt+P[2][1]+dt*q;
  N[2][2] = P[2][2]+q;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    for(j = 0; j < PDL_NAV_KF_STATE_SIZE; j++)
    {
      pKf->P[i][j] = N[i][j];
    }
  }
}

void pdlNavKF_Predict_ConstVelo(pdlNavKF *pKf, pdlNavState *pState, float dt)
{
  int i, j;

  float _dt2;
  float _dt2_div2;
  float _dt3_div2;
  float N[PDL_NAV_KF_STATE_SIZE][PDL_NAV_KF_STATE_SIZE];
  float *P[PDL_NAV_KF_STATE_SIZE];
  float q = pKf->Q_sigma;

  if(!pKf || !pState)
    return;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    P[i] = &pKf->P[0][0] + i * PDL_NAV_KF_STATE_SIZE;
  }

  _dt2 = dt*dt;
  _dt2_div2 = 0.5f*_dt2;
  _dt3_div2 = 0.5f*_dt2*dt;

  pState->pos += pState->vel*dt;

  // P_n+1,n = F * P_n,n * F' + Q

  //            |1   dt   0.5*dt*dt|           |    1       0   0|   |dt^4/4    dt^3/2    dt^2/2|
  // P_n+1,n =  |0   1        dt   | * P_n,n * |    dt      1   0| + |dt^3/2    dt^2      dt    | * Q_sigma
  //            |0   0        1    |           |0.5*dt*dt   dt  1|   |dt^2/2    dt        1     |

  N[0][0] = P[0][0]+P[1][0]*dt+_dt2_div2*P[2][0]+dt*(P[0][1]+P[1][1]*dt+_dt2_div2*P[2][1])+_dt2_div2*(P[0][2]+dt*P[1][2]+P[2][2]*_dt2_div2)+0.25f*_dt2*_dt2*q;
  N[0][1] = P[1][1]*dt+dt*(P[0][2]+P[2][2]*_dt2_div2+dt*P[1][2])+P[0][1]+_dt2_div2*P[2][1]+_dt3_div2*q;
  N[0][2] = P[0][2]+P[2][2]*_dt2_div2+dt*P[1][2]+_dt2_div2*q;
  N[1][0] = P[1][0]+dt*P[2][0]+dt*(dt*P[2][1]+P[1][1])+_dt2_div2*(P[2][2]*dt+P[1][2])+_dt3_div2*q;
  N[1][1] = dt*P[2][1]+dt*(P[2][2]*dt+P[1][2])+P[1][1]+_dt2*q;
  N[1][2] = P[2][2]*dt+P[1][2]+dt*q;
  N[2][0] = P[2][0]+dt*P[2][1]+P[2][2]*_dt2_div2+_dt2_div2*q;
  N[2][1] = P[2][2]*dt+P[2][1]+dt*q;
  N[2][2] = P[2][2]+q;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    for(j = 0; j < PDL_NAV_KF_STATE_SIZE; j++)
    {
      pKf->P[i][j] = N[i][j];
    }
  }
}

void pdlNavKF_CorrectByPosMeasurement(pdlNavKF *pKf, pdlNavState *pState, float p, float R)
{
  int i,j;
  float dz;
  float K[PDL_NAV_KF_STATE_SIZE];
  float Sinv;
  float N[PDL_NAV_KF_STATE_SIZE][PDL_NAV_KF_STATE_SIZE];
  float *P[PDL_NAV_KF_STATE_SIZE];

  if(!pKf || !pState)
    return;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    P[i] = &pKf->P[0][0] + i * PDL_NAV_KF_STATE_SIZE;
  }

  // Calculate Kalman Gain
  // K = P_n-1,n*H'*(H*P_n-1,n*H' + R)^-1

  //                | 1 |                         | 1 |
  // K = P_n-1,n *  | 0 | * ( |1 0 0| * P_n-1,n * | 0 | + R)^-1
  //                | 0 |                         | 0 |

  Sinv = 1.0f / (P[0][0] + R);

  K[0] = P[0][0] * Sinv;
  K[1] = P[1][0] * Sinv;
  K[2] = P[2][0] * Sinv;

  // Update the state estimate
  // X_n,n = X_n-1,n + K(Z - H*X_n-1)

  dz = p - pState->pos;

  pState->pos += K[0]*dz;
  pState->vel += K[1]*dz;
  pState->acc += K[2]*dz;

  // Update state estimate error covariance
  // P_n,n = (I - K*H)*P_n-1,n*(I - K*H)' + K*R*K'

  //           | 1 0 0 |   | K0 |                                     | 1 0 0 |   | K0 |               | R*K0 |
  // P_n,n = ( | 0 1 0 | - | K1 | * |1 0 0| ) * P_n-1,n * transpose ( | 0 1 0 | - | K1 | * |1 0 0| ) + | R*K1 | * |K0 K1 K2|
  //           | 0 0 1 |   | K2 |                                     | 0 0 1 |   | K2 |               | R*K2 |

  N[0][0] = P[0][0]*(1-K[0])*(1-K[0])+K[0]*K[0]*R;
  N[0][1] = P[0][1]*(1-K[0])-P[0][0]*K[1]*(1-K[0])+K[0]*K[1]*R;
  N[0][2] = -P[0][0]*K[2]*(1-K[0])+P[0][2]*(1-K[0])+K[0]*K[2]*R;
  N[1][0] = (1-K[0])*(P[1][0]-P[0][0]*K[1])+K[1]*R*K[0];
  N[1][1] = -K[1]*(P[1][0]-P[0][0]*K[1])-P[0][1]*K[1]+P[1][1]+K[1]*K[1]*R;
  N[1][2] = P[1][2]-P[0][2]*K[1]-K[2]*(P[1][0]-P[0][0]*K[1])+K[1]*K[2]*R;
  N[2][0] = (1-K[0])*(P[2][0]-P[0][0]*K[2])+K[2]*R*K[0];
  N[2][1] = P[2][1]-P[0][1]*K[2]-K[1]*(P[2][0]-P[0][0]*K[2])+K[1]*K[2]*R;
  N[2][2] = -K[2]*(P[2][0]-P[0][0]*K[2])-P[0][2]*K[2]+P[2][2]+K[2]*K[2]*R;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    for(j = 0; j < PDL_NAV_KF_STATE_SIZE; j++)
    {
      pKf->P[i][j] = N[i][j];
    }
  }
}


void pdlNavKF_CorrectByVelMeasurement(pdlNavKF *pKf, pdlNavState *pState, float v, float R)
{
  int i,j;
  float dz;
  float K[PDL_NAV_KF_STATE_SIZE];
  float Sinv;
  float N[PDL_NAV_KF_STATE_SIZE][PDL_NAV_KF_STATE_SIZE];
  float *P[PDL_NAV_KF_STATE_SIZE];

  if(!pKf || !pState)
    return;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    P[i] = &pKf->P[0][0] + i * PDL_NAV_KF_STATE_SIZE;
  }

  // Calculate Kalman Gain
  // K = P_n-1,n*H'*(H*P_n-1,n*H' + R)^-1

  //                | 0 |                         | 0 |
  // K = P_n-1,n *  | 1 | * ( |0 1 0| * P_n-1,n * | 1 | + R)^-1
  //                | 0 |                         | 0 |

  Sinv = 1.0f / (P[1][1] + R);

  K[0] = P[0][1] * Sinv;
  K[1] = P[1][1] * Sinv;
  K[2] = P[2][1] * Sinv;

  // Update the state estimate
  // X_n,n = X_n-1,n + K(Z - H*X_n-1)

  dz = v - pState->vel;

  pState->pos += K[0]*dz;
  pState->vel += K[1]*dz;
  pState->acc += K[2]*dz;

  // Update state estimate error covariance
  // P_n,n = (I - K*H)*P_n-1,n*(I - K*H)' + K*R*K'

  //           | 1 0 0 |   | K0 |                                     | 1 0 0 |   | K0 |               | R*K0 |
  // P_n,n = ( | 0 1 0 | - | K1 | * |0 1 0| ) * P_n-1,n * transpose ( | 0 1 0 | - | K1 | * |0 1 0| ) + | R*K1 | * |K0 K1 K2|
  //           | 0 0 1 |   | K2 |                                     | 0 0 1 |   | K2 |               | R*K2 |

  N[0][0] = -P[1][0]*K[0]-K[0]*(P[0][1]-P[1][1]*K[0])+P[0][0]+K[0]*K[0]*R;
  N[0][1] = (1-K[1])*(P[0][1]-P[1][1]*K[0])+K[0]*R*K[1];
  N[0][2] = P[0][2]-K[0]*P[1][2]-K[2]*(P[0][1]-P[1][1]*K[0])+K[0]*K[2]*R;
  N[1][0] = P[1][0]*(1-K[1])-P[1][1]*K[0]*(1-K[1])+K[0]*K[1]*R;
  N[1][1] = P[1][1]*(1-K[1])*(1-K[1])+K[1]*K[1]*R;
  N[1][2] = P[1][2]*(1-K[1])-P[1][1]*K[2]*(1-K[1])+K[1]*K[2]*R;
  N[2][0] = P[2][0]-P[1][0]*K[2]-K[0]*(P[2][1]-P[1][1]*K[2])+K[0]*K[2]*R;
  N[2][1] = (1-K[1])*(P[2][1]-P[1][1]*K[2])+K[2]*R*K[1];
  N[2][2] = -K[2]*(P[2][1]-P[1][1]*K[2])-K[2]*P[1][2]+P[2][2]+K[2]*K[2]*R;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    for(j = 0; j < PDL_NAV_KF_STATE_SIZE; j++)
    {
      pKf->P[i][j] = N[i][j];
    }
  }
}

void pdlNavKF_CorrectByAccMeasurement(pdlNavKF *pKf, pdlNavState *pState, float a, float R)
{
  int i,j;
  float dz;
  float K[PDL_NAV_KF_STATE_SIZE];
  float Sinv;
  float N[PDL_NAV_KF_STATE_SIZE][PDL_NAV_KF_STATE_SIZE];
  float *P[PDL_NAV_KF_STATE_SIZE];

  if(!pKf || !pState)
    return;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    P[i] = &pKf->P[0][0] + i * PDL_NAV_KF_STATE_SIZE;
  }

  // Calculate Kalman Gain
  // K = P_n-1,n*H'*(H*P_n-1,n*H' + R)^-1

  //                | 0 |                         | 0 |
  // K = P_n-1,n *  | 0 | * ( |0 0 1| * P_n-1,n * | 0 | + R)^-1
  //                | 1 |                         | 1 |

  Sinv = 1.0f / (P[2][2] + R);

  K[0] = P[0][2] * Sinv;
  K[1] = P[1][2] * Sinv;
  K[2] = P[2][2] * Sinv;

  // Update the state estimate
  // X_n,n = X_n-1,n + K(Z - H*X_n-1)

  dz = (a - pState->accBias) - pState->acc;

  pState->pos += K[0]*dz;
  pState->vel += K[1]*dz;
  pState->acc += K[2]*dz;

  // Update state estimate error covariance
  // P_n,n = (I - K*H)*P_n-1,n*(I - K*H)' + K*R*K'

  //           | 1 0 0 |   | K0 |                                     | 1 0 0 |   | K0 |               | R*K0 |
  // P_n,n = ( | 0 1 0 | - | K1 | * |0 0 1| ) * P_n-1,n * transpose ( | 0 1 0 | - | K1 | * |0 0 1| ) + | R*K1 | * |K0 K1 K2|
  //           | 0 0 1 |   | K2 |                                     | 0 0 1 |   | K2 |               | R*K2 |

  N[0][0] = -K[0]*P[2][0]+P[0][0]-K[0]*(P[0][2]-K[0]*P[2][2])+K[0]*K[0]*R;
  N[0][1] = -K[0]*P[2][1]+P[0][1]-K[1]*(P[0][2]-K[0]*P[2][2])+K[0]*K[1]*R;
  N[0][2] = (1-K[2])*(P[0][2]-K[0]*P[2][2])+K[0]*R*K[2];
  N[1][0] = -K[1]*P[2][0]+P[1][0]-K[0]*(P[1][2]-K[1]*P[2][2])+K[0]*K[1]*R;
  N[1][1] = -K[1]*P[2][1]+P[1][1]-K[1]*(P[1][2]-K[1]*P[2][2])+K[1]*K[1]*R;
  N[1][2] = (1-K[2])*(P[1][2]-K[1]*P[2][2])+K[1]*R*K[2];
  N[2][0] = P[2][0]*(1-K[2])-K[0]*P[2][2]*(1-K[2])+K[0]*K[2]*R;
  N[2][1] = P[2][1]*(1-K[2])-K[1]*P[2][2]*(1-K[2])+K[1]*K[2]*R;
  N[2][2] = P[2][2]*(1-K[2])*(1-K[2])+K[2]*K[2]*R;

  for(i = 0; i < PDL_NAV_KF_STATE_SIZE; i++)
  {
    for(j = 0; j < PDL_NAV_KF_STATE_SIZE; j++)
    {
      pKf->P[i][j] = N[i][j];
    }
  }
}
