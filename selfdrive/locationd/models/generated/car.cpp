#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4218974077310003720) {
   out_4218974077310003720[0] = delta_x[0] + nom_x[0];
   out_4218974077310003720[1] = delta_x[1] + nom_x[1];
   out_4218974077310003720[2] = delta_x[2] + nom_x[2];
   out_4218974077310003720[3] = delta_x[3] + nom_x[3];
   out_4218974077310003720[4] = delta_x[4] + nom_x[4];
   out_4218974077310003720[5] = delta_x[5] + nom_x[5];
   out_4218974077310003720[6] = delta_x[6] + nom_x[6];
   out_4218974077310003720[7] = delta_x[7] + nom_x[7];
   out_4218974077310003720[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_591343773823492067) {
   out_591343773823492067[0] = -nom_x[0] + true_x[0];
   out_591343773823492067[1] = -nom_x[1] + true_x[1];
   out_591343773823492067[2] = -nom_x[2] + true_x[2];
   out_591343773823492067[3] = -nom_x[3] + true_x[3];
   out_591343773823492067[4] = -nom_x[4] + true_x[4];
   out_591343773823492067[5] = -nom_x[5] + true_x[5];
   out_591343773823492067[6] = -nom_x[6] + true_x[6];
   out_591343773823492067[7] = -nom_x[7] + true_x[7];
   out_591343773823492067[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1500766596726272852) {
   out_1500766596726272852[0] = 1.0;
   out_1500766596726272852[1] = 0.0;
   out_1500766596726272852[2] = 0.0;
   out_1500766596726272852[3] = 0.0;
   out_1500766596726272852[4] = 0.0;
   out_1500766596726272852[5] = 0.0;
   out_1500766596726272852[6] = 0.0;
   out_1500766596726272852[7] = 0.0;
   out_1500766596726272852[8] = 0.0;
   out_1500766596726272852[9] = 0.0;
   out_1500766596726272852[10] = 1.0;
   out_1500766596726272852[11] = 0.0;
   out_1500766596726272852[12] = 0.0;
   out_1500766596726272852[13] = 0.0;
   out_1500766596726272852[14] = 0.0;
   out_1500766596726272852[15] = 0.0;
   out_1500766596726272852[16] = 0.0;
   out_1500766596726272852[17] = 0.0;
   out_1500766596726272852[18] = 0.0;
   out_1500766596726272852[19] = 0.0;
   out_1500766596726272852[20] = 1.0;
   out_1500766596726272852[21] = 0.0;
   out_1500766596726272852[22] = 0.0;
   out_1500766596726272852[23] = 0.0;
   out_1500766596726272852[24] = 0.0;
   out_1500766596726272852[25] = 0.0;
   out_1500766596726272852[26] = 0.0;
   out_1500766596726272852[27] = 0.0;
   out_1500766596726272852[28] = 0.0;
   out_1500766596726272852[29] = 0.0;
   out_1500766596726272852[30] = 1.0;
   out_1500766596726272852[31] = 0.0;
   out_1500766596726272852[32] = 0.0;
   out_1500766596726272852[33] = 0.0;
   out_1500766596726272852[34] = 0.0;
   out_1500766596726272852[35] = 0.0;
   out_1500766596726272852[36] = 0.0;
   out_1500766596726272852[37] = 0.0;
   out_1500766596726272852[38] = 0.0;
   out_1500766596726272852[39] = 0.0;
   out_1500766596726272852[40] = 1.0;
   out_1500766596726272852[41] = 0.0;
   out_1500766596726272852[42] = 0.0;
   out_1500766596726272852[43] = 0.0;
   out_1500766596726272852[44] = 0.0;
   out_1500766596726272852[45] = 0.0;
   out_1500766596726272852[46] = 0.0;
   out_1500766596726272852[47] = 0.0;
   out_1500766596726272852[48] = 0.0;
   out_1500766596726272852[49] = 0.0;
   out_1500766596726272852[50] = 1.0;
   out_1500766596726272852[51] = 0.0;
   out_1500766596726272852[52] = 0.0;
   out_1500766596726272852[53] = 0.0;
   out_1500766596726272852[54] = 0.0;
   out_1500766596726272852[55] = 0.0;
   out_1500766596726272852[56] = 0.0;
   out_1500766596726272852[57] = 0.0;
   out_1500766596726272852[58] = 0.0;
   out_1500766596726272852[59] = 0.0;
   out_1500766596726272852[60] = 1.0;
   out_1500766596726272852[61] = 0.0;
   out_1500766596726272852[62] = 0.0;
   out_1500766596726272852[63] = 0.0;
   out_1500766596726272852[64] = 0.0;
   out_1500766596726272852[65] = 0.0;
   out_1500766596726272852[66] = 0.0;
   out_1500766596726272852[67] = 0.0;
   out_1500766596726272852[68] = 0.0;
   out_1500766596726272852[69] = 0.0;
   out_1500766596726272852[70] = 1.0;
   out_1500766596726272852[71] = 0.0;
   out_1500766596726272852[72] = 0.0;
   out_1500766596726272852[73] = 0.0;
   out_1500766596726272852[74] = 0.0;
   out_1500766596726272852[75] = 0.0;
   out_1500766596726272852[76] = 0.0;
   out_1500766596726272852[77] = 0.0;
   out_1500766596726272852[78] = 0.0;
   out_1500766596726272852[79] = 0.0;
   out_1500766596726272852[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3667355204630287878) {
   out_3667355204630287878[0] = state[0];
   out_3667355204630287878[1] = state[1];
   out_3667355204630287878[2] = state[2];
   out_3667355204630287878[3] = state[3];
   out_3667355204630287878[4] = state[4];
   out_3667355204630287878[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3667355204630287878[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3667355204630287878[7] = state[7];
   out_3667355204630287878[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1908076773300772117) {
   out_1908076773300772117[0] = 1;
   out_1908076773300772117[1] = 0;
   out_1908076773300772117[2] = 0;
   out_1908076773300772117[3] = 0;
   out_1908076773300772117[4] = 0;
   out_1908076773300772117[5] = 0;
   out_1908076773300772117[6] = 0;
   out_1908076773300772117[7] = 0;
   out_1908076773300772117[8] = 0;
   out_1908076773300772117[9] = 0;
   out_1908076773300772117[10] = 1;
   out_1908076773300772117[11] = 0;
   out_1908076773300772117[12] = 0;
   out_1908076773300772117[13] = 0;
   out_1908076773300772117[14] = 0;
   out_1908076773300772117[15] = 0;
   out_1908076773300772117[16] = 0;
   out_1908076773300772117[17] = 0;
   out_1908076773300772117[18] = 0;
   out_1908076773300772117[19] = 0;
   out_1908076773300772117[20] = 1;
   out_1908076773300772117[21] = 0;
   out_1908076773300772117[22] = 0;
   out_1908076773300772117[23] = 0;
   out_1908076773300772117[24] = 0;
   out_1908076773300772117[25] = 0;
   out_1908076773300772117[26] = 0;
   out_1908076773300772117[27] = 0;
   out_1908076773300772117[28] = 0;
   out_1908076773300772117[29] = 0;
   out_1908076773300772117[30] = 1;
   out_1908076773300772117[31] = 0;
   out_1908076773300772117[32] = 0;
   out_1908076773300772117[33] = 0;
   out_1908076773300772117[34] = 0;
   out_1908076773300772117[35] = 0;
   out_1908076773300772117[36] = 0;
   out_1908076773300772117[37] = 0;
   out_1908076773300772117[38] = 0;
   out_1908076773300772117[39] = 0;
   out_1908076773300772117[40] = 1;
   out_1908076773300772117[41] = 0;
   out_1908076773300772117[42] = 0;
   out_1908076773300772117[43] = 0;
   out_1908076773300772117[44] = 0;
   out_1908076773300772117[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1908076773300772117[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1908076773300772117[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1908076773300772117[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1908076773300772117[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1908076773300772117[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1908076773300772117[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1908076773300772117[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1908076773300772117[53] = -9.8000000000000007*dt;
   out_1908076773300772117[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1908076773300772117[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1908076773300772117[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1908076773300772117[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1908076773300772117[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1908076773300772117[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1908076773300772117[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1908076773300772117[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1908076773300772117[62] = 0;
   out_1908076773300772117[63] = 0;
   out_1908076773300772117[64] = 0;
   out_1908076773300772117[65] = 0;
   out_1908076773300772117[66] = 0;
   out_1908076773300772117[67] = 0;
   out_1908076773300772117[68] = 0;
   out_1908076773300772117[69] = 0;
   out_1908076773300772117[70] = 1;
   out_1908076773300772117[71] = 0;
   out_1908076773300772117[72] = 0;
   out_1908076773300772117[73] = 0;
   out_1908076773300772117[74] = 0;
   out_1908076773300772117[75] = 0;
   out_1908076773300772117[76] = 0;
   out_1908076773300772117[77] = 0;
   out_1908076773300772117[78] = 0;
   out_1908076773300772117[79] = 0;
   out_1908076773300772117[80] = 1;
}
void h_25(double *state, double *unused, double *out_5305392359454170036) {
   out_5305392359454170036[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7752789251071041527) {
   out_7752789251071041527[0] = 0;
   out_7752789251071041527[1] = 0;
   out_7752789251071041527[2] = 0;
   out_7752789251071041527[3] = 0;
   out_7752789251071041527[4] = 0;
   out_7752789251071041527[5] = 0;
   out_7752789251071041527[6] = 1;
   out_7752789251071041527[7] = 0;
   out_7752789251071041527[8] = 0;
}
void h_24(double *state, double *unused, double *out_3456721122253081439) {
   out_3456721122253081439[0] = state[4];
   out_3456721122253081439[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5580139652065541961) {
   out_5580139652065541961[0] = 0;
   out_5580139652065541961[1] = 0;
   out_5580139652065541961[2] = 0;
   out_5580139652065541961[3] = 0;
   out_5580139652065541961[4] = 1;
   out_5580139652065541961[5] = 0;
   out_5580139652065541961[6] = 0;
   out_5580139652065541961[7] = 0;
   out_5580139652065541961[8] = 0;
   out_5580139652065541961[9] = 0;
   out_5580139652065541961[10] = 0;
   out_5580139652065541961[11] = 0;
   out_5580139652065541961[12] = 0;
   out_5580139652065541961[13] = 0;
   out_5580139652065541961[14] = 1;
   out_5580139652065541961[15] = 0;
   out_5580139652065541961[16] = 0;
   out_5580139652065541961[17] = 0;
}
void h_30(double *state, double *unused, double *out_5148205691103040891) {
   out_5148205691103040891[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8175621864131261462) {
   out_8175621864131261462[0] = 0;
   out_8175621864131261462[1] = 0;
   out_8175621864131261462[2] = 0;
   out_8175621864131261462[3] = 0;
   out_8175621864131261462[4] = 1;
   out_8175621864131261462[5] = 0;
   out_8175621864131261462[6] = 0;
   out_8175621864131261462[7] = 0;
   out_8175621864131261462[8] = 0;
}
void h_26(double *state, double *unused, double *out_947078723979288660) {
   out_947078723979288660[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4011285932196985303) {
   out_4011285932196985303[0] = 0;
   out_4011285932196985303[1] = 0;
   out_4011285932196985303[2] = 0;
   out_4011285932196985303[3] = 0;
   out_4011285932196985303[4] = 0;
   out_4011285932196985303[5] = 0;
   out_4011285932196985303[6] = 0;
   out_4011285932196985303[7] = 1;
   out_4011285932196985303[8] = 0;
}
void h_27(double *state, double *unused, double *out_2876030512015577315) {
   out_2876030512015577315[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8096358897777865243) {
   out_8096358897777865243[0] = 0;
   out_8096358897777865243[1] = 0;
   out_8096358897777865243[2] = 0;
   out_8096358897777865243[3] = 1;
   out_8096358897777865243[4] = 0;
   out_8096358897777865243[5] = 0;
   out_8096358897777865243[6] = 0;
   out_8096358897777865243[7] = 0;
   out_8096358897777865243[8] = 0;
}
void h_29(double *state, double *unused, double *out_4277994970166034283) {
   out_4277994970166034283[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7665390519816869278) {
   out_7665390519816869278[0] = 0;
   out_7665390519816869278[1] = 1;
   out_7665390519816869278[2] = 0;
   out_7665390519816869278[3] = 0;
   out_7665390519816869278[4] = 0;
   out_7665390519816869278[5] = 0;
   out_7665390519816869278[6] = 0;
   out_7665390519816869278[7] = 0;
   out_7665390519816869278[8] = 0;
}
void h_28(double *state, double *unused, double *out_448484235210305473) {
   out_448484235210305473[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5698954536823151764) {
   out_5698954536823151764[0] = 1;
   out_5698954536823151764[1] = 0;
   out_5698954536823151764[2] = 0;
   out_5698954536823151764[3] = 0;
   out_5698954536823151764[4] = 0;
   out_5698954536823151764[5] = 0;
   out_5698954536823151764[6] = 0;
   out_5698954536823151764[7] = 0;
   out_5698954536823151764[8] = 0;
}
void h_31(double *state, double *unused, double *out_5030198297169664147) {
   out_5030198297169664147[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3385077829963633827) {
   out_3385077829963633827[0] = 0;
   out_3385077829963633827[1] = 0;
   out_3385077829963633827[2] = 0;
   out_3385077829963633827[3] = 0;
   out_3385077829963633827[4] = 0;
   out_3385077829963633827[5] = 0;
   out_3385077829963633827[6] = 0;
   out_3385077829963633827[7] = 0;
   out_3385077829963633827[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4218974077310003720) {
  err_fun(nom_x, delta_x, out_4218974077310003720);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_591343773823492067) {
  inv_err_fun(nom_x, true_x, out_591343773823492067);
}
void car_H_mod_fun(double *state, double *out_1500766596726272852) {
  H_mod_fun(state, out_1500766596726272852);
}
void car_f_fun(double *state, double dt, double *out_3667355204630287878) {
  f_fun(state,  dt, out_3667355204630287878);
}
void car_F_fun(double *state, double dt, double *out_1908076773300772117) {
  F_fun(state,  dt, out_1908076773300772117);
}
void car_h_25(double *state, double *unused, double *out_5305392359454170036) {
  h_25(state, unused, out_5305392359454170036);
}
void car_H_25(double *state, double *unused, double *out_7752789251071041527) {
  H_25(state, unused, out_7752789251071041527);
}
void car_h_24(double *state, double *unused, double *out_3456721122253081439) {
  h_24(state, unused, out_3456721122253081439);
}
void car_H_24(double *state, double *unused, double *out_5580139652065541961) {
  H_24(state, unused, out_5580139652065541961);
}
void car_h_30(double *state, double *unused, double *out_5148205691103040891) {
  h_30(state, unused, out_5148205691103040891);
}
void car_H_30(double *state, double *unused, double *out_8175621864131261462) {
  H_30(state, unused, out_8175621864131261462);
}
void car_h_26(double *state, double *unused, double *out_947078723979288660) {
  h_26(state, unused, out_947078723979288660);
}
void car_H_26(double *state, double *unused, double *out_4011285932196985303) {
  H_26(state, unused, out_4011285932196985303);
}
void car_h_27(double *state, double *unused, double *out_2876030512015577315) {
  h_27(state, unused, out_2876030512015577315);
}
void car_H_27(double *state, double *unused, double *out_8096358897777865243) {
  H_27(state, unused, out_8096358897777865243);
}
void car_h_29(double *state, double *unused, double *out_4277994970166034283) {
  h_29(state, unused, out_4277994970166034283);
}
void car_H_29(double *state, double *unused, double *out_7665390519816869278) {
  H_29(state, unused, out_7665390519816869278);
}
void car_h_28(double *state, double *unused, double *out_448484235210305473) {
  h_28(state, unused, out_448484235210305473);
}
void car_H_28(double *state, double *unused, double *out_5698954536823151764) {
  H_28(state, unused, out_5698954536823151764);
}
void car_h_31(double *state, double *unused, double *out_5030198297169664147) {
  h_31(state, unused, out_5030198297169664147);
}
void car_H_31(double *state, double *unused, double *out_3385077829963633827) {
  H_31(state, unused, out_3385077829963633827);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
