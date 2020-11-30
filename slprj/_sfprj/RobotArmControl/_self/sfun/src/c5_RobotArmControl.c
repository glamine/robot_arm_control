/* Include files */

#include <stddef.h>
#include "blas.h"
#include "RobotArmControl_sfun.h"
#include "c5_RobotArmControl.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "RobotArmControl_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c5_debug_family_names[19] = { "a1", "a2", "a3", "d1", "d5",
  "t1", "t2", "t3", "t4", "t5", "nargin", "nargout", "q1ref", "q2ref", "q3ref",
  "q4ref", "q5ref", "param", "T50ref" };

/* Function Declarations */
static void initialize_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance);
static void initialize_params_c5_RobotArmControl
  (SFc5_RobotArmControlInstanceStruct *chartInstance);
static void enable_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance);
static void disable_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance);
static void c5_update_debugger_state_c5_RobotArmControl
  (SFc5_RobotArmControlInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_RobotArmControl
  (SFc5_RobotArmControlInstanceStruct *chartInstance);
static void set_sim_state_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct *
  chartInstance, const mxArray *c5_st);
static void finalize_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance);
static void sf_gateway_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance);
static void initSimStructsc5_RobotArmControl(SFc5_RobotArmControlInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static void c5_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_T50ref, const char_T *c5_identifier, real_T
  c5_y[16]);
static void c5_b_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[16]);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static real_T c5_c_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static void c5_info_helper(const mxArray **c5_info);
static const mxArray *c5_emlrt_marshallOut(const char * c5_u);
static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u);
static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_d_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_e_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_RobotArmControl, const char_T
  *c5_identifier);
static uint8_T c5_f_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info(SFc5_RobotArmControlInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c5_is_active_c5_RobotArmControl = 0U;
}

static void initialize_params_c5_RobotArmControl
  (SFc5_RobotArmControlInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c5_update_debugger_state_c5_RobotArmControl
  (SFc5_RobotArmControlInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c5_RobotArmControl
  (SFc5_RobotArmControlInstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  int32_T c5_i0;
  real_T c5_u[16];
  const mxArray *c5_b_y = NULL;
  uint8_T c5_hoistedGlobal;
  uint8_T c5_b_u;
  const mxArray *c5_c_y = NULL;
  real_T (*c5_T50ref)[16];
  c5_T50ref = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellmatrix(2, 1), false);
  for (c5_i0 = 0; c5_i0 < 16; c5_i0++) {
    c5_u[c5_i0] = (*c5_T50ref)[c5_i0];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 4, 4), false);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_hoistedGlobal = chartInstance->c5_is_active_c5_RobotArmControl;
  c5_b_u = c5_hoistedGlobal;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  sf_mex_assign(&c5_st, c5_y, false);
  return c5_st;
}

static void set_sim_state_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct *
  chartInstance, const mxArray *c5_st)
{
  const mxArray *c5_u;
  real_T c5_dv0[16];
  int32_T c5_i1;
  real_T (*c5_T50ref)[16];
  c5_T50ref = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = true;
  c5_u = sf_mex_dup(c5_st);
  c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 0)),
                      "T50ref", c5_dv0);
  for (c5_i1 = 0; c5_i1 < 16; c5_i1++) {
    (*c5_T50ref)[c5_i1] = c5_dv0[c5_i1];
  }

  chartInstance->c5_is_active_c5_RobotArmControl = c5_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 1)),
     "is_active_c5_RobotArmControl");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_RobotArmControl(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c5_RobotArmControl(SFc5_RobotArmControlInstanceStruct
  *chartInstance)
{
  real_T c5_hoistedGlobal;
  real_T c5_b_hoistedGlobal;
  real_T c5_c_hoistedGlobal;
  real_T c5_d_hoistedGlobal;
  real_T c5_e_hoistedGlobal;
  real_T c5_q1ref;
  real_T c5_q2ref;
  real_T c5_q3ref;
  real_T c5_q4ref;
  real_T c5_q5ref;
  int32_T c5_i2;
  real_T c5_param[5];
  uint32_T c5_debug_family_var_map[19];
  real_T c5_a1;
  real_T c5_a2;
  real_T c5_a3;
  real_T c5_d1;
  real_T c5_d5;
  real_T c5_t1;
  real_T c5_t2;
  real_T c5_t3;
  real_T c5_t4;
  real_T c5_t5;
  real_T c5_nargin = 6.0;
  real_T c5_nargout = 1.0;
  real_T c5_T50ref[16];
  real_T c5_x;
  real_T c5_b_x;
  real_T c5_c_x;
  real_T c5_d_x;
  real_T c5_e_x;
  real_T c5_f_x;
  real_T c5_g_x;
  real_T c5_h_x;
  real_T c5_i_x;
  real_T c5_j_x;
  real_T c5_k_x;
  real_T c5_l_x;
  real_T c5_m_x;
  real_T c5_n_x;
  real_T c5_o_x;
  real_T c5_p_x;
  real_T c5_q_x;
  real_T c5_r_x;
  real_T c5_s_x;
  real_T c5_t_x;
  real_T c5_u_x;
  real_T c5_v_x;
  real_T c5_w_x;
  real_T c5_x_x;
  real_T c5_y_x;
  real_T c5_ab_x;
  real_T c5_bb_x;
  real_T c5_cb_x;
  real_T c5_db_x;
  real_T c5_eb_x;
  real_T c5_fb_x;
  real_T c5_gb_x;
  real_T c5_hb_x;
  real_T c5_ib_x;
  real_T c5_jb_x;
  real_T c5_kb_x;
  real_T c5_lb_x;
  real_T c5_mb_x;
  real_T c5_nb_x;
  real_T c5_ob_x;
  real_T c5_pb_x;
  real_T c5_qb_x;
  real_T c5_rb_x;
  real_T c5_sb_x;
  real_T c5_tb_x;
  real_T c5_ub_x;
  real_T c5_vb_x;
  real_T c5_wb_x;
  real_T c5_xb_x;
  real_T c5_yb_x;
  real_T c5_ac_x;
  real_T c5_bc_x;
  real_T c5_cc_x;
  real_T c5_dc_x;
  real_T c5_ec_x;
  real_T c5_fc_x;
  real_T c5_gc_x;
  real_T c5_hc_x;
  real_T c5_ic_x;
  real_T c5_jc_x;
  real_T c5_kc_x;
  real_T c5_lc_x;
  real_T c5_mc_x;
  real_T c5_nc_x;
  real_T c5_oc_x;
  real_T c5_pc_x;
  real_T c5_qc_x;
  real_T c5_rc_x;
  real_T c5_sc_x;
  real_T c5_tc_x;
  real_T c5_uc_x;
  real_T c5_vc_x;
  real_T c5_wc_x;
  real_T c5_xc_x;
  real_T c5_yc_x;
  real_T c5_ad_x;
  real_T c5_bd_x;
  real_T c5_cd_x;
  real_T c5_dd_x;
  real_T c5_ed_x;
  int32_T c5_i3;
  int32_T c5_i4;
  static real_T c5_dv1[4] = { 0.0, 0.0, 0.0, 1.0 };

  int32_T c5_i5;
  int32_T c5_i6;
  int32_T c5_i7;
  real_T *c5_b_q1ref;
  real_T *c5_b_q2ref;
  real_T *c5_b_q3ref;
  real_T *c5_b_q4ref;
  real_T *c5_b_q5ref;
  real_T (*c5_b_T50ref)[16];
  real_T (*c5_b_param)[5];
  c5_b_param = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 5);
  c5_b_q5ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c5_b_q4ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c5_b_q3ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c5_b_q2ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_b_T50ref = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_b_q1ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c5_b_q1ref, 0U);
  chartInstance->c5_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  c5_hoistedGlobal = *c5_b_q1ref;
  c5_b_hoistedGlobal = *c5_b_q2ref;
  c5_c_hoistedGlobal = *c5_b_q3ref;
  c5_d_hoistedGlobal = *c5_b_q4ref;
  c5_e_hoistedGlobal = *c5_b_q5ref;
  c5_q1ref = c5_hoistedGlobal;
  c5_q2ref = c5_b_hoistedGlobal;
  c5_q3ref = c5_c_hoistedGlobal;
  c5_q4ref = c5_d_hoistedGlobal;
  c5_q5ref = c5_e_hoistedGlobal;
  for (c5_i2 = 0; c5_i2 < 5; c5_i2++) {
    c5_param[c5_i2] = (*c5_b_param)[c5_i2];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 19U, 19U, c5_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_a1, 0U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_a2, 1U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_a3, 2U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_d1, 3U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_d5, 4U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_t1, 5U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_t2, 6U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_t3, 7U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_t4, 8U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_t5, 9U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 10U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 11U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_q1ref, 12U, c5_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_q2ref, 13U, c5_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_q3ref, 14U, c5_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_q4ref, 15U, c5_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_q5ref, 16U, c5_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_param, 17U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_T50ref, 18U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 4);
  c5_a1 = c5_param[0];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 5);
  c5_a2 = c5_param[1];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 6);
  c5_a3 = c5_param[2];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 7);
  c5_d1 = c5_param[3];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 8);
  c5_d5 = c5_param[4];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 10);
  c5_t1 = c5_q1ref;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 11);
  c5_t2 = c5_q2ref;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 12);
  c5_t3 = c5_q3ref;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 13);
  c5_t4 = c5_q4ref;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 14);
  c5_t5 = c5_q5ref;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 16);
  c5_x = c5_t1;
  c5_b_x = c5_x;
  c5_b_x = muDoubleScalarSin(c5_b_x);
  c5_c_x = c5_t5;
  c5_d_x = c5_c_x;
  c5_d_x = muDoubleScalarSin(c5_d_x);
  c5_e_x = (c5_t2 + c5_t3) + c5_t4;
  c5_f_x = c5_e_x;
  c5_f_x = muDoubleScalarCos(c5_f_x);
  c5_g_x = c5_t1;
  c5_h_x = c5_g_x;
  c5_h_x = muDoubleScalarCos(c5_h_x);
  c5_i_x = c5_t5;
  c5_j_x = c5_i_x;
  c5_j_x = muDoubleScalarCos(c5_j_x);
  c5_k_x = c5_t5;
  c5_l_x = c5_k_x;
  c5_l_x = muDoubleScalarCos(c5_l_x);
  c5_m_x = c5_t1;
  c5_n_x = c5_m_x;
  c5_n_x = muDoubleScalarSin(c5_n_x);
  c5_o_x = (c5_t2 + c5_t3) + c5_t4;
  c5_p_x = c5_o_x;
  c5_p_x = muDoubleScalarCos(c5_p_x);
  c5_q_x = c5_t1;
  c5_r_x = c5_q_x;
  c5_r_x = muDoubleScalarCos(c5_r_x);
  c5_s_x = c5_t5;
  c5_t_x = c5_s_x;
  c5_t_x = muDoubleScalarSin(c5_t_x);
  c5_u_x = (c5_t2 + c5_t3) + c5_t4;
  c5_v_x = c5_u_x;
  c5_v_x = muDoubleScalarSin(c5_v_x);
  c5_w_x = c5_t1;
  c5_x_x = c5_w_x;
  c5_x_x = muDoubleScalarCos(c5_x_x);
  c5_y_x = c5_t1;
  c5_ab_x = c5_y_x;
  c5_ab_x = muDoubleScalarCos(c5_ab_x);
  c5_bb_x = c5_t2 + c5_t3;
  c5_cb_x = c5_bb_x;
  c5_cb_x = muDoubleScalarCos(c5_cb_x);
  c5_db_x = c5_t2;
  c5_eb_x = c5_db_x;
  c5_eb_x = muDoubleScalarCos(c5_eb_x);
  c5_fb_x = (c5_t2 + c5_t3) + c5_t4;
  c5_gb_x = c5_fb_x;
  c5_gb_x = muDoubleScalarSin(c5_gb_x);
  c5_hb_x = (c5_t2 + c5_t3) + c5_t4;
  c5_ib_x = c5_hb_x;
  c5_ib_x = muDoubleScalarCos(c5_ib_x);
  c5_jb_x = c5_t5;
  c5_kb_x = c5_jb_x;
  c5_kb_x = muDoubleScalarCos(c5_kb_x);
  c5_lb_x = c5_t1;
  c5_mb_x = c5_lb_x;
  c5_mb_x = muDoubleScalarSin(c5_mb_x);
  c5_nb_x = c5_t1;
  c5_ob_x = c5_nb_x;
  c5_ob_x = muDoubleScalarCos(c5_ob_x);
  c5_pb_x = c5_t5;
  c5_qb_x = c5_pb_x;
  c5_qb_x = muDoubleScalarSin(c5_qb_x);
  c5_rb_x = c5_t1;
  c5_sb_x = c5_rb_x;
  c5_sb_x = muDoubleScalarCos(c5_sb_x);
  c5_tb_x = c5_t5;
  c5_ub_x = c5_tb_x;
  c5_ub_x = muDoubleScalarCos(c5_ub_x);
  c5_vb_x = (c5_t2 + c5_t3) + c5_t4;
  c5_wb_x = c5_vb_x;
  c5_wb_x = muDoubleScalarCos(c5_wb_x);
  c5_xb_x = c5_t1;
  c5_yb_x = c5_xb_x;
  c5_yb_x = muDoubleScalarSin(c5_yb_x);
  c5_ac_x = c5_t5;
  c5_bc_x = c5_ac_x;
  c5_bc_x = muDoubleScalarSin(c5_bc_x);
  c5_cc_x = (c5_t2 + c5_t3) + c5_t4;
  c5_dc_x = c5_cc_x;
  c5_dc_x = muDoubleScalarSin(c5_dc_x);
  c5_ec_x = c5_t1;
  c5_fc_x = c5_ec_x;
  c5_fc_x = muDoubleScalarSin(c5_fc_x);
  c5_gc_x = c5_t1;
  c5_hc_x = c5_gc_x;
  c5_hc_x = muDoubleScalarSin(c5_hc_x);
  c5_ic_x = c5_t2 + c5_t3;
  c5_jc_x = c5_ic_x;
  c5_jc_x = muDoubleScalarCos(c5_jc_x);
  c5_kc_x = c5_t2;
  c5_lc_x = c5_kc_x;
  c5_lc_x = muDoubleScalarCos(c5_lc_x);
  c5_mc_x = (c5_t2 + c5_t3) + c5_t4;
  c5_nc_x = c5_mc_x;
  c5_nc_x = muDoubleScalarSin(c5_nc_x);
  c5_oc_x = (c5_t2 + c5_t3) + c5_t4;
  c5_pc_x = c5_oc_x;
  c5_pc_x = muDoubleScalarSin(c5_pc_x);
  c5_qc_x = c5_t5;
  c5_rc_x = c5_qc_x;
  c5_rc_x = muDoubleScalarCos(c5_rc_x);
  c5_sc_x = (c5_t2 + c5_t3) + c5_t4;
  c5_tc_x = c5_sc_x;
  c5_tc_x = muDoubleScalarSin(c5_tc_x);
  c5_uc_x = c5_t5;
  c5_vc_x = c5_uc_x;
  c5_vc_x = muDoubleScalarSin(c5_vc_x);
  c5_wc_x = (c5_t2 + c5_t3) + c5_t4;
  c5_xc_x = c5_wc_x;
  c5_xc_x = muDoubleScalarCos(c5_xc_x);
  c5_yc_x = c5_t2 + c5_t3;
  c5_ad_x = c5_yc_x;
  c5_ad_x = muDoubleScalarSin(c5_ad_x);
  c5_bd_x = c5_t2;
  c5_cd_x = c5_bd_x;
  c5_cd_x = muDoubleScalarSin(c5_cd_x);
  c5_dd_x = (c5_t2 + c5_t3) + c5_t4;
  c5_ed_x = c5_dd_x;
  c5_ed_x = muDoubleScalarCos(c5_ed_x);
  c5_T50ref[0] = c5_b_x * c5_d_x + c5_f_x * c5_h_x * c5_j_x;
  c5_T50ref[4] = c5_l_x * c5_n_x - c5_p_x * c5_r_x * c5_t_x;
  c5_T50ref[8] = -c5_v_x * c5_x_x;
  c5_T50ref[12] = c5_ab_x * (((c5_a1 + c5_a3 * c5_cb_x) + c5_a2 * c5_eb_x) -
    c5_d5 * c5_gb_x);
  c5_T50ref[1] = c5_ib_x * c5_kb_x * c5_mb_x - c5_ob_x * c5_qb_x;
  c5_T50ref[5] = -c5_sb_x * c5_ub_x - c5_wb_x * c5_yb_x * c5_bc_x;
  c5_T50ref[9] = -c5_dc_x * c5_fc_x;
  c5_T50ref[13] = c5_hc_x * (((c5_a1 + c5_a3 * c5_jc_x) + c5_a2 * c5_lc_x) -
    c5_d5 * c5_nc_x);
  c5_T50ref[2] = -c5_pc_x * c5_rc_x;
  c5_T50ref[6] = c5_tc_x * c5_vc_x;
  c5_T50ref[10] = -c5_xc_x;
  c5_T50ref[14] = ((c5_d1 - c5_a3 * c5_ad_x) - c5_a2 * c5_cd_x) - c5_d5 *
    c5_ed_x;
  c5_i3 = 0;
  for (c5_i4 = 0; c5_i4 < 4; c5_i4++) {
    c5_T50ref[c5_i3 + 3] = c5_dv1[c5_i4];
    c5_i3 += 4;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
  for (c5_i5 = 0; c5_i5 < 16; c5_i5++) {
    (*c5_b_T50ref)[c5_i5] = c5_T50ref[c5_i5];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_RobotArmControlMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c5_i6 = 0; c5_i6 < 16; c5_i6++) {
    _SFD_DATA_RANGE_CHECK((*c5_b_T50ref)[c5_i6], 1U);
  }

  _SFD_DATA_RANGE_CHECK(*c5_b_q2ref, 2U);
  _SFD_DATA_RANGE_CHECK(*c5_b_q3ref, 3U);
  _SFD_DATA_RANGE_CHECK(*c5_b_q4ref, 4U);
  _SFD_DATA_RANGE_CHECK(*c5_b_q5ref, 5U);
  for (c5_i7 = 0; c5_i7 < 5; c5_i7++) {
    _SFD_DATA_RANGE_CHECK((*c5_b_param)[c5_i7], 6U);
  }
}

static void initSimStructsc5_RobotArmControl(SFc5_RobotArmControlInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber)
{
  (void)c5_machineNumber;
  (void)c5_chartNumber;
  (void)c5_instanceNumber;
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i8;
  int32_T c5_i9;
  int32_T c5_i10;
  real_T c5_b_inData[16];
  int32_T c5_i11;
  int32_T c5_i12;
  int32_T c5_i13;
  real_T c5_u[16];
  const mxArray *c5_y = NULL;
  SFc5_RobotArmControlInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotArmControlInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i8 = 0;
  for (c5_i9 = 0; c5_i9 < 4; c5_i9++) {
    for (c5_i10 = 0; c5_i10 < 4; c5_i10++) {
      c5_b_inData[c5_i10 + c5_i8] = (*(real_T (*)[16])c5_inData)[c5_i10 + c5_i8];
    }

    c5_i8 += 4;
  }

  c5_i11 = 0;
  for (c5_i12 = 0; c5_i12 < 4; c5_i12++) {
    for (c5_i13 = 0; c5_i13 < 4; c5_i13++) {
      c5_u[c5_i13 + c5_i11] = c5_b_inData[c5_i13 + c5_i11];
    }

    c5_i11 += 4;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 4, 4), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_T50ref, const char_T *c5_identifier, real_T
  c5_y[16])
{
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_T50ref), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_T50ref);
}

static void c5_b_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[16])
{
  real_T c5_dv2[16];
  int32_T c5_i14;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv2, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c5_i14 = 0; c5_i14 < 16; c5_i14++) {
    c5_y[c5_i14] = c5_dv2[c5_i14];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_T50ref;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[16];
  int32_T c5_i15;
  int32_T c5_i16;
  int32_T c5_i17;
  SFc5_RobotArmControlInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotArmControlInstanceStruct *)chartInstanceVoid;
  c5_T50ref = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_T50ref), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_T50ref);
  c5_i15 = 0;
  for (c5_i16 = 0; c5_i16 < 4; c5_i16++) {
    for (c5_i17 = 0; c5_i17 < 4; c5_i17++) {
      (*(real_T (*)[16])c5_outData)[c5_i17 + c5_i15] = c5_y[c5_i17 + c5_i15];
    }

    c5_i15 += 4;
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i18;
  real_T c5_b_inData[5];
  int32_T c5_i19;
  real_T c5_u[5];
  const mxArray *c5_y = NULL;
  SFc5_RobotArmControlInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotArmControlInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i18 = 0; c5_i18 < 5; c5_i18++) {
    c5_b_inData[c5_i18] = (*(real_T (*)[5])c5_inData)[c5_i18];
  }

  for (c5_i19 = 0; c5_i19 < 5; c5_i19++) {
    c5_u[c5_i19] = c5_b_inData[c5_i19];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 5), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_RobotArmControlInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotArmControlInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static real_T c5_c_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_nargout;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_RobotArmControlInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotArmControlInstanceStruct *)chartInstanceVoid;
  c5_nargout = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nargout), &c5_thisId);
  sf_mex_destroy(&c5_nargout);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

const mxArray *sf_c5_RobotArmControl_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  sf_mex_assign(&c5_nameCaptureInfo, sf_mex_createstruct("structure", 2, 4, 1),
                false);
  c5_info_helper(&c5_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs0 = NULL;
  const mxArray *c5_lhs0 = NULL;
  const mxArray *c5_rhs1 = NULL;
  const mxArray *c5_lhs1 = NULL;
  const mxArray *c5_rhs2 = NULL;
  const mxArray *c5_lhs2 = NULL;
  const mxArray *c5_rhs3 = NULL;
  const mxArray *c5_lhs3 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sin"), "name", "name", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c5_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818736U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c5_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("cos"), "name", "name", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343830372U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c5_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818722U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c5_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs3), "lhs", "lhs", 3);
  sf_mex_destroy(&c5_rhs0);
  sf_mex_destroy(&c5_lhs0);
  sf_mex_destroy(&c5_rhs1);
  sf_mex_destroy(&c5_lhs1);
  sf_mex_destroy(&c5_rhs2);
  sf_mex_destroy(&c5_lhs2);
  sf_mex_destroy(&c5_rhs3);
  sf_mex_destroy(&c5_lhs3);
}

static const mxArray *c5_emlrt_marshallOut(const char * c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c5_u)), false);
  return c5_y;
}

static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 7, 0U, 0U, 0U, 0), false);
  return c5_y;
}

static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_RobotArmControlInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotArmControlInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static int32_T c5_d_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i20;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i20, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i20;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_RobotArmControlInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotArmControlInstanceStruct *)chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_e_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_RobotArmControl, const char_T
  *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_RobotArmControl), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_RobotArmControl);
  return c5_y;
}

static uint8_T c5_f_emlrt_marshallIn(SFc5_RobotArmControlInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void init_dsm_address_info(SFc5_RobotArmControlInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c5_RobotArmControl_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1213452204U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(400173893U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(336008782U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(131841656U);
}

mxArray *sf_c5_RobotArmControl_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("NtZ4VzaIceiGiDwfFX4FuD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(5);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c5_RobotArmControl_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c5_RobotArmControl_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c5_RobotArmControl(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"T50ref\",},{M[8],M[0],T\"is_active_c5_RobotArmControl\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_RobotArmControl_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_RobotArmControlInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc5_RobotArmControlInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _RobotArmControlMachineNumber_,
           5,
           1,
           1,
           0,
           7,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_RobotArmControlMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_RobotArmControlMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _RobotArmControlMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"q1ref");
          _SFD_SET_DATA_PROPS(1,2,0,1,"T50ref");
          _SFD_SET_DATA_PROPS(2,1,1,0,"q2ref");
          _SFD_SET_DATA_PROPS(3,1,1,0,"q3ref");
          _SFD_SET_DATA_PROPS(4,1,1,0,"q4ref");
          _SFD_SET_DATA_PROPS(5,1,1,0,"q5ref");
          _SFD_SET_DATA_PROPS(6,1,1,0,"param");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1063);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)
            c5_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c5_q1ref;
          real_T *c5_q2ref;
          real_T *c5_q3ref;
          real_T *c5_q4ref;
          real_T *c5_q5ref;
          real_T (*c5_T50ref)[16];
          real_T (*c5_param)[5];
          c5_param = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 5);
          c5_q5ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c5_q4ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c5_q3ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c5_q2ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c5_T50ref = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
          c5_q1ref = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c5_q1ref);
          _SFD_SET_DATA_VALUE_PTR(1U, *c5_T50ref);
          _SFD_SET_DATA_VALUE_PTR(2U, c5_q2ref);
          _SFD_SET_DATA_VALUE_PTR(3U, c5_q3ref);
          _SFD_SET_DATA_VALUE_PTR(4U, c5_q4ref);
          _SFD_SET_DATA_VALUE_PTR(5U, c5_q5ref);
          _SFD_SET_DATA_VALUE_PTR(6U, *c5_param);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _RobotArmControlMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "RDek3C9Pwc3F3WLLibNITE";
}

static void sf_opaque_initialize_c5_RobotArmControl(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_RobotArmControlInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c5_RobotArmControl((SFc5_RobotArmControlInstanceStruct*)
    chartInstanceVar);
  initialize_c5_RobotArmControl((SFc5_RobotArmControlInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c5_RobotArmControl(void *chartInstanceVar)
{
  enable_c5_RobotArmControl((SFc5_RobotArmControlInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c5_RobotArmControl(void *chartInstanceVar)
{
  disable_c5_RobotArmControl((SFc5_RobotArmControlInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c5_RobotArmControl(void *chartInstanceVar)
{
  sf_gateway_c5_RobotArmControl((SFc5_RobotArmControlInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c5_RobotArmControl(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_RobotArmControl
    ((SFc5_RobotArmControlInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_RobotArmControl();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c5_RobotArmControl(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c5_RobotArmControl();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_RobotArmControl((SFc5_RobotArmControlInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c5_RobotArmControl(SimStruct* S)
{
  return sf_internal_get_sim_state_c5_RobotArmControl(S);
}

static void sf_opaque_set_sim_state_c5_RobotArmControl(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c5_RobotArmControl(S, st);
}

static void sf_opaque_terminate_c5_RobotArmControl(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_RobotArmControlInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_RobotArmControl_optimization_info();
    }

    finalize_c5_RobotArmControl((SFc5_RobotArmControlInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_RobotArmControl((SFc5_RobotArmControlInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_RobotArmControl(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c5_RobotArmControl((SFc5_RobotArmControlInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_RobotArmControl(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_RobotArmControl_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,5,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,5);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2407963701U));
  ssSetChecksum1(S,(110514711U));
  ssSetChecksum2(S,(1197902715U));
  ssSetChecksum3(S,(3576065009U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_RobotArmControl(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_RobotArmControl(SimStruct *S)
{
  SFc5_RobotArmControlInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc5_RobotArmControlInstanceStruct *)utMalloc(sizeof
    (SFc5_RobotArmControlInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_RobotArmControlInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_RobotArmControl;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_RobotArmControl;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_RobotArmControl;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c5_RobotArmControl;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c5_RobotArmControl;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_RobotArmControl;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_RobotArmControl;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_RobotArmControl;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_RobotArmControl;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_RobotArmControl;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_RobotArmControl;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c5_RobotArmControl_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_RobotArmControl(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_RobotArmControl(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_RobotArmControl(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_RobotArmControl_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
