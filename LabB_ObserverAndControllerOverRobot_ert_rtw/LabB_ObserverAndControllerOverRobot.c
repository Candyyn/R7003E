/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LabB_ObserverAndControllerOverRobot.c
 *
 * Code generated for Simulink model 'LabB_ObserverAndControllerOverRobot'.
 *
 * Model version                  : 1.705
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu Jan 22 10:53:52 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Atmel->AVR
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "LabB_ObserverAndControllerOverRobot.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include <stddef.h>
#include "LabB_ObserverAndControllerOverRobot_private.h"
#include <string.h>

/* Block signals (default storage) */
BlockIO_LabB_ObserverAndControl LabB_ObserverAndControllerOve_B;

/* Block states (default storage) */
D_Work_LabB_ObserverAndControll LabB_ObserverAndControlle_DWork;

/* Real-time model */
static RT_MODEL_LabB_ObserverAndContro LabB_ObserverAndControllerOv_M_;
RT_MODEL_LabB_ObserverAndContro *const LabB_ObserverAndControllerOv_M =
  &LabB_ObserverAndControllerOv_M_;

/* Model step function */
void LabB_ObserverAndControllerOverRobot_step(void)
{
  /* local block i/o variables */
  real32_T rtb_Fcn3;
  real32_T rtb_Fcn4;
  real32_T rtb_SaturationVsupplytoVsupply;
  real32_T rtb_Internal_3_2;
  real32_T rtb_Sum2_p;
  real32_T rtb_Sum3;
  real32_T rtb_DataTypeConversion1;
  real32_T rtb_Add1_idx_0;
  real32_T rtb_Add1_idx_1;
  real32_T rtb_Add1_idx_2;
  real32_T rtb_Gain8_idx_0;
  real32_T rtb_Gain8_idx_2;
  real32_T rtb_Gain8_idx_3;
  real32_T tmp;
  int16_T i;
  int16_T rtb_Sum_h_idx_0_tmp;
  uint16_T rtb_DataTypeConversion6;

  /* MATLABSystem: '<S3>/Encoder' */
  /*         %% Define output properties */
  /* 'Encoder_arduino:136' num = 0; */
  /* 'Encoder_arduino:140' num = 1; */
  /* 'Encoder_arduino:121' y = int32(0); */
  /* 'Encoder_arduino:122' if coder.target('Rtw') */
  /*  Call: int enc_output(int enc) */
  /* 'Encoder_arduino:124' y = coder.ceval('enc_output', obj.Encoder); */
  LabB_ObserverAndControllerOve_B.rtb_Encoder_c = enc_output(1.0);

  /* Gain: '<S2>/convert to meters' incorporates:
   *  DataTypeConversion: '<S3>/Data Type  Conversion2'
   *  Gain: '<S2>/convert to  radians'
   *  MATLABSystem: '<S3>/Encoder'
   */
  LabB_ObserverAndControllerOve_B.converttometers =
    LabB_ObserverAndControllerOve_P.converttoradians_Gain * (real32_T)
    LabB_ObserverAndControllerOve_B.rtb_Encoder_c *
    LabB_ObserverAndControllerOve_P.fWheelRadius;

  /* SampleTimeMath: '<S5>/TSamp'
   *
   * About '<S5>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *   */
  LabB_ObserverAndControllerOve_B.TSamp =
    LabB_ObserverAndControllerOve_B.converttometers *
    LabB_ObserverAndControllerOve_P.TSamp_WtEt;

  /* SampleTimeMath: '<S6>/TSamp' incorporates:
   *  DiscreteIntegrator: '<S2>/Discrete-Time  Integrator (convert from theta_b_dot to theta_b)'
   *
   * About '<S6>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *   */
  LabB_ObserverAndControllerOve_B.TSamp_n =
    LabB_ObserverAndControlle_DWork.DiscreteTimeIntegratorconvertfr *
    LabB_ObserverAndControllerOve_P.TSamp_WtEt_m;

  /* DiscreteStateSpace: '<S7>/Internal_1_1' */
  {
    rtb_Fcn3 = LabB_ObserverAndControllerOve_P.Internal_1_1_C*
      LabB_ObserverAndControlle_DWork.Internal_1_1_DSTATE;
  }

  /* DiscreteStateSpace: '<S7>/Internal_1_2' */
  {
    rtb_Internal_3_2 = 0.0;
  }

  /* DiscreteStateSpace: '<S7>/Internal_1_3' */
  {
    rtb_Sum3 = 0.0;
  }

  /* DiscreteStateSpace: '<S7>/Internal_1_4' */
  {
    rtb_Sum2_p = 0.0;
  }

  /* Fcn: '<S9>/Fcn3' incorporates:
   *  Sum: '<S7>/Sum1'
   */
  rtb_Fcn3 = ((rtb_Fcn3 + rtb_Internal_3_2) + rtb_Sum3) + rtb_Sum2_p;

  /* DiscreteStateSpace: '<S7>/Internal_2_1' */
  {
    rtb_Sum2_p = 0.0;
  }

  /* DiscreteStateSpace: '<S7>/Internal_2_2' */
  {
    rtb_Sum3 = LabB_ObserverAndControllerOve_P.Internal_2_2_C*
      LabB_ObserverAndControlle_DWork.Internal_2_2_DSTATE;
  }

  /* DiscreteStateSpace: '<S7>/Internal_2_3' */
  {
    rtb_Internal_3_2 = 0.0;
  }

  /* DiscreteStateSpace: '<S7>/Internal_2_4' */
  {
    rtb_Fcn4 = 0.0;
  }

  /* Sum: '<S7>/Sum2' */
  LabB_ObserverAndControllerOve_B.Sum2 = ((rtb_Sum2_p + rtb_Sum3) +
    rtb_Internal_3_2) + rtb_Fcn4;

  /* DiscreteStateSpace: '<S7>/Internal_3_1' */
  {
    rtb_Fcn4 = 0.0;
  }

  /* DiscreteStateSpace: '<S7>/Internal_3_2' */
  {
    rtb_Sum2_p = 0.0;
  }

  /* DiscreteStateSpace: '<S7>/Internal_3_3' */
  {
    rtb_Sum3 = LabB_ObserverAndControllerOve_P.Internal_3_3_C*
      LabB_ObserverAndControlle_DWork.Internal_3_3_DSTATE;
  }

  /* DiscreteStateSpace: '<S7>/Internal_3_4' */
  {
    rtb_Internal_3_2 = 0.0;
  }

  /* Fcn: '<S9>/Fcn4' incorporates:
   *  Sum: '<S7>/Sum3'
   */
  rtb_Fcn4 = ((rtb_Fcn4 + rtb_Sum2_p) + rtb_Sum3) + rtb_Internal_3_2;

  /* DiscreteStateSpace: '<S7>/Internal_4_1' */
  {
    rtb_Sum2_p = 0.0;
  }

  /* DiscreteStateSpace: '<S7>/Internal_4_2' */
  {
    rtb_Sum3 = 0.0;
  }

  /* DiscreteStateSpace: '<S7>/Internal_4_3' */
  {
    rtb_Internal_3_2 = 0.0;
  }

  /* DiscreteStateSpace: '<S7>/Internal_4_4' */
  {
    rtb_DataTypeConversion1 = LabB_ObserverAndControllerOve_P.Internal_4_4_C*
      LabB_ObserverAndControlle_DWork.Internal_4_4_DSTATE;
  }

  /* SignalConversion generated from: '<S4>/Gain10' incorporates:
   *  Sum: '<S7>/Sum4'
   */
  LabB_ObserverAndControllerOve_B.TmpSignalConversionAtGain10[0] = rtb_Fcn3;
  LabB_ObserverAndControllerOve_B.TmpSignalConversionAtGain10[1] =
    LabB_ObserverAndControllerOve_B.Sum2;
  LabB_ObserverAndControllerOve_B.TmpSignalConversionAtGain10[2] = rtb_Fcn4;
  LabB_ObserverAndControllerOve_B.TmpSignalConversionAtGain10[3] = ((rtb_Sum2_p
    + rtb_Sum3) + rtb_Internal_3_2) + rtb_DataTypeConversion1;

  /* DiscreteStateSpace: '<S8>/Internal_1_1' */
  {
    rtb_DataTypeConversion1 = LabB_ObserverAndControllerOve_P.Internal_1_1_C_h*
      LabB_ObserverAndControlle_DWork.Internal_1_1_DSTATE_m;
  }

  /* DiscreteStateSpace: '<S8>/Internal_1_2' */
  {
    rtb_Sum2_p = 0.0;
  }

  /* DiscreteStateSpace: '<S8>/Internal_1_3' */
  {
    rtb_Sum3 = 0.0;
  }

  /* DataTypeConversion: '<S3>/Data Type  Conversion1' incorporates:
   *  Sum: '<S8>/Sum1'
   */
  rtb_DataTypeConversion1 = (rtb_DataTypeConversion1 + rtb_Sum2_p) + rtb_Sum3;

  /* DiscreteStateSpace: '<S8>/Internal_2_1' */
  {
    rtb_Sum2_p = 0.0;
  }

  /* DiscreteStateSpace: '<S8>/Internal_2_2' */
  {
    rtb_Sum3 = LabB_ObserverAndControllerOve_P.Internal_2_2_C_p*
      LabB_ObserverAndControlle_DWork.Internal_2_2_DSTATE_p;
  }

  /* DiscreteStateSpace: '<S8>/Internal_2_3' */
  {
    rtb_Internal_3_2 = 0.0;
  }

  /* Sum: '<S8>/Sum2' */
  rtb_Sum2_p = (rtb_Sum2_p + rtb_Sum3) + rtb_Internal_3_2;

  /* DiscreteStateSpace: '<S8>/Internal_3_1' */
  {
    rtb_Sum3 = 0.0;
  }

  /* DiscreteStateSpace: '<S8>/Internal_3_2' */
  {
    rtb_Internal_3_2 = 0.0;
  }

  /* DiscreteStateSpace: '<S8>/Internal_3_3' */
  {
    rtb_SaturationVsupplytoVsupply =
      LabB_ObserverAndControllerOve_P.Internal_3_3_C_h*
      LabB_ObserverAndControlle_DWork.Internal_3_3_DSTATE_o;
  }

  /* Sum: '<S8>/Sum3' */
  rtb_Sum3 = (rtb_Sum3 + rtb_Internal_3_2) + rtb_SaturationVsupplytoVsupply;

  /* Sum: '<S4>/Add1' incorporates:
   *  Gain: '<S4>/Gain4'
   */
  rtb_Add1_idx_0 = LabB_ObserverAndControllerOve_P.Md5[0] *
    LabB_ObserverAndControllerOve_B.converttometers + rtb_DataTypeConversion1;
  rtb_Add1_idx_1 = LabB_ObserverAndControllerOve_P.Md5[1] *
    LabB_ObserverAndControllerOve_B.converttometers + rtb_Sum2_p;
  rtb_Add1_idx_2 = LabB_ObserverAndControllerOve_P.Md5[2] *
    LabB_ObserverAndControllerOve_B.converttometers + rtb_Sum3;

  /* Sum: '<S4>/Add2' incorporates:
   *  Gain: '<S4>/Gain5'
   *  Gain: '<S4>/Gain6'
   *  Sum: '<S4>/Add1'
   */
  for (i = 0; i < 4; i++) {
    LabB_ObserverAndControllerOve_B.Add2[i] =
      ((LabB_ObserverAndControllerOve_P.Md7[i + 4] * rtb_Add1_idx_1 +
        LabB_ObserverAndControllerOve_P.Md7[i] * rtb_Add1_idx_0) +
       LabB_ObserverAndControllerOve_P.Md7[i + 8] * rtb_Add1_idx_2) +
      LabB_ObserverAndControllerOve_P.Md6[i] *
      LabB_ObserverAndControllerOve_B.converttometers;
  }

  /* End of Sum: '<S4>/Add2' */

  /* ManualSwitch: '<S4>/Manual Switch1' incorporates:
   *  DiscreteIntegrator: '<S2>/Discrete-Time  Integrator (convert from theta_b_dot to theta_b)'
   *  Gain: '<S4>/Gain8'
   *  ManualSwitch: '<S4>/Manual Switch'
   *  Sum: '<S4>/Add2'
   *  Sum: '<S5>/Diff'
   *  Sum: '<S6>/Diff'
   *  UnitDelay: '<S5>/UD'
   *  UnitDelay: '<S6>/UD'
   *
   * Block description for '<S5>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S6>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S5>/UD':
   *
   *  Store in Global RAM
   *
   * Block description for '<S6>/UD':
   *
   *  Store in Global RAM
   */
  if (LabB_ObserverAndControllerOve_P.ManualSwitch1_CurrentSetting == 1) {
    rtb_Gain8_idx_0 = LabB_ObserverAndControllerOve_B.converttometers;
    LabB_ObserverAndControllerOve_B.Sum2 = LabB_ObserverAndControllerOve_B.TSamp
      - LabB_ObserverAndControlle_DWork.UD_DSTATE;
    rtb_Gain8_idx_2 =
      LabB_ObserverAndControlle_DWork.DiscreteTimeIntegratorconvertfr;
    rtb_Gain8_idx_3 = LabB_ObserverAndControllerOve_B.TSamp_n -
      LabB_ObserverAndControlle_DWork.UD_DSTATE_o;
  } else if (LabB_ObserverAndControllerOve_P.ManualSwitch_CurrentSetting == 1) {
    /* ManualSwitch: '<S4>/Manual Switch' incorporates:
     *  Gain: '<S4>/Gain8'
     *  SignalConversion generated from: '<S4>/Gain10'
     */
    rtb_Gain8_idx_0 = rtb_Fcn3;
    rtb_Gain8_idx_2 = rtb_Fcn4;
    rtb_Gain8_idx_3 =
      LabB_ObserverAndControllerOve_B.TmpSignalConversionAtGain10[3];
  } else {
    rtb_Gain8_idx_0 = LabB_ObserverAndControllerOve_B.Add2[0];
    LabB_ObserverAndControllerOve_B.Sum2 = LabB_ObserverAndControllerOve_B.Add2
      [1];
    rtb_Gain8_idx_2 = LabB_ObserverAndControllerOve_B.Add2[2];
    rtb_Gain8_idx_3 = LabB_ObserverAndControllerOve_B.Add2[3];
  }

  /* End of ManualSwitch: '<S4>/Manual Switch1' */

  /* Saturate: '<S53>/Saturation -Vsupply to Vsupply' incorporates:
   *  DiscreteIntegrator: '<S2>/Discrete-Time  Integrator (convert from theta_b_dot to theta_b)'
   *  Fcn: '<S1>/turn off the motor if the measured angle is greater than X degrees'
   *  Gain: '<S1>/controller'
   *  Gain: '<S4>/Gain8'
   *  Product: '<S1>/Product1'
   */
  rtb_SaturationVsupplytoVsupply = ((LabB_ObserverAndControllerOve_P.Kd[0] *
    rtb_Gain8_idx_0 + LabB_ObserverAndControllerOve_P.Kd[1] *
    LabB_ObserverAndControllerOve_B.Sum2) + LabB_ObserverAndControllerOve_P.Kd[2]
    * rtb_Gain8_idx_2) + LabB_ObserverAndControllerOve_P.Kd[3] * rtb_Gain8_idx_3;
  rtb_SaturationVsupplytoVsupply *= (real32_T)((real32_T)fabs
    (LabB_ObserverAndControlle_DWork.DiscreteTimeIntegratorconvertfr) < 0.75F);

  /* DataTypeConversion: '<S3>/Data Type  Conversion1' incorporates:
   *  Fcn: '<S9>/Fcn'
   */
  rtb_DataTypeConversion1 = (rtb_SaturationVsupplytoVsupply + 10.0F) * 3276.8F;

  /* DataTypeConversion: '<S9>/Data Type Conversion' */
  LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)floor(rtb_DataTypeConversion1);
  if (rtIsNaNF(LabB_ObserverAndControllerOve_B.Sum2) || rtIsInfF
      (LabB_ObserverAndControllerOve_B.Sum2)) {
    LabB_ObserverAndControllerOve_B.Sum2 = 0.0F;
  } else {
    LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)fmod
      (LabB_ObserverAndControllerOve_B.Sum2, 65536.0);
  }

  rtb_DataTypeConversion6 = LabB_ObserverAndControllerOve_B.Sum2 < 0.0F ?
    (uint16_T)-(int16_T)(uint16_T)-LabB_ObserverAndControllerOve_B.Sum2 :
    (uint16_T)LabB_ObserverAndControllerOve_B.Sum2;

  /* End of DataTypeConversion: '<S9>/Data Type Conversion' */

  /* MATLABSystem: '<S9>/Serial Transmit' incorporates:
   *  DataTypeConversion: '<S39>/Extract Desired Bits'
   *  DataTypeConversion: '<S40>/Extract Desired Bits'
   */
  LabB_ObserverAndControllerOve_B.dataIn[1] = (uint8_T)(rtb_DataTypeConversion6 >>
    8);
  LabB_ObserverAndControllerOve_B.dataIn[2] = (uint8_T)rtb_DataTypeConversion6;

  /* DataTypeConversion: '<S3>/Data Type  Conversion1' incorporates:
   *  Fcn: '<S9>/Fcn1'
   */
  rtb_DataTypeConversion1 = (LabB_ObserverAndControllerOve_B.converttometers +
    10.0F) * 3276.8F;

  /* DataTypeConversion: '<S9>/Data Type Conversion1' */
  LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)floor(rtb_DataTypeConversion1);
  if (rtIsNaNF(LabB_ObserverAndControllerOve_B.Sum2) || rtIsInfF
      (LabB_ObserverAndControllerOve_B.Sum2)) {
    LabB_ObserverAndControllerOve_B.Sum2 = 0.0F;
  } else {
    LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)fmod
      (LabB_ObserverAndControllerOve_B.Sum2, 65536.0);
  }

  rtb_DataTypeConversion6 = LabB_ObserverAndControllerOve_B.Sum2 < 0.0F ?
    (uint16_T)-(int16_T)(uint16_T)-LabB_ObserverAndControllerOve_B.Sum2 :
    (uint16_T)LabB_ObserverAndControllerOve_B.Sum2;

  /* End of DataTypeConversion: '<S9>/Data Type Conversion1' */

  /* MATLABSystem: '<S9>/Serial Transmit' incorporates:
   *  DataTypeConversion: '<S45>/Extract Desired Bits'
   *  DataTypeConversion: '<S46>/Extract Desired Bits'
   */
  LabB_ObserverAndControllerOve_B.dataIn[3] = (uint8_T)(rtb_DataTypeConversion6 >>
    8);
  LabB_ObserverAndControllerOve_B.dataIn[4] = (uint8_T)rtb_DataTypeConversion6;

  /* DataTypeConversion: '<S3>/Data Type  Conversion1' incorporates:
   *  DiscreteIntegrator: '<S2>/Discrete-Time  Integrator (convert from theta_b_dot to theta_b)'
   *  Fcn: '<S9>/Fcn2'
   */
  rtb_DataTypeConversion1 =
    (LabB_ObserverAndControlle_DWork.DiscreteTimeIntegratorconvertfr + 10.0F) *
    3276.8F;

  /* DataTypeConversion: '<S9>/Data Type Conversion2' */
  LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)floor(rtb_DataTypeConversion1);
  if (rtIsNaNF(LabB_ObserverAndControllerOve_B.Sum2) || rtIsInfF
      (LabB_ObserverAndControllerOve_B.Sum2)) {
    LabB_ObserverAndControllerOve_B.Sum2 = 0.0F;
  } else {
    LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)fmod
      (LabB_ObserverAndControllerOve_B.Sum2, 65536.0);
  }

  rtb_DataTypeConversion6 = LabB_ObserverAndControllerOve_B.Sum2 < 0.0F ?
    (uint16_T)-(int16_T)(uint16_T)-LabB_ObserverAndControllerOve_B.Sum2 :
    (uint16_T)LabB_ObserverAndControllerOve_B.Sum2;

  /* End of DataTypeConversion: '<S9>/Data Type Conversion2' */

  /* MATLABSystem: '<S9>/Serial Transmit' incorporates:
   *  DataTypeConversion: '<S47>/Extract Desired Bits'
   *  DataTypeConversion: '<S48>/Extract Desired Bits'
   */
  LabB_ObserverAndControllerOve_B.dataIn[5] = (uint8_T)(rtb_DataTypeConversion6 >>
    8);
  LabB_ObserverAndControllerOve_B.dataIn[6] = (uint8_T)rtb_DataTypeConversion6;

  /* Fcn: '<S9>/Fcn3' */
  rtb_Fcn3 = (rtb_Fcn3 + 10.0F) * 3276.8F;

  /* DataTypeConversion: '<S9>/Data Type Conversion3' */
  LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)floor(rtb_Fcn3);
  if (rtIsNaNF(LabB_ObserverAndControllerOve_B.Sum2) || rtIsInfF
      (LabB_ObserverAndControllerOve_B.Sum2)) {
    LabB_ObserverAndControllerOve_B.Sum2 = 0.0F;
  } else {
    LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)fmod
      (LabB_ObserverAndControllerOve_B.Sum2, 65536.0);
  }

  rtb_DataTypeConversion6 = LabB_ObserverAndControllerOve_B.Sum2 < 0.0F ?
    (uint16_T)-(int16_T)(uint16_T)-LabB_ObserverAndControllerOve_B.Sum2 :
    (uint16_T)LabB_ObserverAndControllerOve_B.Sum2;

  /* End of DataTypeConversion: '<S9>/Data Type Conversion3' */

  /* MATLABSystem: '<S9>/Serial Transmit' incorporates:
   *  DataTypeConversion: '<S49>/Extract Desired Bits'
   *  DataTypeConversion: '<S50>/Extract Desired Bits'
   */
  LabB_ObserverAndControllerOve_B.dataIn[7] = (uint8_T)(rtb_DataTypeConversion6 >>
    8);
  LabB_ObserverAndControllerOve_B.dataIn[8] = (uint8_T)rtb_DataTypeConversion6;

  /* Fcn: '<S9>/Fcn4' */
  rtb_Fcn4 = (rtb_Fcn4 + 10.0F) * 3276.8F;

  /* DataTypeConversion: '<S9>/Data Type Conversion4' */
  LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)floor(rtb_Fcn4);
  if (rtIsNaNF(LabB_ObserverAndControllerOve_B.Sum2) || rtIsInfF
      (LabB_ObserverAndControllerOve_B.Sum2)) {
    LabB_ObserverAndControllerOve_B.Sum2 = 0.0F;
  } else {
    LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)fmod
      (LabB_ObserverAndControllerOve_B.Sum2, 65536.0);
  }

  rtb_DataTypeConversion6 = LabB_ObserverAndControllerOve_B.Sum2 < 0.0F ?
    (uint16_T)-(int16_T)(uint16_T)-LabB_ObserverAndControllerOve_B.Sum2 :
    (uint16_T)LabB_ObserverAndControllerOve_B.Sum2;

  /* End of DataTypeConversion: '<S9>/Data Type Conversion4' */

  /* MATLABSystem: '<S9>/Serial Transmit' incorporates:
   *  DataTypeConversion: '<S51>/Extract Desired Bits'
   *  DataTypeConversion: '<S52>/Extract Desired Bits'
   */
  LabB_ObserverAndControllerOve_B.dataIn[9] = (uint8_T)(rtb_DataTypeConversion6 >>
    8);
  LabB_ObserverAndControllerOve_B.dataIn[10] = (uint8_T)rtb_DataTypeConversion6;

  /* DataTypeConversion: '<S3>/Data Type  Conversion1' incorporates:
   *  Fcn: '<S9>/Fcn5'
   */
  rtb_DataTypeConversion1 = (LabB_ObserverAndControllerOve_B.Add2[0] + 10.0F) *
    3276.8F;

  /* DataTypeConversion: '<S9>/Data Type Conversion5' */
  LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)floor(rtb_DataTypeConversion1);
  if (rtIsNaNF(LabB_ObserverAndControllerOve_B.Sum2) || rtIsInfF
      (LabB_ObserverAndControllerOve_B.Sum2)) {
    LabB_ObserverAndControllerOve_B.Sum2 = 0.0F;
  } else {
    LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)fmod
      (LabB_ObserverAndControllerOve_B.Sum2, 65536.0);
  }

  rtb_DataTypeConversion6 = LabB_ObserverAndControllerOve_B.Sum2 < 0.0F ?
    (uint16_T)-(int16_T)(uint16_T)-LabB_ObserverAndControllerOve_B.Sum2 :
    (uint16_T)LabB_ObserverAndControllerOve_B.Sum2;

  /* End of DataTypeConversion: '<S9>/Data Type Conversion5' */

  /* MATLABSystem: '<S9>/Serial Transmit' incorporates:
   *  DataTypeConversion: '<S41>/Extract Desired Bits'
   *  DataTypeConversion: '<S42>/Extract Desired Bits'
   */
  LabB_ObserverAndControllerOve_B.dataIn[11] = (uint8_T)(rtb_DataTypeConversion6
    >> 8);
  LabB_ObserverAndControllerOve_B.dataIn[12] = (uint8_T)rtb_DataTypeConversion6;

  /* DataTypeConversion: '<S3>/Data Type  Conversion1' incorporates:
   *  Fcn: '<S9>/Fcn6'
   */
  rtb_DataTypeConversion1 = (LabB_ObserverAndControllerOve_B.Add2[2] + 10.0F) *
    3276.8F;

  /* DataTypeConversion: '<S9>/Data Type Conversion6' */
  LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)floor(rtb_DataTypeConversion1);
  if (rtIsNaNF(LabB_ObserverAndControllerOve_B.Sum2) || rtIsInfF
      (LabB_ObserverAndControllerOve_B.Sum2)) {
    LabB_ObserverAndControllerOve_B.Sum2 = 0.0F;
  } else {
    LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)fmod
      (LabB_ObserverAndControllerOve_B.Sum2, 65536.0);
  }

  rtb_DataTypeConversion6 = LabB_ObserverAndControllerOve_B.Sum2 < 0.0F ?
    (uint16_T)-(int16_T)(uint16_T)-LabB_ObserverAndControllerOve_B.Sum2 :
    (uint16_T)LabB_ObserverAndControllerOve_B.Sum2;

  /* End of DataTypeConversion: '<S9>/Data Type Conversion6' */

  /* MATLABSystem: '<S9>/Serial Transmit' incorporates:
   *  Constant: '<S9>/Constant'
   *  DataTypeConversion: '<S43>/Extract Desired Bits'
   *  DataTypeConversion: '<S44>/Extract Desired Bits'
   */
  LabB_ObserverAndControllerOve_B.dataIn[0] =
    LabB_ObserverAndControllerOve_P.Constant_Value_d;
  LabB_ObserverAndControllerOve_B.dataIn[13] = (uint8_T)(rtb_DataTypeConversion6
    >> 8);
  LabB_ObserverAndControllerOve_B.dataIn[14] = (uint8_T)rtb_DataTypeConversion6;
  MW_Serial_write(LabB_ObserverAndControlle_DWork.obj.port,
                  &LabB_ObserverAndControllerOve_B.dataIn[0], 15.0,
                  LabB_ObserverAndControlle_DWork.obj.dataSizeInBytes,
                  LabB_ObserverAndControlle_DWork.obj.sendModeEnum,
                  LabB_ObserverAndControlle_DWork.obj.dataType,
                  LabB_ObserverAndControlle_DWork.obj.sendFormatEnum, 2.0,
                  '\x00', NULL, 0.0, NULL, 0.0);
  for (i = 0; i < 3; i++) {
    /* Sum: '<S4>/Add' incorporates:
     *  DiscreteIntegrator: '<S2>/Discrete-Time  Integrator (convert from theta_b_dot to theta_b)'
     *  Gain: '<S4>/Gain'
     *  Gain: '<S4>/Gain1'
     *  Gain: '<S4>/Gain2'
     *  Gain: '<S4>/Gain3'
     *  Sum: '<S4>/Add1'
     */
    LabB_ObserverAndControllerOve_B.Add[i] =
      ((LabB_ObserverAndControllerOve_P.Md1[i + 3] * rtb_Add1_idx_1 +
        LabB_ObserverAndControllerOve_P.Md1[i] * rtb_Add1_idx_0) +
       LabB_ObserverAndControllerOve_P.Md1[i + 6] * rtb_Add1_idx_2) +
      ((LabB_ObserverAndControllerOve_P.Md4[i] *
        LabB_ObserverAndControlle_DWork.DiscreteTimeIntegratorconvertfr +
        LabB_ObserverAndControllerOve_P.Md3[i] *
        LabB_ObserverAndControllerOve_B.converttometers) +
       LabB_ObserverAndControllerOve_P.Md2[i] * rtb_SaturationVsupplytoVsupply);
  }

  /* Gain: '<S4>/Gain10' */
  rtb_Add1_idx_0 = 0.0F;
  rtb_Add1_idx_1 = 0.0F;
  for (i = 0; i < 4; i++) {
    LabB_ObserverAndControllerOve_B.Sum2 =
      LabB_ObserverAndControllerOve_B.TmpSignalConversionAtGain10[i];
    rtb_Sum_h_idx_0_tmp = i << 1;
    rtb_Add1_idx_0 += LabB_ObserverAndControllerOve_P.Cd[rtb_Sum_h_idx_0_tmp] *
      LabB_ObserverAndControllerOve_B.Sum2;
    rtb_Add1_idx_1 += LabB_ObserverAndControllerOve_P.Cd[rtb_Sum_h_idx_0_tmp + 1]
      * LabB_ObserverAndControllerOve_B.Sum2;
  }

  /* End of Gain: '<S4>/Gain10' */

  /* Sum: '<S4>/Sum' incorporates:
   *  DiscreteIntegrator: '<S2>/Discrete-Time  Integrator (convert from theta_b_dot to theta_b)'
   */
  LabB_ObserverAndControllerOve_B.rtb_converttometers_m[0] =
    LabB_ObserverAndControllerOve_B.converttometers - rtb_Add1_idx_0;
  LabB_ObserverAndControllerOve_B.rtb_converttometers_m[1] =
    LabB_ObserverAndControlle_DWork.DiscreteTimeIntegratorconvertfr -
    rtb_Add1_idx_1;

  /* Gain: '<S4>/Gain9' */
  LabB_ObserverAndControllerOve_B.converttometers = 0.0F;
  rtb_Add1_idx_0 = 0.0F;
  rtb_Add1_idx_1 = 0.0F;
  rtb_Add1_idx_2 = 0.0F;
  for (i = 0; i < 2; i++) {
    LabB_ObserverAndControllerOve_B.Sum2 =
      LabB_ObserverAndControllerOve_B.rtb_converttometers_m[i];
    rtb_Sum_h_idx_0_tmp = i << 2;
    LabB_ObserverAndControllerOve_B.converttometers +=
      LabB_ObserverAndControllerOve_P.Ld[rtb_Sum_h_idx_0_tmp] *
      LabB_ObserverAndControllerOve_B.Sum2;
    rtb_Add1_idx_0 += LabB_ObserverAndControllerOve_P.Ld[rtb_Sum_h_idx_0_tmp + 1]
      * LabB_ObserverAndControllerOve_B.Sum2;
    rtb_Add1_idx_1 += LabB_ObserverAndControllerOve_P.Ld[rtb_Sum_h_idx_0_tmp + 2]
      * LabB_ObserverAndControllerOve_B.Sum2;
    rtb_Add1_idx_2 += LabB_ObserverAndControllerOve_P.Ld[rtb_Sum_h_idx_0_tmp + 3]
      * LabB_ObserverAndControllerOve_B.Sum2;
  }

  /* End of Gain: '<S4>/Gain9' */

  /* Gain: '<S4>/Gain7' */
  rtb_Gain8_idx_0 = 0.0F;
  rtb_Gain8_idx_2 = 0.0F;
  rtb_Gain8_idx_3 = 0.0F;
  tmp = 0.0F;
  for (i = 0; i < 4; i++) {
    LabB_ObserverAndControllerOve_B.Sum2 =
      LabB_ObserverAndControllerOve_B.TmpSignalConversionAtGain10[i];
    rtb_Sum_h_idx_0_tmp = i << 2;
    rtb_Gain8_idx_0 += LabB_ObserverAndControllerOve_P.Ad[rtb_Sum_h_idx_0_tmp] *
      LabB_ObserverAndControllerOve_B.Sum2;
    rtb_Gain8_idx_2 += LabB_ObserverAndControllerOve_P.Ad[rtb_Sum_h_idx_0_tmp +
      1] * LabB_ObserverAndControllerOve_B.Sum2;
    rtb_Gain8_idx_3 += LabB_ObserverAndControllerOve_P.Ad[rtb_Sum_h_idx_0_tmp +
      2] * LabB_ObserverAndControllerOve_B.Sum2;
    tmp += LabB_ObserverAndControllerOve_P.Ad[rtb_Sum_h_idx_0_tmp + 3] *
      LabB_ObserverAndControllerOve_B.Sum2;
  }

  /* End of Gain: '<S4>/Gain7' */

  /* Sum: '<S4>/Add3' incorporates:
   *  Gain: '<S4>/Gain8'
   */
  LabB_ObserverAndControllerOve_B.Add3[0] = (LabB_ObserverAndControllerOve_P.Bd
    [0] * rtb_SaturationVsupplytoVsupply +
    LabB_ObserverAndControllerOve_B.converttometers) + rtb_Gain8_idx_0;
  LabB_ObserverAndControllerOve_B.Add3[1] = (LabB_ObserverAndControllerOve_P.Bd
    [1] * rtb_SaturationVsupplytoVsupply + rtb_Add1_idx_0) + rtb_Gain8_idx_2;
  LabB_ObserverAndControllerOve_B.Add3[2] = (LabB_ObserverAndControllerOve_P.Bd
    [2] * rtb_SaturationVsupplytoVsupply + rtb_Add1_idx_1) + rtb_Gain8_idx_3;
  LabB_ObserverAndControllerOve_B.Add3[3] = (LabB_ObserverAndControllerOve_P.Bd
    [3] * rtb_SaturationVsupplytoVsupply + rtb_Add1_idx_2) + tmp;

  /* MATLABSystem: '<S3>/Gyroscope' */
  /*  initialize output to a single (float) with the value zero */
  /* 'soMPU6050Gyro:59' out = int16(zeros(3,1)); */
  /* 'soMPU6050Gyro:60' if coder.target('Rtw') */
  /*  done only for code gen */
  /* 'soMPU6050Gyro:61' coder.cinclude('MPU6050wrapper.h'); */
  /*  get the current value of the sensor */
  /* 'soMPU6050Gyro:63' coder.ceval('MPU6050Gyro_Read', coder.wref(out)); */
  MPU6050Gyro_Read(&LabB_ObserverAndControllerOve_B.out[0]);

  /* DataTypeConversion: '<S3>/Data Type  Conversion1' incorporates:
   *  MATLABSystem: '<S3>/Gyroscope'
   * */
  /*  pull the data appart */
  /* 'soMPU6050Gyro:68' xvel = out(1); */
  /* 'soMPU6050Gyro:69' yvel = out(2); */
  /* 'soMPU6050Gyro:70' zvel = out(3); */
  rtb_DataTypeConversion1 = LabB_ObserverAndControllerOve_B.out[0];

  /* MATLABSystem: '<S53>/Digital Output1' incorporates:
   *  Constant: '<S54>/Constant'
   *  RelationalOperator: '<S54>/Compare'
   */
  writeDigitalPin(8, (uint8_T)(rtb_SaturationVsupplytoVsupply <
    LabB_ObserverAndControllerOve_P.Constant_Value));

  /* Saturate: '<S53>/Saturation -Vsupply to Vsupply' */
  if (rtb_SaturationVsupplytoVsupply >
      LabB_ObserverAndControllerOve_P.M1V4LeftMotorDriverPWM6D8FST_Vs) {
    /* Saturate: '<S53>/Saturation -Vsupply to Vsupply' */
    rtb_SaturationVsupplytoVsupply =
      LabB_ObserverAndControllerOve_P.M1V4LeftMotorDriverPWM6D8FST_Vs;
  } else if (rtb_SaturationVsupplytoVsupply <
             LabB_ObserverAndControllerOve_P.SaturationVsupplytoVsupply_Lowe) {
    /* Saturate: '<S53>/Saturation -Vsupply to Vsupply' */
    rtb_SaturationVsupplytoVsupply =
      LabB_ObserverAndControllerOve_P.SaturationVsupplytoVsupply_Lowe;
  }

  /* End of Saturate: '<S53>/Saturation -Vsupply to Vsupply' */

  /* MATLABSystem: '<S53>/PWM' */
  LabB_ObserverAndControlle_DWork.obj_fn.PWMDriverObj.MW_PWM_HANDLE =
    MW_PWM_GetHandle(6UL);

  /* Gain: '<S53>/conversion to dutycycle (convert to uint8, overflow will provide reverse polairty magnitued)' */
  LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)floor
    (LabB_ObserverAndControllerOve_P.conversiontodutycycleconverttou *
     rtb_SaturationVsupplytoVsupply);
  if (rtIsNaNF(LabB_ObserverAndControllerOve_B.Sum2) || rtIsInfF
      (LabB_ObserverAndControllerOve_B.Sum2)) {
    LabB_ObserverAndControllerOve_B.Sum2 = 0.0F;
  } else {
    LabB_ObserverAndControllerOve_B.Sum2 = (real32_T)fmod
      (LabB_ObserverAndControllerOve_B.Sum2, 256.0);
  }

  /* MATLABSystem: '<S53>/PWM' incorporates:
   *  Gain: '<S53>/conversion to dutycycle (convert to uint8, overflow will provide reverse polairty magnitued)'
   */
  MW_PWM_SetDutyCycle
    (LabB_ObserverAndControlle_DWork.obj_fn.PWMDriverObj.MW_PWM_HANDLE, (real_T)
     (LabB_ObserverAndControllerOve_B.Sum2 < 0.0F ? (int16_T)(uint8_T)-(int8_T)
      (uint8_T)-LabB_ObserverAndControllerOve_B.Sum2 : (int16_T)(uint8_T)
      LabB_ObserverAndControllerOve_B.Sum2));

  /* Update for UnitDelay: '<S5>/UD'
   *
   * Block description for '<S5>/UD':
   *
   *  Store in Global RAM
   */
  LabB_ObserverAndControlle_DWork.UD_DSTATE =
    LabB_ObserverAndControllerOve_B.TSamp;

  /* Update for DiscreteIntegrator: '<S2>/Discrete-Time  Integrator (convert from theta_b_dot to theta_b)' incorporates:
   *  Constant: '<S2>/gyro bias'
   *  Gain: '<S2>/convert to radians//sec'
   *  Sum: '<S2>/Sum'
   */
  LabB_ObserverAndControlle_DWork.DiscreteTimeIntegratorconvertfr +=
    (rtb_DataTypeConversion1 - LabB_ObserverAndControllerOve_P.fGyroBias) *
    LabB_ObserverAndControllerOve_P.converttoradianssec_Gain *
    LabB_ObserverAndControllerOve_P.DiscreteTimeIntegratorconvertfr;

  /* Update for UnitDelay: '<S6>/UD'
   *
   * Block description for '<S6>/UD':
   *
   *  Store in Global RAM
   */
  LabB_ObserverAndControlle_DWork.UD_DSTATE_o =
    LabB_ObserverAndControllerOve_B.TSamp_n;

  /* Update for DiscreteStateSpace: '<S7>/Internal_1_1' */
  {
    LabB_ObserverAndControlle_DWork.Internal_1_1_DSTATE =
      LabB_ObserverAndControllerOve_B.Add3[0] +
      (LabB_ObserverAndControllerOve_P.Internal_1_1_A)*
      LabB_ObserverAndControlle_DWork.Internal_1_1_DSTATE;
  }

  /* Update for DiscreteStateSpace: '<S7>/Internal_2_2' */
  {
    LabB_ObserverAndControlle_DWork.Internal_2_2_DSTATE =
      LabB_ObserverAndControllerOve_B.Add3[1] +
      (LabB_ObserverAndControllerOve_P.Internal_2_2_A)*
      LabB_ObserverAndControlle_DWork.Internal_2_2_DSTATE;
  }

  /* Update for DiscreteStateSpace: '<S7>/Internal_3_3' */
  {
    LabB_ObserverAndControlle_DWork.Internal_3_3_DSTATE =
      LabB_ObserverAndControllerOve_B.Add3[2] +
      (LabB_ObserverAndControllerOve_P.Internal_3_3_A)*
      LabB_ObserverAndControlle_DWork.Internal_3_3_DSTATE;
  }

  /* Update for DiscreteStateSpace: '<S7>/Internal_4_4' */
  {
    LabB_ObserverAndControlle_DWork.Internal_4_4_DSTATE =
      LabB_ObserverAndControllerOve_B.Add3[3] +
      (LabB_ObserverAndControllerOve_P.Internal_4_4_A)*
      LabB_ObserverAndControlle_DWork.Internal_4_4_DSTATE;
  }

  /* Update for DiscreteStateSpace: '<S8>/Internal_1_1' */
  {
    LabB_ObserverAndControlle_DWork.Internal_1_1_DSTATE_m =
      LabB_ObserverAndControllerOve_B.Add[0] +
      (LabB_ObserverAndControllerOve_P.Internal_1_1_A_m)*
      LabB_ObserverAndControlle_DWork.Internal_1_1_DSTATE_m;
  }

  /* Update for DiscreteStateSpace: '<S8>/Internal_2_2' */
  {
    LabB_ObserverAndControlle_DWork.Internal_2_2_DSTATE_p =
      LabB_ObserverAndControllerOve_B.Add[1] +
      (LabB_ObserverAndControllerOve_P.Internal_2_2_A_o)*
      LabB_ObserverAndControlle_DWork.Internal_2_2_DSTATE_p;
  }

  /* Update for DiscreteStateSpace: '<S8>/Internal_3_3' */
  {
    LabB_ObserverAndControlle_DWork.Internal_3_3_DSTATE_o =
      LabB_ObserverAndControllerOve_B.Add[2] +
      (LabB_ObserverAndControllerOve_P.Internal_3_3_A_a)*
      LabB_ObserverAndControlle_DWork.Internal_3_3_DSTATE_o;
  }
}

/* Model initialize function */
void LabB_ObserverAndControllerOverRobot_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(LabB_ObserverAndControllerOv_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&LabB_ObserverAndControlle_DWork, 0,
                sizeof(D_Work_LabB_ObserverAndControll));

  /* Start for MATLABSystem: '<S3>/Encoder' */
  /*  Constructor */
  /* 'Encoder_arduino:36' coder.allowpcode('plain'); */
  /* 'Encoder_arduino:1' matlab.System */
  /* 'Encoder_arduino:2' coder.ExternalDependency */
  /* 'Encoder_arduino:3' matlab.system.mixin.Propagates */
  /* 'Encoder_arduino:4' matlab.system.mixin.CustomIcon */
  /*  Support name-value pair arguments when constructing the object. */
  /* 'Encoder_arduino:39' setProperties(obj,nargin,varargin{:}); */
  LabB_ObserverAndControlle_DWork.obj_g.matlabCodegenIsDeleted = false;

  /* 'Encoder_arduino:69' validateattributes(value,... */
  /* 'Encoder_arduino:70'                 {'numeric'},... */
  /* 'Encoder_arduino:71'                 {'real','nonnegative','integer','scalar','>=',0,'<=',obj.MaxNumEncoder},... */
  /* 'Encoder_arduino:72'                 '', ... */
  /* 'Encoder_arduino:73'                 'Encoder'); */
  /* 'Encoder_arduino:74' obj.Encoder = value; */
  /* 'Encoder_arduino:43' coder.extrinsic('sprintf') */
  /*  Do not generate code for sprintf */
  /* 'Encoder_arduino:44' validateattributes(value,... */
  /* 'Encoder_arduino:45'                 {'numeric'},... */
  /* 'Encoder_arduino:46'                 {'real','nonnegative','integer','scalar'},... */
  /* 'Encoder_arduino:47'                 '', ... */
  /* 'Encoder_arduino:48'                 'PinA'); */
  /* 'Encoder_arduino:49' assert(any(value == obj.AvailablePin), ... */
  /* 'Encoder_arduino:50'                 'Invalid value for Pin. Pin must be one of the following: %s', ... */
  /* 'Encoder_arduino:51'                 sprintf('%d ', obj.AvailablePin)); */
  /* 'Encoder_arduino:52' obj.PinA = value; */
  /* 'Encoder_arduino:56' coder.extrinsic('sprintf') */
  /*  Do not generate code for sprintf */
  /* 'Encoder_arduino:57' validateattributes(value,... */
  /* 'Encoder_arduino:58'                 {'numeric'},... */
  /* 'Encoder_arduino:59'                 {'real','nonnegative','integer','scalar'},... */
  /* 'Encoder_arduino:60'                 '', ... */
  /* 'Encoder_arduino:61'                 'PinB'); */
  /* 'Encoder_arduino:62' assert(any(value == obj.AvailablePin), ... */
  /* 'Encoder_arduino:63'                 'Invalid value for Pin. Pin must be one of the following: %s', ... */
  /* 'Encoder_arduino:64'                 sprintf('%d ', obj.AvailablePin)); */
  /* 'Encoder_arduino:65' obj.PinB = value; */
  LabB_ObserverAndControlle_DWork.obj_g.isInitialized = 1L;

  /*         %% Define output properties */
  /* 'Encoder_arduino:136' num = 0; */
  /*         %% Define output properties */
  /* 'Encoder_arduino:136' num = 0; */
  /* 'Encoder_arduino:108' if coder.target('Rtw') */
  /*    Call: void enc_init(int enc, int pinA, int pinB) */
  /* 'Encoder_arduino:110' coder.cinclude('encoder_arduino.h'); */
  /* 'Encoder_arduino:111' coder.ceval('enc_init', obj.Encoder, obj.PinA, obj.PinB); */
  enc_init(1.0, 19.0, 18.0);

  /*                  if(obj.PWMTimer> 0) */
  /*                      coder.cinclude('PWMFSelect.h'); */
  /*                      coder.ceval('PWM_Select', obj.PWMFSelect, obj.PWMTimer); */
  /*                      disp('skipp!!!') */
  /*                  end */
  LabB_ObserverAndControlle_DWork.obj_g.isSetupComplete = true;

  /* Start for MATLABSystem: '<S9>/Serial Transmit' */
  LabB_ObserverAndControlle_DWork.obj.matlabCodegenIsDeleted = false;
  LabB_ObserverAndControlle_DWork.obj.isInitialized = 1L;
  LabB_ObserverAndControlle_DWork.obj.port = 0.0;
  LabB_ObserverAndControlle_DWork.obj.dataSizeInBytes = 1.0;
  LabB_ObserverAndControlle_DWork.obj.dataType = 0.0;
  LabB_ObserverAndControlle_DWork.obj.sendModeEnum = 0.0;
  LabB_ObserverAndControlle_DWork.obj.sendFormatEnum = 0.0;
  MW_SCI_Open(0);
  LabB_ObserverAndControlle_DWork.obj.isSetupComplete = true;

  /* Start for MATLABSystem: '<S3>/Gyroscope' */
  /*  Constructor */
  /* 'soMPU6050Gyro:27' coder.allowpcode('plain'); */
  /* 'soMPU6050Gyro:1' matlab.System */
  /* 'soMPU6050Gyro:1' coder.ExternalDependency */
  /*  Support name-value pair arguments when constructing the object. */
  /* 'soMPU6050Gyro:30' setProperties(obj,nargin,varargin{:}); */
  LabB_ObserverAndControlle_DWork.obj_f.matlabCodegenIsDeleted = false;

  /* 'soMPU6050Gyro:34' coder.extrinsic('sprintf'); */
  /*  Do not generate code for sprintf */
  /* 'soMPU6050Gyro:35' validateattributes(value,... */
  /* 'soMPU6050Gyro:36'                 {'numeric'},... */
  /* 'soMPU6050Gyro:37'                 {'real','nonnegative','integer','scalar'},... */
  /* 'soMPU6050Gyro:38'                 '', ... */
  /* 'soMPU6050Gyro:39'                 'DLPFmode'); */
  /* 'soMPU6050Gyro:40' obj.DLPFmode = value; */
  LabB_ObserverAndControlle_DWork.obj_f.isInitialized = 1L;

  /* 'soMPU6050Gyro:46' if coder.target('Rtw') */
  /*  done only for code gen */
  /* 'soMPU6050Gyro:47' coder.cinclude('MPU6050wrapper.h'); */
  /*  initialize the sensor */
  /*                  coder.ceval('MPU6050Accel_Init'); */
  /* 'soMPU6050Gyro:51' coder.ceval('MPU6050Gyro_Init', obj.DLPFmode); */
  MPU6050Gyro_Init(0.0);
  LabB_ObserverAndControlle_DWork.obj_f.isSetupComplete = true;

  /* Start for MATLABSystem: '<S53>/Digital Output1' */
  LabB_ObserverAndControlle_DWork.obj_j.matlabCodegenIsDeleted = false;
  LabB_ObserverAndControlle_DWork.obj_j.isInitialized = 1L;
  digitalIOSetup(8, 1);
  LabB_ObserverAndControlle_DWork.obj_j.isSetupComplete = true;

  /* Start for MATLABSystem: '<S53>/PWM' */
  LabB_ObserverAndControlle_DWork.obj_fn.matlabCodegenIsDeleted = false;
  LabB_ObserverAndControlle_DWork.obj_fn.isInitialized = 1L;
  LabB_ObserverAndControlle_DWork.obj_fn.PWMDriverObj.MW_PWM_HANDLE =
    MW_PWM_Open(6UL, 0.0, 0.0);
  LabB_ObserverAndControlle_DWork.obj_fn.isSetupComplete = true;

  /* InitializeConditions for UnitDelay: '<S5>/UD'
   *
   * Block description for '<S5>/UD':
   *
   *  Store in Global RAM
   */
  LabB_ObserverAndControlle_DWork.UD_DSTATE =
    LabB_ObserverAndControllerOve_P.DiscreteDerivative_ICPrevScaled;

  /* InitializeConditions for DiscreteIntegrator: '<S2>/Discrete-Time  Integrator (convert from theta_b_dot to theta_b)' */
  LabB_ObserverAndControlle_DWork.DiscreteTimeIntegratorconvertfr =
    LabB_ObserverAndControllerOve_P.DiscreteTimeIntegratorconvert_e;

  /* InitializeConditions for UnitDelay: '<S6>/UD'
   *
   * Block description for '<S6>/UD':
   *
   *  Store in Global RAM
   */
  LabB_ObserverAndControlle_DWork.UD_DSTATE_o =
    LabB_ObserverAndControllerOve_P.DiscreteDerivative_ICPrevScal_n;
}

/* Model terminate function */
void LabB_ObserverAndControllerOverRobot_terminate(void)
{
  /* Terminate for MATLABSystem: '<S3>/Encoder' */
  if (!LabB_ObserverAndControlle_DWork.obj_g.matlabCodegenIsDeleted) {
    LabB_ObserverAndControlle_DWork.obj_g.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S3>/Encoder' */

  /* Terminate for MATLABSystem: '<S9>/Serial Transmit' */
  if (!LabB_ObserverAndControlle_DWork.obj.matlabCodegenIsDeleted) {
    LabB_ObserverAndControlle_DWork.obj.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S9>/Serial Transmit' */

  /* Terminate for MATLABSystem: '<S3>/Gyroscope' */
  if (!LabB_ObserverAndControlle_DWork.obj_f.matlabCodegenIsDeleted) {
    LabB_ObserverAndControlle_DWork.obj_f.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S3>/Gyroscope' */

  /* Terminate for MATLABSystem: '<S53>/Digital Output1' */
  if (!LabB_ObserverAndControlle_DWork.obj_j.matlabCodegenIsDeleted) {
    LabB_ObserverAndControlle_DWork.obj_j.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S53>/Digital Output1' */

  /* Terminate for MATLABSystem: '<S53>/PWM' */
  if (!LabB_ObserverAndControlle_DWork.obj_fn.matlabCodegenIsDeleted) {
    LabB_ObserverAndControlle_DWork.obj_fn.matlabCodegenIsDeleted = true;
    if ((LabB_ObserverAndControlle_DWork.obj_fn.isInitialized == 1L) &&
        LabB_ObserverAndControlle_DWork.obj_fn.isSetupComplete) {
      LabB_ObserverAndControlle_DWork.obj_fn.PWMDriverObj.MW_PWM_HANDLE =
        MW_PWM_GetHandle(6UL);
      MW_PWM_SetDutyCycle
        (LabB_ObserverAndControlle_DWork.obj_fn.PWMDriverObj.MW_PWM_HANDLE, 0.0);
      LabB_ObserverAndControlle_DWork.obj_fn.PWMDriverObj.MW_PWM_HANDLE =
        MW_PWM_GetHandle(6UL);
      MW_PWM_Close
        (LabB_ObserverAndControlle_DWork.obj_fn.PWMDriverObj.MW_PWM_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S53>/PWM' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
