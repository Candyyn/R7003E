/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LabB_CheckCommunications.c
 *
 * Code generated for Simulink model 'LabB_CheckCommunications'.
 *
 * Model version                  : 18.0
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Wed Jan 21 10:38:09 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Atmel->AVR
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "LabB_CheckCommunications.h"
#include "LabB_CheckCommunications_private.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include <string.h>
#include "rt_defines.h"

/* Block signals (default storage) */
BlockIO_LabB_CheckCommunication LabB_CheckCommunications_B;

/* Block states (default storage) */
D_Work_LabB_CheckCommunications LabB_CheckCommunications_DWork;

/* Real-time model */
static RT_MODEL_LabB_CheckCommunicatio LabB_CheckCommunications_M_;
RT_MODEL_LabB_CheckCommunicatio *const LabB_CheckCommunications_M =
  &LabB_CheckCommunications_M_;
real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int16_T tmp;
  int16_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model step function */
void LabB_CheckCommunications_step(void)
{
  real_T rtb_Switch;
  real_T u0;
  int16_T out[3];
  int16_T rtb_Abs;
  uint8_T tmp;

  /* MATLABSystem: '<S1>/M2V3 Left Connector 18, 19' */
  /*         %% Define output properties */
  /* 'Encoder_arduino:136' num = 0; */
  /* 'Encoder_arduino:140' num = 1; */
  /* 'Encoder_arduino:121' y = int32(0); */
  /* 'Encoder_arduino:122' if coder.target('Rtw') */
  /*  Call: int enc_output(int enc) */
  /* 'Encoder_arduino:124' y = coder.ceval('enc_output', obj.Encoder); */
  LabB_CheckCommunications_B.M2V3LeftConnector1819 = enc_output(1.0);

  /* Gain: '<S3>/Gain' incorporates:
   *  Constant: '<S1>/angle offset'
   *  DiscreteIntegrator: '<S1>/angular velocity to angle'
   *  Sum: '<S1>/Sum'
   *  Sum: '<S1>/Sum4'
   */
  LabB_CheckCommunications_B.Gain =
    ((LabB_CheckCommunications_P.angleoffset_Value +
      LabB_CheckCommunications_DWork.angularvelocitytoangle_DSTATE) +
     LabB_CheckCommunications_P.angleoffset_Value) *
    LabB_CheckCommunications_P.Gain_Gain;

  /* MATLABSystem: '<S1>/Accelerometer' */
  /*  initialize output to a single (float) with the value zero */
  /* 'soMPU6050Accel:40' out = int16(zeros(3,1)); */
  /* 'soMPU6050Accel:41' if (coder.target('Rtw')) */
  /*  done only for code gen */
  /* 'soMPU6050Accel:42' coder.cinclude('MPU6050wrapper.h'); */
  /*  get the current value of the sensor */
  /* 'soMPU6050Accel:44' coder.ceval('MPU6050Accel_Read', coder.wref(out)); */
  MPU6050Accel_Read(&out[0]);

  /* Gain: '<S2>/Gain' incorporates:
   *  Constant: '<S1>/accelerometer angle bias'
   *  Constant: '<S1>/angle offset'
   *  DataTypeConversion: '<S1>/Data Type Conversion1'
   *  DataTypeConversion: '<S1>/Data Type Conversion2'
   *  Gain: '<S1>/raw y acceleration to g_y'
   *  Gain: '<S1>/raw z acceleration to g_z'
   *  MATLABSystem: '<S1>/Accelerometer'
   *  Sum: '<S1>/Sum1'
   *  Sum: '<S1>/Sum3'
   *  Trigonometry: '<S1>/g_y and g_z to angle in radians'
   * */
  /*  pull the data appart */
  /* 'soMPU6050Accel:49' xaccel = out(1); */
  /* 'soMPU6050Accel:50' yaccel = out(2); */
  /* 'soMPU6050Accel:51' zaccel = out(3); */
  LabB_CheckCommunications_B.Gain_j = ((rt_atan2d_snf
    (LabB_CheckCommunications_P.rawyaccelerationtog_y_Gain * (real_T)out[1],
     LabB_CheckCommunications_P.rawzaccelerationtog_z_Gain * (real_T)out[2]) +
    LabB_CheckCommunications_P.accelerometeranglebias_Value) +
    LabB_CheckCommunications_P.angleoffset_Value) *
    LabB_CheckCommunications_P.Gain_Gain_g;

  /* Sin: '<Root>/Sine Wave' */
  if (LabB_CheckCommunications_DWork.systemEnable != 0L) {
    rtb_Switch = LabB_CheckCommunications_P.SineWave_Freq *
      ((LabB_CheckCommunications_M->Timing.clockTick0) * 0.01);
    LabB_CheckCommunications_DWork.lastSin = sin(rtb_Switch);
    LabB_CheckCommunications_DWork.lastCos = cos(rtb_Switch);
    LabB_CheckCommunications_DWork.systemEnable = 0L;
  }

  /* Sin: '<Root>/Sine Wave' */
  LabB_CheckCommunications_B.SineWave = ((LabB_CheckCommunications_DWork.lastSin
    * LabB_CheckCommunications_P.SineWave_PCos +
    LabB_CheckCommunications_DWork.lastCos *
    LabB_CheckCommunications_P.SineWave_PSin) *
    LabB_CheckCommunications_P.SineWave_HCos +
    (LabB_CheckCommunications_DWork.lastCos *
     LabB_CheckCommunications_P.SineWave_PCos -
     LabB_CheckCommunications_DWork.lastSin *
     LabB_CheckCommunications_P.SineWave_PSin) *
    LabB_CheckCommunications_P.SineWave_Hsin) *
    LabB_CheckCommunications_P.SineWave_Amp +
    LabB_CheckCommunications_P.SineWave_Bias;

  /* MATLABSystem: '<S1>/Gyroscope' */
  /*  initialize output to a single (float) with the value zero */
  /* 'soMPU6050Gyro:59' out = int16(zeros(3,1)); */
  /* 'soMPU6050Gyro:60' if coder.target('Rtw') */
  /*  done only for code gen */
  /* 'soMPU6050Gyro:61' coder.cinclude('MPU6050wrapper.h'); */
  /*  get the current value of the sensor */
  /* 'soMPU6050Gyro:63' coder.ceval('MPU6050Gyro_Read', coder.wref(out)); */
  MPU6050Gyro_Read(&out[0]);

  /* Gain: '<S1>/conversion to duty cycle (convert to int)' */
  /*  pull the data appart */
  /* 'soMPU6050Gyro:68' xvel = out(1); */
  /* 'soMPU6050Gyro:69' yvel = out(2); */
  /* 'soMPU6050Gyro:70' zvel = out(3); */
  u0 = floor(LabB_CheckCommunications_P.conversiontodutycycleconverttoi *
             LabB_CheckCommunications_B.SineWave);
  if (rtIsNaN(u0) || rtIsInf(u0)) {
    u0 = 0.0;
  } else {
    u0 = fmod(u0, 65536.0);
  }

  rtb_Abs = u0 < 0.0 ? -(int16_T)(uint16_T)-u0 : (int16_T)(uint16_T)u0;

  /* End of Gain: '<S1>/conversion to duty cycle (convert to int)' */

  /* Switch: '<S1>/Switch' incorporates:
   *  Constant: '<S1>/Constant1'
   *  Constant: '<S1>/Constant2'
   */
  if (rtb_Abs >= LabB_CheckCommunications_P.Switch_Threshold) {
    rtb_Switch = LabB_CheckCommunications_P.Constant2_Value;
  } else {
    rtb_Switch = LabB_CheckCommunications_P.Constant1_Value;
  }

  /* End of Switch: '<S1>/Switch' */

  /* MATLABSystem: '<S1>/Digital Output1' */
  u0 = rt_roundd_snf(rtb_Switch);
  if (u0 < 256.0) {
    if (u0 >= 0.0) {
      tmp = (uint8_T)u0;
    } else {
      tmp = 0U;
    }
  } else {
    tmp = MAX_uint8_T;
  }

  writeDigitalPin(6, tmp);

  /* End of MATLABSystem: '<S1>/Digital Output1' */

  /* MATLABSystem: '<S1>/PWM' */
  LabB_CheckCommunications_DWork.obj_i.PWMDriverObj.MW_PWM_HANDLE =
    MW_PWM_GetHandle(8UL);

  /* Abs: '<S1>/Abs' */
  if (rtb_Abs < 0) {
    rtb_Abs = -rtb_Abs;
  }

  /* Sum: '<S1>/Add2' incorporates:
   *  Abs: '<S1>/Abs'
   *  Constant: '<S1>/Constant3'
   */
  u0 = LabB_CheckCommunications_P.Constant3_Value + (real_T)rtb_Abs;

  /* Saturate: '<S1>/Saturation 0 to 255' */
  if (u0 > LabB_CheckCommunications_P.Saturation0to255_UpperSat) {
    u0 = LabB_CheckCommunications_P.Saturation0to255_UpperSat;
  } else if (u0 < LabB_CheckCommunications_P.Saturation0to255_LowerSat) {
    u0 = LabB_CheckCommunications_P.Saturation0to255_LowerSat;
  }

  /* Abs: '<S1>/Abs3' incorporates:
   *  Gain: '<S1>/Gain1'
   *  Saturate: '<S1>/Saturation 0 to 255'
   *  Sum: '<S1>/Add1'
   */
  u0 = fabs(LabB_CheckCommunications_P.Gain1_Gain_b * rtb_Switch - u0);

  /* Start for MATLABSystem: '<S1>/PWM' */
  if (!(u0 <= 255.0)) {
    u0 = 255.0;
  }

  /* MATLABSystem: '<S1>/PWM' */
  MW_PWM_SetDutyCycle
    (LabB_CheckCommunications_DWork.obj_i.PWMDriverObj.MW_PWM_HANDLE, u0);

  /* Update for DiscreteIntegrator: '<S1>/angular velocity to angle' incorporates:
   *  Constant: '<S1>/gyro velocity bias'
   *  DataTypeConversion: '<S1>/Data Type Conversion'
   *  Gain: '<S1>/raw x velocity to deg. // sec.'
   *  Gain: '<S4>/Gain1'
   *  MATLABSystem: '<S1>/Gyroscope'
   *  Sum: '<S1>/Sum2'
   * */
  LabB_CheckCommunications_DWork.angularvelocitytoangle_DSTATE +=
    (LabB_CheckCommunications_P.rawxvelocitytodegsec_Gain * (real_T)out[0] *
     LabB_CheckCommunications_P.Gain1_Gain +
     LabB_CheckCommunications_P.gyrovelocitybias_Value) *
    LabB_CheckCommunications_P.angularvelocitytoangle_gainval;

  /* Update for Sin: '<Root>/Sine Wave' */
  rtb_Switch = LabB_CheckCommunications_DWork.lastSin;
  LabB_CheckCommunications_DWork.lastSin =
    LabB_CheckCommunications_DWork.lastSin *
    LabB_CheckCommunications_P.SineWave_HCos +
    LabB_CheckCommunications_DWork.lastCos *
    LabB_CheckCommunications_P.SineWave_Hsin;
  LabB_CheckCommunications_DWork.lastCos =
    LabB_CheckCommunications_DWork.lastCos *
    LabB_CheckCommunications_P.SineWave_HCos - rtb_Switch *
    LabB_CheckCommunications_P.SineWave_Hsin;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  LabB_CheckCommunications_M->Timing.clockTick0++;
}

/* Model initialize function */
void LabB_CheckCommunications_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)LabB_CheckCommunications_M, 0,
                sizeof(RT_MODEL_LabB_CheckCommunicatio));
  rtmSetTFinal(LabB_CheckCommunications_M, -1);

  /* External mode info */
  LabB_CheckCommunications_M->Sizes.checksums[0] = (1222194246U);
  LabB_CheckCommunications_M->Sizes.checksums[1] = (3515119507U);
  LabB_CheckCommunications_M->Sizes.checksums[2] = (1792380461U);
  LabB_CheckCommunications_M->Sizes.checksums[3] = (4168725295U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[8];
    LabB_CheckCommunications_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(LabB_CheckCommunications_M->extModeInfo,
      &LabB_CheckCommunications_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(LabB_CheckCommunications_M->extModeInfo,
                        LabB_CheckCommunications_M->Sizes.checksums);
    rteiSetTFinalTicks(LabB_CheckCommunications_M->extModeInfo, -1);
  }

  /* block I/O */
  (void) memset(((void *) &LabB_CheckCommunications_B), 0,
                sizeof(BlockIO_LabB_CheckCommunication));

  /* states (dwork) */
  (void) memset((void *)&LabB_CheckCommunications_DWork, 0,
                sizeof(D_Work_LabB_CheckCommunications));

  /* Start for MATLABSystem: '<S1>/M2V3 Left Connector 18, 19' */
  /*  Constructor */
  /* 'Encoder_arduino:36' coder.allowpcode('plain'); */
  /* 'Encoder_arduino:1' matlab.System */
  /* 'Encoder_arduino:2' coder.ExternalDependency */
  /* 'Encoder_arduino:3' matlab.system.mixin.Propagates */
  /* 'Encoder_arduino:4' matlab.system.mixin.CustomIcon */
  /*  Support name-value pair arguments when constructing the object. */
  /* 'Encoder_arduino:39' setProperties(obj,nargin,varargin{:}); */
  LabB_CheckCommunications_DWork.obj_c.matlabCodegenIsDeleted = false;

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
  LabB_CheckCommunications_DWork.obj_c.isInitialized = 1L;

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
  LabB_CheckCommunications_DWork.obj_c.isSetupComplete = true;

  /* Start for MATLABSystem: '<S1>/Accelerometer' */
  /* 'soMPU6050Accel:1' matlab.System */
  /* 'soMPU6050Accel:1' coder.ExternalDependency */
  LabB_CheckCommunications_DWork.obj.matlabCodegenIsDeleted = false;
  LabB_CheckCommunications_DWork.obj.isInitialized = 1L;

  /* 'soMPU6050Accel:29' if coder.target('Rtw') */
  /*  done only for code gen */
  /* 'soMPU6050Accel:30' coder.cinclude('MPU6050wrapper.h'); */
  /*  initialize the potentiometer */
  /* 'soMPU6050Accel:32' coder.ceval('MPU6050Accel_Init'); */
  MPU6050Accel_Init();
  LabB_CheckCommunications_DWork.obj.isSetupComplete = true;

  /* Start for MATLABSystem: '<S1>/Gyroscope' */
  /*  Constructor */
  /* 'soMPU6050Gyro:27' coder.allowpcode('plain'); */
  /* 'soMPU6050Gyro:1' matlab.System */
  /* 'soMPU6050Gyro:1' coder.ExternalDependency */
  /*  Support name-value pair arguments when constructing the object. */
  /* 'soMPU6050Gyro:30' setProperties(obj,nargin,varargin{:}); */
  LabB_CheckCommunications_DWork.obj_p.matlabCodegenIsDeleted = false;

  /* 'soMPU6050Gyro:34' coder.extrinsic('sprintf'); */
  /*  Do not generate code for sprintf */
  /* 'soMPU6050Gyro:35' validateattributes(value,... */
  /* 'soMPU6050Gyro:36'                 {'numeric'},... */
  /* 'soMPU6050Gyro:37'                 {'real','nonnegative','integer','scalar'},... */
  /* 'soMPU6050Gyro:38'                 '', ... */
  /* 'soMPU6050Gyro:39'                 'DLPFmode'); */
  /* 'soMPU6050Gyro:40' obj.DLPFmode = value; */
  LabB_CheckCommunications_DWork.obj_p.isInitialized = 1L;

  /* 'soMPU6050Gyro:46' if coder.target('Rtw') */
  /*  done only for code gen */
  /* 'soMPU6050Gyro:47' coder.cinclude('MPU6050wrapper.h'); */
  /*  initialize the sensor */
  /*                  coder.ceval('MPU6050Accel_Init'); */
  /* 'soMPU6050Gyro:51' coder.ceval('MPU6050Gyro_Init', obj.DLPFmode); */
  MPU6050Gyro_Init(0.0);
  LabB_CheckCommunications_DWork.obj_p.isSetupComplete = true;

  /* Start for MATLABSystem: '<S1>/Digital Output1' */
  LabB_CheckCommunications_DWork.obj_l.matlabCodegenIsDeleted = false;
  LabB_CheckCommunications_DWork.obj_l.isInitialized = 1L;
  digitalIOSetup(6, 1);
  LabB_CheckCommunications_DWork.obj_l.isSetupComplete = true;

  /* Start for MATLABSystem: '<S1>/PWM' */
  LabB_CheckCommunications_DWork.obj_i.matlabCodegenIsDeleted = false;
  LabB_CheckCommunications_DWork.obj_i.isInitialized = 1L;
  LabB_CheckCommunications_DWork.obj_i.PWMDriverObj.MW_PWM_HANDLE = MW_PWM_Open
    (8UL, 0.0, 0.0);
  LabB_CheckCommunications_DWork.obj_i.isSetupComplete = true;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/angular velocity to angle' */
  LabB_CheckCommunications_DWork.angularvelocitytoangle_DSTATE =
    LabB_CheckCommunications_P.angularvelocitytoangle_IC;

  /* Enable for Sin: '<Root>/Sine Wave' */
  LabB_CheckCommunications_DWork.systemEnable = 1L;
}

/* Model terminate function */
void LabB_CheckCommunications_terminate(void)
{
  /* Terminate for MATLABSystem: '<S1>/M2V3 Left Connector 18, 19' */
  if (!LabB_CheckCommunications_DWork.obj_c.matlabCodegenIsDeleted) {
    LabB_CheckCommunications_DWork.obj_c.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S1>/M2V3 Left Connector 18, 19' */
  /* Terminate for MATLABSystem: '<S1>/Accelerometer' */
  if (!LabB_CheckCommunications_DWork.obj.matlabCodegenIsDeleted) {
    LabB_CheckCommunications_DWork.obj.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S1>/Accelerometer' */
  /* Terminate for MATLABSystem: '<S1>/Gyroscope' */
  if (!LabB_CheckCommunications_DWork.obj_p.matlabCodegenIsDeleted) {
    LabB_CheckCommunications_DWork.obj_p.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S1>/Gyroscope' */

  /* Terminate for MATLABSystem: '<S1>/Digital Output1' */
  if (!LabB_CheckCommunications_DWork.obj_l.matlabCodegenIsDeleted) {
    LabB_CheckCommunications_DWork.obj_l.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S1>/Digital Output1' */

  /* Terminate for MATLABSystem: '<S1>/PWM' */
  if (!LabB_CheckCommunications_DWork.obj_i.matlabCodegenIsDeleted) {
    LabB_CheckCommunications_DWork.obj_i.matlabCodegenIsDeleted = true;
    if ((LabB_CheckCommunications_DWork.obj_i.isInitialized == 1L) &&
        LabB_CheckCommunications_DWork.obj_i.isSetupComplete) {
      LabB_CheckCommunications_DWork.obj_i.PWMDriverObj.MW_PWM_HANDLE =
        MW_PWM_GetHandle(8UL);
      MW_PWM_SetDutyCycle
        (LabB_CheckCommunications_DWork.obj_i.PWMDriverObj.MW_PWM_HANDLE, 0.0);
      LabB_CheckCommunications_DWork.obj_i.PWMDriverObj.MW_PWM_HANDLE =
        MW_PWM_GetHandle(8UL);
      MW_PWM_Close
        (LabB_CheckCommunications_DWork.obj_i.PWMDriverObj.MW_PWM_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S1>/PWM' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
