 iNumberOfEncoderSteps	= 720;
fGyroConversionFactor	= 250 / 32768 % -1/131;
%fGyroConversionFactor	= -1/131;
fWheelRadius			= 0.0216; % [m]
load('GyroBias.mat');

LabC_solutions;

%%
Nxd = 0
Nud= inv(C_acc*inv(eye(4)-Ad+Bd*Kd)*Bd)
