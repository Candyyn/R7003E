clear all;
close all;
clc;

% DO NOT MODIFY THIS!
iNumberOfEncoderSteps	= 720;
fGyroConversionFactor	= -1/131; % Change this based on id on robot
fWheelRadius			= 0.0216; % [m]
load('GyroBias.mat');

% if you want, modify this
fSamplingPeriod			= 0.005; % note: 0.005 is the fastest sampling time with the default mpu5060 library settings

% load the PID values
kP = -511.1067 %-69.6802;
kI = -2857.5569%-2.86*10^3%-390.4363;
kD = -1.0633%-1.68 %-0.1302;

