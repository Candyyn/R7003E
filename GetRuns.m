%% cleaning
clear all;
clc;
close all;

%fSamplingPeriod = 0.01;   % 10 Hz

%%% NEW: title/label for this run
runTitle = '200Hz';
freq = 200;

fSamplingPeriod = 1/freq;

%% basic parameters
iCommunicationTime  = 30;      % [sec]
iCOMPort            = '/dev/ttyACM0';
fPlotsUpdatesPeriod = 1;         % [sec]

%% advanced parameters
iNumberOfSignals    = 7;
strByteOrder        = 'bigEndian';
iBaudRate           = 115200;
strDataType         = 'uint8';

%% placeholders
U_INDEX                     = 1;
MEASURED_X_W_INDEX          = 2;
MEASURED_THETA_B_INDEX      = 3;
X_W_HAT_FULL_INDEX          = 4;
THETA_B_HAT_FULL_INDEX      = 5;
X_W_HAT_REDUCED_INDEX       = 6;
THETA_B_HAT_REDUCED_INDEX   = 7;
MAIN_FIGURE_INDEX           = 8;

%% allocate memory
iNumberOfPackets                = ceil(iCommunicationTime / fSamplingPeriod);
iNumberOfBytesPerPacket         = 1 + 2 * iNumberOfSignals;
aafReceivedInformation          = zeros(iNumberOfBytesPerPacket, iNumberOfPackets);
aafProcessedInformation         = zeros(iNumberOfSignals, iNumberOfPackets);
iPacketIndexForStartingPlotting = 20;
iInitialValidPacket             = 5;
afTimes                         = (1:iNumberOfPackets) * fSamplingPeriod;
iNumberOfPacketsToPlotPerTime   = ceil(fPlotsUpdatesPeriod / fSamplingPeriod);

%%% NEW: storage container for this run
runData = struct();

%% serial communication (modern MATLAB)
if ~(exist('tSerialCommunication', 'var') && isvalid(tSerialCommunication))
    fprintf('Opening the serial communications...\n');

    tSerialCommunication = serialport(iCOMPort, iBaudRate, ...
        "ByteOrder", "big-endian");

    flush(tSerialCommunication);
    pause(1);

    fprintf('...done\n');
end

%% initialize plotting
InitializePlotDataFromSerial;

%%% NEW: set figure title
sgtitle(runTitle);

%% receive packets
fprintf('Receiving the serial packets:\n');

for iPacketIndex = 1:iNumberOfPackets

    afCurrentPacket = fread(tSerialCommunication, ...
        iNumberOfBytesPerPacket, strDataType);

    if length(afCurrentPacket) == iNumberOfBytesPerPacket
        aafReceivedInformation(:, iPacketIndex) = afCurrentPacket';
    end

    if iPacketIndex == iPacketIndexForStartingPlotting
        [~, iIndexOf1] = min( ...
            sum(aafReceivedInformation(:, ...
            iInitialValidPacket:iPacketIndexForStartingPlotting)' - 1).^2 );
    end

    if iPacketIndex > iPacketIndexForStartingPlotting
        for iSignal = 1:iNumberOfSignals

            iHighByteIndex = iIndexOf1 + (iSignal - 1) * 2 + 1;
            iLowByteIndex  = iIndexOf1 + (iSignal - 1) * 2 + 2;

            if iHighByteIndex > iNumberOfBytesPerPacket
                iHighByteIndex = iHighByteIndex - iNumberOfBytesPerPacket;
            end
            if iLowByteIndex > iNumberOfBytesPerPacket
                iLowByteIndex = iLowByteIndex - iNumberOfBytesPerPacket;
            end

            aafProcessedInformation(iSignal, iPacketIndex) = ...
                ((aafReceivedInformation(iLowByteIndex, iPacketIndex) + ...
                 256 * aafReceivedInformation(iHighByteIndex, iPacketIndex)) ...
                 / 3276.8) - 10;
        end
    end

    if ~mod(iPacketIndex, iNumberOfPacketsToPlotPerTime)
        fprintf('%3d out of %3d\n', iPacketIndex, iNumberOfPackets);
        PlotDataFromSerial;
    end
end

%% close serial
fprintf('Closing the serial communications...');
clear tSerialCommunication;
fprintf('...done\n');

%% ===========================
%% SAVE RUN DATA
%% ===========================

%%% NEW: store everything from this run
runData.title           = runTitle;
runData.samplingPeriod  = fSamplingPeriod;
runData.time            = afTimes;
runData.rawBytes        = aafReceivedInformation;
runData.signals         = aafProcessedInformation;
load('allRuns.mat', 'allRuns');
%%% NEW: append to collection of runs
if ~exist('allRuns', 'var')
    allRuns = {};
end
allRuns{end+1} = runData;

%%% OPTIONAL: save to disk
save('allRuns.mat', 'allRuns');
