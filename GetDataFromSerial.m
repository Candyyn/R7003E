% % cleaning
clear all;
clc;
close all;
fSamplingPeriod = 0.01


if exist('tSerialCommunication', 'var') && isvalid(tSerialCommunication)
    clear tSerialCommunication
end

% basic parameters
% fSamplingPeriod	= should be already loaded in the workspace
iCommunicationTime	= 30;		% [sec]
iCOMPort			= '/dev/ttyACM0';
%iCOMPort			= '3';
fPlotsUpdatesPeriod	= 1;		% [sec]
%
% advanced parameters
iNumberOfSignals	= 7;
strByteOrder		= 'bigEndian';
iBaudRate			= 115200;
strDataType			= 'uint8';


% placeholders
U_INDEX						= 1;
MEASURED_X_W_INDEX			= 2;
MEASURED_THETA_B_INDEX		= 3;
X_W_HAT_FULL_INDEX			= 4;
THETA_B_HAT_FULL_INDEX		= 5;
X_W_HAT_REDUCED_INDEX		= 6;
THETA_B_HAT_REDUCED_INDEX	= 7;
MAIN_FIGURE_INDEX			= 8;


% allocate the memory for receiving the information
% and some auxiliary variables
iNumberOfPackets				= ceil(iCommunicationTime / fSamplingPeriod);
iNumberOfBytesPerPacket			= 1 + 2 * iNumberOfSignals;	% the first byte is a "1" useful for understanding where the packet starts
aafReceivedInformation			= zeros(iNumberOfBytesPerPacket, iNumberOfPackets);
aafProcessedInformation			= zeros(iNumberOfSignals, iNumberOfPackets);
iPacketIndexForStartingPlotting	= 20;
iInitialValidPacket				= 5;
afTimes							= (1:iNumberOfPackets) * fSamplingPeriod;
iNumberOfPacketsToPlotPerTime	= ceil( fPlotsUpdatesPeriod / fSamplingPeriod );


% allocate the structure for the serial communication 
%if( ~exist('tSerialCommunication') )
	%
%	fprintf('Opening the serial communications...');
	%
%	tSerialCommunication = serial(iCOMPort);
%	set(tSerialCommunication, 'ByteOrder', strByteOrder);
%	set(tSerialCommunication, 'BaudRate', iBaudRate); 
% 	tSerialCommunication.ReadAsyncMode = 'continuous';
% 	fprintf(tSerialCommunication,'*IDN?')
% 	tSerialCommunication.BytesAvailable
%	fopen(tSerialCommunication);
%	pause(1); % ???
	%
%	fprintf('...done\n');
	%
%end;%

% allocate the structure for the serial communication  fix cause old sstuff
if ~(exist('tSerialCommunication', 'var') && isvalid(tSerialCommunication))
    fprintf('Opening the serial communications...\n');

    % Use modern serialport
    tSerialCommunication = serialport(iCOMPort, iBaudRate, ...
        "ByteOrder", "big-endian");   % matches strByteOrder

    %configureTerminator(tSerialCommunication, "none");  % binary protocol, no terminator
    flush(tSerialCommunication);                        % clear any old bytes

    pause(1);  % tiny delay for the connection to settle
    fprintf('...done\n');
end


% initialize the data plotting
InitializePlotDataFromSerial;
 

% cycle on receiving the information
fprintf('Receiving the serial packets:\n');
for iPacketIndex = 1:iNumberOfPackets;
	%
	% read the current packet
	afCurrentPacket = fread(tSerialCommunication, iNumberOfBytesPerPacket, strDataType);
	%
	if(length(afCurrentPacket) == iNumberOfBytesPerPacket)
		%
		aafReceivedInformation(:, iPacketIndex) = afCurrentPacket';
		%
	end;%
	%
	% when one has received a sufficient number of packets then
	% find where the "1" is inside the aafInformation matrix
	% (truncate also the received information by eliminating
	% the first initial packets, that may be corrupted)
	if( iPacketIndex == iPacketIndexForStartingPlotting )
		%
		[~, iIndexOf1] = min( sum(aafReceivedInformation(:, iInitialValidPacket:iPacketIndexForStartingPlotting)' - 1).^2 );
		%
	end;% if find the index of 1
	%
	% if we have received enough packets then reconstruct the signals
	if( iPacketIndex > iPacketIndexForStartingPlotting )
		%
		% do the reconstruction one signal per time
		for iSignal = 1:iNumberOfSignals;
			%
			% indexes of the signal starting from iIndexOf1
			iHighByteIndex	= iIndexOf1 + (iSignal - 1) * 2 + 1;
			iLowByteIndex	= iIndexOf1 + (iSignal - 1) * 2 + 2;
			%
			% re-map the indexes so to point inside the aafInformation matrix
			if( iHighByteIndex > iNumberOfBytesPerPacket);
				%
				iHighByteIndex = iHighByteIndex - iNumberOfBytesPerPacket;
				%
			end;%
			if( iLowByteIndex > iNumberOfBytesPerPacket );
				%
				iLowByteIndex = iLowByteIndex - iNumberOfBytesPerPacket;
				%
			end;%
			%
			% do some magic tricks (that means fix the fact that we got the data in bytes)
			aafProcessedInformation(iSignal, iPacketIndex)	=							...
				(																		...
					(																	...
							aafReceivedInformation(iLowByteIndex, iPacketIndex)			...
						+	256 * aafReceivedInformation(iHighByteIndex, iPacketIndex)	...
					) / 3276.8															...
				) - 10;
			%
		end;% for on the individual signals
		%
	end;% if we have received enough packets
	%
	% print some useful information on the shell
	if( ~mod(iPacketIndex, iNumberOfPacketsToPlotPerTime) )
		%
		fprintf('%3d out of %3d\n', iPacketIndex, iNumberOfPackets);
		%
		% plot also the current estimation errors
		PlotDataFromSerial;
		%
	end;% print on the shell
	%
end;% cycle on the packets
 

%% ===============================
% SELECT WHICH EXPERIMENT THIS RUN IS
% ===============================

EXPERIMENT = 2;     % <-- CHANGE: 1 or 2
MYGROUP = 7;        % <-- your group number

% Only relevant for experiment 2
r_max = 0.130;       % <-- set your found value


%% ===============================
% Convert signals to column vectors
% ===============================

times_col      = afTimes';
encoder_col    = aafProcessedInformation(MEASURED_X_W_INDEX, :)';
angle_col      = aafProcessedInformation(MEASURED_THETA_B_INDEX, :)';
actuation_col  = aafProcessedInformation(U_INDEX, :)';


%% ===============================
% Load previous file if it exists
% ===============================

filename = sprintf('group_%d_results.mat', MYGROUP);

if isfile(filename)

    load(filename)

    fprintf("Loaded existing results file\n")

else

    fprintf("Creating new results file\n")

end


%% ===============================
% Store data depending on experiment
% ===============================

if EXPERIMENT == 1

    fprintf("Saving Experiment 1 data\n")

    eval(sprintf('group_%d_experiment_1_times = times_col;', MYGROUP));
    eval(sprintf('group_%d_experiment_1_encoder = encoder_col;', MYGROUP));
    eval(sprintf('group_%d_experiment_1_angle = angle_col;', MYGROUP));
    eval(sprintf('group_%d_experiment_1_actuation = actuation_col;', MYGROUP));

elseif EXPERIMENT == 2

    fprintf("Saving Experiment 2 data\n")

    eval(sprintf('group_%d_experiment_2_times = times_col;', MYGROUP));
    eval(sprintf('group_%d_experiment_2_encoder = encoder_col;', MYGROUP));
    eval(sprintf('group_%d_experiment_2_angle = angle_col;', MYGROUP));
    eval(sprintf('group_%d_experiment_2_actuation = actuation_col;', MYGROUP));
    eval(sprintf('group_%d_r_max = r_max;', MYGROUP));

else

    error("EXPERIMENT must be 1 or 2")

end



%% ===============================
% Ensure all required variables exist
% ===============================

varNames = {
    sprintf('group_%d_experiment_1_times', MYGROUP)
    sprintf('group_%d_experiment_1_encoder', MYGROUP)
    sprintf('group_%d_experiment_1_angle', MYGROUP)
    sprintf('group_%d_experiment_1_actuation', MYGROUP)
    sprintf('group_%d_experiment_2_times', MYGROUP)
    sprintf('group_%d_experiment_2_encoder', MYGROUP)
    sprintf('group_%d_experiment_2_angle', MYGROUP)
    sprintf('group_%d_experiment_2_actuation', MYGROUP)
    sprintf('group_%d_r_max', MYGROUP)
};

for i = 1:length(varNames)

    if ~exist(varNames{i}, 'var')

        if contains(varNames{i}, 'r_max')
            eval([varNames{i} ' = [];']);
        else
            eval([varNames{i} ' = [];']);
        end

    end

end


%% ===============================
% Save ALL required variables
% ===============================


save(filename, ...
    sprintf('group_%d_experiment_1_times', MYGROUP), ...
    sprintf('group_%d_experiment_1_encoder', MYGROUP), ...
    sprintf('group_%d_experiment_1_angle', MYGROUP), ...
    sprintf('group_%d_experiment_1_actuation', MYGROUP), ...
    sprintf('group_%d_experiment_2_times', MYGROUP), ...
    sprintf('group_%d_experiment_2_encoder', MYGROUP), ...
    sprintf('group_%d_experiment_2_angle', MYGROUP), ...
    sprintf('group_%d_experiment_2_actuation', MYGROUP), ...
    sprintf('group_%d_r_max', MYGROUP) ...
);

fprintf("Saved file:\n")
disp(filename)









% close the serial communications
fprintf('Closing the serial communications...');
fclose(tSerialCommunication);
delete(tSerialCommunication);
clear tSerialCommunication;
fprintf('...done\n');



