function out = InputCalib(date, iter, mac6)
fileStr = ['measurements/' date '-' iter '-' mac6 '.txt'];
file_input = fopen(fileStr, 'r');

formatSpec = ['%f' '%f'];
sizeInput = [7 Inf];

TIME = null(1);
LOC = null(1, 2);
RSS = null(1);
MAC_OWN = null(1);
AP = null(1);

while ~feof(file_input)
	input = fscanf(file_input, '%f');
	TIME = [TIME; input];
	input = fscanf(file_input, '%[0123456789ABCDEF:]');
	MAC_OWN = [MAC_OWN; input];
	input = fscanf(file_input, '%f %f %f');
	LOC = [LOC; input(2) input(3)];
	input = fscanf(file_input, '%[0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ_"()]');
	AP = [AP; input];
	input = fscanf(file_input, ['%f' '\n'], [1]);
	RSS = [RSS; input];
end
fclose(file_input);

AP_NUM = zeros(length(AP(:,1)), 1);

for i = 1:length(AP(:, 1))
	if AP(i, :) == '"MWNL_MSJANG(5G)"'
		AP_NUM(i) = 15;
	elseif AP(i, :) == '"MWNL_MAP1(5G)"'
		AP_NUM(i) = 0;
	elseif AP(i, :) == '"MWNL_MAP2(5G)"'
		AP_NUM(i) = 1;
	end
end

DEVICE_MAC = null(1,1);
DEVICE_MAC = [DEVICE_MAC; MAC_OWN(1,:)];
for i = 1:length(MAC_OWN(:,1))
	new_dev = 1;
	for j = 1:length(DEVICE_MAC(:,1))
		if DEVICE_MAC(j,:) == MAC_OWN(i,:)
			new_dev = 0;
		end
	end
	if new_dev == 1
		DEVICE_MAC = [DEVICE_MAC; MAC_OWN(i,:)];
	end
end

for i = 1:length(DEVICE_MAC(:,1))
	fileStr = ['coef_results/' DEVICE_MAC(i,:) '.txt'];
	file_output = fopen(fileStr, 'w');
	for j = 1:length(MAC_OWN(:,1))
		if MAC_OWN(j,:) == DEVICE_MAC(i,:)
			fprintf(file_output, '%.f %.f %.f %.f %.f \n', [TIME(j) LOC(j,1) LOC(j,2) AP_NUM(j) RSS(j)]);
		end
	end
	fclose(file_output);
end

% out = Calibration(LOC, RSS, AP_NUM, date);
end
