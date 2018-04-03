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

out = Calibration(LOC, RSS, AP_NUM, date);
end
