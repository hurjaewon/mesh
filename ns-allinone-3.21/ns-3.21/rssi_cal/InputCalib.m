function out = InputCalib(str)
fileStr = ['measurements/' str '-log.dat'];
file_input = fopen(fileStr, 'r');

input = fscanf(file_input, '%f');

LOC = zeros(length(input)/4, 2);
RSS = zeros(length(input)/4, 1);
AP = zeros(length(input)/4, 1);

a = 1;
b = 1;
c = 1;
d = 1;
for i=1:length(input)
	if mod(i, 4) == 1
		LOC(a, 1) = input(i);
		a = a + 1;
	elseif mod(i, 4) == 2
		LOC(b, 2) = input(i);
		b = b + 1;
	elseif mod(i, 4) == 3
		RSS(c) = input(i);
		c = c + 1;
	else
		AP(d) = input(i);
		d = d + 1;
	end
end

fclose(file_input);
str = str(1:6);
out = Calibration(LOC, RSS, AP, str);
end
