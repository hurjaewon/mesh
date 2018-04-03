function out = Calibration(LOC, RSS, AP, str)

%%% Argument check
%---------------------------------------------------------------------------

validateattributes(LOC, {'numeric'}, {'2d', 'finite'});
validateattributes(RSS, {'numeric'}, {'2d', 'finite'});
validateattributes(AP, {'numeric'}, {'2d', 'finite'});

[n_rows_loc, n_cols_loc] = size(LOC);
[n_rows_rss, n_cols_rss] = size(RSS);
[n_rows_ap, n_cols_ap] = size(AP);

if n_cols_loc ~= 2 || n_cols_rss ~= 1 || n_cols_ap ~= 1
    error('Arguments must be Nx2, Nx1, Nx1 matrces.');
end

if n_rows_loc ~= n_rows_rss || n_rows_rss ~= n_rows_ap || n_rows_ap ~= n_rows_loc
    error('Num of rows of arguments must be same');
end

outlet = [12 21; 18 21; 8 17; 15 14.5; 13 13; 8 10.5; 13 10.5; 18 10.5; 3 9; 7 8; 10 6; 14 6; 4 5; 11 5; 9.1 5.5];
wall = [0 0 0 21.6; 3.6 0 3.6 10.8; 3.6 13.7 3.6 21.6; 9 5.4 9 10.8; 9 13.7 9 21.6; 11.7 0 11.7 5.4; 19.8 0 19.8 10.8; 19.8 13.7 19.8 21.6; 
    3.6 0 19.8 0; 3.6 5.4 19.8 5.4; 3.6 10.8 19.8 10.8; 3.6 13.7 19.8 13.7; 0 13.7 3.6 13.7; 0 17.3 3.6 17.3; 0 21.6 19.8 21.6]; 

AP_TX_POW = 13;
AP_FREQ = 2.41;
TMP = 20 * log10(AP_FREQ/5);



AP_LOC = zeros(n_rows_ap, 2);
AP_LOC(:, 1) = outlet(AP, 1);
AP_LOC(:, 2) = outlet(AP, 2);

loc = zeros(n_rows_loc, 4);
loc(:, 1) = LOC(:, 1);
loc(:, 2) = LOC(:, 2);
loc(:, 3) = AP_LOC(:, 1);
loc(:, 4) = AP_LOC(:, 2);

ints = lineSegmentIntersect(loc, wall);

dist = sqrt((loc(:, 1) - loc(:, 3)).^2 + (loc(:, 2) - loc(:, 4)).^2);
logd = log10(dist);

num_ints = zeros(n_rows_loc, 1);
for i = 1:n_rows_loc
    num_ints(i) = sum(ints.intAdjacencyMatrix(i, :));
end

logd_los = zeros(1, 1);
LOS_RSS = zeros(1, 1);
for i = 1:n_rows_loc
    if num_ints(i) == 0
        logd_los = [logd_los; logd(i)];
        LOS_RSS = [LOS_RSS; RSS(i)];
    end
end
logd_los = logd_los(2:length(logd_los), 1);
LOS_RSS = LOS_RSS(2:length(LOS_RSS), 1);

H1 = zeros(length(logd_los), 2);
H1(:, 1) = 1;
H1(:, 2) = logd_los;

y1 = AP_TX_POW - LOS_RSS - TMP;

pinvH1 = pinv(H1);
xhat1 = pinvH1 * y1;

H2 = zeros(n_rows_loc, 17);
H2(:, 1) = 1;
H2(:, 2) = logd;
H2(:, 3:17) = ints.intAdjacencyMatrix;

y2 = AP_TX_POW - RSS - TMP;

pinvH2 = pinv(H2);
xhat2 = pinvH2 * y2;

for i = 1:length(xhat2)
	if xhat2(i) == 0
		xhat2(i) = 12;
	end
end

xhat1
xhat2

out.xhat1 = xhat1;
out.xhat2 = xhat2;
out.H1 = H1;
out.H2 = H2;
out.ints = ints.intAdjacencyMatrix;

outStr = ['coef_results/Result-' str '.txt'];
file_output = fopen(outStr, 'w');
fprintf(file_output, '%f %f\n', out.xhat1);
fprintf(file_output, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n', out.xhat2);
fclose(file_output);

end



