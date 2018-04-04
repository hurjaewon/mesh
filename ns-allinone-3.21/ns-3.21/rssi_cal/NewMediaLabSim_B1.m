function out = NewMediaLabSim_B1(TX_X, TX_Y, TX_POW, TX_GAIN, f)

% Building size 30.6m x 21.6m
% Wall edge (0,0), (10.8, 0), (14.4, 13.5), (23.4, 13.5), (30.6, 13.5), (0,
% 21.6), (10.8, 21.6), (23.4, 21.6), (30.6, 21.6)
% MPP location : (0.1, 0.1)
% grid step 1m

freq = f; % GHz

tx_x = TX_X;
tx_y = TX_Y;
len_x = 197;
len_y = 215;

wall = [0 0 0 21.6; 3.6 0 3.6 10.8; 3.6 13.7 3.6 21.6; 9 5.4 9 10.8; 9 13.7 9 21.6; 11.7 0 11.7 5.4; 19.8 0 19.8 10.8; 19.8 13.7 19.8 21.6; 
    3.6 0 19.8 0; 3.6 5.4 19.8 5.4; 3.6 10.8 19.8 10.8; 3.6 13.7 19.8 13.7; 0 13.7 3.6 13.7; 0 17.3 3.6 17.3; 0 21.6 19.8 21.6]; 

loc = zeros(len_x * len_y, 4);
loc(:, 1) = tx_x;
loc(:, 2) = tx_y;

grid_x = zeros(len_x*len_y, 1);
grid_y = zeros(len_x*len_y, 1);

for i = 1:len_x
    for j = 1:len_y
        grid_x(len_y * (i-1) + j) = i / 10;
        grid_y(len_y * (i-1) + j) = j / 10;
    end
end

loc(:, 3) = grid_x;
loc(:, 4) = grid_y;

ints = lineSegmentIntersect(loc, wall);

num_intersect = zeros(len_x * len_y, 1);
wall_loss = zeros(length(wall), 1);
wallpl = zeros(len_x * len_y, 1);

for i = 1:len_x*len_y
    num_intersect(i) = sum(ints.intAdjacencyMatrix(i, :));
end

wall_loss(:) = 12;

% LSE calibration LOS W1 = 8.02 W2 = 5.66 // NLOS W1 = 7.90, W2 = 5.58;
wall_loss(11) = 7.58;
wall_loss(12) = 5.66;

wallpl = ints.intAdjacencyMatrix * wall_loss;

dist = zeros(len_x * len_y, 1);
for i = 1:len_x*len_y
    dist(i) = sqrt((tx_x - loc(i,3))^2 + (tx_y - loc(i,4))^2);
end

A = zeros(len_x * len_y, 1);
B = zeros(len_x * len_y, 1);

for i = 1:len_x*len_y
    
    if num_intersect(i) == 0
        A(i) = 59.53;
        B(i) = 20.98;
    else
        A(i) = 59.19;
        B(i) = 21.52;
    end
end

% LSE calibrated LOS A :59.53  B :20.98 // NLOS A : 59.19, B : 21.52
% Winner 2 model LOS A: 46.8, B :18.7  // NLOS A : 46.4, B :20 
% Following WINNER 2 model, room to room LOS/NLOS scenario, same floor,
% heavy wall loss

dist2log = zeros(len_x * len_y, 1);
for i = 1:len_x*len_y
    if dist(i) < 1
        dist2log(i) = 0;
    else
        dist2log(i) = log10(dist(i));
    end
end

openpl = A + B.*dist2log + 20*log10(freq/5);   %dB, (following WINNER 2)
              
% wallpl = wall_loss * num_intersect; %dB, (following WINNER 2)
realpl = -openpl - wallpl;

map_pl = zeros(len_x, len_y);
for i = 1:len_x
    for j = 1:len_y
        map_pl(i, j) = realpl(len_y * (i-1) + j);
    end
end

surf(map_pl')

out.real_pl = realpl;
out.map_pl = map_pl;
out.map_pow = TX_POW + TX_GAIN + map_pl;  %dBm

end