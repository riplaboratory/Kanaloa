% Sliding observation profiling function
yMax = 1;       % max value of function (usually 1)
yMin = 0;       % min value of function (usually 0)
tDown = 1;      % when the function should reach y = min [s]
tUp = 0.1;      % when the function should reach y = max [s]

slopeUp = yMax/tUp;
slopeDown = -yMax/tDown;
flip = tUp;


x = linspace(0,2,1000);
% y = piecewise(x < 0, yMin,...
%     0 < x < tUp, slopeUp*x,...
%     tUp < x < tDown, slopeDown*x,...
%     x > tDown, yMin);

test(10);

function test(asdf)

    disp(asdf);

end