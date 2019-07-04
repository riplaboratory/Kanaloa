% logarithmic mapping

x = linspace(-10,10,10000);
y = log(x);
y2 = 2*log(x);
y3 = log(2*x);

close all;
figure(1);
plot(x,y,...
    x,y2,...
    x,y3);