purge

time = 0:0.0001:10;
l = length(time);

rt = 200*cos(time);
a = 1000;
b = 1;
sys = tf([64*64 0],[1 2*0.7*64 64*64])

bode(sys);

%%use filter
yt = lsim(sys,rt,time);

figure()
plot(time,rt)
hold on
plot(time,yt,'r-')
legend('input','output')
