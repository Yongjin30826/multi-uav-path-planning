y1 = load('ACOPA4PathPlanning\results\ACOPAProb1Dim20Data.txt');
y11=y1(100002:150001);
log10_y11 = log10(y11);
x = 1:50000;
plot(x,log10_y11);
xlabel('FEs');
ylabel('Cost (log) ');
legend('ACOPA4');