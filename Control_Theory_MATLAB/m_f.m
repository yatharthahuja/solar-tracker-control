x = linspace(-255, 255, 10000000);
y1 = trimf(x, [-1700,-255,-205] );
y2 = trimf(x, [-225,-155,-75]);
y3 = trimf(x, [-105,0,105]);
y4 = trimf(x, [75,155,225]);
y5 = trimf(x, [205,255,1700]);
p1 = plot(x,y1)
hold on 
p2 = plot(x,y2)
hold on 
p3 = plot(x,y3)
hold on
p4 = plot(x,y4)
hold on 
p5 = plot(x,y5)
%xlabel('Triangular MF');
legend([p1 p2 p3 p4 p5],'NL','NS','ZE','PS','PL')
ylim([0 1]);
hold off