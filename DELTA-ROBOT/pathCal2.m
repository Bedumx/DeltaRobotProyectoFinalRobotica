function pose2=pathCal2(n,r)

p=1*pi/n;
t = 0 : p : 1 * pi;
x = 0.001*t%r .* cos(t);
y = r .* cos(t)%r .* sin(t);
z = r .* sin(t)%1.*t.^2;
pose2=[x'-50,y',z'-400];

% figure(1)
% plot(x,'b','LineWidth', 1);
% hold on
% grid on
% title('X obtained vs X desired');legend('X-ob','Xdes');xlabel('t');ylabel('X-ob, X-des');
% figure(2)
% plot(y,'b','LineWidth', 1);
% hold on
% grid on
% title('X obtained vs X desired');legend('X-ob','Xdes');xlabel('t');ylabel('X-ob, X-des');
% figure(3)
% plot(z,'b','LineWidth', 1);
% hold on
% grid on
% title('X obtained vs X desired');legend('X-ob','Xdes');xlabel('t');ylabel('X-ob, X-des');

end