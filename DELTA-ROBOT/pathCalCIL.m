function pose=pathCalCIL(n,r,x0,y0,z0,s)

p=1*pi/n;
t = 0 : p : 1 * pi;
x = 0.001*t%r .* cos(t);
y = s*r .* cos(t)%r .* sin(t);
z = r .* sin(t)%1.*t.^2;
pose=[x'+x0,y'+y0,z'+z0];

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