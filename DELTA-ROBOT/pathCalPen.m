function pose3=pathCalPen(n,ax,ay,az,b,a)

p=1/n;
t = 0 : p : 1;
x = ax*t+a;
y = ay*t+b;
z = az*t;
pose3=[x',y',z'-350];

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