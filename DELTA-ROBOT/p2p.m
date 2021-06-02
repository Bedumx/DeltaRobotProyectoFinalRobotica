%function p2p=pathCalCIL(n,x1,y1,z1,x0,y0,z0,s)
dof=6

q0=[0.0,0.0,0.0,0.0,0.0,0.0];
v0=[.0,.0,.0,.0,.0,.0];
ac0=[.0,.0,.0,.0,.0,.0];
t0=0;

q1=[.5,.6,.65,.7,.75,.8];
v1=[.0,.0,.0,.0,.0,.0];
ac1=[.0,.0,.0,.0,.0,.0];
tf=1;


t=linspace(t0,tf,100*(tf-t0));

s=length(t);
c = [1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]

qd_l=[];
vd_l=[];
ad_l=[];

M=[[1, t0, t0*t0, t0*t0*t0, t0*t0*t0*t0, t0*t0*t0*t0*t0],
   [0, 1, 2*t0, 3*t0*t0, 4*t0*t0*t0, 5*t0*t0*t0*t0],
   [0, 0, 2, 6*t0, 12*t0*t0, 20*t0*t0*t0],     
   [1, tf, tf*tf, tf*tf*tf, tf*tf*tf*tf, tf*tf*tf*tf*tf],
   [0, 1, 2*tf, 3*tf*tf, 4*tf*tf*tf, 5*tf*tf*tf*tf],
   [0, 0, 2, 6*tf, 12*tf*tf, 20*tf*tf*tf]];

for i=1:dof
    
   
    B=[[q0(i)],
       [v0(i)],
       [ac0(i)],
       [q1(i)],
       [v1(i)],
       [ac1(i)]];
   
    A=inv(M)*B;
    
    qd= A(1,1)*c + A(2,1).*t + A(3,1)*t.*t+ A(4,1)*t.*t.*t + A(5,1)*t.*t.*t.*t + A(6,1)*t.*t.*t.*t.*t
    vd= A(2,1)*c+ 2*A(3,1)*t + 3*A(4,1)*t.*t + 4*A(5,1)*t.*t.*t + 5*A(6,1)*t.*t.*t.*t
    ad= 2*A(3,1)*c + 6*A(4,1)*t + 12*A(5,1)*t.*t + 20*A(6,1)*t.*t.*t
    if i==1
        qd_f1 = qd;
    end
    if i==2
        qd_f2 = qd;
    end
    if i==3
        qd_f3 = qd;
    end
    if i==4
        qd_f4 = qd;
    end
    if i==5
        qd_f5 = qd;
    end
    if i==6
        qd_f6 = qd;
    end
    qd_l = [qd_l;qd];
end

%end