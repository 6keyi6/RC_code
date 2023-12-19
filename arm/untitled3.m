untitled;
function untitled
syms   alpha1       a1       d1       theta1;%DH参数表   基于改进表达法
syms   alpha2       a2       d2       theta2;
syms   alpha3       a3       d3       theta3;
syms   alpha4       a4       d4       theta4;
syms   alpha5       a5       d5       theta5;
syms   alpha6       a6       d6       theta6;
syms c1 c2 c3 c4 c5 c6 s1 s2 s3 s4 s5 s6 ca1 ca2 ca3 ca4 ca5 ca6 sa1 sa2 sa3 sa4 sa5 sa6;
syms r p y;
r =-55.6;%y
p =-70.8;%z轴
y =52.0;%y
theta1 =36*pi/180;   d1 = 0;   a1 = 0;     alpha1 = 0;          offset1 = 0;
theta2 =36*pi/180;   d2 = 0;     a2 = 0;     alpha2 = -pi/2;      offset2 = 0;
theta3 =36*pi/180;   d3 = 0;     a3 = 180;   alpha3 = 0;          offset3 = 0;
theta4 =36*pi/180;   d4 = 170.8; a4 = 0;     alpha4 = -pi/2;      offset4 = 0;
theta5 =36*pi/180;   d5 = 0;     a5 = 0;     alpha5 = pi/2;       offset5 = 0;
theta6 =36*pi/180;   d6 = 0;     a6 = 0;     alpha6 = -pi/2;      offset6 = 0;
MDH = [theta1          d1       a1       alpha1;
       theta2          d2       a2       alpha2;    
       theta3          d3       a3       alpha3;
       theta4          d4       a4       alpha4;
       theta5          d5       a5       alpha5;
       theta6          d6       a6       alpha6];
ca1 = cos(MDH(1,4)); sa1 = sin(MDH(1,4)) ;
ca2 = cos(MDH(2,4)); sa2 = sin(MDH(2,4)) ;
ca3 = cos(MDH(3,4)); sa3 = sin(MDH(3,4)) ;
ca4 = cos(MDH(4,4)); sa4 = sin(MDH(4,4)) ;
ca5 = cos(MDH(5,4)); sa5 = sin(MDH(5,4)) ;
ca6 = cos(MDH(6,4)); sa6 = sin(MDH(6,4)) ;
c1 = cos(MDH(1,1));  s1 = sin(MDH(1,1)) ; 
c2 = cos(MDH(2,1));  s2 = sin(MDH(2,1)) ;
c3 = cos(MDH(3,1));  s3 = sin(MDH(3,1)) ;
c4 = cos(MDH(4,1));  s4 = sin(MDH(4,1)) ;
c5 = cos(MDH(5,1));  s5 = sin(MDH(5,1)) ;
c6 = cos(MDH(6,1));  s6 = sin(MDH(6,1)) ;
T01=[c1            -s1              0              a1;
    s1*ca1       c1*ca1          -sa1            -sa1*d1;
    s1*sa1       c1*sa1           ca1            ca1*d1;
    0             0                0              1];

T12=[c2             -s2            0              a2;
    s2*ca2      c2*ca2           -sa2          -sa2*d2;
    s2*sa2      c2*sa2           ca2             ca2*d2;
    0            0               0              1];

T23=[c3               -s3           0              a3;
    s3*ca3         c3*ca3         -sa3        -sa3*d3;
    s3*sa3         c3*sa3         ca3          ca3*d3;
    0                0             0           1];
T34=[c4           -s4              0              a4;
    s4*ca4       c4*ca4        -sa4          -sa4*d4;
    s4*sa4       c4*sa4         ca4           ca4*d4;
    0             0               0              1];
T45=[c5           -s5               0              a5;
    s5*ca5      c5*ca5           -sa5         -sa5*d5;
    s5*sa5      c5*sa5            ca5          ca5*d5;
    0             0                0              1];
T56=[c6           -s6               0              a6;
    s6*ca6      c6*ca6           -sa6        -sa6*d6;
    s6*sa6      c6*sa6            ca6         ca6*d6;
    0             0                 0            1];
T06 = T01*T12*T23*T34*T45*T56;
T03 = T01*T12*T23;
% disp(T06);
disp(T06);

	 Rx = [ 1 0 0;
		 0 cos(y*pi/180) -sin(y*pi/180);
		 0 sin(y*pi/180)  cos(y*pi/180)];



	 Ry= [ cos(p*pi/180) 0 sin(p*pi/180);
		 0 1 0;
         -sin(p*pi/180) 0 cos(p*pi/180)];

	 Rz =[ cos(r*pi/180) -sin(r*pi/180) 0;
		 sin(r*pi/180) cos(r*pi/180) 0; 
         0 0 1];
R06 = Rx*Ry*Rz;
disp(R06);
end

