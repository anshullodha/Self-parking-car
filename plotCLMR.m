function  plotCLMR(xr,yr,theta,phi)

wheel_length=7;
wheel_width=2;
track=15;
wheel_base=15;
Rear_clearance=5;
Front_clearance=5;
body_length=wheel_base+Rear_clearance+Front_clearance;

theta=pi/2-theta;
d=sqrt((track/2)^2+Rear_clearance^2);
x=xr-(d*sin(theta+atan(track/2/Rear_clearance)));
y=yr-(d*cos(theta+atan(track/2/Rear_clearance)));

% x=xr-track/2*cos(theta);
% y=yr-Rear_clearance*cos(theta);

xa1=x;
ya1=y;


xa2=xa1+track*cos(theta);
ya2=ya1-track*sin(theta);

xb1=x+body_length*sin(theta);
yb1=y+body_length*cos(theta);

xb2=xb1+track*cos(theta);
yb2=yb1-track*sin(theta);

patch([xa1 xa2 xb2 xb1],[ya1 ya2 yb2 yb1],[0 1 0]);

% The left back wheel
xref=x+(Rear_clearance-wheel_length/2)*sin(theta);
yref=y+(Rear_clearance-wheel_length/2)*cos(theta);

xa11=xref-(wheel_width/2)*cos(theta);
ya11=yref+(wheel_width/2)*sin(theta);

xa21=xa11+(wheel_width)*cos(theta);
ya21=ya11-(wheel_width)*sin(theta);

xb11=xa11+wheel_length*sin(theta);
yb11=ya11+wheel_length*cos(theta);

xb21=xb11+wheel_width*cos(theta);
yb21=yb11-wheel_width*sin(theta);

patch([xa11 xa21 xb21 xb11],[ya11 ya21 yb21 yb11],[0 0 0]);



% The Right back wheel

xref=xref+(track)*cos(theta);
yref=yref-(track)*sin(theta);

xa11=xref-(wheel_width/2)*cos(theta);
ya11=yref+(wheel_width/2)*sin(theta);

xa21=xa11+(wheel_width)*cos(theta);
ya21=ya11-(wheel_width)*sin(theta);

xb11=xa11+wheel_length*sin(theta);
yb11=ya11+wheel_length*cos(theta);

xb21=xb11+wheel_width*cos(theta);
yb21=yb11-wheel_width*sin(theta);

patch([xa11 xa21 xb21 xb11],[ya11 ya21 yb21 yb11],[0 0 0]);



% The left front wheel
xref=x+(Rear_clearance+wheel_base)*sin(theta);
yref=y+(Rear_clearance+wheel_base)*cos(theta);

Wheel_diag=sqrt((wheel_length/2)^2+(wheel_width/2)^2);


xa11=xref-Wheel_diag*sin(theta-phi+atan(wheel_width/wheel_length));
ya11=yref-Wheel_diag*cos(theta-phi+atan(wheel_width/wheel_length));

xa21=xa11+(wheel_width)*cos(theta-phi);
ya21=ya11-(wheel_width)*sin(theta-phi);

xb11=xa11+wheel_length*sin(theta-phi);
yb11=ya11+wheel_length*cos(theta-phi);

xb21=xb11+wheel_width*cos(theta-phi);
yb21=yb11-wheel_width*sin(theta-phi);

patch([xa11 xa21 xb21 xb11],[ya11 ya21 yb21 yb11],[0 0 0]);


% The Right front wheel
xref=xref+(track)*cos(theta);
yref=yref-(track)*sin(theta);

Wheel_diag=sqrt((wheel_length/2)^2+(wheel_width/2)^2);


xa11=xref-Wheel_diag*sin(theta-phi+atan(wheel_width/wheel_length));
ya11=yref-Wheel_diag*cos(theta-phi+atan(wheel_width/wheel_length));

xa21=xa11+(wheel_width)*cos(theta-phi);
ya21=ya11-(wheel_width)*sin(theta-phi);

xb11=xa11+wheel_length*sin(theta-phi);
yb11=ya11+wheel_length*cos(theta-phi);

xb21=xb11+wheel_width*cos(theta-phi);
yb21=yb11-wheel_width*sin(theta-phi);

patch([xa11 xa21 xb21 xb11],[ya11 ya21 yb21 yb11],[0 0 0]);
