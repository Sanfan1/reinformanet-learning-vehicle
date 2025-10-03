%% Introduction
% this m-file script is to provide you with an environment as an example to
% write your own Reinforcement learning controller. This code provides you
% with a dynamic line follower robots with 9 sensors. The sensor number
% will give you the information regarding how far the sensor is from the
% line. You need to develope your own code for "Reinforcement_Method"
% function written and update it as you like. 
% You can define your Reinforcement_Method function in a way to receive as
% many inputs as possible to serve for your objective 

%% Initialization
close all
% clear all

% clc

%% global variables
global w_max
global x_0 y_0
global QValues
global epsilon
global steps
global s_old
global w_l
global w_r 
addpath('')
w_max = 400;
w_l = 0;
w_r = 0;
steps = 10;
%initializatipon
epsilon = 0.99;
QValues = Q_Table_Creator();

L=2;   % length of the robot
w=2;    % Width of the robot
s=1;    %initialize sensor location
dir = 0;  % initial direction 0 is clockwise and 1 is counter clock wise
x = 0;
y = 0;
delta = 0;  %angle
Ts=8e-4;
x_0=0;
y_0=-10;
s_old=1;

%% Selecting the controller and the Map
controller_type=input(' Enter the nuber regarding your controller and press enter: \n 1: Reinforcement Method \n 2: P controller \n' );
map_type=input(' Select the Map type: \n 1: Simple rectangle \n 2: bow tie \n 3: Saw tooth \n 4: Octangle \n 5: Advanced \n' );



%% Main Loop

%figure
figure('Renderer', 'painters', 'Position', [10 10 1200 1000])
%try
    while (1)         
        clf 
        [s,er,line_s,line_angle]=robot_map(x,y,delta,L,w,dir,map_type) ;    % plot the robot based on its position, angle, and size
        line_s_old=line_s;
                    
        angle_dif=max([line_angle,delta])-min([line_angle,delta]);    
        if (angle_dif>=pi)
            angle_dif=abs(angle_dif-2*pi);
        end
            
        switch controller_type
           case 1
               [w_l, w_r]=ReinforcementMethod (epsilon, s, s_old);
           case 2
               [w_l, w_r]=P_control(s); 
        end 
        % shows the parameter on the figure 
        text(6,17,sprintf("W_L=%d  W_r=%d" ,w_l,w_r))
        text(-12,16,sprintf("angle dif=%1.2f" ,angle_dif))
        text(-12,15,sprintf("line angle=%1.2f" ,line_angle))
        text(-12,14,sprintf("Delta=%1.2f" ,delta))
        
        pause(0.0001); 
        clc;
        
        
        % select the controller
        
               
        
        s_old=s;
        
        % Generate the next angle and position of the robot
        [x,y,delta]=robot_move(w_l,w_r,x,y,delta,L,w,Ts);
        
        % s=12 means that the robot has lost the line 
        % This if restart the robot when it loses the line
        if (s==12)  
            [x, y, s, s_old, delta, dir] = New_Episode( x_0, y_0, dir) ;       
        end
        
    end

%catch
%    warning('code stopped')
%end







%% Controller Functions
%This controller is a simple Proportional controller to give you an understanding
%The input of the controller is the sensor on the line and the output is
%the speed of left and right motor
function [w_l, w_r]=P_control(s)
global w_max
if s~=12
    err=[0 -.05 .05 -.3 .3 -.75 .75 -1 1 -2 2];
    w_r_c=-err(s)*w_max;
    w_l_c=err(s)*w_max;

    w_r=satur((w_max+w_r_c),-w_max,w_max);
    w_l=satur((w_max+w_l_c),-w_max,w_max);
else
    w_r=0;
    w_l=0;
end
    


end




%Place your code here
% This function should have atleast one input, sensor, and two outputs
% (motor speeds) feel free to add as many input/output you require to
% implement your reinforcement learning controller
function [w_l, w_r]=ReinforcementMethod (epsilon, s, s_old)
global w_l
global w_r
global QValues
global w_max
if s~=12
       wheels_val =[w_l, w_r];
    r = reward(s_old , s, wheels_val);
    updateQtble( s_old, wheels_val,r);
     if (s==1 && s_old == 1&& w_l>0 && w_r>0)
        w_l =w_max;
        w_r =w_max;
     else if(s==1 && w_l>0 && w_r>0)
        w_l =w_max/2;
        w_r =w_max/2;
     
     else
        [w_l, w_r]= chooseNextState(epsilon, s, QValues);
     end
     end
else
    w_r=0;
    w_l=0;
end

end




%% Neseccary functions for Drawing the Map, robot, sensor reading, and robot movement
% You do not need to change anything on this part!!! This is to have the
% environment ready for you
function u = updateQtble(state, wheels_val, reward)
    u=0;
    global QValues
    w_l=wheels_val(1);
    w_r=wheels_val(2);
    QTableLen = length(QValues);
    for i = 1: 1: QTableLen
        if (QValues(i,1)==state) && (QValues(i,2) == w_l) && (QValues(i,3) == w_r)
            QValues(i,4)=reward;
            u = 1;  %update sucessfully
        end
    end
end

%   reward funtion
%   input: s_old, s
%   output: integer value reward
%   bounds: s_old and s are in the range of [1,11]
function r = reward(obs_old , obs, wheel_vals)
    global steps
    w_l__reward_val = int8(wheel_vals(1) / steps);
    w_r__reward_val = int8(wheel_vals(2) / steps);
    average_Wheel_Reward = int8((w_l__reward_val + w_r__reward_val) / 2);

    if obs== 12
        r = -20;
    else
        x = int8((obs-1)/2)+1;      % x = current state
        y = int8((obs_old-1)/2)+1;  % y = previous state
        if x == x
           t = (0.5*x*x-7.5*x+27); 
           r= int8(t) + average_Wheel_Reward;
        end
        if x < y
            t = 0.5*(x^2)-7.5*x+20+y;
            r= int8(t) + average_Wheel_Reward;
        end
        if x > y
            t = (-0.5*(x^2)+0.5*x-1+y);
            r= int8(t) + average_Wheel_Reward;
        end
    end
end

function reward = r(obs_old , obs)
    x = int8(obs-1)/2;      % x = previous state
    y = int8(obs_old-1)/2;  % y = current state
    if obs == obs_old
       reward = int8(0.5*(x^2)-6.5*x+20); 
    end
    %if line move from outer sensor to middle, give it extra reward 
    if obs < obs_old
        reward = 10 - x + y;
    end
    %if line move from middle to outer sensor, penalize the algorithm
    if obs > obs_old
        reward = y - x
    end

end

function [w_l, w_r]= chooseNextState(epsilon, state, QValues)
    % choose best QValue given state
    if rand(1, 1) <= epsilon
        wheel_vals = chooseBestState(QValues, state);
        w_l = wheel_vals(1);
        w_r = wheel_vals(2);
    else
    % Choose random QValue given state
        wheel_vals = chooseRandValueForState(QValues, state);
        w_l = wheel_vals(1);
        w_r = wheel_vals(2);
    end

end


function wheels_val = chooseRandValueForState(QValues, state)
    total_states = 11;
    QTableLen = length(QValues);
    QStateSize = QTableLen / total_states;
    
    end_of_possible_loc = (QStateSize * state) + 1;
    start_of_possible_loc = end_of_possible_loc - QStateSize;

    randValueLocForState = int32(rand(1, 1)*QStateSize + start_of_possible_loc);
    wheels_val = QValues(randValueLocForState, 2:3);
end

function wheels_val = chooseBestState(QValues, state)
    % Given a state s
    % search for max value

    total_states = 11;
    QTableLen = length(QValues);
    QStateSize = QTableLen / total_states;
    end_of_possible_loc = (QStateSize * state) + 1;
    start_of_possible_loc = end_of_possible_loc - QStateSize;
    
    max = 0;
    bestValueLocForState = 1;
    for i=start_of_possible_loc:1:(end_of_possible_loc-1)
        curr_state = QValues(i, 1:4);
        if curr_state(4) > max
            bestValueLocForState = i;
            max = curr_state(4);
        end
    end
    wheels_val = QValues(bestValueLocForState, 2:3);
end

% disp(find(QValues(1:length(QValues)-1) ))

function [w_l, w_r]= getNextAction(epsilon, state)
    % choose best QValue given state
    if rand(1, 1) <= epsilon
        [w_l, w_r] = chooseBestState(QValues, state);
    else
    % Choose random QValue given state
        [w_l, w_r] = chooseRandValueForState(QValues, state);
    end

end

function Q_Table = Q_Table_Creator (~)
    global steps
    global w_max
    w_l = w_max;
    w_r = w_max;
    
    
    minMax = 2 * w_max;
    bucketSize = minMax / steps;
    
    w_l_potential = [];
    w_r_potential = [];
    
    bucket_pointer = -1*w_max;
    
    sensorOptions = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
    % Find each possible new state for r_wheel and l_wheel
    for i=1:1:steps+1
        w_l_potential = [w_l_potential; [bucket_pointer]];
        w_r_potential = [w_r_potential; [bucket_pointer]];
        bucket_pointer = bucket_pointer + bucketSize;
    end
    
    l_r_combined_options = [];
    
    for i=1:1:steps+1
        for j=1:1:steps+1
            potential_group = [];
            potential_group = [potential_group; [w_l_potential(i), w_r_potential(j)]];
            l_r_combined_options = [l_r_combined_options; potential_group];
        end
    end
    
    Q_Table = [];
    
    for i=1:1:11
        for k=1:1:11
            for j=1:1:((steps+1)*(steps+1))
                potential_group_with_sensor = [];
                potential_group_with_sensor = [potential_group_with_sensor; i, k, l_r_combined_options(j, :)];
                Q_Table = [Q_Table; [potential_group_with_sensor, 0]];
            end
        end
    end
    
end




% draw the MAP and new location of the robot and returns the sensor on the
% line and robot angle
function [s,er,line_s,line_angle]=robot_map(x,y,delta,L,w,dir,map_type)
global x_0 y_0
D=0.5*sqrt(L^2+w^2);
x_edge=[w/2 -w/2 -w/2 w/2];
y_edge=[L/2 L/2 -L/2 -L/2];

% ***************************** Line Map
switch map_type
    case 1
        L_map=[-10,-10;-10,15;10,15;10,-10;-10,-10;]; % simple rectangle
        
    case 2
        L_map=[-10,-10;-10,0;0,0;10,0;10,10;0,10;0,-10;-10,-10;]; % bow tie
    case 3
        L_map=[-10,-10;-10,15;7,15;10,10;0,10;0,7;10,5;10,0;7,-5;10,-10;-10,-10;];  % saw tooth
    case 4
        L_map=[ 5  -10; -4.9  -10; -12  -3; -12   7;...
         -5   14; 5   14; 12   7; 12 -3; 5  -10];   % octangle
    case 5
        L_map=[ -10 -10;-10 7;-2 0;-7 0;0 7;-5 7;-5 13;-2 13;-5 18;14.5 18;14.5 13;...
            9 13;11 15;5 15;7 13;2 13;2 7;5 7;5 10;8 10;8 7;14.5 7;14.5 -7;.5 -7;...
            0.5 3.5;11 3.5;11 -3.5;4 -3.5;4 0;7.5 0;7.5,-10;-10 -10;];  %advanced
end




plot(L_map(:,1),L_map(:,2),'k', 'LineWidth', 3);
%axis off
hold on
plot([x_0, x_0],[y_0-1 , y_0+1],'b', 'LineWidth', 3);
text(x_0,y_0+2,'start','Color','blue','FontSize',14)

[x_edge_rotate, y_edge_rotate]=rot_point(x_edge,y_edge,delta);
x_edge_rotate=[x_edge_rotate; x_edge_rotate(1)]+x;
y_edge_rotate=[y_edge_rotate ;y_edge_rotate(1)]+y;
plot(x_edge_rotate,y_edge_rotate,'b-', 'LineWidth', 3);


x_r_w=[w/2 w/2];
y_r_w=[-.3*L -.5*L];


[x_r_w_rotate, y_r_w_rotate]=rot_point(x_r_w,y_r_w,delta);
x_r_w_rotate=[x_r_w_rotate; x_r_w_rotate(1)]+x;
y_r_w_rotate=[y_r_w_rotate ;y_r_w_rotate(1)]+y;
plot(x_r_w_rotate,y_r_w_rotate,'g-', 'LineWidth', 6);

x_l_w=[-w/2 -w/2];
y_l_w=[-.3*L -.5*L];


[x_l_w_rotate, y_l_w_rotate]=rot_point(x_l_w,y_l_w,delta);
x_l_w_rotate=[x_l_w_rotate; x_l_w_rotate(1)]+x;
y_l_w_rotate=[y_l_w_rotate ;y_l_w_rotate(1)]+y;
plot(x_l_w_rotate,y_l_w_rotate,'g-', 'LineWidth', 6);



 
 
 
% x_sens=w*0.5*[-.8 -.6 -.4 -.2 0 .2 .4 .6 .8]; 
% y_sens=L*0.5*[0 0.25 0.5 .8 .8 .8  .5 .25 0]; 
x_sens=w*0.5*[0 -.2  .2  -.4  .4 -.6 .6 -.8 .8 -.8 .8]; 
y_sens=L*0.5*[.8 .8 .8 0.5  .5 0.25 .25 0 0 -0.25 -.25];
[x_sens_rotate, y_sens_rotate]=rot_point(x_sens,y_sens,delta);
x_sens_rotate=x_sens_rotate+x;
y_sens_rotate=y_sens_rotate+y;
plot(x_sens_rotate,y_sens_rotate,'r*', 'LineWidth', 1);
sensor_loc=[x_sens_rotate,y_sens_rotate];
line_loc=L_map;
[s,min_dis,line_s,line_angle,er] = sensor(sensor_loc,line_loc,D,x,y,dir);
text(x+D,y+D,num2str(s),'Color','red')
text(x+D,y-D,num2str(line_s))

%title("Machine Learning Application in  Line Follower  Designed by Sepehr Saadatmand")
%grid on
%set(gca,'visible','off')
 set(gca,'xtick',[])
 set(gca,'ytick',[])

axis equal
%  xlim([x-4*D, x+4*D]);
%  ylim([y-4*D, y+4*D]);
axis([-15 17 -13 20])
 
end

%returning the sensor on the line

function [s,min_dis,line_s,line_angle,er] = sensor(sensor_loc,line_loc,D,x,y,dir)
    s=12;
    err=[0 .05 -.05 .3 -.3 .75 -.75 1 -1 2 -2 20];
    min_dis=100;
    line_s=0;
    line_angle=pi;
    [n_sen,~]=size(sensor_loc);
    [n_line,~]=size(line_loc);
    
    
    j=1;
    while (j<12 && s==12)
        for i=1:n_line-1
            v1=[line_loc(i,:),0];
            v2=[line_loc(i+1,:),0];
            pt=[sensor_loc(1,:), 0];
            d = point_to_line(pt, v1, v2);
            if (d<4*D)
            
              pt=[sensor_loc(j,:), 0];
              d = point_to_line(pt, v1, v2);
              d_tr1 = norm(pt-v2);
              d_tr2 = norm(pt-v1);
              d_tr1_tr2=norm(v1-v2);
              var1_1=d_tr1^2;
              var2_1=(d_tr2^2)+(d_tr1_tr2^2);%-(2*d_tr1_tr2*d_tr2);
              var1_2=d_tr2^2;
              var2_2=(d_tr1^2)+(d_tr1_tr2^2);%-(2*d_tr1_tr2*d_tr1);
              if (var1_1<var2_1)&&(var1_2<var2_2)
                  if (d<.2)
                      min_dis=d;
                      s=j;
                      line_s=i;
                      line_angle=.5*pi+atan2((v1(2)-v2(2)),(v1(1)-v2(1)));
                      if dir==1
                          line_angle=line_angle+pi;
                          if i~=1
                             line_s=10-line_s;
                          end
                      end
                          
                      line_angle= angle_correction(line_angle);
                  end
              end
              
            end
        end
    j=j+1; 
    end

    er=err(s);

end



function d = point_to_line(pt, v1, v2)
% pt should be nx3
% v1 and v2 are vertices on the line (each 1x3)
% d is a nx1 vector with the orthogonal distances
v1 = repmat(v1,size(pt,1),1);
v2 = repmat(v2,size(pt,1),1);
a = v1 - v2;
b = pt - v2;
d = sqrt(sum(cross(a,b,2).^2,2)) ./ sqrt(sum(a.^2,2));
end


%move robot to the next location
function [x_o,y_o,delta_o]=robot_move(w_l,w_r,x,y,delta,L,w,Ts)
    
    r=0.2;
    A_W=2*pi*r;        %area of wheels
    
    if(w_l>=0 && w_r>=0)
        % *************************** Forward computation
        w_f=A_W*min(w_r,w_l);
        v_f=w_f*A_W;
        x_fw=-v_f*Ts*sin(delta);
        y_fw=v_f*Ts*cos(delta);


        % ************************** Turning
        w_t=w_r-w_l;
        alpha=2*pi*(r/w)*w_t*Ts;
        gamma=atan(L/w);
        sigma=gamma+alpha;
        B=0.5*sqrt(w^2+L^2);
        a=0.5*w;
        C=sqrt(a^2+B^2-(2*a*B*cos(sigma)));
        x_rot_org=-C*sin(alpha);
        y_rot_org=C*cos(alpha)-0.5*L;
        [x_rot,y_rot]=rot_point(x_rot_org,y_rot_org,delta);

        % ************************** Final Computatoion
        x_o=x_fw+x_rot+x;
        y_o=y_fw+y_rot+y;
        delta_o=delta+alpha;
    elseif (w_l<0 && w_r<0)
        % *************************** Backward computation
        w_f=A_W*max(w_r,w_l);
        v_f=w_f*A_W;
        x_fw=-v_f*Ts*sin(delta);
        y_fw=v_f*Ts*cos(delta);


        % ************************** Turning
        w_t=w_r-w_l;
        alpha=2*pi*(r/w)*w_t*Ts;
        gamma=atan(L/w);
        sigma=gamma+alpha;
        B=0.5*sqrt(w^2+L^2);
        a=0.5*w;
        C=sqrt(a^2+B^2-(2*a*B*cos(sigma)));
        x_rot_org=-C*sin(alpha);
        y_rot_org=C*cos(alpha)-0.5*L;
        [x_rot,y_rot]=rot_point(x_rot_org,y_rot_org,delta);

        % ************************** Final Computatoion
        x_o=x_fw+x_rot+x;
        y_o=y_fw+y_rot+y;
        delta_o=delta+alpha;
        
        
    else
        % ************************** Turning
        w_t=w_r+w_l;
        alpha=2*pi*(r/w)*w_t*Ts;
        gamma=atan(L/w);
        sigma=gamma+alpha;
        B=0.5*sqrt(w^2+L^2);
        a=0.5*w;
        C=sqrt(a^2+B^2-(2*a*B*cos(sigma)));
        x_rot_org=-C*sin(alpha);
        y_rot_org=C*cos(alpha)-0.5*L;
        [x_rot,y_rot]=rot_point(x_rot_org,y_rot_org,delta);
        
        % ************************** rotating 
        if w_r<0
            w_rotate=min(w_r,w_l);
        else
            w_rotate=-min(w_r,w_l);
        end
            
        alpha_rot=2*pi*(r/(0.5*w))*w_rotate*Ts;
        % ************************** Final Computatoion
        x_o=x_rot+x;
        y_o=y_rot+y;
        delta_o=delta+alpha+alpha_rot;
        
    end
    delta_o=angle_correction(delta_o);
    
end
    
function [x_o, y_o]=rot_point(x,y,delta)
lngth=length(x);
x_o=zeros(lngth,1);
y_o=zeros(lngth,1);
for i=1:lngth
    D=sqrt(x(i)^2+y(i)^2);
    theta=atan2(y(i),x(i));
    angle=theta+delta;
    x_o(i)=D*cos(angle);
    y_o(i)=D*sin(angle);
end
end

% To make sure the angle is between -pi and pi
function o=angle_correction(angle)
o=angle;
while o>pi || o<-pi
    if o>pi
        o=o-2*pi;
    elseif o<-pi
        o=o+2*pi;
    end
end
end


%saturation function
function o=satur(x,mmin,mmax)
    if x<mmin
        o=mmin;
    elseif x>mmax
        o=mmax;
    else
        o=x;
    end
end


% To bring the robot to the starting point and change the direction and
% start from a random angle 
function [x, y, s, s_old, delta, dir] = New_Episode(x_0, y_0, dir)

dir = ~dir;
s=1;
s_old=1;

delta_0=(1-dir*2)*(pi/2) + (pi/6)*(rand-.5);
%delta_0=delta_0+pi;
delta_0=angle_correction(delta_0);
delta=delta_0;
x=x_0;
y=y_0;
end