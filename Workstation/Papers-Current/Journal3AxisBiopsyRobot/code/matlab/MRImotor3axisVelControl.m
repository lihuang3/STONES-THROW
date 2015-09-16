function MRImotor3axisVelControl
% CLOSED-LOOP CONTROL:  controls the velocity of 3 orthogonal rotors
% simultaneously by using a control law constructed using a
% control-Lyapunov function.
%
%  three orthogonal motor drivers (with balanced rotors) are driven by the same
%  magnetic gradient.  We simulate the rotation speed and position of each rotor.
%  Author: Aaron Becker
%  Date: 10/18/2013

close all
format compact
clc
set(0,'DefaultAxesFontSize',14)

%  Seed the random number generator
rng(10);

% CONSTANTS, section IV, IJRR article
steelSaturatedMag = 1.36*10^6;      % A/m  http://hyperphysics.phy-astr.gsu.edu/hbase/tables/magprop.html
maxMRIgrad = 0.040;                 %T/m  (page 4, IJRR)
steelBearingDiamInmm = 6;           %= [1,1.5,2,2.5,3,3.5,4,4.5,6,6.5,7,8,9,10,12,13,14,15,16,18,24]; % mm, bearing sizes available from McMaster
r = steelBearingDiamInmm/2*0.001;   % radius in m
Tfr =  8.0*10^-5;                   % 7*10^-5N    =  kg*m/(s^2) (friction PAGE 8, IJRR)
Tl = 0;                              % load torque
m = sphereVol(r)*8530;              %m^3* kg / (m^3), mass of bearing
r1 = 0.018;                         %m, radius rotor arm  with steel bearing.
J = 2*m*r1^2;                       % total inertia
b = 10.0*10^-7;                     %1.0*10^-7 Nms/rad     (PAGE 8, IJRR)
bearingVolMag = sphereVol(r)*steelSaturatedMag; %bearing volume* magnetic susceptibility
velDes = 1*[20,-20,20]; %desired velocity  (can't go 40 rad/s

% SET INITIAL CONDITIONS
%Conventions: theta is angle of rotor
% phi is magnetic angle
% psi is slip angle
%[thx, vx, thy, vy, thz, vz]
%state0 = zeros(6,1);
state0 = randn(6,1);

%%%% Simulate applying a constantly rotating control law to 3 rotors:
options = odeset('RelTol',1e-4,'AbsTol',1e-4);
figure(1)
plot(1,1)

title('Rotate Orthogonal Rotors');
tic
[T,Y] = ode45(@MRI3orthonogalRotors,[0:0.005:2],state0,options); %#ok<NBRAK>

%lin prog, 0.01 s tictoc is: Elapsed time is 14.641404 seconds.
%sign(), 0.01 s tictoc is:   Elapsed time is 0.083913 seconds.
toc
%[T,Y] = ode45(@MRI3orthonogalRotors,[0:0.05:0.1],state0,options);
% t=2s needs 22.024832s

PHIs = zeros(numel(T),3);
for i = 1:length(T)
    PHIs(i,:) = calcControlLaw(T(i),Y(i,:)')';
end
subplot(4,1,1)
plot(T,PHIs(:,1),T,PHIs(:,2),'.-',T,PHIs(:,3),':');
legend('X','Y','Z')
ylabel('gradient orientation')
subplot(4,1,2)
plot(T,Y(:,1),'-',T,Y(:,2),'-',T,velDes(1)*ones(size(T)),'--')
legend('rad', 'rad/s','goal','Location','NorthWest')

ylabel('x-axis')
subplot(4,1,3)
plot(T,Y(:,3),'-',T,Y(:,4),'-',T,velDes(2)*ones(size(T)),'--')
ylabel('y-axis')
subplot(4,1,4)
plot(T,Y(:,5),'-',T,Y(:,6),'-',T,velDes(3)*ones(size(T)),'--')
ylabel('z-axis')
xlabel('time (s)')

set(gcf,'position',[195    49   560   948])
figure(5)
clf
n=3;
plot(T,sum((Y(:,2*(1:n))-repmat(velDes(1:n), size(T))).^2,2))
xlabel('Time (s)')
ylabel('Error (rad/s)^2')

    function v = sphereVol(r)
        v = 4/3*pi*r.^3;
    end

    function Phi = calcControlLaw(~,Y)
        % solves a linear program to minimize the candidate Lyapunov
        % function
        %input Y = [thx, vx, thy, vy, thz, vz]
        
        %Velocity errors for each rotor.  
        %If rotor does not exist, set error to 0
        ex = velDes(1)-Y(2);
        ey = velDes(2)-Y(4);
        ez = velDes(3)-Y(6);
        %Trig shorthand
        Phi = -sign(...
            [-ey*cos(Y(3)) + ez*sin(Y(5));
            -ez*cos(Y(5)) + ex*sin(Y(1));
            -ex*cos(Y(1)) + ey*sin(Y(3))]);
    end

    function Fout = staticFriction(Fapplied)
        % combines Static Friction and Applied Force
        if  Fapplied > Tfr
            Fout = Fapplied-Tfr;
        elseif Fapplied < -Tfr
            Fout = Fapplied+Tfr;
        else
            Fout = 0;
        end
    end

    function dy = MRI3orthonogalRotors(t,y)
        % ODE45 differential equations for 3 orthogonal rotors

        PHI = calcControlLaw(t,y);
        
        % magnetic gradients
        dBx = maxMRIgrad*PHI(1);
        dBy = maxMRIgrad*PHI(2);
        dBz = maxMRIgrad*PHI(3);
        
        %forces on balls
        Fx = bearingVolMag*dBx;
        Fy = bearingVolMag*dBy;
        Fz = bearingVolMag*dBz;
        
        dy = zeros(6,1);    % a column vector
        
        %project force onto perpendicular vector
        %[thx, vx, thy, vy, thz, vz]
        dy([1,3,5]) = y([2,4,6]);
        
        dy([2,4,6]) = 1/J*(  -b*y([2,4,6]) -Tl +[staticFriction(r1*(Fz*cos(y(1))-Fy*sin(y(1))))  ;
            staticFriction(r1*(Fx*cos(y(3))-Fz*sin(y(3))))  ;
            staticFriction(r1*(Fy*cos(y(5))-Fx*sin(y(5))))]  );
    end


end




