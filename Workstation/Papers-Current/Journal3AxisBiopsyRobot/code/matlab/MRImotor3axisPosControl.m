function MRImotor3axisPosControl
% CLOSED-LOOP CONTROL:  controls the position of 3 orthogonal rotors
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

MAKE_MOVIE = false;

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
posDes = [10,-10,5]; %desired positions

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
[T,Y] = ode45(@MRI3orthonogalRotors,[0:0.01:4],state0,options);

%lin prog, 0.01 s tictoc is: Elapsed time is 14.641404 seconds.
%sign(), 0.01 s tictoc is:   Elapsed time is 0.083913 seconds.
toc
%[T,Y] = ode45(@MRI3orthonogalRotors,[0:0.05:0.1],state0,options);
% t=2s needs 22.024832s

PHIs = zeros(numel(T),3);
for i = 1:length(T)
    PHIs(i,:) = calcPhiVel(T(i),Y(i,:)')';
end
subplot(4,1,1)
plot(T,PHIs(:,1),T,PHIs(:,2),'.-',T,PHIs(:,3),':');
legend('X','Y','Z')
ylabel('gradient orientation')
subplot(4,1,2)
plot(T,Y(:,1),'-',T,Y(:,2),'-',T,posDes(1)*ones(size(T)),'--')
legend('rad', 'rad/s','goal','Location','NorthWest')
ylabel('x-axis')
subplot(4,1,3)
plot(T,Y(:,3),'-',T,Y(:,4),'-',T,posDes(2)*ones(size(T)),'--')
ylabel('y-axis')
subplot(4,1,4)
plot(T,Y(:,5),'-',T,Y(:,6),'-',T,posDes(3)*ones(size(T)),'--')
ylabel('z-axis')
xlabel('time (s)')

set(gcf,'position',[195    49   560   948])
figure(5)
clf
n=3;
plot(T,sum((Y(:,2*(1:n)-1)-repmat(posDes(1:n), size(T))).^2,2))
xlabel('Time (s)')
ylabel('Error (rad)$^2$')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% MAKE A MOVIE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if MAKE_MOVIE
    import tacho.*
    import mArrow3class.*
    f2 = figure(2);
    
    
    FrameCount = 1;
    MOVIE_NAME = 'LyapunovPosControl3rotors4s2';
    G.fig = figure(2);
    set(G.fig,'Units','normalized','outerposition',[0 0 1 1],'NumberTitle','off');%'MenuBar','none',
    writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(writerObj,'Quality',100)
    %set(writerObj, 'CompressionRatio', 2);
    open(writerObj);
    
    
    OrientRange = ceil(max( max(abs(Y(:,2*(1:n)-1))))/5)*5; %rad
    %SETUP PLOTS
    hx = subplot(3,3,1);
    %initTacho(axeshandle,minvalue,maxvalue,steps,substeps,textsteps,unit,needlecolor)
    tacho.initTacho(hx,-OrientRange,OrientRange,12,24,3,'x (rad)','r');
    tacho.addGoalTacho(hx,posDes(1),[0.5,0,0])
    hy = subplot(3,3,4);
    tacho.initTacho(hy,-OrientRange,OrientRange,12,24,3,'y (rad)','g');
    tacho.addGoalTacho(hy,posDes(2),[0,0.5,0])
    hz = subplot(3,3,7);
    tacho.initTacho(hz,-OrientRange,OrientRange,12,24,3,'z (rad)','b');
    tacho.addGoalTacho(hz,posDes(3),[0,0,0.5])
    
    subplot(3,3,[2:3,5:6,8:9])
    htitle = title(sprintf('Orthogonal Rotor Position Control, t=%2.1fs',0));
    harrow = mArrow3class.mArrow3([0 0 0],maxMRIgrad*calcPhiVel(T(1),Y(1,:)'),'color','k','stemWidth',0.001,'facealpha',.5);%,'edgecolor','r')
    
    lighting phong
    axis equal
    axis( maxMRIgrad*[-1,1,-1,1,-1,1] );
    grid on
    hold on
    
    [xc, yc, zc] = cylinder;
    rf = maxMRIgrad/20;
    surf(xc*rf+0,yc*rf+0,maxMRIgrad*(zc*0.1-1),'FaceColor',[0 0 0]);
    surf(xc*rf+0,maxMRIgrad*(zc*-0.1+1),yc*rf+0,'FaceColor',[0 0 0]);
    surf(maxMRIgrad*(zc*-0.1+1),xc*rf+0,yc*rf+0,'FaceColor',[0 0 0]);
    %end caps
    phi = linspace(0,2*pi,40);
    xcap = rf*cos(phi);
    ycap = rf*sin(phi);
    grey = [0.8,0.8,0.8];
    patch(xcap,ycap, -maxMRIgrad*0.9*ones(size(xcap)),.5*grey);
    patch(xcap,maxMRIgrad*0.9*ones(size(xcap)),ycap,.5*grey);
    patch(maxMRIgrad*0.9*ones(size(xcap)),xcap,ycap,.5*grey);
    
    % plot the three rotors with magnetic balls on each face
    rRadius = maxMRIgrad/8;
    pts = 25;
    xPx = [ rRadius*cos(linspace(pi/2,3*pi/2,pts)),maxMRIgrad*3/4+ rRadius*cos(linspace(3*pi/2,5*pi/2,pts)),maxMRIgrad*3/4+rRadius/2*cos(linspace(pi/2,-3*pi/2,pts)),maxMRIgrad*3/4+rRadius/3*cos(linspace(pi/2,5*pi/2,pts)),maxMRIgrad*3/4+rRadius*cos(pi/2),rRadius*cos(pi/2),rRadius/2*cos(linspace(pi/2,-3*pi/2,pts))];
    yPx = [ rRadius*sin(linspace(pi/2,3*pi/2,pts)),                rRadius*sin(linspace(3*pi/2,5*pi/2,pts)),               rRadius/2*sin(linspace(pi/2,-3*pi/2,pts)),               rRadius/3*sin(linspace(pi/2,5*pi/2,pts)),               rRadius*sin(pi/2),rRadius*sin(pi/2),rRadius/2*sin(linspace(pi/2,-3*pi/2,pts))];
    
    xPx = [ rRadius*cos(linspace(pi/2,3*pi/2,pts)),maxMRIgrad*3/4+ rRadius*cos(linspace(3*pi/2,5*pi/2,pts)),maxMRIgrad*3/4+rRadius*cos(pi/2),rRadius*cos(pi/2)];
    yPx = [ rRadius*sin(linspace(pi/2,3*pi/2,pts)),                rRadius*sin(linspace(3*pi/2,5*pi/2,pts)),                 rRadius*sin(pi/2),rRadius*sin(pi/2)];
    
    [xsp,ysp,zsp] = sphere;
    xsp = rRadius/2*xsp;
    ysp = rRadius/2*ysp;
    zsp = rRadius/2*zsp;
    
    %draw ball bearings
    hspx = surf(xsp-maxMRIgrad*3/4,ysp+maxMRIgrad*3/4,zsp,'FaceColor','r');
    hspy = surf(xsp+maxMRIgrad*3/4,ysp-maxMRIgrad*3/4,zsp,'FaceColor','g');
    hspz = surf(xsp,ysp+maxMRIgrad*3/4,zsp-maxMRIgrad*3/4,'FaceColor','b');
    set(hspx,'linestyle','none')
    set(hspy,'linestyle','none')
    set(hspz,'linestyle','none')
    
    set(gca,'xTick',-1:0.02:1,'yTick',-1:0.02:1,'zTick',-1:0.02:1)
    axis([-0.04001,0.04001,-0.04001,0.04001,-0.04001,0.04001])
    set(gca,'gridlinestyle','-')
 %   rRadius/2
  %  sphere(
    
%     figure(4)
%     for i = 1:numel(xPx)
%         plot(xPx(i),yPx(i),'.');
%         hold on
%         text(xPx(i),yPx(i),num2str(i));
%         axis equal
%         patch(xPx, yPx,'b');
%     end

    arrX = [0,0,    0.7,0.7,  1,   0.7, 0.7, 0,   0];
    arrY = [0,0.1,  0.1,0.2,  0,  -0.2,-0.1,-0.1, 0];
    arrZ = maxMRIgrad*ones(size(arrX));
    grey = [0.6,0.6,0.6];
    mX = patch(arrZ,arrY,arrX,grey);
    mY = patch(arrY,arrZ,arrX,grey);
    mZ = patch(arrX,arrY,-arrZ,grey);

    armx = patch( maxMRIgrad*0.98*ones(size(xPx)), xPx, yPx,[0.5,0,0]);
    army = patch(xPx,  maxMRIgrad*0.98*ones(size(xPx)), yPx,[0,0.5,0]);
    armz = patch(xPx, yPx, -maxMRIgrad*0.98*ones(size(xPx)),[0,0,0.5]);
    

    
    % mZ = patch(arrX,arrY,-arrZ,grey);
    
    xlabel('x')
    ylabel('y')
    zlabel('z')
   drawnow 
    
    % UPDATE PLOTS
    for i = 1:length(T)
        %phi = calcPhi(T(i));
        phi = calcPhiVel(T(i),Y(i,:)');
        tacho.updateTacho(hx,Y(i,1));
        tacho.updateTacho(hy,Y(i,3));
        tacho.updateTacho(hz,Y(i,5));
        %mArrow3([0 0 0],maxMRIgrad*phi,'color','red','stemWidth',0.001,'facealpha',0.5);
        mArrow3class.updateArrow3(harrow,[0,0,0],maxMRIgrad*phi);
        
        %update projections
        %calculate the magnetic angle in each direction.
        ax = atan2(phi(3),phi(2));  mx = maxMRIgrad*sqrt( phi(3)^2+phi(2)^2);
        ay = atan2(phi(1),phi(3));  my = maxMRIgrad*sqrt( phi(1)^2+phi(3)^2);
        az = atan2(phi(2),phi(1));  mz = maxMRIgrad*sqrt( phi(2)^2+phi(1)^2);
        set(mX, 'YData', mx*(cos(ax)*arrX-sin(ax)*arrY),'ZData',  mx*(sin(ax)*arrX+cos(ax)*arrY), 'XData',arrZ);
        set(mY, 'ZData', my*(cos(ay)*arrX-sin(ay)*arrY), 'XData', my*(sin(ay)*arrX+cos(ay)*arrY),'YData', arrZ);
        set(mZ, 'XData', mz*(cos(az)*arrX-sin(az)*arrY), 'YData', mz*(sin(az)*arrX+cos(az)*arrY),'ZData', -arrZ);
        
        %update rotor arms
        set(armx, 'XData',maxMRIgrad*0.95*ones(size(xPx)),'YData', cos(Y(i,1))*xPx-sin(Y(i,1))*yPx,'ZData',  sin(Y(i,1))*xPx+cos(Y(i,1))*yPx );
        set(army, 'ZData', cos(Y(i,3))*xPx-sin(Y(i,3))*yPx, 'XData', sin(Y(i,3))*xPx+cos(Y(i,3))*yPx ,'YData', +maxMRIgrad*0.95*ones(size(xPx)));
        set(armz, 'XData', cos(Y(i,5))*xPx-sin(Y(i,5))*yPx, 'YData', sin(Y(i,5))*xPx+cos(Y(i,5))*yPx ,'ZData', -maxMRIgrad*0.95*ones(size(xPx)));
        
        set(hspx, 'XData', xsp+maxMRIgrad*.95, 'YData', ysp+cos(Y(i,1))*maxMRIgrad*3/4,'ZData', zsp+sin(Y(i,1))*maxMRIgrad*3/4);
        set(hspy, 'XData', xsp+sin(Y(i,3))*maxMRIgrad*3/4, 'YData', ysp+maxMRIgrad*.95,'ZData', zsp+cos(Y(i,3))*maxMRIgrad*3/4);
        set(hspz, 'XData', xsp+cos(Y(i,5))*maxMRIgrad*3/4, 'YData', ysp+sin(Y(i,5))*maxMRIgrad*3/4,'ZData', zsp-maxMRIgrad*.95);
        
        set(htitle,'string',sprintf('Orthogonal Rotor Position Control, t=%2.1fs',T(i)));
        %drawnow
        %pause(0.2)
        updateDrawing
       % pause(0.01)
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%% END MAKE A MOVIE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

    function v = sphereVol(r)
        v = 4/3*pi*r.^3;
    end


    function Phi = calcPhiVel(~,Y)
        % solves a linear program to minimize the candidate Lyapunov
        % function
        gain = 100;
        % velocity errors
        ex = gain*(posDes(1)-Y(1))-Y(2);
        ey = gain*(posDes(2)-Y(3))-Y(4);
        ez = gain*(posDes(3)-Y(5))-Y(6);
        %Trig shorthand
        F =  [-ey*cos(Y(3)) + ez*sin(Y(5));
            -ez*cos(Y(5)) + ex*sin(Y(1));
            -ex*cos(Y(1)) + ey*sin(Y(3))];
        Phi = -sign( F);
        %Phi = -sign(F).*(abs(F)>1); 
        
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
        
        % I tried this, but no speed up
        %        Ibig = Fapplied > Tfr;
        %         Isml = Fapplied <-Tfr;
        %         Fout = Fapplied.*(Ibig+Isml)-Tfr*Ibig + Tfr*Isml;
        %         
        %     if vel > 1;
        %         Fout = Fout-FstaticFriction;
        %     elseif vel < -1
        %         Fout = Fout+FstaticFriction;
        %     end
        
    end

    function dy = MRI3orthonogalRotors(t,y)
        % ODE45 differential equations for 3 orthogonal rotors
        %1 Hz magnetic field rotation around world y-axis (coordinates follow Fig.
        %1 of IJRR 2013 article.)
        %PHI = calcPhi(t);
        PHI = calcPhiVel(t,y);
        
        % magnetic gradients
        dBx = maxMRIgrad*PHI(1);
        dBy = maxMRIgrad*PHI(2);
        dBz = maxMRIgrad*PHI(3);
        
        %forces on balls
        Fx = bearingVolMag*dBx;
        Fy = bearingVolMag*dBy;
        Fz = bearingVolMag*dBz;
        
        %calculate the magnetic angle in each direction.
        %         phix = atan2(Fz,Fy);
        %         phiy = atan2(Fx,Fz);
        %         phiz = atan2(Fy,Fx);
        %
        dy = zeros(6,1);    % a column vector
        
        %project force onto perpendicular vector
        %[thx, vx, thy, vy, thz, vz]
        dy([1,3,5]) = y([2,4,6]);
        
        dy([2,4,6]) = 1/J*(  -b*y([2,4,6]) -Tl +[staticFriction(r1*(Fz*cos(y(1))-Fy*sin(y(1))))  ;
                                                 staticFriction(r1*(Fx*cos(y(3))-Fz*sin(y(3))))  ;
                                                 staticFriction(r1*(Fy*cos(y(5))-Fx*sin(y(5))))]  );
    end
    function updateDrawing
        drawnow
        if(MAKE_MOVIE)
            FrameCount=FrameCount+1;
            figure(f2)
        %    set(gcf,'renderer','painters')  %can  cause an error if arrows
        %    disappear.
            tfig = myaa(4);
          %  F = getframe_nosteal_focus; %
            F = getframe;
           % writeVideo(writerObj,F.cdata);
            close(tfig)
            while(FrameCount < 10)
                updateDrawing
            end
            
        end
    end

end




