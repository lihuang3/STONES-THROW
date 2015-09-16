function pileDriver
    % Design our spring at http://springipedia.com/extension-design-theory.asp
    %
    %  I used 0.010" Nitinol wire for the spring.  I could purchase 0.008, 0.006, or 0.004"
    %  wire (each is $40)
    close all
    
    %TODO add rolling resistance.
    
    %Compare momentum for first 100 strikes as a function of open loop
    %driving frequency.
    
    steelSaturatedMag = 1.36*10^6;      % A/m  http://hyperphysics.phy-astr.gsu.edu/hbase/tables/magprop.html
    maxMRIgrad = 0.023;                 %T/m  (page 4, IJRR)
    l = 0.030;                         %m, length ball can roll (inside capsule).
    steelBearingDiamInmm = 5;           %bearing sizes available from McMaster
    r = steelBearingDiamInmm/2*0.001;   % radius in m
    massBall = 3/4*pi*r^3*8530;              %m^3* kg / (m^3), mass of bearing
    SpringRate = 50;  % N/m
    %ls = 0.0025; %m SpringRestLength
    bearingVolMag = 3/4*pi*r^3*steelSaturatedMag; %bearing volume* magnetic susceptibility
   
    
    
    % Diagram
    %   spring    ball           ImpactPlate
    %   [\\\\|    (_)             | >
    %    -ls 0                    l
    
    refine = 4;
    
    
    numContacts = 100;
    frequencies = 0.5:0.5:10;
    
    for  CoeR = 0.2:0.1:0.9; %0.4; %Coefficient of restitution
    
    
    ImpactVelocities = zeros(numContacts, numel(frequencies));
    totalTime = zeros(1,numel(frequencies));
    
    tic
    for fr = 1:numel(frequencies);
        display(['frequency ',num2str(frequencies(fr)), ' [',num2str(fr),'/',num2str(numel(frequencies)),']']);
        
        options = odeset('Events',@events,'OutputFcn', @odeplot,...
        'OutputSel',1,'Refine',refine);
        
        y0 = [0; 0];
        tstart = 0;
        tfinal = 300;
        tout = tstart; %position [m], velocity [m/s]
        yout = y0.';
        teout = [];
        yeout = [];
        ieout = [];
        
        figure(fr);clf;
        %set(gca,'xlim',[0 1],'ylim',[-0.2*l 1.1*l]);  %TODO: fix
        box on
        hold on;
        
        for i = 1:numContacts
            
            
            
            % Solve until the first terminal event. capsuleODE
            [t,y,te,ye,ie] = ode23(@capsuleODE,[tstart tfinal],y0,options);
            if ~ishold
                hold on
            end
            % Accumulate output.
            nt = length(t);
            tout = [tout; t(2:nt)];  %#ok<*AGROW>
            yout = [yout; y(2:nt,:)];
            teout = [teout; te];    % Events at tstart are never reported.
            yeout = [yeout; ye];
            ieout = [ieout; ie];
            
            ud = get(gcf,'UserData');
            if ud.stop
                break;
            end

            % Set the new initial conditions, with .9 attenuation.
            y0(1) = l;
            y0(2) = -CoeR*y(nt,2);
            
            
            ImpactVelocities(i,fr) = y(nt,2);
            
            % A good guess of a valid first time step is the length of
            % the last valid time step, so use it for faster computation.
            options = odeset(options,'InitialStep',t(nt)-t(nt-refine),...
                'MaxStep',t(nt)-t(1));
            tstart = t(nt);
            
            if abs(y(nt,2)) < 0.01
                %zeno's paradox:  the sphere is practically stuck to the top, find next time instant where applied force is
                %negative
                tstart = ceil(2*tstart*fq)/(2*fq);
            end
            
            
%             if i >= numContacts && teout(end) > 10
%                 numContacts = numContacts+1;
%             end
            
            
        end
        
        hl = plot(teout,yeout(:,1),'ro');
        uistack(hl,'bottom');
        
        plot(tout,openLoopControl(tout)*l/3+0.5*l,'g')
        
        legend('impacts','ball trajectory','Location','East')
        xlabel('time [s]');
        ylabel('length [m]');
        title(['Sphere trajectory and the events, frequency =',num2str(frequencies(fr)),' Hz, ', num2str(i),' Contacts']);
        hold off
        odeplot([],[],'done');
        uistack(hl,'top');

        totalTime(fr) =teout(end);
        
        save(['piledriverCoeR',num2str(CoeR),'.mat'],'numContacts','frequencies',...
            'ImpactVelocities','totalTime','steelSaturatedMag','maxMRIgrad','l',...
            'steelBearingDiamInmm','r','massBall','SpringRate','bearingVolMag','CoeR');
    end
    
    % final plots
    figure(fr + 10);clf;
    plot(frequencies, totalTime,'-o')
    xlabel('Open-loop driving frequency [Hz]');
    ylabel(['Time for ',num2str(numContacts),' contacts']);
    
    figure(fr + 11);clf;
    plot(frequencies,sum(ImpactVelocities)/numContacts,'-o')
    xlabel('Open-loop driving frequency [Hz]');
    ylabel('Average Impact Velocity');
    
    figure(fr + 12);clf;
    plot(frequencies,sum(ImpactVelocities)./totalTime,'-o')
    xlabel('Open-loop driving frequency [Hz]');
    ylabel('momentum transfer per second');
    end
    
    % --------------------------------------------------------------
    function [value,isterminal,direction] = events(~,y)
        % Locate the time when height passes through zero in a
        % decreasing direction and stop integration.
        value = l-y(1);     % Detect height = 0
        isterminal = 1;   % Stop the integration
        direction = -1;   % Negative direction only
    end
    
    function sign = openLoopControl(t)
        fq = frequencies(fr);  %excitation frequency
        sign = -mod(floor(2*t*fq),2)*2+1;
    end
    
    function dydt = capsuleODE(t,y)
        forceOnBall = bearingVolMag*maxMRIgrad;
        
        openLoop = 1;
        if ~openLoop
            if y(2) < 0
                forceOnBall = -forceOnBall;
            end
        else
            forceOnBall = openLoopControl(t)*forceOnBall;
        end
        
        
        springforce = 0;
        if(y(1)<0)  %spring is only engaged when position < 0.
            springforce = -SpringRate*y(1);
        end
        
        dydt = [y(2); 1/massBall*(forceOnBall+springforce)];
        
    end
    
end

%end