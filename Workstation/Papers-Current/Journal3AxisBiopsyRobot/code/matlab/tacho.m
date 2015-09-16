classdef tacho % 
    %TACHO Utility functions drawing and updating a tachometer guage drawing in a figure axis
    % modified from
    % http://www.mathworks.com/matlabcentral/fileexchange/authors/150715
    % Main change was by making it a class
    % http://stackoverflow.com/questions/5973699/is-there-any-way-to-call-a-matlab-subfunction-from-the-outside-the-file
    %
    methods (Static = true)
        
        
        function initTacho(axeshandle,minvalue,maxvalue,steps,substeps,textsteps,unit,needlecolor)%unit ist beschriftung
            if nargin < 8
                needlecolor = 'k';
            end
            %value increment with one step
            stepvalue=(maxvalue-minvalue)/steps;
            stepangle=1.5*pi/steps;
            
            %set gca
            set(gcf,'CurrentAxes',axeshandle);
            
            %real world ratio
            set(axeshandle,'DataAspectRatio',[1 1 1]);
            
            %no axis shown
            axis off;
            
            %set boundaries
            axis([-1.1 1.1 -1.1 1.1]);
            
            %white background
            %rectangle('Position',[-1.1,-1.1,2.2,2.2],'Curvature',[1,1],'FaceColor','w');
            th = -pi:0.1:pi;
            patch(1.1*cos(th),1.1*sin(th),'w')
            
            %create tacho object
            objTacho.linehandle=line('linewidth',2,'xdata',[0 cos(1.25*pi)],'ydata',[0 sin(1.25*pi)],'color',needlecolor);
            objTacho.axeshandle=axeshandle;
            objTacho.minvalue=minvalue;
            objTacho.maxvalue=maxvalue;
            objTacho.stepvalue=stepvalue;
            objTacho.stepangle=stepangle;
            objTacho.steps=steps;
            objTacho.unit=unit;
            
            %create scala
            for i=0:steps
                %the curent angle
                %phi=1.5*pi/steps*(steps-i)-0.25*pi;
                phi=-1.5*pi/steps*i+1.25*pi;
                
                x=cos(phi);
                y=sin(phi);
                
                %long or short scala lines?
                if(mod(i,substeps))
                    line([0.95*x x],[0.95*y y]);
                else
                    line([0.9*x x],[0.9*y y]);
                end
                
                %print values
                if(~mod(i,textsteps))
                    text(x*0.7,y*0.7,num2str(i*stepvalue+minvalue),'HorizontalAlignment','center','fontsize',get(0,'DefaultAxesFontSize'));
                end
            end
%             for i=0:textsteps
%                 text(x*0.7,y*0.7,num2str(i*stepvalue+minvalue),'HorizontalAlignment','center','fontsize',get(0,'DefaultAxesFontSize'));
%             end
            
            text(0,-0.3,unit,'HorizontalAlignment','center','fontsize',get(0,'DefaultAxesFontSize'));
            
            set(axeshandle,'UserData',objTacho);
            
            %default value
            tacho.updateTacho(axeshandle,0);
        end
        
        function updateTacho(axeshandle,value)
            objTacho=get(axeshandle,'UserData');
            
            if(value<objTacho.minvalue)
                value=objTacho.minvalue;
            end
            
            if(value>objTacho.maxvalue)
                value=objTacho.maxvalue;
            end
            
            phi=-objTacho.stepangle*(value-objTacho.minvalue)/objTacho.stepvalue+1.25*pi;
            set(objTacho.linehandle,'xdata',[0 cos(phi)],'ydata',[0 sin(phi)]);
        end
        
        
        function addGoalTacho(axeshandle,value,needlecolor)
            objTacho=get(axeshandle,'UserData');
            
            if(value<objTacho.minvalue)
                value=objTacho.minvalue;
            end
            
            if(value>objTacho.maxvalue)
                value=objTacho.maxvalue;
            end
            
            phi=-objTacho.stepangle*(value-objTacho.minvalue)/objTacho.stepvalue+1.25*pi;
            if exist('objTacho.goalhandle')
                 set(objTacho.goalhandle,'xdata',[0 1.05*cos(phi)],'ydata',[0 1.05*sin(phi)]);
            else
                 objTacho.goalhandle=line('linewidth',6,'xdata',[0 1.05*cos(phi)],'ydata',[0 1.05*sin(phi)],'color',needlecolor);
                 uistack(objTacho.linehandle,'top')
            end
        end
        
    end
end
