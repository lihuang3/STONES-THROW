function AllSteelProbes
    % what forces can we make with steel implements?
    format compact
    close all
    
    
    % current chrome steel balls: Bearing-Quality E52100 Alloy Steel
    % http://www.mcmaster.com/#chrome-steel-balls/=oylk49
    % Density: 7.81 g/cc  7.81 (g / cc) =7810 kg / (m^3)
    % [http://www.matweb.com/search/datasheet.aspx?matguid=d0b0a51bff894778a97f5b72e7317d85&ckck=1]
    
    
    
    %% WHY are the bearings used thus far so small?
    
    % is ti because of the static gradient field?
    % Plot forces on steel bearings
    figure(2)
    steelSaturatedMag = 1.36*10^6; % A/m
    maxMRIgrad = 40; %mT/m
    maxMRIgradEntrance = 7200; %mT/m    720 gauss/cm = 7200 mT / m  http://www.mrisafety.com/safety_article.asp?subject=249  THIS IS 180x stronger!
    % A/m * mT/m * m^3 == milli-Newtons.  1 Newton is 1/4 pound, 10 N = 2.25
    % lbs,  10 lbs = 44.48222 N, 70 N = 15.7lbs
    % plot(steelSizeDiamMM/2, 0.001*maxMRIgrad*steelSaturatedMag*sphereVol(steelSizeDiamMM/2*0.001),'-o',steelSizeDiamMM/2, 0.001*maxMRIgradEntrance*steelSaturatedMag*sphereVol(steelSizeDiamMM/2*0.001),'-o')
    
    %{1~{\rm N} = 1~{\rm kg}{\cdot}{\rm m}} / {{\rm s}^2}
    steelSizeDiamMM = 6; %6mm
    
    % 7810 *9.81*sphereVol(steelSizeDiamMM/2*0.001)
    % 0.001*maxMRIgrad*steelSaturatedMag*sphereVol(steelSizeDiamMM/2*0.001)
    %
    
    display([' Magnetic force from MRI 40mT is ',num2str(0.001*maxMRIgrad*steelSaturatedMag/(7810 *9.81) *100),'% of gravity'])
    display([' Magnetic force from MRI 23mT is ',num2str(0.001*23*steelSaturatedMag/(7810 *9.81) *100),'% of gravity'])
    
    function v = sphereVol(r)
        v = 4/3*pi*r.^3;
    end
    
    % A porcupine quill is 1.5mm in diameter, and the cone is three parts
    % (Figure 4, PNAS)
    % part 1 : diam 0, .51mm; ;length 3.62
    % part 2 : 0.51, 1.22; 3.66
    % part 3 : 1.22,1.53; 3.66
    tipVolmm3 = truncatedCone(3.62,0,0.51)+truncatedCone(3.66,0.51,1.22)+truncatedCone(3.66,1.22,1.53);
    tipForce = 0.001*maxMRIgrad*steelSaturatedMag*tipVolmm3/1000/1000/1000;
    penetrationForce  = 0.33;
    display([' Penetration needs ',num2str(tipForce),'  N with porcupine quill'])
    display([' Magnetic force from MRI 40mT on steel porcupine quill tip: ',num2str(tipForce),' N'])
   
    shaftLengthm = ( penetrationForce - tipForce) /( pi*(1.53/1000/2)^2*0.001*maxMRIgrad*steelSaturatedMag);
    
    shaftLengthGRAVm = ( penetrationForce - tipVolmm3/1000/1000/1000*7810 *9.81) /( pi*(1.53/1000/2)^2*7810 *9.81);
    
    display(['Required quill length with MRI 40mT on steel porcupine quill: ',num2str(shaftLengthm),' m'])
    display(['Required quill length with gravity on steel porcupine quill: ',num2str(shaftLengthGRAVm),' m'])
    
    function v = truncatedCone(h,d,D)
        r = d/2;
        R = D/2;
        v = 1/3*pi*h*(r^2+r*R+R^2);
    end
    
end



