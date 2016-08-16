clc

disp('Connecting to Arduino...')
arduino=serial('COM4','BaudRate',9600); % create serial communication object on port COM4
arduino.InputBufferSize = 20;
arduino.OutputBufferSize = 20;
fopen(arduino); % initiate arduino communication
disp('Arduino connected!')


hour = 0;
hour_echo = 0;
minu = 0;
minu_echo = 0;
flag = 0;
pause(10);

while 1
    c = clock;
    
    hour = uint8(c(4));
    minu = uint8(c(5));

    if(hour ==9 || hour == 19)
        flag = 0;
    end
        
    if(flag ==0 && (hour ==8 || hour == 18))
        flag = 1;
        fwrite(arduino,uint8(hour),'sync');
        fwrite(arduino,uint8(minu),'sync');
        echo_data=fread(arduino,2,'uint8');
%         disp('[hour minutes] =')
%         disp([hour, minu]);
        hour_echo = echo_data(1);
        minu_echo = echo_data(2);
        disp('echo [hour minutes] =')
        disp([hour_echo, minu_echo]);
    end

pause(2400);
end
 
fclose(arduino);
delete(arduino)
clear arduino
clear all