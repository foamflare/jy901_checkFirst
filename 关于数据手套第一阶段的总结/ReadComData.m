%% ֱ�ӴӴ��ڶ�ȡjy901���ݣ�ʱ��� roll yaw p itch��
%% ͨ���õ��ĽǶ�ֵ���ƾ��εķ�ת
%% 2018-3-8  http://blog.csdn.net/android_lover2014/article/details/52122357
clear  
clc
Serial_obj=serial('COM3');%�������ڶ���
Serial_obj.baudrate = 9600;%���ò�����,ȱʡ9600bit/s
%% �򿪴��ڣ���д�������ݡ�  
fopen(Serial_obj);
%% ����txt�ĵ���   
FileName='data.txt';
FileID=fopen(FileName,'a+');
k = 1; %��һ�������ǲ���ȷ��
data1_in = 0;
data2_in = 0;
data3_in = 0;
%% ����Ծ��ν��ж��壬����������ת��
[x,y,z]=meshgrid([-1 1]);
hr=slice(x,y,z,z,[-1 1],[-1 1],[-1 1]);
hold off
p=[0 0 0]; %��ת��׼��
while true
     rawData = fscanf(Serial_obj,'%s')
     fprintf(FileID,'%s\n',rawData);%������д����
     if k>5 
        S = regexp(rawData,',','split');%�õ��������ݣ����������ǽǶ�
        time = S(1);
        data1=str2num(char(S(2)));
        data2=str2num(char(S(3)));
        data3=str2num(char(S(4)));
        
%         data1=str2num(char(S(2)))-data1_in;
%         data2=str2num(char(S(3)))-data2_in;
%         data3=str2num(char(S(4)))-data3_in;
%         data1_in = str2num(char(S(2)));
%         data2_in = str2num(char(S(3)));
%         data3_in = str2num(char(S(4)));
        %���õ���ԭʼ���ݽ��д���
        direct=[1 0 0];  % direction��set as you will
        hr=slice(x,y,z,z,[-1 1],[-1 1],[-1 1]);
        rotate(hr,direct,data1,p) % api �õ����ǽǶ�����
        
        direct=[0 1 0];  % direction��set as you will
        rotate(hr,direct,data2,p)

        direct=[0 0 1];  % direction��set as you will
        rotate(hr,direct,data3,p)
        hold off
    else
        k = k+1;
     end
     pause(0.0025);
end
%% �رմ��ڣ��������� 
fclose(Serial_obj);
delete(Serial_obj);  
clear Serial_obj
fclose(FileID)
%%   