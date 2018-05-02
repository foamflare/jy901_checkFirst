%% 直接从串口读取jy901数据（时间戳 roll yaw p itch）
%% 通过得到的角度值控制矩形的翻转
%% 2018-3-8  http://blog.csdn.net/android_lover2014/article/details/52122357
clear  
clc
Serial_obj=serial('COM3');%创建串口对象
Serial_obj.baudrate = 9600;%设置波特率,缺省9600bit/s
%% 打开串口，读写串口内容。  
fopen(Serial_obj);
%% 创建txt文档。   
FileName='data.txt';
FileID=fopen(FileName,'a+');
k = 1; %第一次数据是不正确的
data1_in = 0;
data2_in = 0;
data3_in = 0;
%% 下面对矩形进行定义，并且设置旋转点
[x,y,z]=meshgrid([-1 1]);
hr=slice(x,y,z,z,[-1 1],[-1 1],[-1 1]);
hold off
p=[0 0 0]; %旋转基准点
while true
     rawData = fscanf(Serial_obj,'%s')
     fprintf(FileID,'%s\n',rawData);%将数据写到文
     if k>5 
        S = regexp(rawData,',','split');%得到串口数据，串口数据是角度
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
        %将得到的原始数据进行处理
        direct=[1 0 0];  % direction，set as you will
        hr=slice(x,y,z,z,[-1 1],[-1 1],[-1 1]);
        rotate(hr,direct,data1,p) % api 得到的是角度数据
        
        direct=[0 1 0];  % direction，set as you will
        rotate(hr,direct,data2,p)

        direct=[0 0 1];  % direction，set as you will
        rotate(hr,direct,data3,p)
        hold off
    else
        k = k+1;
     end
     pause(0.0025);
end
%% 关闭串口，清理缓存区 
fclose(Serial_obj);
delete(Serial_obj);  
clear Serial_obj
fclose(FileID)
%%   