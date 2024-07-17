clear all;
clc;
close all;

%%the data logging frequeny is once every 30 seconds 
%%this code was originally written for a 72h cycling between 36C to 38.5C
%%with 5 pads 

%%Read the TXT file containing the rawdata 
data = readtable("5pads_72h_Cycling.TXT");
numberofpads = 5;
data.Properties.VariableNames = ["Temp1","Temp2","Temp3","Temp4","Temp5",...
    "Output1","Output2","Output3","Output4","Output5","Miliseconds"]; %%user might need to change the variable names based on their data collection setup in the Arduino IDE sketch 
data.Minutes=data.Miliseconds./60000;%convert time from Miliseconds to Minutes
data.Hours=data.Miliseconds./3600000;%convert time from Miliseconds to Hours

% %Trim off extra data (example below is to trim off data after hour 75)
% indtoKeep=find(data.Hours>=75,1,'first');
% %data.Hours=data.Hours-data.Hours(indtoKeep);
% data=data(1:indtoKeep,:);

%%plot data 
figure(1)
plot(data,"Hours",["Temp1","Temp2","Temp3","Temp4","Temp5"]);
ylabel("temperature/celsius");
xlabel("time/hour");
xlim([0 90]);
ylim([35.5 39.5]);
title("5pads 72h Cycling");
% plot setpoints
hold on
x=[0 12 24 36 48 60 72 75];%%setpoint change timepoints (in hours)
y=[36 38.5 36 38.5 36 38.5 36 36]; %%setpoint temperatures 
stairs(x,y);
hold off
saveas(figure(1),'5pads_72h_cycling.png');
%plot outputs
figure(2)
plot(data,"Hours",["Output1","Output2","Output3","Output4","Output5"]);
ylabel("Command Output");
xlabel("time/hour");
title("5pads 72h cycling output");

%%calculate RMSE between each pad to the setpoints 

for i=1:(length(x)-1)
    [M,I]=min((data.Hours-x(i)).^2);  
    [M2,I2]=min((data.Hours-x(i+1)).^2);
    data.Setpoint(I:I2)=y(i);
end
Rtmeansq=[];
for i=1:numberofpads
    Actual=table2array(data(:,i));
    Setpoint=data.Setpoint;
    Rtmeansq=[Rtmeansq rmse(Setpoint,Actual)];
end


%%ramp time analysis
%%get the average and the mean absolute deviation of the steady states 
tmp=[];
dv=[];
for i=1:6
    avgtemp=[];
    md=[];
    for j=1:numberofpads
    [M,I]=min((data.Hours-i*12).^2);    
    range=data(I-60:I-2,j);%%range is from 30 mins before setpoint change to 1 mins before setpoint change 
    avgtemp=[avgtemp,mean(range)];%average temperature of 30 mins to 1mins before the set point  
    end
    tmp=[tmp;avgtemp];
end

time=[];
for i=[1,3,5] %heating
    timepoint=[];
    for j=1:numberofpads 
    [M,I]=min((data.Hours-i*12).^2);    
    range=data(I:I+360,j);%search up to 3 hours after the setpoint 
    temperature=movmean(table2array(range),[0 20]);%moving window average (size:10 mins)
    averagetemperature=table2array(tmp(i+1,j));
    inx=find(abs(temperature-averagetemperature)<=0.5,1,'first');%find timepoint when the moving window average is within 0.5 C from the steady state temperature 
    timepoint=[timepoint,data.Minutes(I+inx)-data.Minutes(I)];%calculate the ramp time
    end
    time=[time;timepoint];
end

for i=[2,4,6] %cooling
    timepoint=[];
    for j=1:numberofpads 
    [M,I]=min((data.Hours-i*12).^2);    
    range=data(I:I+360,j);%search up to 3 hours after the setpoint 
    temperature=movmean(table2array(range),[0 20]);%%moving window average (size:10 mins)
    averagetemperature=table2array(tmp(i-1,j));
    inx=find(abs(temperature-averagetemperature)<=0.5,1,'first');%find timepoint when the moving window average is within 0.5 C from the steady state temperature 
    timepoint=[timepoint,data.Minutes(I+inx)-data.Minutes(I)];%calculate the ramp time
    end
    time=[time;timepoint];
end

%%plot the ramp times (average of 5 pads)  
figure(3);
time_mean=mean(time,2);
bar(time_mean);
xticklabels(["heating1","heating2","heating3","cooling1","cooling2","cooling3"]);



