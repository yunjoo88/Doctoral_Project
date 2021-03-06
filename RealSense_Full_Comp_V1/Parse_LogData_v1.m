clear all
fin=fopen('LogData_DirectAll.txt');
while(~feof(fin)) 
    try
        timecnt = fscanf(fin, 'Saved Data in ID %d \n ',1)+1;
        HumanHandPos(:,timecnt) = fscanf(fin, 'HumanHandPos : %f, %f, %f \n',3);
        Gradient(:,timecnt) = fscanf(fin, 'Gradinet : %f, %f, %f \n',3);
        RobotHandPos(:,timecnt) = fscanf(fin, 'RobotHandPos : %d, %d \n ',2); 
        RobotHandCurrent(:,timecnt) = fscanf(fin, 'RobotHandCurrnet : %d, %d \n ',2); 
        RobotAmrPos(:,timecnt) = fscanf(fin, 'RobotAmrPos : %f, %f, %f, %f, %f, %f \n',6);
        RobotArmTorque(:,timecnt) = fscanf(fin, 'RobotArmTorque : %f, %f, %f, %f, %f, %f, %f \n',7);
        TactileData(:,timecnt) = fscanf(fin, 'TactileData : %f, %f, %f, %f, %f, %f, %f, %f \n\n',8);
    catch
    end
end
fclose all;

if (length(TactileData)~=timecnt)
    HumanHandPos=HumanHandPos(:,1:timecnt-1);
    Gradient=Gradient(:,1:timecnt-1);
    RobotHandPos=RobotHandPos(:,1:timecnt-1);
    RobotHandCurrent=RobotHandCurrent(:,1:timecnt-1);
    RobotAmrPos=RobotAmrPos(:,1:timecnt-1);
    RobotArmTorque=RobotArmTorque(:,1:timecnt-1);
    TactileData=TactileData(:,1:timecnt-1);
end
figure(1);
subplot(311); plot(HumanHandPos');
subplot(312); plot(RobotAmrPos(1:3,:)'); ylim([-1 1]);
subplot(313); plot(Gradient');

figure(2);
subplot(311); plot(TactileData');
subplot(312); plot(RobotAmrPos(1:3,:)');
subplot(313); plot(RobotAmrPos(4:6,:)');