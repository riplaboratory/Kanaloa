% Runs estimator(s) on 'sim.mat' and 'sen.mat' files (must be in working
% directory).  Run 'getData.m' first, then run this.

function runEstOnData()
    
    % Create new wamv simulation
    wamv = usvSim;
    
    % Load existing sim and sen structures from file
    load('sen.mat','sen');
    load('sim.mat','sim');
    
    % Reset sensor incrementors
    sen.gps.k = 1;
    sen.gps.update = 0;
    sen.imu.k = 1;
    sen.imu.update = 0;
    sen.gps.lastUpdateTime = 0;
    sen.imu.lastUpdateTime = 0;
    
    % Setup wamv simulation
    [t,lp,ic] = wamv.simDefaults();         % set default values
    p = wamv.createPlant(lp);               % create dynamical plant
    sim = wamv.insertIC(sim,ic,wamv);       % insert intial conditions into output vectors
            
    % Setup state estimators
    caEst = wamv.caEstDefaults();               % set default estimator values
    caEst = wamv.caEstOutputSetup(t,caEst);     % create estimator output vectors    
    mcaEst = wamv.mcaEstDefaults();             % set default estimator values
    mcaEst = wamv.mcaEstOutputSetup(t,mcaEst);  % create estimator output vectors

    % Run simulation
    for k = 1:1:t.N
        
        % Update sensor incrementors
        sen = wamv.checkSensorUpdate(t,sen,k);
        
        % Executes when estimator should update                
        if mod(k*t.dt,caEst.dt) == 0
            
            % Estimate state using constant-acceleration model
            caEst = wamv.caEst(caEst,t,sen,wamv,k);

            % Estimate state using modified constant-acceleration model
            mcaEst = wamv.mcaEstMeas(mcaEst,t,sen);
%             [mcaEst,meas,RGain] = wamv.mcaEstMeas(mcaEst,t,sen);
            mcaEst = wamv.mcaEst(mcaEst,t,wamv,k);
%             mcaEst = wamv.mcaEst(mcaEst,t,meas,RGain,wamv,k);
        
        end 
            
        % Increment time
        t.now = t.now+t.dt;
        
    end
    
    % Send stuff to workspace for debugging
    assignin('base','simxr',sim.xr);
    assignin('base','simxm',sim.xm);
    assignin('base','gpsxr',sen.gps.xr);
    assignin('base','imuxr',sen.imu.xr);
    assignin('base','gpsxm',sen.gps.xm);
    assignin('base','imuxm',sen.imu.xm);
    assignin('base','caest',caEst);
    assignin('base','mcaest',mcaEst);
   
    % Plot stuff
    p.sim = true;
    p.sen = true;
    p.caEst = true;
    p.mcaEst = true;
    close all;
    plotXY(p,sim,sen,caEst,mcaEst,1);
    plotMapFrame(p,t,sim,sen,caEst,mcaEst,2);
    plotRobotFrame(p,t,sim,sen,caEst,mcaEst,3);
    
end

function plotXY(p,sim,sen,caEst,mcaEst,figN)
    
    % Robot x vs y
    figure(figN);
    hold on;
    if p.sim; plot(sim.ym(1,:),sim.ym(2,:),'k'); end
    if p.sen; plot(sen.gps.xm(1,:),sen.gps.xm(2,:),'ro'); end
    if p.caEst; plot(caEst.xm(1,:),caEst.xm(2,:),'g-'); end
    if p.mcaEst; plot(mcaEst.xm(1,:),mcaEst.xm(2,:),'b-'); end
    hold off;
    xlabel('x-position [m]');
    ylabel('y-position [m]');
    grid on;
    axis equal;
    
end

function plotMapFrame(p,t,sim,sen,caEst,mcaEst,figN)
      
    % Robot positions vs. time (map frame)
    figure(figN);
    tt = 0:t.dt:t.end;
    ttGps = 0:1/sen.gps.rr:t.end;
    ttImu = 0:1/sen.imu.rr:t.end;
    ttCaEst = 0:caEst.dt:t.end;
    ttMcaEst = 0:mcaEst.dt:t.end;
    xm = subplot(3,3,1);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.ym(1,:),'k'); end
    if p.sen; plot(ttGps(1:end-1),sen.gps.xm(1,:),'r.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xm(1,:),'g-'); end    
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xm(1,:),'b-'); end
    hold off;
    ylabel('x pos [m]');
    grid on;
    title('Map frame positions');
    ym = subplot(3,3,4);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.ym(2,:),'k'); end
  	if p.sen; plot(ttGps(1:end-1),sen.gps.xm(2,:),'r.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xm(2,:),'g-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xm(2,:),'b-'); end
    hold off;
    ylabel('y pos [m]');
    grid on;
    tzm = subplot(3,3,7);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.ym(3,:),'k'); end
    if p.sen; plot(ttImu(1:end-1),sen.imu.xm(3,:),'m.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xm(3,:),'g-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xm(3,:),'b-'); end
    hold off;
    xlabel('time [s]');
    ylabel('angle [rad]');
    grid on;
    dxm = subplot(3,3,2);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.ym(4,:),'k'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xm(4,:),'b-'); end
    hold off;
    ylabel('x vel [m/s]');
    grid on;
    title('Map frame velocities');
    dym = subplot(3,3,5);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.ym(5,:),'k'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xm(5,:),'b-'); end
    hold off;
    ylabel('y vel [m/s]');
    grid on;
    dtzm = subplot(3,3,8);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.ym(6,:),'k'); end
    if p.sen; plot(ttImu(1:end-1),sen.imu.xm(6,:),'m.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xm(6,:),'g-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xm(6,:),'b-'); end
    hold off;
    xlabel('time [s]');
    ylabel('angular vel [rad]');
    grid on;
    ddxm = subplot(3,3,3);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.ym(7,:),'k'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xm(7,:),'b-'); end
    hold off;
    ylabel('x acc [m/s^2]');
    grid on;
    title('Map frame accelerations');
    ddym = subplot(3,3,6);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.ym(8,:),'k'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xm(8,:),'b-'); end
    hold off;
    ylabel('y acc [m/s^2]');
    grid on;
    ddtzm = subplot(3,3,9);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.ym(9,:),'k'); end
    hold off;
    xlabel('time [s]');
    ylabel('angular acc [rad/s^2]');
    grid on;
    linkaxes([xm,ym,tzm,dxm,dym,dtzm,ddxm,ddym,ddtzm],'x');
    linkaxes([xm,ym],'y');
    linkaxes([dxm,dym],'y');
    linkaxes([ddxm,ddym],'y');
    
end

function plotRobotFrame(p,t,sim,sen,caEst,mcaEst,figN)

    figure(figN);
    tt = 0:t.dt:t.end;
    ttGps = 0:1/sen.gps.rr:t.end;
    ttImu = 0:1/sen.imu.rr:t.end;
    ttCaEst = 0:caEst.dt:t.end;
    ttMcaEst = 0:mcaEst.dt:t.end;
    xr = subplot(3,3,1);
    hold on;        
    if p.sim; plot(tt(1:end-1),sim.yr(1,:),'k'); end
    if p.sen; plot(ttGps(1:end-1),sen.gps.xr(1,:),'m.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xr(1,:),'g-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xr(1,:),'b-'); end
    grid on;
    ylabel('surge pos [m]');
    grid on;
    title('Robot frame positions');
    yr = subplot(3,3,4);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.yr(2,:),'k'); end
    if p.sen; plot(ttGps(1:end-1),sen.gps.xr(2,:),'m.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xr(2,:),'g-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xr(2,:),'b-'); end
    grid on;
    ylabel('sway pos [m]');
    grid on;
    tzr = subplot(3,3,7);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.yr(3,:),'k'); end
    if p.sen; plot(ttImu(1:end-1),sen.imu.xr(3,:),'m.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xr(3,:),'g-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xr(3,:),'b-'); end
    grid on;
    xlabel('time [s]');
    ylabel('angle [rad]');
    grid on;
    dxr = subplot(3,3,2);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.yr(4,:),'k'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xr(4,:),'b-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.meas.imu(4,:),'c.'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.meas.gps(4,:),'y.'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.meas.all(4,:),'m.'); end
    hold off;
    ylabel('surge vel [m/s]');
    grid on;
    title('Robot frame velocities');
    dyr = subplot(3,3,5);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.yr(5,:),'k'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xr(5,:),'b-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.meas.imu(5,:),'c.'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.meas.gps(5,:),'y.'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.meas.all(5,:),'m.'); end
    hold off;
    ylabel('sway vel [m/s]');
    grid on;
    dtzr = subplot(3,3,8);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.yr(6,:),'k'); end
    if p.sen; plot(ttImu(1:end-1),sen.imu.xr(6,:),'r.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xr(6,:),'g-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xr(6,:),'b-'); end
    hold off
    xlabel('time [s]');
    ylabel('angular vel [rad]');
    grid on;
    ddxr = subplot(3,3,3);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.yr(7,:),'k'); end
    if p.sen; plot(ttImu(1:end-1),sen.imu.xr(7,:),'r.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xr(7,:),'g-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xr(7,:),'b-'); end
    hold off;
    ylabel('surge acc [m/s^2]');
    grid on;
    title('Robot frame accelerations');
    ddyr = subplot(3,3,6);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.yr(8,:),'k'); end
    if p.sen; plot(ttImu(1:end-1),sen.imu.xr(8,:),'r.'); end
    if p.caEst; plot(ttCaEst(1:end-1),caEst.xr(8,:),'g-'); end
    if p.mcaEst; plot(ttMcaEst(1:end-1),mcaEst.xr(8,:),'b-'); end
    hold off;
    ylabel('sway acc [m/s^2]');
    grid on;
    ddtzr = subplot(3,3,9);
    hold on;
    if p.sim; plot(tt(1:end-1),sim.yr(9,:),'k'); end
    hold off;
    xlabel('time [s]');
    ylabel('angular acc [rad/s^2]');
    grid on;
    linkaxes([xr,yr,tzr,dxr,dyr,dtzr,ddxr,ddyr,ddtzr],'x');
    linkaxes([xr,yr],'y');
    linkaxes([dxr,dyr],'y');
    linkaxes([ddxr,ddyr],'y');

end