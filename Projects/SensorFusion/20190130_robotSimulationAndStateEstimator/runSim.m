function runSim()
    
    % Create new wamv simulation
    wamv = usvSim;    
    
    % Setup wamv simulation
    [t,lp,ic,sen] = wamv.simDefaults();         % set default values
    [sim,sen] = wamv.simOutputSetup(t,sen);     % create simulation output vectors
    p = wamv.createPlant(lp);                   % create dynamical plant
    sim = wamv.insertIC(sim,ic,wamv);           % insert intial conditions into output vectors
    
    % Setup state estimators
    caest = wamv.caEstDefaults();               % set default estimator values
    caest = wamv.caEstOutputSetup(caest,t);     % create estimator output vectors
    ddest = wamv.ddEstDefaults();               % set default estimator values
    ddest = wamv.ddEstOutputSetup(ddest,t);     % create estimator output vectors
    
    % Run simulation
    for k = 1:1:t.N
        
        % Controller
        sim = wamv.controller(sim,k);
        
        % Simulate robot
        sim = wamv.runSim(t,p,sim,wamv,k);
        
        % Update simulated sensor data
        [sim,sen] = wamv.updateSensors(t,sim,sen,wamv,k);
        
        % Estimate state
        caest = wamv.caEst(caest,t,sen,wamv,k);
        [ddest,sen] = wamv.ddEst(ddest,t,sen,wamv,k);
%         ddest = wamv.ddEst(ddest,t,sen,wamv,k);

    end
    
    % Send stuff to workspace for debugging
    assignin('base','simxr',sim.xr);
    assignin('base','simyr',sim.yr);
    assignin('base','simxm',sim.xm);
    assignin('base','simym',sim.ym);
    assignin('base','gpsyr',sen.gps.xr);
    assignin('base','imuyr',sen.imu.xr);
    assignin('base','gpsym',sen.gps.xm);
    assignin('base','imuym',sen.imu.xm);
    assignin('base','caestxr',caest.xr);
    assignin('base','caestxm',caest.xm);
    assignin('base','ddestxr',ddest.xr);
    assignin('base','ddestxm',ddest.xm);
    assignin('base','ddestyr',ddest.yr);
    assignin('base','ddestym',ddest.ym);
    
    % Which plots to show
    simPlot = true;
    senPlot = true;
    caEstPlot = false;
    ddEstPlot = true;
    
    % Robot x vs y
    figure(1);
    hold on;
    if simPlot; plot(sim.ym(1,:),sim.ym(2,:),'b'); end
    if senPlot; plot(sen.gps.xm(1,:),sen.gps.xm(2,:),'ro'); end
    if caEstPlot; plot(caest.xm(1,:),caest.xm(2,:),'g-'); end
    if ddEstPlot; plot(ddest.ym(1,:),ddest.ym(2,:),'c-'); end
    hold off;
    xlabel('x-position [m]');
    ylabel('y-position [m]');
    grid on;
    axis equal;
    
    % Robot positions vs. time (map frame)
    figure(2);
    tt = 0:t.dt:t.tend;
    xm = subplot(3,3,1);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.ym(1,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.gps.xm(1,:),'r.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xm(1,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.ym(1,:),'c-'); end
    hold off;
    ylabel('x pos [m]');
    grid on;
    ym = subplot(3,3,4);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.ym(2,:),'b'); end
  	if senPlot; plot(tt(1:end-1),sen.gps.xm(2,:),'r.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xm(2,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.ym(2,:),'c-'); end
    hold off;
    ylabel('y pos [m]');
    grid on;
    tzm = subplot(3,3,7);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.ym(3,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.imu.xm(3,:),'m.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xm(3,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.ym(3,:),'c-'); end
    hold off;
    xlabel('time [s]');
    ylabel('angle [rad]');
    grid on;
    dxm = subplot(3,3,2);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.ym(4,:),'b'); end
    if caEstPlot; plot(tt(1:end-1),caest.xm(4,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.ym(4,:),'c-'); end
    hold off;
    ylabel('x vel [m/s]');
    grid on;
    dym = subplot(3,3,5);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.ym(5,:),'b'); end
    if caEstPlot; plot(tt(1:end-1),caest.xm(5,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.ym(5,:),'c-'); end
    hold off;
    ylabel('y vel [m/s]');
    grid on;
    dtzm = subplot(3,3,8);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.ym(6,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.imu.xm(6,:),'m.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xm(6,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.ym(6,:),'c-'); end
    hold off;
    xlabel('time [s]');
    ylabel('angular vel [rad]');
    grid on;
    ddxm = subplot(3,3,3);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.ym(7,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.imu.xm(7,:),'m.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xm(7,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.ym(7,:),'c-'); end
    hold off;
    ylabel('x acc [m/s^2]');
    grid on;
    ddym = subplot(3,3,6);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.ym(8,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.imu.xm(8,:),'m.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xm(8,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.ym(8,:),'c-'); end
    hold off;
    ylabel('y acc [m/s^2]');
    grid on;
    ddtzm = subplot(3,3,9);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.ym(9,:),'b'); end
    if caEstPlot; plot(tt(1:end-1),caest.xm(9,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.ym(9,:),'c-'); end
    hold off;
    xlabel('time [s]');
    ylabel('angular acc [rad/s^2]');
    grid on;
    linkaxes([xm,ym,tzm,dxm,dym,dtzm,ddxm,ddym,ddtzm],'x');
    linkaxes([xm,ym],'y');
    linkaxes([dxm,dym],'y');
    linkaxes([ddxm,ddym],'y');
    suptitle('Robot Position vs. Time (Map Frame)');

    % Robot positions vs. time (robot frame)
    figure(3);
    tt = 0:t.dt:t.tend;
    xr = subplot(3,3,1);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.yr(1,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.gps.xr(1,:),'m.-'); end
    if caEstPlot; plot(tt(1:end-1),caest.xr(1,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.yr(1,:),'c-'); end
    grid on;
    ylabel('surge pos [m]');
    grid on;
    yr = subplot(3,3,4);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.yr(2,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.gps.xr(2,:),'m.-'); end
    if caEstPlot; plot(tt(1:end-1),caest.yr(2,:),'g-'); end
    plot(tt(1:end-1),ddest.xr(2,:),'c-');
    grid on;
    ylabel('sway pos [m]');
    grid on;
    tzr = subplot(3,3,7);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.yr(3,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.imu.xr(3,:),'r.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xr(3,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.yr(3,:),'c-'); end
    grid on;
    xlabel('time [s]');
    ylabel('angle [rad]');
    grid on;
    dxr = subplot(3,3,2);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.yr(4,:),'b'); end
    if caEstPlot; plot(tt(1:end-1),caest.xr(4,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.yr(4,:),'c-'); end
    hold off;
    ylabel('surge vel [m/s]');
    grid on;
    dyr = subplot(3,3,5);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.yr(5,:),'b'); end
    if caEstPlot; plot(tt(1:end-1),caest.xr(5,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.yr(5,:),'c-'); end
    hold off;
    ylabel('sway vel [m/s]');
    grid on;
    dtzr = subplot(3,3,8);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.yr(6,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.imu.xr(6,:),'r.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xr(6,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.yr(6,:),'c-'); end
    hold off;
    xlabel('time [s]');
    ylabel('angular vel [rad]');
    grid on;
    ddxr = subplot(3,3,3);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.yr(7,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.imu.xr(7,:),'r.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xr(7,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.yr(7,:),'c-'); end
    hold off;
    ylabel('surge acc [m/s^2]');
    grid on;
    ddyr = subplot(3,3,6);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.yr(8,:),'b'); end
    if senPlot; plot(tt(1:end-1),sen.imu.xr(8,:),'r.'); end
    if caEstPlot; plot(tt(1:end-1),caest.xr(8,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.yr(8,:),'c-'); end
    hold off;
    ylabel('sway acc [m/s^2]');
    grid on;
    ddtzr = subplot(3,3,9);
    hold on;
    if simPlot; plot(tt(1:end-1),sim.yr(9,:),'b'); end
    if caEstPlot; plot(tt(1:end-1),caest.xr(9,:),'g-'); end
    if ddEstPlot; plot(tt(1:end-1),ddest.yr(9,:),'c-'); end
    hold off;
    xlabel('time [s]');
    ylabel('angular acc [rad/s^2]');
    grid on;
    linkaxes([xr,yr,tzr,dxr,dyr,dtzr,ddxr,ddyr,ddtzr],'x');
    linkaxes([xr,yr],'y');
    linkaxes([dxr,dyr],'y');
    linkaxes([ddxr,ddyr],'y');
    linkaxes([xr,yr,tzr,dxr,dyr,dtzr,ddxr,ddyr,ddtzr,...
              xm,ym,tzm,dxm,dym,dtzm,ddxm,ddym,ddtzm],'x');
    linkaxes([xr,yr,xm,ym],'y');
    linkaxes([dxr,dyr,dxm,dym],'y');
    linkaxes([ddxr,ddyr,ddxm,ddym],'y');
    suptitle('Robot Position vs. Time (Robot Frame)');
    
end