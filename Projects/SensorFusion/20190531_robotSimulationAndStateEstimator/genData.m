function genData()
    
    % Create new wamv simulation
    wamv = usvSim;    
    
    % Setup wamv simulation
    [t,lp,ic] = wamv.simDefaults();         % set default values
    [sim,sen] = wamv.simOutputSetup(t);     % create simulation output vectors
    p = wamv.createPlant(lp);               % create dynamical plant
    sim = wamv.insertIC(sim,ic,wamv);       % insert intial conditions into output vectors
        
    % Run simulation
    for k = 1:1:t.N
    
        % Controller
        sim = wamv.controller(sim,k);
        
        % Simulate robot
        sim = wamv.runSim(t,p,sim,wamv,k);
        
        % Update simulated sensor data
        sen = wamv.checkSensorUpdate(t,sen,k);
        sen = wamv.updateSensors(t,sim,sen,wamv,k);
        
        % Increment time
        t.now = t.now+t.dt;
        
    end
    
    % Save output to file    
    save('sim.mat','sim');
    save('sen.mat','sen');

end