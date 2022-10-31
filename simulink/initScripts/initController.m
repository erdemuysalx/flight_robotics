function controller = initController(vehicle, propeller)
                      

    controller.dt = 1/300;
    


    % T = alpha_t * Omega^2
    % Q = alpha_q * Omega^2
    

    % speedix 250
    alpha_t = 2.281e-10; 
    alpha_q = 3.1117e-12;
    
    % Dies ist die Steuermatrix aus Übung 2
    Phi = [
        -propeller.positions(:,2)';
         propeller.positions(:,1)';
         propeller.rotDirs' *(alpha_q/alpha_t);
        ones(1,length(propeller.rotDirs))
    ];
    
    % Für die Verteilung der Momente auf die einzelnen Motoren
    % benutzen wir die Inverse (für n>4 eine Pseudoinverse)
    controller.allocation = single(pinv(Phi));
    
    mass_error = 0;
    controller.m = vehicle.m * (1+mass_error);
    
    
end

