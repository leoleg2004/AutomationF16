function dcm = calcola_dcm(best_theta)
    % La variabile in ingresso (best_theta) viene usata per calcolare la DCM
    dcm = [ cos(best_theta),  0, -sin(best_theta);
            0,                1,  0;
            sin(best_theta),  0,  cos(best_theta)];
end