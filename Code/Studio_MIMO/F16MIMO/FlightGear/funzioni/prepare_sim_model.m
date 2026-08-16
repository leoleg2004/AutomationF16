function prepare_sim_model()
    % Prepara un modello Simulink con gli input collegati al Workspace
    origModel = 'F16';
    newModel = 'F16_MPC';
    
    try bdclose(newModel); catch; end
    addpath(fullfile(pwd, 'Modelli_Simulink'));
    load_system(origModel);
    save_system(origModel, fullfile(pwd, 'Modelli_Simulink', [newModel, '.slx']));
    
    inports = {'Thrust', 'ele', 'ail', 'rud', 'lef', 'gust'};
    vars = {'Thrust_ts', 'ele_ts', 'ail_ts', 'rud_ts', 'dlef_ts', 'gust_ts'};
    dstBlocks = {'Mu x', 'Nonlinear F16 Model', 'Nonlinear F16 Model', 'Nonlinear F16 Model', 'Nonlinear F16 Model', 'Nonlinear F16 Model'};
    dstPorts = {'1', '4', '5', '6', '7', '1'};
    
    for i = 1:length(inports)
        blk = [newModel, '/', inports{i}];
        pos = get_param(blk, 'Position');
        delete_line(newModel, [inports{i}, '/1'], [dstBlocks{i}, '/', dstPorts{i}]);
        delete_block(blk);
        add_block('simulink/Sources/From Workspace', blk, 'Position', pos);
        set_param(blk, 'VariableName', vars{i});
        add_line(newModel, [inports{i}, '/1'], [dstBlocks{i}, '/', dstPorts{i}], 'autorouting', 'on');
    end
    
    save_system(newModel);
    disp('Modello di simulazione pronto!');
end
