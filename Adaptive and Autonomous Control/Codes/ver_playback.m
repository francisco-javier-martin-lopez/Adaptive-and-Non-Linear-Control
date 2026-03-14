function ver_playback(simout)
    % VER_PLAYBACK Visualizador interactivo de trayectoria con Slider
    % Llama a esta función pasando tu resultado de simulación:
    % >> ver_playback(simout)

    %% 1. Preparación de Datos
    % Extraemos tiempo y posiciones de forma segura
    try
        t_data = simout.tout;
        % Posición Real (Dron)
        p_real = squeeze(simout.p_i.Data); 
        if size(p_real, 1) == 3, p_real = p_real'; end % Forzamos a Nx3
        
        % Posición Deseada (Fantasma)
        p_des = squeeze(simout.p_d.Data);
        if size(p_des, 1) == 3, p_des = p_des'; end % Forzamos a Nx3
    catch
        error('No se encuentran los datos p_i o p_d_sim en simout. Revisa los nombres.');
    end

    %% 2. Crear la Figura Interactiva
    f = figure('Name', 'Playback de Trayectoria', 'Color', 'w');
    ax = axes('Parent', f, 'Position', [0.1 0.25 0.8 0.7]);
    grid(ax, 'on'); axis(ax, 'equal'); hold(ax, 'on');
    view(3); xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

    % Dibujar las trayectorias completas (el "camino")
    plot3(ax, p_des(:,1), p_des(:,2), p_des(:,3), '--k', 'LineWidth', 1, 'DisplayName', 'Referencia');
    plot3(ax, p_real(:,1), p_real(:,2), p_real(:,3), '-b', 'LineWidth', 1, 'DisplayName', 'Trayectoria Real');

    % Crear los marcadores de los Drones
    h_des = plot3(ax, p_des(1,1), p_des(1,2), p_des(1,3), 'og', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Obj. Deseado');
    h_real = plot3(ax, p_real(1,1), p_real(1,2), p_real(1,3), 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'UAV Real');

    legend(ax, 'Location', 'best');
    title(ax, sprintf('Tiempo: %.2f s', t_data(1)));

    %% 3. Crear el Slider (La barra de tiempo)
    sld = uicontrol('Parent', f, 'Style', 'slider', ...
        'Position', [100, 50, 400, 20], ...
        'value', 1, 'min', 1, 'max', length(t_data), ...
        'SliderStep', [1/length(t_data) 0.1]); % Paso fino y paso grueso

    % Texto que dice el tiempo al lado del slider
    txt = uicontrol('Parent', f, 'Style', 'text', ...
        'Position', [250, 25, 100, 20], ...
        'String', 'Tiempo: 0.00 s');

    %% 4. Asignar el Callback
    % Aquí conectamos el movimiento del slider con la función de actualización
    sld.Callback = @(es,ed) updatePlot(es, t_data, p_real, p_des, h_real, h_des, ax, txt);

    %% ---------------------------------------------------------
    %% FUNCION LOCAL DE ACTUALIZACION (Ahora sí funciona aquí)
    %% ---------------------------------------------------------
    function updatePlot(slider, time, posR, posD, headR, headD, axisHandle, textHandle)
        idx = round(slider.Value); % Índice actual basado en la barra
        
        % Protección para no salirnos del índice
        if idx < 1, idx = 1; end
        if idx > length(time), idx = length(time); end
        
        % Actualizar posición del punto rojo (Real)
        set(headR, 'XData', posR(idx,1), 'YData', posR(idx,2), 'ZData', posR(idx,3));
        
        % Actualizar posición del punto verde (Deseado)
        set(headD, 'XData', posD(idx,1), 'YData', posD(idx,2), 'ZData', posD(idx,3));
        
        % Actualizar título y texto
        curr_t = time(idx);
        title(axisHandle, sprintf('Tiempo: %.2f s', curr_t));
        set(textHandle, 'String', sprintf('t = %.2f s', curr_t));
        
        drawnow limitrate; % Forzar dibujado eficiente
    end

end