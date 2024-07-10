% Crear una tabla vacía
load('evaluation_result.mat', 'ATE_POSE');
% Obtener una lista de todos los nombres de slam
slam_celdas = ATE_POSE(:, 2);
slam_nombres_celda = cellfun(@(x) x{1}, [slam_celdas{:}], 'UniformOutput', false);
slam_nombres = unique(slam_nombres_celda);

% Inicializar la tabla de datos con las variables esperadas
datos_excel = table(cell(size(ATE_POSE, 1), 1), 'VariableNames', {'nombre_test'});
for slam_idx = 1:numel(slam_nombres)
    datos_excel.(slam_nombres{slam_idx}) = cell(size(ATE_POSE, 1), 1);
end

% Iterar sobre los datos de ATE_POSE
for i = 1:size(ATE_POSE, 1)
    % Obtener el nombre del test
    nombre_test = ATE_POSE{i, 1};
    
    % Obtener los resultados para cada slam
    resultados_slam = ATE_POSE{i, 2};
    datos_excel.nombre_test{i} = nombre_test;
    % Iterar sobre los resultados para cada slam
    for j = 1:numel(resultados_slam)
        % Obtener el nombre del slam y el resultado
        nombre_slam = resultados_slam{j}{1};
        resultado = resultados_slam{j}{2};
        
        % Agregar el resultado a la fila
        datos_excel.(nombre_slam){i} = resultado;
    end
end

% Escribir la tabla en un archivo Excel
nombre_archivo_excel = 'resultados_ATE_POSE.xlsx';
writetable(datos_excel, nombre_archivo_excel);


