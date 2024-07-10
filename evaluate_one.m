function P_est_ate_array = evaluate_one(test_id, test_fullname)
warning('off', 'all');
close all;

fighd = [];
colors = [
   
    0 1 0;  % Verde
    0 0 1;  % Azul
    1 0 1;  % Magenta
    0 1 1;  % Cian
    0.5 0.5 0.5;  % Gris
    0 0 0;  % Negro
    1 1 0;  % Amarillo
    % Puedes agregar más colores según sea necesario
];
    colors2= [".b",".y",'.black'];

%% get the exp name number
exp_name =  test_fullname.name;
exp_path = [test_fullname.folder '/' test_fullname.name '/'];

gndtr_pos_fn     = [exp_path 'leica_pose.csv'];
pose_est_fn      = [exp_path 'predict_odom.csv'];
trans_B2prism_fn = [exp_path '../trans_B2prism.csv'];

% Obtener una lista de todos los archivos que terminen en 'predict_odom.csv'
odom_files = dir([exp_path '*predict_odom.csv']);
comp_time_files = dir([exp_path '*comp_time.csv']);
% if empty files return
if isempty(comp_time_files)
    return;
end

% Rest of the code goes here...
odom_file_names = cell(1, length(odom_files));
comp_time_file_names = cell(1, length(comp_time_files));
% P_est_array =[];
% Iterar sobre cada archivo encontrado
for i = 1:length(odom_files)
    pose_est_fn = [exp_path odom_files(i).name];
    
    % Guardar el nombre del archivo en el array
    odom_file_names{i} = pose_est_fn;
end
for i = 1:length(comp_time_files)
    comp_time_fn = [exp_path comp_time_files(i).name];
    
    % Guardar el nombre del archivo en el array
    comp_time_file_names{i} = comp_time_fn;
end
disp('Nombres de archivos encontrados:');
disp(odom_file_names);

%% Read the gndtr data from logs

% Position groundtr
gndtr_pos_data = csvread(gndtr_pos_fn,  1, 0);

% First sample time used for offsetting all others
t0_ns = gndtr_pos_data(1, 1);

% pos groundtruthdata
t = (gndtr_pos_data(:, 1) - t0_ns)/1e9;
P = gndtr_pos_data(:, 4:6);

% Delete the duplicate in position groundtruth data
[~, Px_unq_idx] = unique(P(:, 1));
[~, Py_unq_idx] = unique(P(:, 2));
[~, Pz_unq_idx] = unique(P(:, 3));

P_unq_idx = union(union(Px_unq_idx, Py_unq_idx), Pz_unq_idx);
P = P(P_unq_idx, :);
t = t(P_unq_idx, :);


%% Read the viralslam estimate data from logs
% SLAM estimate
for i = 1:length(odom_file_names)
    pose_est_fn = odom_file_names{i} ;
    pose_est_data = csvread(pose_est_fn, 1, 0);
    t_est = (pose_est_data(:, 1) - t0_ns)/1e9;
    P_est =  pose_est_data(:, 4:6);
    Q_est = (pose_est_data(:, [10, 7:9]));
    V_est =  pose_est_data(:, 11:13);
    comp_time_data = csvread(comp_time_file_names{i}, 1, 0);
    comp_time_t = (comp_time_data(:, 1) - t0_ns)/1e9;
    comp_time = comp_time_data(:, 2);
    % Transform from body frame to the prism
    trans_B2prism = csvread(trans_B2prism_fn, 0, 0);
    
    % Compensate the position estimate with the prism displacement
    P_est = P_est + quatconv(Q_est, trans_B2prism);
    
    
    %% Resample the ground truth data by estimate data sample times
    
    % Note affix rs[x] is for resampled by [x]
    
    % Find the interpolated time stamps
    clear rsest_pos_itp_idx;
    [rsest_pos_itp_idx(:, 1), rsest_pos_itp_idx(:, 2)] = combteeth(t_est, t, 0.1);
    
    % Remove the un-associatable samples
    rsest_nan_idx = find(isnan(rsest_pos_itp_idx(:, 1)) | isnan(rsest_pos_itp_idx(:, 2)));
    
    t_est_full = t_est;
    P_est_full = P_est;
    Q_est_full = Q_est;
    V_est_full = V_est;
    comp_time_full = comp_time;
    
    rsest_pos_itp_idx(rsest_nan_idx, :) = [];
    t_est(rsest_nan_idx, :)     = [];
    t_est_array{i}=t_est;
    P_est(rsest_nan_idx, :)     = [];
    Q_est(rsest_nan_idx, :)     = [];
    V_est(rsest_nan_idx, :)     = [];
    
    % interpolate the pos gndtr state
    P_rsest = vecitp(P, t, t_est, rsest_pos_itp_idx);

    
    % find the optimal alignment
    [rot_align_est, trans_align_est] = traj_align(P_rsest, P_est);
    
    % Align the position estimate
    P_est      = (rot_align_est*P_est'      + trans_align_est)';
    P_est_array{i} = P_est;
    P_est_full = (rot_align_est*P_est_full' + trans_align_est)';
    P_est_full_array{i} = P_est_full;
    % Align the orientaton estimate
    Q_est      = quatmultiply(rotm2quat(rot_align_est), Q_est);
    Q_est_full = quatmultiply(rotm2quat(rot_align_est), Q_est);
    
    % Align the velocity estimate
    V_est      = (rot_align_est*V_est')';
    V_est_full = (rot_align_est*V_est_full')';
    
    
    % Export the leica transform to a yaml file
    fileID = fopen([exp_path 'leica_tf.yaml'], 'w');
    fprintf(fileID, ['%%YAML:1.0\n'...
                     'T_W_Wleica: !!opencv-matrix\n'...
                     '  rows: 4\n'...
                     '  cols: 4\n'...
                     '  dt: d\n']);
    R_W2L   =  rot_align_est';
    t_W2L   = -rot_align_est'*trans_align_est;
    T_W2L   = [R_W2L, t_W2L; 0 0 0 1];
    T_W2L_str = sprintf(['  data: [ %0.9f, %0.9f, %0.9f, %0.9f,\n'...
                         '          %0.9f, %0.9f, %0.9f, %0.9f,\n'...
                         '          %0.9f, %0.9f, %0.9f, %0.9f,\n'...
                         '          %0.9f, %0.9f, %0.9f, %0.9f ]'],...
        T_W2L(1, 1), T_W2L(1, 2), T_W2L(1, 3), T_W2L(1, 4),...
        T_W2L(2, 1), T_W2L(2, 2), T_W2L(2, 3), T_W2L(2, 4),...
        T_W2L(3, 1), T_W2L(3, 2), T_W2L(3, 3), T_W2L(3, 4),...
        T_W2L(4, 1), T_W2L(4, 2), T_W2L(4, 3), T_W2L(4, 4));
    fprintf(fileID, T_W2L_str);
    fclose(fileID);
    
    % Note: this transform can transform the leica estimate to the slam local
    % frame, which can be convenient if you want to record the simulation on
    % rviz
    
    
    %% Calculate the position and rotation errors
    
    
    %% Calculate the absolute trajectory error of position estimate
    P_est_err     = P_rsest - P_est;
    P_est_err_array{i}=P_est_err;
    P_est_rmse    = rms(P_est_err);
    P_est_ate     = norm(P_est_rmse);
    name =odom_files(i).name;
    slam_names{i} = regexprep(name, {'predict_odom.csv', '[^a-zA-Z0-9_]'}, '');
    P_est_ate_array{i}{1} = slam_names{i};
    P_est_ate_array{i}{2} = P_est_ate;
    disp(odom_files(i).name)
    disp ("ate")
    disp(P_est_ate)
      end
    %% Print the result
    fprintf('test: %2d. %s. Err: P_est_ate: %6.3f\n',...
              test_id, exp_name(8:end), P_est_ate);
    
    
  
    
    
    %% Calculate the maximum time
    t_max = max([t; t_est]);
    
    
    % ba_plot_style = {'linestyle', 'none',...
    %                   'marker', 'diamond',...
    %                   'markerfacecolor', myorange,...
    %                   'markeredgecolor', myorange,...
    %                   'markersize', 5};
    

    %% Plot the 3D trajectory
    figpos = [1920 0 0 0] + [0, 480, 630, 400];
    figure('position', figpos, 'color', 'w', 'paperpositionmode', 'auto');
    fighd = [fighd gcf];
    % figure;
    % Plot the signal point to let the legend generator use the line symbol
    plot3(P(1:2, 1), P(1:2, 2),P(1:2, 3), 'r', 'linewidth', 3);
    hold on;
    for i = 1:length(P_est_array)
    % Obtienes el color correspondiente para este conjunto de datos
    color = colors(i, :);
    
    % Ploteas cada conjunto de datos P_est en una iteración separada del bucle
    plot3(P_est_array{i}(1:2, 1),P_est_array{i}(1:2, 2),P_est_array{i}(1:2, 3), 'linewidth', 2, 'color', color);
    hold on;
    end
%     plot3(P_est(1:2, 1),  P_est(1:2, 2),  P_est(1:2, 3), 'b', 'linewidth', 3);
    
    % Plot the full trajectory in '.' style to avoid messy gaps
    plot3(P(:, 1), P(:, 2), P(:, 3), '.r', 'markersize', 6);
    hold on;
   for i = 1:length(P_est_full_array)
    % Obtienes el color correspondiente para este conjunto de datos
    color = colors(i,:);

    % Ploteas cada conjunto de datos P_est en una iteración separada del bucle
    plot3(P_est_full_array{i}(:, 1),P_est_full_array{i}(:, 2),P_est_full_array{i}(:, 3),'.','color',color, 'markersize', 6);
    hold on;
   end
   
%     plot3(P_est_full(:, 1),  P_est_full(:, 2),  P_est_full(:, 3),'.b', 'markersize', 6);
    
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    grid on;
    daspect([1 1 1]);
    % view([-21 15]);
    tightfig;
    set(gca, 'fontsize', 13);
    % lg_hd = legend('Leica', 'LOAM (H)', 'LOAM (V)', 'viralslam');

    lg_hd = legend(['Groundtruth', slam_names]);
    
    % Save the plot as .fig as well as .png
    saveas(gcf, [exp_path exp_name '_traj.fig']);
    img = getframe(gcf);
    imwrite(img.cdata, [exp_path exp_name '_traj.png']);



%% Plot the time evolution of position

figpos = [1920 0 0 0] + [0, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(4, 1, 1);
hold on;
axgndtr = plot(t,     P(:, 1),     'r', 'linewidth', 4);
g_size= size(P);

% Iteras sobre los conjuntos de datos P_est
for i = 1:length(P_est_array)
    % Obtienes el color correspondiente para este conjunto de datos
    color = colors(i, :);
    
    % Ploteas cada conjunto de datos P_est en una iteración separada del bucle
    axest = plot(t_est_array{i}, P_est_array{i}(:, 1), 'linewidth', 2, 'color', color);
end
uistack(axgndtr, 'top');
% uistack(axest, 'top');
ylabel('X [m]');
lg_hd = legend([slam_names,'Groundtruth']);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(4, 1, 2);
hold on;
axgndtr = plot(t,     P(:, 2),     'r', 'linewidth', 4);
% Iteras sobre los conjuntos de datos P_est
for i = 1:length(P_est_array)
    % Obtienes el color correspondiente para este conjunto de datos
    color = colors(i, :);
    
    % Ploteas cada conjunto de datos P_est en una iteración separada del bucle
    axest = plot(t_est_array{i}, P_est_array{i}(:, 2), 'linewidth', 2, 'color', color);
end
uistack(axgndtr, 'top');
uistack(axest,   'top');
ylabel('Y [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(4, 1, 3);
hold on;
axgndtr = plot(t,   P(:, 3),       'r', 'linewidth', 3);
% Iteras sobre los conjuntos de datos P_est
for i = 1:length(P_est_array)
    % Obtienes el color correspondiente para este conjunto de datos
    color = colors(i, :);
    
    % Ploteas cada conjunto de datos P_est en una iteración separada del bucle
    axest = plot(t_est_array{i}, P_est_array{i}(:, 3), 'linewidth', 2, 'color', color);
end
uistack(axgndtr, 'top');
uistack(axest,   'top');
xlabel('Time [s]');
ylabel('Z [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(4, 1, 4);
hold on;
for i = 1:length(comp_time_file_names)
    comp_time_fn = comp_time_file_names{i};
    comp_time_data = csvread(comp_time_fn, 1, 0);
    comp_time_t = (comp_time_data(:, 1) - t0_ns) / 1e9;
    comp_time = comp_time_data(:, 2);
    plot(comp_time_t, comp_time, 'linewidth', 2, 'color', colors(i, :));
end
xlabel('Time [s]');
ylabel('CT [ms]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyzt.fig']);
% saveas(gcf, [exp_path exp_name '_xyzt.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyzt.png']);



%% Plot the time evolution of position estimation error
figpos = [1920 0 0 0] + [630, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
for i = 1:length(P_est_array)
    % Obtienes el color correspondiente para este conjunto de datos
    color = colors(i, :);
    
    % Ploteas cada conjunto de datos P_est en una iteración separada del bucle
    plot(t_est_array{i}, P_est_err_array{i}(:, 3), 'linewidth', 2, 'color', color);
end

% plot(t_est,  P_est_err(:, 1),  'b', 'linewidth', 2);
ylabel('X Err. [m]');
lg_hd = legend([slam_names]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
for i = 1:length(P_est_array)
    % Obtienes el color correspondiente para este conjunto de datos
    color = colors(i, :);
    
    % Ploteas cada conjunto de datos P_est en una iteración separada del bucle
    plot(t_est_array{i}, P_est_err_array{i}(:, 3), 'linewidth', 2, 'color', color);
end
% plot(t_est,  P_est_err(:, 2),  'b', 'linewidth', 2);
ylabel('Y Err [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
for i = 1:length(P_est_array)
    % Obtienes el color correspondiente para este conjunto de datos
    color = colors(i, :);
    
    % Ploteas cada conjunto de datos P_est en una iteración separada del bucle
    plot(t_est_array{i}, P_est_err_array{i}(:, 3), 'linewidth', 2, 'color', color);
end
% plot(t_est,  P_est_err(:, 3),  'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Z Err [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);
% Ajustar la separación entre subplots
h = findobj(gcf, 'Type', 'axes');
for k = 1:length(h)
    pos = get(h(k), 'Position');
    set(h(k), 'Position', [pos(1), pos(2) + 0.02, pos(3), pos(4) - 0.02]);
end
tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyz_err_t.fig']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyz_err_t.png']);



%% Plot the combined time evolution of position estimation error
% figpos = [1920 0 0 0] + [630, 480, 630, 200];
% figure('position', figpos, 'color', 'w');
% fighd = [fighd gcf];
% 
% hold on;
% plot(t_est, P_est_err(:, 1), 'r', 'linewidth', 2);
% plot(t_est, P_est_err(:, 2), 'g', 'linewidth', 2);
% plot(t_est, P_est_err(:, 3), 'b', 'linewidth', 2);
% xlabel('Time [s]');
% ylabel('Error [m]');
% grid on;
% set(gca, 'fontsize', 13);
% xlim([0 t_max]);
% 
% lg_hd = legend('Px error', 'Py error', 'Pz error');
% 
% tightfig(gcf);
% saveas(gcf, [exp_path exp_name '_xyz_h_err_t.fig']);
% img = getframe(gcf);
% imwrite(img.cdata, [exp_path exp_name '_xyz_h_err_t.png']);

warning('on', 'all');
end