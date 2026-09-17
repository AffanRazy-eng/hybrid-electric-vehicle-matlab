%% Hybrid Electric Vehicle (HEV) Rule-Based Simulation
% A simplified parallel HEV model with vehicle dynamics, engine fuel use,
% motor assistance, battery SOC tracking, and regenerative braking.

clear; clc; close all;

%% Simulation settings
dt = 1;                         % Time step (s)
t = 0:dt:600;                  % Simulation time (s)

%% Vehicle parameters
m = 1500;                      % Vehicle mass (kg)
g = 9.81;                      % Gravity (m/s^2)
Cd = 0.29;                     % Drag coefficient
A = 2.2;                       % Frontal area (m^2)
rho = 1.225;                   % Air density (kg/m^3)
Cr = 0.012;                    % Rolling-resistance coefficient

%% Powertrain parameters
motor_max_power_kW = 50;
motor_efficiency = 0.90;
regen_efficiency = 0.70;
engine_max_power_kW = 35;
gasoline_energy_density_J_per_L = 34.2e6;

%% Battery parameters
battery_capacity_kWh = 10;
SOC_initial = 0.80;
SOC_min_limit = 0.70;
SOC_max_limit = 0.90;

%% Engine efficiency map
engine_power_map_kW = [0 5 10 15 20 25 30 35];
engine_efficiency_map = [0 0.20 0.25 0.30 0.32 0.31 0.29 0.27];

%% Driving cycle
speed_kmh = zeros(size(t));
speed_kmh(t <= 100) = 50 * t(t <= 100) / 100;
idx = t > 100 & t <= 250; speed_kmh(idx) = 50;
idx = t > 250 & t <= 350; speed_kmh(idx) = 50 + 30 * (t(idx) - 250) / 100;
idx = t > 350 & t <= 450; speed_kmh(idx) = 80;
idx = t > 450 & t <= 550; speed_kmh(idx) = 80 * (550 - t(idx)) / 100;
% Remaining samples stay at zero.
v = speed_kmh / 3.6;           % Speed (m/s)

%% Vehicle dynamics and required wheel power
acceleration = [0 diff(v) / dt];
F_rolling = m * g * Cr;
F_aero = 0.5 * rho * Cd * A .* v.^2;
F_acceleration = m .* acceleration;
F_total = F_rolling + F_aero + F_acceleration;
P_vehicle_kW = F_total .* v / 1000;

%% Conventional vehicle fuel consumption
P_conventional_kW = min(max(P_vehicle_kW, 0), engine_max_power_kW);
fuel_conventional_L = 0;

for i = 1:length(t)
    if P_conventional_kW(i) > 0
        eff = interp1(engine_power_map_kW, engine_efficiency_map, ...
            P_conventional_kW(i), 'linear');
        eff = max(0.05, min(eff, 0.35));
        fuel_conventional_L = fuel_conventional_L + ...
            (P_conventional_kW(i) * 1000 / eff) * dt / ...
            gasoline_energy_density_J_per_L;
    end
end

%% HEV energy-management simulation
P_engine_kW = zeros(size(t));
P_motor_kW = zeros(size(t));
SOC_HEV = zeros(size(t));
engine_efficiency_HEV = zeros(size(t));
SOC_HEV(1) = SOC_initial;
fuel_HEV_L = 0;
regen_stored_kWh = 0;
braking_mechanical_kWh = 0;

for i = 1:length(t)
    demand_kW = P_vehicle_kW(i);

    % Rule-based EMS: determine engine/motor split.
    if demand_kW < 0
        P_engine_kW(i) = 0;
        P_motor_kW(i) = demand_kW;       % Motor as generator during braking
        braking_mechanical_kWh = braking_mechanical_kWh + ...
            abs(P_motor_kW(i)) * dt / 3600;
        regen_stored_kWh = regen_stored_kWh + ...
            abs(P_motor_kW(i)) * regen_efficiency * dt / 3600;
    elseif demand_kW < 5
        P_engine_kW(i) = 0;
        P_motor_kW(i) = demand_kW;       % Electric-only low-load driving
    elseif demand_kW < 10
        P_engine_kW(i) = 10;
        P_motor_kW(i) = demand_kW - P_engine_kW(i);
    elseif demand_kW < 15
        P_engine_kW(i) = 15;
        P_motor_kW(i) = demand_kW - P_engine_kW(i);
    else
        P_engine_kW(i) = min(demand_kW, engine_max_power_kW);
        P_motor_kW(i) = demand_kW - P_engine_kW(i);
    end

    % Respect motor rated power. This cycle does not reach the limit.
    P_motor_kW(i) = max(-motor_max_power_kW, ...
        min(P_motor_kW(i), motor_max_power_kW));

    % Engine fuel use at this time step.
    if P_engine_kW(i) > 0
        engine_efficiency_HEV(i) = interp1(engine_power_map_kW, ...
            engine_efficiency_map, P_engine_kW(i), 'linear');
        engine_efficiency_HEV(i) = max(0.05, ...
            min(engine_efficiency_HEV(i), 0.35));
        fuel_HEV_L = fuel_HEV_L + ...
            (P_engine_kW(i) * 1000 / engine_efficiency_HEV(i)) * dt / ...
            gasoline_energy_density_J_per_L;
    end

    % Motor power to battery power: positive means battery discharge.
    if P_motor_kW(i) > 0
        battery_power_kW = P_motor_kW(i) / motor_efficiency;
    elseif P_motor_kW(i) < 0
        battery_power_kW = P_motor_kW(i) * regen_efficiency;
    else
        battery_power_kW = 0;
    end

    % SOC update and protection limits.
    SOC_change = battery_power_kW * dt / (battery_capacity_kWh * 3600);
    SOC_HEV(i) = max(SOC_min_limit, min(SOC_HEV(i) - SOC_change, SOC_max_limit));
    if i < length(t)
        SOC_HEV(i + 1) = SOC_HEV(i);
    end
end

%% Results
distance_km = trapz(t, v) / 1000;
positive_traction_energy_kWh = sum(max(P_vehicle_kW, 0)) * dt / 3600;
fuel_economy_conventional = fuel_conventional_L / distance_km * 100;
fuel_economy_HEV = fuel_HEV_L / distance_km * 100;
fuel_saved_L = fuel_conventional_L - fuel_HEV_L;
fuel_saving_percent = (1 - fuel_HEV_L / fuel_conventional_L) * 100;
battery_net_energy_kWh = (SOC_initial - SOC_HEV(end)) * battery_capacity_kWh;
engine_running = P_engine_kW > 0;

fprintf('\nHEV MATLAB SIMULATION RESULTS\n');
fprintf('--------------------------------\n');
fprintf('Distance: %.4f km\n', distance_km);
fprintf('Conventional fuel: %.4f L (%.4f L/100 km)\n', ...
    fuel_conventional_L, fuel_economy_conventional);
fprintf('HEV fuel: %.4f L (%.4f L/100 km)\n', fuel_HEV_L, fuel_economy_HEV);
fprintf('Fuel saving: %.4f L (%.2f%%)\n', fuel_saved_L, fuel_saving_percent);
fprintf('Initial / final SOC: %.2f%% / %.2f%%\n', ...
    SOC_initial * 100, SOC_HEV(end) * 100);
fprintf('Battery net energy used: %.4f kWh\n', battery_net_energy_kWh);
fprintf('Braking mechanical energy: %.4f kWh\n', braking_mechanical_kWh);
fprintf('Energy stored from braking: %.4f kWh\n', regen_stored_kWh);
fprintf('Engine operating time: %d s (%.2f%%)\n', ...
    sum(engine_running), mean(engine_running) * 100);
fprintf('Average engine efficiency while running: %.2f%%\n', ...
    mean(engine_efficiency_HEV(engine_running)) * 100);

%% Plots
figure('Name', 'HEV Driving Cycle');
plot(t, speed_kmh, 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Speed (km/h)'); title('Driving Cycle');

figure('Name', 'HEV Power Flow');
plot(t, P_vehicle_kW, 'LineWidth', 1.5); hold on;
plot(t, P_engine_kW, 'LineWidth', 1.5);
plot(t, P_motor_kW, 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Power (kW)'); title('HEV Power Flow');
legend('Vehicle demand', 'Engine', 'Motor', 'Location', 'best'); hold off;

figure('Name', 'Battery SOC');
plot(t, SOC_HEV * 100, 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('SOC (%)'); title('Battery State of Charge');

figure('Name', 'Fuel Comparison');
bar([fuel_conventional_L fuel_HEV_L]); grid on;
set(gca, 'XTickLabel', {'Conventional', 'HEV'});
ylabel('Fuel consumption (L)'); title('Fuel Consumption Comparison');
