# Hybrid Electric Vehicle MATLAB Simulation

A rule-based parallel HEV simulation that compares fuel use of a conventional gasoline vehicle and a hybrid vehicle over a 600-second custom driving cycle.

## Features

- Longitudinal vehicle dynamics: rolling resistance, aerodynamic drag, and acceleration force
- Conventional-vehicle fuel calculation using an engine efficiency map
- Rule-based engine/motor power split
- Battery state-of-charge (SOC) tracking
- Regenerative braking energy recovery
- Fuel-economy and power-flow plots

## Requirements

MATLAB R2016b or later is recommended. No toolboxes are required.

## Run the model

1. Open MATLAB.
2. Set the Current Folder to this repository.
3. Run `hev_simulation.m`.

The command window prints a summary and MATLAB opens four figures.

## Energy-management strategy

| Vehicle power demand | Engine command | Motor role |
|---|---:|---|
| Negative | Off | Generator; braking energy recovery |
| 0 to <5 kW | Off | Electric traction |
| 5 to <10 kW | 10 kW | Balances wheel demand / charges battery if surplus |
| 10 to <15 kW | 15 kW | Balances wheel demand |
| >=15 kW | Up to 35 kW | Assists if needed |

## Model notes

This is an educational, simplified model. It does not include road grade, transmission losses, battery internal resistance, auxiliaries, or a full charge-sustaining controller. Final SOC should be matched to initial SOC for a rigorous fuel-economy comparison.

## Expected baseline results

With the included parameters and one-second time step, expected values are approximately:

- Distance: 7.9167 km
- Conventional fuel: 0.3812 L
- HEV fuel: 0.2591 L
- Fuel saving: 32.04%
- Final battery SOC: 78.95%

## Project structure

```
hev_simulation.m                 Main MATLAB simulation
outputs/hev_project_study_guide.md  Project study notes
README.md                        Repository documentation
```

## License

This repository is released under the MIT License. See `LICENSE`.
