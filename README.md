# HI

This project is inspired by Cantnavet's [BMSolver](https://github.com/cantnavet/BmSolver).

The program inputs mm(also supports backwalled mm) and mm airtime.
Update: You could set speed/slowness effect via ZSolver::setEffect(int speed, int slowness)

1. Outputs the velocity and strategy type for optimal delayed and nondelayed strat.
2. Find jumps within offset threshold (picking the best out of delayed and nondelayed strat).
3. Logs stratfind decision making.

**(Considers inertia)**

## Examples

### 0.1875bm loop (beats Benja's "optimal loop")

```
Optimal Solver ----------------------- 
Target mm: 0.187500, airtime: 12

- Delayed section: 
Estimates BW speed lowerBound: -0.269265
Required BW speed: -0.573786
Inertia triggered at t = 2 during delayed pendulum simulation.
Vz on inertia tick: pos
Avoid inertia on upperbound, and slow down afterward 

- Nondelayed section: 
Max BW speed: -0.290802
Required BW speed: -0.520678
Inertia triggered at t = 1 during nondelayed pendulum simulation.
Vz on inertia tick: neg
Hit inertia on lowerbound, and slow down afterward 

-------------------------------------------
For mm = 0.187500 (airtime = 12), t <= 25, threshold = 0.010000
- NonDelayedSpeed: 0.185487, Type: Pendulum
- DelayedSpeed: 0.290802, Type: Pendulum
t = 2: 1.375000 + 0.001979 b
t = 5: 2.250000 + 0.000667 b
t = 16: 5.437500 + 0.004807 b
t = 24: 7.750000 + 0.007449 b
```

### 3bcmm 1.25bm (A casual e-5 double chilling here)

```
-------------------------------------------
For mm = 1.250000 (airtime = 11), t <= 25, threshold = 0.001000
- NonDelayedSpeed: 0.232769, Type: Slingshot
- DelayedSpeed: 0.334247, Type: Slingshot
(Nondelayed is better than Delayed at t = 8)
t = 9: 3.562500 + 0.000980 b
t = 15: 5.375000 + 0.000034 b
t = 19: 6.562500 + 0.000391 b
```

### A weird true penta(6-2) featured in my [unlisted video](https://youtu.be/uTz3sbWMWuI)

```
Optimal Backwalled Solver ----------------------- 
Target mm: 9.875000, airtime: 7

- Delayed section: 
pessi speed: 0.439366
run speed: 0.439368

- Nondelayed section: 
Inertia triggered during pessi backwall solve.
pessi speed: 0.340315
a7run speed: 0.340377
run speed: 0.340366

-------------------------------------------
For backwalled mm = 9.875000 (airtime = 7), t <= 25, threshold = 0.010000
- NonDelayedSpeed: 0.340377, Type: A7Run
- DelayedSpeed: 0.439368, Type: Run
(Nondelayed is better than Delayed at t = 3)
t = 4: 2.312500 + 0.005133 b
t = 7: 3.375000 + 0.002011 b
t = 15: 6.000000 + 0.000305 b
t = 16: 6.312500 + 0.001635 b
t = 17: 6.625000 + 0.000720 b
t = 24: 8.750000 + 0.008839 b
```

### TODO: GUI, ice/slime support, slime bounce jump finding?

### Future Plan: 45bwmm finder, noturn (on mm) finder, input bruteforcer ... 
