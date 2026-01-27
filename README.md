# HI

Here is a distance jump solver in Minecraft 1.8.9. (inspired by Cantnavet's [BMSolver](https://github.com/cantnavet/BmSolver))

The program takes in mm(normal/backwalled) and mm airtime(constant).

1. Outputs the velocity and strategy type for optimal delayed and nondelayed strat.
2. Find jumps within offset threshold (picking the best out of delayed and nondelayed strat).
3. Logs stratfind decision making.

***Feature**: You could set speed/slowness effect via `ZSolver::setEffect(int speed, int slowness)`*

**(Considers inertia)**

## Examples

### 0.1875bm loop (beats Benja's "optimal loop")

```
Optimal Solver ----------------------- 
Target mm: 0.1875, airtime: 12

- Delayed section: 
delayTick = 1
BW speed lowerBound: -0.269265
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
For mm = 0.1875 (airtime = 12), t <= 25, threshold = 0.01, offset:0.6
- NonDelayedSpeed: 0.185487, Type: Pendulum
- DelayedSpeed(dt=1): 0.290802, Type: Pendulum
t = 2: 1.375 + 0.001979 b
t = 5: 2.25 + 0.000667 b
t = 16: 5.4375 + 0.004807 b
t = 24: 7.75 + 0.007449 b
```

### Slowness I 1.5bm 6-1 to ladder (perfect double 45.01)

```
Set Speed: 0, Slowness: 1

Optimal Solver ----------------------- 
Target mm: 1.5, airtime: 12

- Delayed section: 
delayTick = 1
BW speed lowerBound: -0.303187
Required BW speed: -0.192072

- Nondelayed section: 
Max BW speed: -0.319588
Required BW speed: -0.127684

-------------------------------------------
For mm = 1.5 (airtime = 12), t <= 25, threshold = 0.01, offset:0.3
- NonDelayedSpeed: 0.237242, Type: Slingshot
- DelayedSpeed(dt=1): 0.319588, Type: Slingshot
(Nondelayed is better than Delayed at t = 4)
t = 10: 3.5 + 0.009758 b
t = 15: 5 + 0.000000137 b
```

### A weird true penta(6-2) featured in my [unlisted video](https://youtu.be/uTz3sbWMWuI)

```
Optimal Backwalled Solver ----------------------- 
Target mm: 9.875, airtime: 7

- Delayed section: 
delayTick = 1
Fit at most s45(0) r(sj45(7), 4)
pessi speed: 0.439366
run speed: 0.439368

- Nondelayed section: 
Fit at most s45(1) r(sj45(7), 4)
Inertia triggered during pessi backwall solve.
pessi speed: 0.340315
a7run speed: 0.340377
run speed: 0.340366

-------------------------------------------
For backwalled mm = 9.875 (airtime = 7), t <= 25, threshold = 0.001, offset:0.6
- NonDelayedSpeed: 0.340377, Type: A7Run
- DelayedSpeed(dt=1): 0.439368, Type: Run
(Nondelayed is better than Delayed at t = 3)
t = 15: 6 + 0.000305 b
t = 17: 6.625 + 0.00072 b
```

### TODOs:
1. input bruteforcer (WIP)
2. GUI
3. ice/slime support
4. slime bounce jump finding?
5. 45bwmm finder (stratfind with limited turns on mm)
