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

## Inputfinder(New): Finding inputs that matches the speed and fits in the mm

### Slowness I 1.5bm 6-1 to ladder, bwmm strat for facing = 0.015 

**(The full search with settings below took ~9500 micro-seconds on average on my M1 Mac)**

```
Input Finder: 
TargetVz: (-0.1276846279184921, -0.1276844242999637), Interval Width: 2.03619e-07, MM: -1.5, Airtime: 12, AllowStrafe: 1

LOG ----------------------- 

Settings: 
maxDepth = 2, maxTicks = 50
(speed, slow) = (0, 1)
Facing = 0.015 deg 
-------------------------------------------------
Try searching depth = 1 inputs
-------------------------------------------------
Try searching depth = 2 inputs

Found Seqeunce: sa.wa(2) sta(3) wa.sd(3) w.sd(5) 
t = 13(+4), Vz: -0.1276845089733

Found Seqeunce: s.wa(1) sj.wa(3) sta(9) st(2) stj(5) wa.s(7) wj.s(10) sta(2) st(1) 
t = 40(+0), Vz: -0.1276845156104653

Found Seqeunce: sa.wa(3) sj.wa(3) sta(9) st(2) stj(5) wa.s(7) wj.s(10) sta(2) st(1) 
t = 42(+9), Vz: -0.1276846239996365

Found Seqeunce: sa.w(1) s.w(7) st(1) stj(4) wa.s(8) wj.s(10) sta(2) st(1) 
t = 34(+11), Vz: -0.1276844736885106
```

### TODOs:
1. GUI
2. 45bwmm finder (stratfind with limited turns on mm)
3. Full distance jump finder
