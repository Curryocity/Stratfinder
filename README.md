# HI

Inspired by Cantnavet's [BMSolver](https://github.com/cantnavet/BmSolver)

The program inputs mm and mm airtime, output the velocity for optimal delayed and nondelayed strat.

**(Considers inertia)**

## Example outputs

### 0.125bm

```
LOG ----------------------- 
- Delayed section: 
Estimates BW speed lowerBound: -0.257038
Required BW speed: -0.590749
Inertia triggered at t = 2 during delayed pendulum simulation.
Vz on inertia tick: neg
Hit inertia on lowerbound, and slow down afterward 

- Nondelayed section: 
Max BW speed: -0.288891
Required BW speed: -0.538192

-------------------------------------------
For mm = 0.125, mm airtime = 12
Optimal nonDelayedSpeed: 0.182381
Strat Type: Pendulum
Optimal delayedSpeed: 0.288891
Strat Type: Pendulum
```

### 3bm

```
LOG ----------------------- 
- Delayed section: 
Estimates BW speed lowerBound: -0.32824
Required BW speed: -0.796698
Required FW airspeed: 0.127565
Required BW speed for boomerang: 0.198887

- Nondelayed section: 
Max BW speed: -0.385537
Required BW speed: -0.727874
Required FW airspeed: 0.180741
Required BW speed for boomerang: -0.977608

-------------------------------------------
For mm = 3, mm airtime = 12
Optimal nonDelayedSpeed: 0.286919
Strat Type: Pendulum
Optimal delayedSpeed: 0.385537
Strat Type: Boomerang
```

### STATUS: STILL WORK IN PROCESS
