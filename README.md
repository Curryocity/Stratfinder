# HI

Inspired by Cantnavet's [BMSolver](https://github.com/cantnavet/BmSolver/tree/main)

Currently, the program inputs mm and mm airtime, output the velocity for optimal delayed and nondelayed strat (Imagine the inertia threshold = 0 )

Example output:
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

WORK IN PROCESS
