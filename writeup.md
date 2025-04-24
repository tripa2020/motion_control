I began with the Ziegler–Nichols method but found determining the oscillation period cumbersome. Instead, I tuned the gains by hand. I know what each term does Kp is responsive to the actual error, kd is how fast the error is changing, and ki is the accumlation of err. I raised the proportional gain (Kp) until the vehicle slightly overshot the wave path, then increased the derivative gain (Kd) to damp the overshoot. Since there was no steady-state error, I left the integral gain (Ki) at zero. Finally, I focused on the car’s behavior after its first cycle—despite its initial orientation mismatch—to ensure it tracked the wavy path consistently.

qu2.3
the performance was the same. The reason being is the Ki is responsible for steady state error becuase we are using pure pursuit there isnt a consistent
steady state error and therefor no impact by the Ki term

qu4
when tuning mpc from an inuitive standpoint it makes sense to just increase all the paramaters, increasing the sampling points and space should lead to better 
results; however this was not the case, I started out with just increasing the lookahead distance but that didn't work so I increase T as well, and that was when I started to get better results I ended up using 10 and 10 and 1 for K T lookahead respectively although the car did follow the obstance directly, it was close enough and within the testing parameters.