when examing the problem of comptuing PID I first start with the Ziegler–Nichols 
but after realizing that I had to find the period of oscillation I decided that I can tune by hand
I know what each term does Kp is responsive to the actual error, kd is how fast the error is changing, and ki is the accumlation of err 
from vizualizing the intial PID parameters set {.5,.5,.5} I noticed that car isnt being responsive enough to the wave path so I elected to start by 
increasing P until the robot overshot the path initially, then use the kd term to make sure I am not overshooting the goal there is no steady state error
so change k_i was not necessary. another issue what the car pose and path alway started in different direction so it took until the end of the first oscillation until the car followed the path consistently so I choose to focuse on the wavy path after the first cycle.

qu2.3
the performance was the same. The reason being is the Ki is responsible for steady state error becuase we are using pure pursuit there isnt a consistent
steady state error and therefor no impact by the Ki term

qu4

when tuning mpc from an inuitive standpoint it makes sense to just increase all the paramaters, increasing the sampling points and space should lead to better 
results; however this was not the case, I started out with just increasing the lookahead distance but that didn't work so I increase T as well, and that was when I started to get better results I ended up using 10 and 10 and 1 for K T lookahead respectively although the car did follow the obstance directly, it was close enough and within the testing parameters.