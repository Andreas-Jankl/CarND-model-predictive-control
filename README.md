# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

Implement a model predictive controller in C++ that can navigate a car safely around the track of the simulator given.

---

## Review

* **Student describes their model in detail. This includes the state, actuators and update equations.**

    * These are the update equations for the used kinematic model being used
        * x​t+1​​=x​t​​+v​t​​∗cos(ψ​t​​)∗dt
        * y​t+1​​=y​t​​+v​t​​∗sin(ψ​t​​)∗dt
        * ψ​t+1​​=ψ​t​​+​L​f​​​​v​t​​​​∗δ∗dt
        * v​t+1​​=v​t​​+a​t​​∗dt
    -----------------------------
    * [x,y,ψ,v] is the state of the vehicle, L​f​​ is a physical characteristic of the vehicle, and [δ,a] are the actuators, or control inputs, to our system.
    * Additionally the cross track error and the orientation error are are also considered t obe states
    * This means there are 6 states and 2 actuators
    * In addition the factor dt which is the timestep and Lf which is defined as the distance between the vehicle's front and its center of gravity. Lf, together with the current velocity v_t, determine the current turning radius of the vehicle.
    -----------------------------
    * These are the update eqations for the error
        * cte​t+1​​=f(x​t​​)−y​t​​+(v​t​​∗sin(eψ​t​​)∗dt)
        * eψ​t+1​​=ψ​t​​−ψdes​t​​+(​L​f​​​​v​t​​​​∗δ​t​​∗dt)


* **Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried..**

    * I started out simply trying some parameters. Meaning I altered one parameter at the time
    * I did start with the values from the MPC quizzes
    * I tried N [5 - 40] / dt [0.05 - 0.2]
    * If I take a large elapsed timespan and long timesteps, the model will be quite unresponsive.
            * This means until a certain point it will get through the curve but I will several times touch the curb
            * The reason is that it will generally use lower steering angles since its looking much further ahead
            * The big advantage is though that there will be no oscilliations
            * At one point though it won't make it throug the curve anymore
    * If I take a small elapsed timespan and small timesteps, the model will be very unresponsive.
            * It will be steering fast and will correct cross track errors very fast
            * At one point though it will simply start to oscilliate heavily or even worse it will simply steer itself of the track
    * Finally I chose a dt = 0.1 and N = 10. This turned out to be perfect cause I correct steering every 0.1s and I always look ahead 1sec
    

* **Is a polynomial fitted to waypoints?**

    * Yes I fitted a polynomial of 3rd order to the reference points:
        * y = a*x^3 + b*x^2 + c*x + d
    * In code this can be found here:
        * auto coeffs = polyfit(ptsxv, ptsyv, 3);
        

* **If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.**

    * Yes I did preprocess the waypoints. I did convert them from global state to the vehicle state
    * Its important to note that in vehicle space the vehicle is at position 0,0 and orientation 0 means the first 3 states of the state vector are 0
    * Below is the code where this gets done
```sh
          // convert the reference points from global to vehicle space
          for (int i = 0; i < ptsxv.size(); i++) {
            double x = ptsxv[i] - px;
            double y = ptsyv[i] - py;
            ptsxv[i] = x * cos(psi) + y * sin(psi);
            ptsyv[i] = - x * sin(psi) + y * cos(psi);
          }
```


* **The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.** 

    * This is being handled by not just taking the solution at timestep = 0 which won't be taking into action before timestep 0.1
    * I therefore did go ahead and not just used the solution at timestep = 0 but also timstep = 0.07 and timestep = 0.14. Then I averaged it out.
	* This should lead to a better set steering and acceleration since I look further ahead and try to set the values so that even if there is a latency I make sure that the vehicle gets back to the center
	* I think an even better method might have been to not just take the state at timestep 0 but to actually predict the state in 0.1sec and then run the mpc solve algorithm with the state at 0.1sec as initial state
	* In code one finds my implementation here:
```sh
	  //Take care of Latency
	  //This is being done by calculating the mean of the next 3 steps.
	  //The reason I did this is because its (3-1)*0.07 = 0.14 sec that we look ahead and since we have 100ms latence one looks to take a value in the future not at timestept = 0
	  int n_mean = 3;
	  for (int i = 0; i < n_mean; i++)
	  {
		delta += solution.x[delta_start + i] / n_mean;
		a_acc += solution.x[a_start + i] / n_mean;
	  }
```	