# To do list for hardware testing of arduino quadcopter

* [ ] You should also be able to fly the drone at this point. So it would be good to go ahead and fly the drone

* [ ] Plot data from this flight test

* [ ] Using a scale and the pass through Arduino routine get the thrust of the motor as a function of microsend pulse and add that to the aerodynamics routine for the quadcopter. While you're doing this experiment figure out how long it takes to go from full throttle on your control stick to full throttle on the drone. That is the amount of delay we need to introduce into our actuators.

* [ ] Wooden Drone waypoint flights - Can focus on outerloop control laws (Make sure to update blog for this one)
    * [ ] Attempt to code altitude hold for drone using deltaP and Az
    * [ ] Station keeping
    * [ ] Point to a desired heading
    * [ ] Waypoint controller
