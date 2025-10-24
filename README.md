# Team Carltron: Future Engineers 2025
This is the Future Engineers project repository by team Carltron for the 2025 World Robot Olympiad International Final.

### Our Journey so far...
The 2025 season is actually our first season both in Future Engineers and WRO in general. After participating in multiple robotics competitions over the past few years, we were looking for a new challenge, when we found out about the Future Engineers Category in december 2024. Due to exams and other obligations though, we were only able to start working on our robot in April 2025, one month before our regional competition in Aachen, Germany. This challenged us to make our design strategies more efficient and also streamline our project management. After getting a working robot ready just in time, our first big moment came on the 17th of May at our regionals, where we managed to secure the qualification for the german finals by taking 1st place. 

Our robot had jumped over the first hurdle, which was a big relieve and motivated us to go all in on the national competition. But not everything was well and good. Our first robot (V1) had also shown some flaws during its live competition runs, so we already had a long list of ideas on how to improve it and diminish or remove these issues. 


# Design Strategy
For the general design of both our hardware and software systems, we always try to simplify all solutions to their base requirements. For example this means solving issues on the lowest level possible (e.g. hardware > software). 

# Motorization
For the choice of our drive system, the central factors were power and compactness. At first we considered implementing an all-wheel drive with two steering axles for maximum maneuverability. But our research showed that our robot would have to be significantly larger than planned for that solution.

From the outset we set a guiding principle for the robot’s development: every aspect of the design must aim for the simplest solution with the lowest susceptibility to failure. Accordingly, we performed calculations and experiments on turning circles and performance for different drive options to determine what the simplest overall system would look like that still fully meets our requirements:

It must reliably achieve a usable speed range of 0.12 to 1.4 m/s.

It must have sufficient traction to transfer motor power to the ground.

It must have a maximum turning radius of 0.55 m.

The steering must be able to transition fully from one extreme deflection to the other within 0.3 s.

In the comparison we found that a conventional approach—with the rear axle as the drive axle and the front axle used purely for steering—offered the best compromise between compactness and performance. This configuration also gave us the opportunity to develop a fully custom steering system. We then assembled the corresponding drive system to meet the requirements.

To ensure high agility despite using front-wheel steering only, a differential on the rear axle is very helpful because it allows the driving wheels to move independently. This lets the inner wheel follow a smaller turning radius without unnecessary slip and without the wheels losing traction. At the regional competition we used an off-the-shelf rear axle with a differential. However, its precision and reliability did not meet our requirements. In addition, using this and other standard components in the drivetrain forced us to design the rest of the robot around those parts rather than integrating them well, since they were not developed specifically for our use case.

To keep the robot’s form factor small and maintain full flexibility in the overall design, we decided to develop our own differential gearbox that would fit perfectly into the rest of the robot’s layout. We tested various bearings, gears, and motor couplings for this. Ultimately we developed the differential gearbox shown in cross section in Fig. 1, which—like most of the robot’s other components—was designed from the ground up by us in the 3D CAD software Onshape.
