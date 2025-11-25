# Team Carltron - Future Engineers 2025
![nice_robot.JPG](other%2Fnice_robot.JPG)
<center>This is the Future Engineers project repository by team Carltron for the 2025 World Robot Olympiad International Final.
</center>


### Our Journey so far...
The 2025 season is actually our first season both in Future Engineers and WRO in general. After participating in multiple robotics competitions over the past few years, we were looking for a new challenge, when we found out about the Future Engineers Category in december 2024. Due to exams and other obligations though, we were only able to start working on our robot in April 2025, one month before our regional competition in Aachen, Germany. This challenged us to make our design strategies more efficient and also streamline our project management. After getting a working robot ready just in time, our first big moment came on the 17th of May at our regionals, where we managed to secure the qualification for the german finals by taking 1st place.
Our robot had jumped over the first hurdle, which was a big relieve and motivated us to go all in on the national competition. But not everything was well and good. Our first robot (V1) had also shown some flaws during its live competition runs, so we already had a long list of ideas on how to improve it and diminish or remove these issues. 


## Design Strategy
For the general design of both our hardware and software systems, we always try to simplify all solutions to their base requirements. For example this means solving issues on the lowest level possible (e.g. hardware > software). 

# Mobility Management
## Motorization
For the choice of our drive system, the central factors were power and compactness. At first we considered implementing an all-wheel drive with two steering axles for maximum maneuverability. But our research showed that our robot would have to be significantly larger than planned for that solution.

From the outset we set a guiding principle for the robot’s development: every aspect of the design must aim for the simplest solution with the lowest susceptibility to failure. Accordingly, we performed calculations and experiments on turning circles and performance for different drive options to determine what the simplest overall system would look like that still fully meets our requirements:

1. It must reliably achieve a usable speed range of 0.12 to 1.4 m/s.

2. It must have sufficient traction to transfer motor power to the ground.

3. It must have a maximum turning radius of 0.55 m.

4. The steering must be able to transition fully from one extreme deflection to the other within 0.3 s.

In the comparison we found that a conventional approach—with the rear axle as the drive axle and the front axle used purely for steering—offered the best compromise between compactness and performance. This configuration also gave us the opportunity to develop a fully custom steering system. We then assembled the corresponding drive system to meet the requirements.

To ensure high agility despite using front-wheel steering only, a differential on the rear axle is very helpful because it allows the driving wheels to move independently. This lets the inner wheel follow a smaller turning radius without unnecessary slip and without the wheels losing traction. At the regional competition we used an off-the-shelf rear axle with a differential. However, its precision and reliability did not meet our requirements. In addition, using this and other standard components in the drivetrain forced us to design the rest of the robot around those parts rather than integrating them well, since they were not developed specifically for our use case.

To keep the robot’s form factor small and maintain full flexibility in the overall design, we decided to develop our own differential gearbox that would fit perfectly into the rest of the robot’s layout. We tested various bearings, gears, and motor couplings for this. Ultimately we developed the differential gearbox shown in cross section in Fig. 1, which—like most of the robot’s other components—was designed from the ground up by us in the 3D CAD software Onshape.


# Power and Sense Management

## Sensors

The selection of sensors is obviously one of the most defining characteristics in the concept of any Future 
Engineers robot. So let's start with a first principles approach to figure out, what kind of sensors are required to 
achieve the strategy we intended to implement.

Since our goal was to design a robot that could reliably localise itself on the playing field in 
any place or situation, we found that we would need sensors that could give us a reading of the robots 
physical surroundings in real time, which could be used for multiple purposes such as obstacle detection, 
localisation and collision avoidance. By taking a closer look at the actual playing field were working with in the 
Future Engineers category, we can see that all walls, barriers and obstacles are on a single flat level, or in other 
words a single plane. This is because all of these relevant objects on the playing field have the same height of 100 mm.

If we can get a planar 360° reading of our physical surroundings we can from any point of the playing field see so- 
called "characteristic spots". Examples of such spots would be the corners of the outer and inner walls, the 
barriers of the parking lot or the placement of specific obstacles. As long as our planar reading is precise enough 
and has an adequate resolution, we, theoretically, can do all localisation, navigation and path planning based on 
this single 360° sensor reading.

Well, the great thing is, because we are not the first humans tackling challenges of autonomous navigation, there 
already are quite mature sensor technologies that can give us the exact 360° physical reading that we need: They're 
called 360° Lidar sensors. 

Lidar (Light Detection and Ranging) is a Laser-based distance measuring technology that works by sending out light 
beams and measuring the time their reflections from any objects take to return. Since lightspeed is constant, this 
technology can give pretty accurate distance readings, whilst having a much more targeted measuring window (e.g. 
compared to ultrasound distance sensors). 

But what makes these sensors give 360° readings of their surroundings if they have such a targeted range? It's 
kind of simple but also genius: You simply spin the sensor really fast (multiple times per second). By doing that 
whilst taking multiple thousand readings every revolution of the sensor, you get a full 360° view with distance 
measurements in all directions.

>![LIDAR_animation.gif](other%2FLIDAR_animation.gif)
>Animation of the concept of a 360° Lidar Sensor

After finding the right type of sensor for our application, the next step is selecting a specific model that 
perfectly suits the requirements set by our specific challenge.

Since our robot will have to use the sensor readings while driving, we need the sensor to provide us with readings 
regularly. Our testing showed that a rate of 10 hz worked to an acceptable extent while best results could be 
achieved using sensors running at 15 hz or more. 

Concerning the resolution of the sensor, we need it to detect the obstacles that only have a 5 cm width from the 
robots' perspective, at a range of up to 2 m.

We also need a sensor performance, that can reliably detect the black walls of the game field both at ranges of up 
to 4 m and shallow angles, which could arise when driving close to a wall.

Also, importantly, we need it to have a formfactor, that lets us easily integrate it into our robot and that fits 
under the 10 cm height, so we can actually get the sensor reading in the plane of the game field.

We created this decision matrix to help us find the ideal 360° Lidar sensor:

| Criterion                                      | **RPLIDAR S3**                     | **RPLIDAR S2**                      | **D900 / LD19 Plus**              | **RPLIDAR A2**                    |
|-----------------------------------------------|------------------------------------|-------------------------------------|-----------------------------------|-----------------------------------|
| **Update rate ≥10 Hz**                        | 🟢 10–20 Hz adjustable             | 🟢 10 Hz fixed                      | 🟡 6–13 Hz range                  | 🟢 Up to 10 Hz                    |
| **Best at ≥15 Hz**                            | 🟢 Supports ≥15 Hz                 | 🟡 Limited to ~10 Hz                | 🟡 Upper end ~13 Hz               | 🟡 10 Hz max typical             |
| **5 cm @ 2 m resol.**                         | 🟢 0.1125° ultra fine              | 🟢 0.1125° ultra fine               | 🟢 ~0.7–0.8° sufficient           | 🟢 Fine enough for 5 cm          |
| **Range margin beyond 4 m**                   | 🟢 40 m max range                  | 🟢 30–50 m variants                 | 🟢 12 m radius                    | 🟢 12–16 m range                 |
| **Black wall @ 4 m**                          | 🟢 10% refl. to 15 m               | 🟢 10% refl. to 10–15 m             | 🟡 Spec at 70% refl.              | 🟡 Dark targets less specified    |
| **Shallow-angle dark surfaces**               | 🟢 Optimized low-reflectivity dtof | 🟢 Good low-reflectivity handling   | 🟡 DTOF, less data published      | 🟡 Triangulation, more sensitive |
| **Height < 10 cm**                            | 🟢 ~41 mm total height             | 🟢 ~38.9 mm height                  | 🟢 ~33.5 mm height                | 🟢 ~41 mm height                 |
| **Compact footprint for small robot**         | 🟢 55×56 mm compact body           | 🟡 77×77 mm footprint               | 🟢 ~38×38 mm footprint            | 🟡 Larger A-series footprint     |
| **Eye safety, outdoor light**                 | 🟢 Class 1, 80 kLux resistant      | 🟢 Class 1, IP65 rated              | 🟢 Class 1, 30–60 kLux resistant  | 🟡 Indoor-oriented, less robust  |
| **Integration ecosystem, ROS support**        | 🟢 Strong ecosystem, drivers       | 🟢 Same Slamtec ecosystem           | 🟡 Good, but more fragmented      | 🟢 Widely used, many examples    |
| **Cost level**                                | 🟡 Higher, premium Slamtec         | 🟡 Mid-high price tier              | 🟢 Low-mid price range            | 🟢 Affordable A-series choice    |
| **Form factor for 10 cm plane**               | 🟢 Ideal low profile scanner       | 🟢 Low optical window height        | 🟢 Very low, cube-like            | 🟢 Fits under 10 cm              |
| **Overall match to requirements**             | 🟢 Best fit for challenge          | 🟡 Very strong alternative          | 🟡 Good budget compromise         | 🟡 Usable, but less optimized    |

So the choice fell to the RPlidar S3 Sensor. Especially its ability to provide us with adequate resolution at a 15 
hz rotation rate due to its high sample rate, made the best choice clear to be this one. 

> ![lidar_mount.jpg](other%2Flidar_mount.jpg)
> RPlidar S3 Sensor installed on its mounting hardware

Since we want to be able to use the 360° Lidar capabilities of this sensor to its fullest abilities, we decided to 
design our robot, so that the Lidar sensor would have an unobstructed surround view. This simply means that now 
components of the robot should reach into the field of view of the Lidar. But the sensor also should still sit well 
underneath the 10 cm top height mark of the walls and obstacles. Sitting to high might cause the sensor to simply 
look "over" obstacles and walls, as soon as the lidar sensors mounting is not perfectly plane. the closer we can 
bring the Lidar sensors measurement plane to the height middle of the 10 cm game field height, the better.



Getting physical 360° planar readings already gets us quite far in perceiving precisely what kind of obstacles are 
placed where and how to avoid them. But one crucial bit of information for the Future Engineers Challenge is still 
missing and can not be determined just by the readings from our Lidar sensor: The **color** of the obstacles. 


As the primary sensor for detecting the course and obstacles, we use a 2D 360° LiDAR sensor that generates a precise point cloud of the robot’s surroundings at a rate of 15 Hz. This sensor detects all physical objects visible from its perspective in a plane parallel to the ground by continuously rotating and measuring distances with a laser in each direction. The LiDAR is positioned in the robot in such a way that it has an almost completely unobstructed 360° field of view. Since the camera is mounted above the LiDAR, the only obstruction is directly behind the LiDAR — the so-called “shark fin” of our robot — through which the camera’s data cable is safely routed. However, this narrow “shark fin” occupies only about 4° of the LiDAR’s field of view and can therefore be completely and losslessly filtered out in software. This allows obstacles and walls to be continuously tracked in all directions, improving overall reliability.


## Power Supply

### Selection of batteries

In order to determine the kind of batteries we would need to adequately power our robot, our first step was getting an overview of the power consimption of all the components we intended to use:

| Component                                     | Power in W (peak, including voltage conversion losses) |
|-----------------------------------------------|------------------------------------------------------|
| Drive Motor                                   | 14                                                   |
| Steering Motor                                | 11                                                   | 
| Raspery Pi                                    | 15                                                   |
| Lidar Sensor                                  | 4                                                    |
| Various Sensors / IO (IMU, LED, Camera, etc.) | 4                                                    |
| **Sum**                                       | **48**                                               | 

>*These wattage values were obtained by combining values from the specific data sheets of the components as well as our own measurements.*

So, our batteries would need to provide ~50 Watts at peak. Our own measurements show, that over long term running, our specific system requires an average of 22 Watts constantly.

Additionally, we want to run our drive motor directly off the batteries without a voltage conversion. This is because any conversion losses would directly impede the performance of our drive train. Minimizing the required components (another converter in this case) also follows our general design strategy to keep all system aspects as simple as possible. To run our drive motor off the batteries directly, they need to provide provide a voltage between 14 - 18V.

As additional requirements, we want our batteries to be widely available in case we need to get spares, not too expensive, safe while in use and charging and we want to use a quick connect/disconnect system, that lets us swap out batteries with ease. They should also give us enough capacity to do at least an hour of testing on one battery to save time, so the batteries need to provide a capacity of at least ~30 Wh.

So we compared different kinds of commonly used batteries for our requirements:


| Criterion         | 2× **LP-E6** series           | 4S **LiPo RC** pack           | 4S **18650 Li-ion** pack           | 12-cell **NiMH** RC pack                |
|------------------|-------------------------------|-------------------------------|------------------------------------|-----------------------------------------|
| **Voltage**      | 🟢 16 V nominal               | 🟢 14.8 V nominal             | 🟢 14.8 V nominal                  | 🟢 14.4 V nominal                       |
| **Energy**       | 🟢 ~39 Wh pack                | 🟢 ~33 Wh pack                | 🟢 ~38 Wh pack                     | 🟢 ~43 Wh pack                          |
| **Density**      | 🟢 High Li-ion density        | 🟢 High LiPo density          | 🟢 Very high density               | 🟡 Lower specific energy                |
| **Peak power**   | 🟢 3.5 A, ~56 W               | 🟢 Very high current          | 🟢 Several amps continuous         | 🟢 High discharge capability            |
| **Availability** | 🟢 Widely sold camera battery | 🟡 Hobby / RC channels only   | 🟡 Cells common, packs niche       | 🟡 Declining RC availability            |
| **Safety**       | 🟢 Protected, low risk        | 🔴 Damage, fire risk higher   | 🟡 Needs BMS protection            | 🟢 Chemically very robust               |
| **Charging**     | 🟢 Simple LP-E6 chargers      | 🟡 Requires balance charger   | 🔴 BMS-aware charger needed        | 🟡 Dedicated NiMH charger               |
| **Swap mech.**   | 🟢 Latching sled, foolproof   | 🟢 XT plug quick swap         | 🟡 Less standardized packs         | 🟡 Bulky stick swapping                 |
| **Durability**   | 🟢 Hard plastic shell         | 🟡 Soft pouch, fragile        | 🟡 Depends on pack case            | 🟢 Rugged cylindrical cells             |
| **Cost**         | 🟡 Higher cost per Wh         | 🟢 Lowest cost per Wh         | 🟡 Moderate cost per Wh            | 🟡 Moderate, heavier packs              |
| **Form factor**  | 🟢 Compact brick              | 🟡 Flat pouch, padding        | 🟡 Cylinder cluster, bulky         | 🟡 Long stick, heavy                    |
| **Overall match**| 🟢 Best system-level fit      | 🟡 Acceptable, worse usabiliy | 🟡 Acceptable, complex integration | 🟡 Not adequate, heavy and bad usablity |


Due to their safety track record, wide availability, flexibly integrable form factor and standardized mounting system, we decided to use two LP-E6 type batteries running in Series. These provide up to 6 A at approximately 8 V each. Therefore, when connected in series, these two batteries giving us a total power capacity of around 56 W at 16V, nicely fitting our power requirements with some margin. 


### Voltage Management

Our different components each require specifc voltages which we need to supply to them reliably:

| Voltage | Components                                           |
|---------|------------------------------------------------------|
| 3.3 V   | IMU, Camera, Motor Encoder, IO-LEDs, Lighting Bridge |
| 5.1 V   | Raspberry Pi, Lidar Sensor                           |
| 8.4 V   | Steering Servo, Cooling Fans                         |
| 16.0 V  | Drive Motor                                          |

So there are four discreet voltage rails that need to be supplied reliably:

Our Raspberry Pi already provides an internal 3.3 V rail, that is used both for logic IO and power supply up to 500 mA for external components via the GPIO, which is more than enough for the Sensors and IO we want to run at 3.3 V, since they require a combined maximum of ~380 mA.

Both the Lidar Sensor and the Raspberry Pi need to be supplied with 5.1 V. While the Raspberry Pi can be supplied with up to 5 A, this is only necessary when running power hungry external components on its USB Ports. Since we do not use the USB ports during operation, we can still run the Pi 5 at full power when supplying int with 3 A. Our Lidar Sensor requires a maximum of 0.8 A on startup, bringing our total required power on the 5 V rail to 3.8 A. We decided to use a 5 A Step-Down Buck Converter for the 5 V Supply that is connected as seen in our Power Rails diagram.

At 8.4 V, the cooling fans of our robot use ~250 mA each and our Steering Servo needs ~1.4 A at peak. They are supplied by a smaller 3 A Step-Down Buck Converter.

The drive Motor is run directly off the Batteries, removing the need for any additional converters. Changes in the supply voltage based on the batteries state of charge are fully mitigated by our encoder-based drivetrain controller algorithms, as will be discussed later in the documentation.

This schematic provides an overview of how the different voltage rails are connected:
![Schematic_Power_Rails_Diagram.png](schemes%2FSchematic_Power_Rails_Diagram.png)

The separate Buck Converters used for the 8.4 V and 5.1 V rails also decouple all the sensitive electronics and sensors from our drive train power systems, preventing any voltage ripple and inconsistencies caused by sudden power changes in our drivetrain and steering systems.

To operate the Raspberry Pi and the batteries under optimal conditions, we also installed an actively air-cooled heatsink on the Raspberry Pi, as well as passive cooling elements on the voltage converters. In addition, we mounted the two main cooling fans under the rear of the robot to provide constant airflow from the front wheel housings, across all electronic components and converters, and out through the bottom of the chassis. These fans fully replace the air inside the robots bodywork every 4 seconds, making sure that all components can run at peak performance without getting hot.


### Custom Mainboard

For distributing power to the various components and ensuring communication between the electronic modules and sensors, we designed our own custom mainboard and had it professionally manufactured. 

This was especially interesting to us, since we learned a lot about electrical engineering by doing all the conceptualisation, design, layout and manufacturing information on our own.

>![Schematic_Mainboard_wiring.png](schemes%2FSchematic_Mainboard_wiring.png)
>Wiring schematic of our custom Mainboard


> ![Mainboard_layout.png](schemes%2FMainboard_layout.png)
> Actual layout and routing of the PCB (Printed Circuit Board)

This custom Mainboard PCB provides a significantly more efficient and reliable solution compared to the numerous 
cables we previously used to connect the components.

| PCB Top Side, no components                     | PCB Bottom Side, no components                        |
|-------------------------------------------------|-------------------------------------------------------|
| ![pcd_top_empty.jpg](other%2Fpcd_top_empty.jpg) | ![pcb_bottom_empty.jpg](other%2Fpcb_bottom_empty.jpg) |


Since a lot of data connections had to be routed through this Mainboard without interfering with each other, we decided to make it 4-layered with components placed on both sides of the board for maximum compactness.

Both the two voltage converters, the drive motor bridge, the IMU and the Raspberry Pi are mounted directly onto the Mainboard, while cable conectors on the Mainboard facilitate the connections to the drive motor itself, the lidar sensor, the steering servo and the cooling fans.

The Mainboard also directly integrates seamlessly with our custom battery holder. On the bottom side of the Mainboard, small connection pins allow the batteries to directly connect to the Mainboard when inserted.

| PCB Top Side, assembled (with battery holdder)                    | PCB Bottom Side, assembled (with battery holdder)                           |
|-------------------------------------------------------------------|-------------------------------------------------------|
| ![pcb_top_view_assembled.jpg](other%2Fpcb_top_view_assembled.jpg) | ![pcb_bottom_view_assembled.jpg](other%2Fpcb_bottom_view_assembled.jpg) |

In order to connect the Raspberry Pi with the Mainboard, we also designed a custom adapter PCB, that directly 
connects all GPIO pins of the Pi.

>TODO: Picture of bridge

For all components that were directly mounted onto our custom Mainboard, we used soldering for the connectors.

![soldering.jpg](other%2Fsoldering.jpg)


# Obstacles
For precise navigation on the parcours the robot primarily uses the walls for orientation. In the opening race the robot therefor constantly monitors and analyzes the position of the for it visible walls to drive straight foreward in the straight sections and anticipate curves. This way it can drive efficiently on the parcours with a high velocity. 

## Concept


# Obstacle Mangement


## Drivecontroller


## General


# Pictures


# Engineering/Design


# Attachment
