<h1 style="text-align:center;">FRC Team 1902 - Exploding Bacon - 2025 Reefscape</h1>

![Reefscape](resources/Reefscape.png)

This project contains the code for team 1902's 2025 FRC robot "WaterPig". The code for our robot is written in Java and uses several external libraries including [WPILib](https://docs.wpilib.org/en/stable/), [REVLib](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html), [PathPlanner](https://github.com/mjansen4857/pathplanner), [PhotonVision](https://github.com/PhotonVision/photonvision), [YAGSL](https://github.com/BroncBotz3481/YAGSL), [MapleSim](https://shenzhen-robotics-alliance.github.io/maple-sim/), and [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit). We also use [Elastic Dashboard](https://github.com/Gold872/elastic-dashboard) and [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) as for viewing live and logged robot data and telemetry. All robot code is organzied in the standard command-based layout.
## Code Highlights
- Object Detection
  
    Our robot is capable of autonomously detecting and collecting game pieces from the floor during both the Autonomous and Teleoperated periods. This functionality is powered by an Orange Pi 5 running [Photonvisons object detection pipeline](https://docs.photonvision.org/en/latest/docs/objectDetection/about-object-detection.html), utilizing a YOLOv11 model trained in-house for the specific task of identifying game pieces on the field.


- Autonmous Pathing and Alignemnt

    We developed a simple, PID-based pathing system that enables fast and precise alignment to preset waypoints. This allows the robot to execute fully autonomous cycles by snapping to either the nearest calculated waypoint or a predefined path—effective in both Autonomous and Teleoperated modes allowing the robot to precisely allign with the scoring elements.

- AdvantageScope and MapleSim Simulation

    To support the development and testing of both robot code and autonomous routines, our team implemented basic simulation capabilities using AdvantageScope as the primary simulation environment and MapleSim for simple physics-based simulation of robot interactions with the simulated field. This approach allowed us to begin testing code before the robot was fully manufactured and safely develop autonomous routines without risking robot or field damage.

    We also use AdvantageScope to review past matches logged to a USB via AdvantageKit. This includes vision estimates, sensor values, and motor readings. It has been very usefully for debugging issues that occur during matches.
## About the Team
FRC Team 1902, Exploding Bacon, is a high school robotics team based in Orlando, Florida. We are a community-driven team that focuses on STEM education and outreach. Our team is dedicated to creating a positive impact in our community through the power of robotics. We run a variety of programs, outreaches, and more, so feel free to visit us on our website:
[explodingbacon.com](explodingbacon.com)
## Contributing

Contributions are always welcome! If you find a bug in our code, or if you have an idea for a new feature, please open an issue on this repository. If you'd like to contribute code, please fork this repository and submit a pull request.

## Acknowledgements
This code was written by:
 - [Samuel Styles](https://github.com/purerandomgit)
 - [Gustavo Cortes](https://github.com/chefgusta)

This code was reviewed by:
 - [Tyler Waddell](https://github.com/brothersw)

We would also like to thank our sponsours, team members, parents and mentors for their help and support.