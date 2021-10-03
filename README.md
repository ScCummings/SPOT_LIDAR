# SPOT_LIDAR

LIDAR research for the AI for Robotics course at Chapman University. 

## Useful Links

- [Getting Started with Autowalk](https://support.bostondynamics.com/s/article/Getting-Started-with-Autowalk)
- [Mission Recorder Example](https://dev.bostondynamics.com/python/examples/mission_recorder/readme)


## Proposed Design

When script is run:

- Turn off e-stop
- Power on the robot
- Self right
- Sit down
- Stand up
- Relocalize. If no fiducial is nearby, cancel the script since it cannot start recording
- Start recording
- Generate the mission & stop recording
- Sit down
- Stop program
