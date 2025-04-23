# GliderControl
Code to control the gradual descent of the X-1 glider into the scoring zone for Iowa State University's 2024-2025 Design Build Fly (DBF) team.

This repository contains both the team's custom code for glider control, code to test the glider's individual componenets,
and helpful references such as hardware datasheets and  Nick Rehm's open source dRehmFlight. More info about this dRehmFlight software can be found at its GitHub repository here: https://github.com/nickrehm/dRehmFlight/tree/master

Directory Descriptions:
- Archive: Contains old versions of flight control programs for the X-1's descent
- Intelligent_Descent: Sensor-based descent program
- References: Contains schematics and datasheets for the X-1's electronics
- Timed_Descent_Logging: Simplified descent program that runs off of timing, not sensor input, but logs sensor data to the Teensy's on-board microSD card throughout the flight
- Timed_Descent_NoLogging: Simplified descent program that runs off of timing, not sensor input, and does not log sensor data. Can be used for a "barebones" flight descent program.
- Unit_Tests: Contains unit tests for the different modules of the X-1's electronics (GPS, IMU, servo, etc.) to be tested independently and in combination.