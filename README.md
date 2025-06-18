# Seesaw Balancer: Your First Robot Brain

This project is about teaching an Arduino how to balance a wooden beam using two propellers. It's a fun and safe way to learn about control systems before trying to build a full drone.

### The Goal

An electronic sensor (the MPU6050) feels the tilt of the beam. The Arduino (the "brain") runs code that reads this tilt and decides how fast to spin each propeller to bring the beam back to level.

### How to Get Started

Your main instruction manual is the **`DIAGNOSIS_GUIDE.md`** file.

Open it and follow the steps carefully. The most important file you will change is **`src/main.cpp`**, which contains the logic for the controller.
