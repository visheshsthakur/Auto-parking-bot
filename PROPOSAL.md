# Path Finder

Team Members:
- Vishesh Singh Thakur, vthakur@buffalo.edu
- Ashwin Nair, anair3@buffalo.edu

--- 

## Project Objective
To make two robots, one of which would be a ground robot able to manually and autonomously controlled. The second robot would be an aerial robot that would go high in the air to visualize and create a map of a place, analyze the data, and would then send signals to the ground robot to move around a specified path


## Contributions
This would be a kind of a maze solving robot, where instead of the ground bot wandering around a maze to map the place and find the exit, the aerial bot would do the mapping and analyzing of that data, and would tell the ground bot where to go. This topic is interesting because it can be implemented in self driving cars. A vehicle can go to a parking lot and on connecting with the in-built aerial cameras in the parking lot can generate the real-time map of the lot and direct the vehicle to park it in the closest empty space it can find


## Project Plan
We will need to program an aerial robot to use the mounted camera to map the surface, analyze objects and boundaries, and then find a way out of the area. We will also need to program a ground robot that will be able to recieve commands from another robot and use the commands to wander around the mapped area, and perform a specific action, like exiting a maze, finding a specific object, auto-parking a car, etc.

We will need to learn how a robot can perform the mapping of a space using its camera, and analyze the data, so that it can be used to command the ground bot to perform certain tasks. We also need to learn how to analyze a live data stream which can then be filtered and used to command our ground bot. We need to learn how to program the ground bot to receive the incoming data and move accordingly, while also mapping its surroundings to move on its own if the data is only sent once from the aerial bot.


## Milestones/Schedule Checklist
{What are the tasks that you need to complete?  Who is going to do them?  When will they be completed?}
- [x] Complete this proposal document.  *Due Oct. 25*
- [x] Creating a map. * Due November 4
- [x] Mounted the fixed camera bot to analyse the map
- [x] Mapping and analysing the map created. *Due november 12th
- [x] Spawn multiple robots onto the map using a launch file
- [ ] Sending this mapped data to the moving bot. *Due november 19th
- [ ] Calibrate according to the map and test the robot.
- [x] Create progress report.  *Due Nov. 25*
- [ ] Final test of the project.
- [ ] Create final presentation.  *Due Dec. 6*
- [ ] Provide system documentation (README.md).  *Due Dec. 13*


## Measures of Success
- [x] Create a proper map.
- [x] Installing a camera to analyse the map continuously
- [x] Spawing of robots which would receive the data from the mounted camera
- [x] Getting the mounted camera to publish a topic
- [ ] Getting the model lets say a warehouse or a parking lot mapped.
- [ ] Transferring the data mapped data to the moving bot.
- [ ] Robot able to tackle the obstructions using the mapped data.
- [ ] Creating a continous interaction between the two bots so that if an obstruction comes out all of a sudden the moving bot could be able to avoid and get a new path for its goal.





