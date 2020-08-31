# fabrik-ik-motor6d

FABRIK based inverse kinematics solver made to work with roblox's motor6d rigging system.

If you want to experiment and use the project then I recommend you download the releases and download the entire source-code in a zip file and open up the place file.

Partial Documentation can be found in my Roblox thread post on community resources:
https://devforum.roblox.com/t/fabrik-ik-solver-intended-for-any-motor6d-rig-open-source/742227

Current To Do List:

Brainstorm ways to better constraint a vector in a region:
Why? Currently the conical constraint method is confusing and activity intensive(Goes up to 10% Activity)

1. (NVM translating this idea from 2D to 3D is way too difficult as it requires finding the intersection between 3d Parts) Utilize rotated region3 to create space where vector can reside in order to constraint the vector

2. Use parts and welds to set the constraint X,Y,Z Axis so its easier for the user to control the constraint

3. Now!!: creating a hinge joint with planes to prevent sideways movement of the vector Limb

4. Debug mode to visualize the conical constraint using Parts
