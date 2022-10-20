# ME 495 Embedded Systems - homework 2
Author: `${Ayush Gaggar}`

This homework is designed to launch a robot in rviz2, place a brick at a specified location, and
drive the robot to catch the brick (after determining whether it's within range and speed limits).
After catching the brick, drive it back to the spawn position and tilt it off.

## Quickstart
1. Use ros2 launch `${turtle_brick} ${turtle_arena.launch.py} ${use_jsp:=none}` to start the
   arena and turtle simulation. (Note that use_jsp has options gui, jsp, and none, depending on
   whether the user wants to use the joint state publisher gui or not)
2. Use `${ros2 service call /brick_place turtle_brick_interfaces/srv/Place '{brick_x: 8.0, brick_y: 3.0, brick_z: 25.0}'}`
   to drop the brick
3. Here is a video of the robot when the brick is within catching range: 
<video src="https://user-images.githubusercontent.com/10903052/196907853-3992735b-3157-46f3-bc37-a55f83ecf7cf.mp4" data-canonical-src="https://user-images.githubusercontent.com/10903052/196907853-3992735b-3157-46f3-bc37-a55f83ecf7cf.mp4" controls="controls" muted="muted" class="d-block rounded-bottom-2 border-top width-fit" style="max-height:640px;">
</video>

4. Here is a video of the turtle when the brick cannot be caught:
<video src="https://user-images.githubusercontent.com/10903052/196906638-de139174-6b0d-4970-855e-551b21fe407d.mp4" data-canonical-src="https://user-images.githubusercontent.com/10903052/196906638-de139174-6b0d-4970-855e-551b21fe407d.mp4" controls="controls" muted="muted" class="d-block rounded-bottom-2 border-top width-fit" style="max-height:640px;">
</video>

Worked with: `${Nick Morales, Katie Hughes, Marnu Nel, Allan Garcia-casal, Ava Zahedi, David Dorf}`