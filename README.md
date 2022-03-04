## Teaching tools

1.  Teaching marker
2.  Teaching ghost
3.  Teaching tfbc

### Teaching marker
TODO: add gif\
TODO: marker reset service\
TODO: marker pose preset service (and later gui)

An interactive marker that can be moved around in the world.
The poseition of this marker is published and it can later be
used to save new frames in the world, or update existing ones. 

### Teaching ghost
TODO: ghost reset service\
TODO: change tcp service (actually, this will be in the new broadcaster server)

An inverse kinematics solution is calulated all the time so that
a ghost of the robot can follow the teaching marker around. 
This can be used to verify the joint pose of the robot for a
marker pose, and also to move the actual (or simulated) robot
to the ghosts joint pose as thepose is published with ns /ghost.

### Teaching TFBC
The Teaching Test Frame BroadCaster publishes a few simple frames
to tf so that these packages can independently be run and tested.
It is not used in an actual launch or docker scenario where the 
actual tf broadcaster server is managing all the frames instead.  