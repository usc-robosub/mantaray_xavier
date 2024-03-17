'''
This will create a ros topic that will act as a limit controller, setting the hard velocity and acceleration
limits of the thrusters. This controller also handles the conversion between simulation message types and
real hardware outputs.

Publishers:
- This is the topic that the user or GNC will communicate with. So publishers to this topic are the teleop
programs or higher level control code.

Subscriber:
- The /mantaray/thruster/[] topics

Todo:
- Rewrite this in CPP
'''

