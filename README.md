# Inverse-Kinematics-2019-Arm

Core Functions:

DHmatrix - returns modified DH Matrix given DH Parameters
FK_2019 - Forward Kinematics Function, returns T matrices for each joint given theta values of arm
Jacobian3DOF_2019 - Returns 3x6 Jacobian matrix for 3DOF operation (xyz)
Jacobian6DOF_2019 - Returns 6x6 Jacobain matrix for 6DOF operation (xyz,euler angles)
angular2linear - Converts angular velocity to linear actuator velocity given geometric parameters of triangle


Auxiliary Functions:

plotArm_2019 - plots current position of arm and returns position of end effectors
EulerAngles - returns euler angles for given transformation matrix


IK Functions:

IK_Position_2019 + IK_3DPosition_2019 (3DOF control)
- function and script that is based on desired position
- script returns plot of arm

IK_Velocity_2019 + IK_3DVelocity_2019 (3DOF control)
- function and script based on desired velocity
- script returns plot of arm

IK_6DOF + IK_3D_6DOF (6DOF control)
- function and script based on desired velocity
- script returns plot of arm



