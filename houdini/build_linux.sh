#!/bin/csh -f

set INCLUDEBULLET  = /usr/local/include

set LIBBULLET = /usr/local/lib

hcustom -g -I$INCLUDEBULLET \
         \
         -L$LIBBULLET \
         -lBulletDynamics \
         -lBulletCollision \
         -lLinearMath \
         src/SIM_SnowSolverBullet.cpp



# this is an example line for compiling Bullet for linux.
# please go here for more info: http://odforce.net/wiki/index.php/BulletPhysicsRBDSolverDOP

# set up to use a symbolic link for the bullet libs called "bullet-current" installed in an location next to this Bullet DOP code

#hcustom -g -i . -I ../../bullet-current/src -l LibBulletDynamics -l LibBulletCollision -l LibLinearMath -L  ../../bullet-current/src/BulletDynamics/  -L  ../../bullet-current/src/BulletCollision/  -L ../../bullet-current/src/LinearMath/ SIM_SolverBullet.cpp

