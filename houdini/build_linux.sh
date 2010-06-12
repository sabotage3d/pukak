#!/bin/csh -f

set INCLUDEBULLET  = /usr/local/include
set BULLET_LIB = /usr/local/lib

hcustom -I$INCLUDEBULLET -L$BULLET_LIB -lBulletDynamics -lBulletCollision -lLinearMath src/SIM_SnowSolverBullet.cpp
         
#set BULLET_SRC=/dist/bullet/bullet-2.76-r1967/src
#set BULLET_LIBS=/dist/bullet/bullet-2.76-r1967/lib
#set BULLET_LIBS=/usr/local/lib

# this is an example line for compiling Bullet for linux.
# please go here for more info: http://odforce.net/wiki/index.php/BulletPhysicsRBDSolverDOP

# set up to use a symbolic link for the bullet libs called "bullet-current" installed in an location next to this Bullet DOP code

#hcustom -g -i . -I ../../bullet-current/src -l LibBulletDynamics -l LibBulletCollision -l LibLinearMath -L  ../../bullet-current/src/BulletDynamics/  -L  ../../bullet-current/src/BulletCollision/  -L ../../bullet-current/src/LinearMath/ SIM_SolverBullet.cpp

#hcustom -I$BULLET_SRC -L$BULLET_LIBS -lBulletCollision -lBulletDynamics -lLinearMath src/SIM_SnowSolverBullet.cpp

#hcustom -I$BULLET_SRC -L$BULLET_LIBS -lBulletSoftBody -lBulletCollision -lBulletDynamics -lLinearMath src/SIM_SnowSolverBullet.cpp
