
# this is an example line for compiling Bullet for linux.
# please go here for more info: http://odforce.net/wiki/index.php/BulletPhysicsRBDSolverDOP

# set up to use a symbolic link for the bullet libs called "bullet-current" installed in an location next to this Bullet DOP code

cd C:\Program Files\Side Effects Software\Houdini 10.0.401\toolkit\SIM_SolverBullet-0.11\src
hcustom -g -i . -I C:\PROGRA~1\bullet-2.74\src -l LibBulletDynamics -l LibBulletCollision -l LibLinearMath -L ..\bulletLibraries -L ..\bulletLibraries -L ..\bulletLibraries SIM_SolverBullet.cpp
