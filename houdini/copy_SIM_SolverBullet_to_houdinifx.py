import getopt
import os
import shutil
import sys


def usage():
    print( 'Usage: SIM_SolverBullet_to_houdinifx.py' )


houdinifxPath = 'J:/groups/houdinifx/tools/bullet'
if not os.path.exists( houdinifxPath ):
    print( 'Error: Cannot find houdinifx group folder' )
    sys.exit(1)

houdiniPath = ''
simSolverBulletSrcPath = ''
try:    # If you have access to the Houdini environment variables, find the files relative to HFS
    houdiniPath = os.environ['HFS']
    simSolverBulletSrcPath = houdiniPath + '\\toolkit\\SIM_SnowBulletSolver\\src'
    simSolverBulletSrcPath = simSolverBulletSrcPath.replace( '\\', '/' )
except( KeyError ):   # Otherwise, find the files relative to the current working directory
    houdiniPath = os.getcwd()
    simSolverBulletSrcPath = houdiniPath + '\\src'
    simSolverBulletSrcPath = simSolverBulletSrcPath.replace( '\\', '/' )

simSolverBullet_h = simSolverBulletSrcPath + '/SIM_SnowSolverBullet.h'
simSolverBullet_cpp = simSolverBulletSrcPath + '/SIM_SnowSolverBullet.cpp'

shutil.copy( simSolverBullet_h, houdinifxPath )
shutil.copy( simSolverBullet_cpp, houdinifxPath )
