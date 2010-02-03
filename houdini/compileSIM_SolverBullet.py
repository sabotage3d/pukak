

import os
import shutil

# do a "try" here, if "BULLET_PATH" is not found, prompt for a BULLET_PATH (raw_input) and then create one.
bulletPath = None
houdiniPath = None
try:
    bulletPath = os.environ['BULLET_PATH']
    bulletPath = bulletPath.replace( 'Program Files', 'PROGRA~1' )
except( ValueError ):
    print( 'Set up a variable called BULLET_PATH, then set BULLET_PATH to the path to your main bullet folder.' )

try:
    houdiniPath = os.environ['HFS']
except( ValueError ):
    print( 'Error: Cannot find environment variable $HFS.  Run this script from SideFX "Command Line Tools."' )

if bulletPath and houdiniPath:
    
    bulletDynamicsPath = bulletPath + '\\src\\BulletDynamics'
    bulletCollisionPath = bulletPath + '\\src\\BulletCollision'
    linearMathPath = bulletPath + '\\src\\LinearMath'
    
    simSolverBulletSrcPath = houdiniPath + '\\toolkit\\SIM_SolverBullet-0.11\\src'
    
    os.chdir( simSolverBulletSrcPath )
    
    bulletSrcPath = bulletPath + '\\src'
    bulletSrcPath = bulletSrcPath.replace( '/', '\\' )
    
    hcustomCommand = 'hcustom -g -i . -I ' + bulletSrcPath + ' -l BulletDynamics -l BulletCollision -l LinearMath -L ' + bulletDynamicsPath + ' -L ' + bulletCollisionPath + ' -L ' + linearMathPath + ' SIM_SnowSolverBullet.cpp'
    os.system( hcustomCommand )

    print( 'Bullet src path = ' + bulletSrcPath )
    print( 'Sim solver bullet src path = ' + simSolverBulletSrcPath )
    
    userPath = os.environ['USERPROFILE']
    houdiniDllPath = userPath + '\\My Documents\\houdini10.0\\dso'
    if not os.path.exists( houdiniDllPath ):
        houdiniUserPath = userPath + '\\My Documents\\houdini10.0'
        if not os.path.exists( houdiniUserPath ):
            os.mkdir( houdiniUserPath )
        os.mkdir( houdiniDllPath )
    shutil.copy( 'SIM_SnowSolverBullet.dll', houdiniDllPath )
