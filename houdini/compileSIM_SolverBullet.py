import getopt
import os
import shutil
import sys


def usage():
    print( 'Usage: compileSIM_SolverBullet.py' )
    print( '    -h, --help           Print this help.' )
    print( '    -c, --copyfiles      Copy plug-in over to the home houdini plug-in folder (default does not copy).' )
    

doCopy = False

try:                                
    opts, args = getopt.getopt( sys.argv[1:], "hc", ["help", "copyfiles"] )
except getopt.GetoptError:
    print( 'Bad arg' )
    usage()
    sys.exit(2)
    
for (opt, arg) in opts:
    if opt in ( '-h', '--help' ):
        usage()
        sys.exit(2)
    elif opt in ( '-c', '--copyfiles'):
        doCopy = True


# do a "try" here, if "BULLET_PATH" is not found, prompt for a BULLET_PATH (raw_input) and then create one.
bulletPath = None
houdiniPath = None
try:
    bulletPath = os.environ['BULLET_PATH']
    bulletPath = bulletPath.replace( 'Program Files (x86)', 'PROGRA~2' )
    bulletPath = bulletPath.replace( 'Program Files', 'PROGRA~1' )
except( KeyError ):
    print( 'Set up a variable called BULLET_PATH, then set BULLET_PATH to the path to your main bullet folder.' )

try:
    houdiniPath = os.environ['HFS']
except( KeyError ):
    print( 'Error: Cannot find environment variable $HFS.  Run this script from SideFX "Command Line Tools."' )

if bulletPath and houdiniPath:
    
    bulletDynamicsPath = bulletPath + '\\src\\BulletDynamics'
    bulletCollisionPath = bulletPath + '\\src\\BulletCollision'
    linearMathPath = bulletPath + '\\src\\LinearMath'
    
    simSolverBulletSrcPath = houdiniPath + '\\toolkit\\SIM_SnowBulletSolver\\src'
    simSolverBulletSrcPath = simSolverBulletSrcPath.replace( '\\', '/' )
    
    bulletSrcPath = bulletPath + '\\src'
    bulletSrcPath = bulletSrcPath.replace( '/', '\\' )
    
    hcustomCommand = 'hcustom -i . -I ' + bulletSrcPath + ' -l BulletDynamics -l BulletCollision -l LinearMath -L ' + bulletDynamicsPath + ' -L ' + bulletCollisionPath + ' -L ' + linearMathPath + ' SIM_SnowSolverBullet.cpp'
    os.chdir( simSolverBulletSrcPath )
    #hcustomCommand = 'hcustom -g -i C:\Users\srh43\Documents\houdini10.0\dso -I ' + bulletSrcPath + ' -l BulletDynamics -l BulletCollision -l LinearMath -L ' + bulletDynamicsPath + ' -L ' + bulletCollisionPath + ' -L ' + linearMathPath + ' SIM_SnowSolverBullet.cpp'
    os.system( hcustomCommand )

    #print( 'Bullet src path = ' + bulletSrcPath )
    #print( 'Sim solver bullet src path = ' + simSolverBulletSrcPath )
    
    if doCopy:
        userPath = os.environ['USERPROFILE']
        houdiniDllPath = userPath + '\\My Documents\\houdini11.0\\dso'
        if not os.path.exists( houdiniDllPath ):
            houdiniUserPath = userPath + '\\My Documents\\houdini11.0'
            if not os.path.exists( houdiniUserPath ):
                os.mkdir( houdiniUserPath )
            os.mkdir( houdiniDllPath )
        shutil.copy( 'SIM_SnowSolverBullet.dll', houdiniDllPath )
