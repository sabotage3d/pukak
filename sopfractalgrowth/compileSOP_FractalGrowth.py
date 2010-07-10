import getopt
import os
import shutil
import sys


def usage():
    print( 'Usage: compileSOP_FractalGrowth.py [options]' )
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

houdiniPath = None

try:
    houdiniPath = os.environ['HFS']
except( KeyError ):
    print( 'Error: Cannot find environment variable $HFS.  Run this script from SideFX "Command Line Tools."' )
    sys.exit(1)

# Go to the directory containing the SOP_FractalGrowth code
sopFractalGrowthSrcPath = houdiniPath + '\\toolkit\\SOP_FractalGrowth\\src'
sopFractalGrowthSrcPath = sopFractalGrowthSrcPath.replace( '\\', '/' )
os.chdir( sopFractalGrowthSrcPath )

hcustomCommand = 'hcustom -i ../lib SOP_FractalGrowth.cpp'
os.system( hcustomCommand )

if doCopy:
    os.chdir( "../lib" )
    userPath = os.environ['USERPROFILE']
    houdiniDllPath = userPath + '\\My Documents\\houdini10.0\\dso'
    if not os.path.exists( houdiniDllPath ):
        houdiniUserPath = userPath + '\\My Documents\\houdini10.0'
        if not os.path.exists( houdiniUserPath ):
            os.mkdir( houdiniUserPath )
        os.mkdir( houdiniDllPath )
    shutil.copy( 'SOP_FractalGrowth.dll', houdiniDllPath )
