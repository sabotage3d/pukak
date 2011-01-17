import getopt
import os
import shutil
import sys


def usage():
    print( 'Usage: compileDOP_GroupShellGranules.py [options]' )
    print( '    -h, --help           Print this help.' )
    

doCopy = False

try:                                
    opts, args = getopt.getopt( sys.argv[1:], "hc", ["help"] )
except getopt.GetoptError:
    print( 'Bad arg' )
    usage()
    sys.exit(2)
    
for (opt, arg) in opts:
    if opt in ( '-h', '--help' ):
        usage()
        sys.exit(2)

houdiniPath = None

try:
    houdiniPath = os.environ['HFS']
except( KeyError ):
    print( 'Error: Cannot find environment variable $HFS.  Run this script from SideFX "Command Line Tools."' )
    sys.exit(1)

# Go to the directory containing the SOP_FractalGrowth code
dopSrcPath = houdiniPath + '/toolkit/DOP_GroupShellGranules/src'
dopSrcPath = dopSrcPath.replace( '\\', '/' )
os.chdir( dopSrcPath )

hcustomCommand = 'hcustom DOP_GroupShellGranules.cpp'
os.system( hcustomCommand )

