import getopt
import os
import shutil
import sys


def usage():
    print( 'Usage: compileSOP_GroupPointsWithMatchingAttribs.py [options]' )
    print( '    -h, --help           Print this help.' )
    

doCopy = False

try:                                
    opts, args = getopt.getopt( sys.argv[1:], "h", ["help"] )
except getopt.GetoptError:
    print( 'Bad arg' )
    usage()
    sys.exit(2)
    
for (opt, arg) in opts:
    if opt in ( '-h', '--help' ):
        usage()
        sys.exit(2)


# do a "try" here, if "BULLET_PATH" is not found, prompt for a BULLET_PATH (raw_input) and then create one.

houdiniPath = None

try:
    houdiniPath = os.environ['HFS']
except( KeyError ):
    print( 'Error: Cannot find environment variable $HFS.  Run this script from SideFX "Command Line Tools."' )
    sys.exit(1)

# Go to the directory containing the SOP_GroupPointsWithMatchingAttribs code
sopSrcPath = houdiniPath + '\\toolkit\\SOPs\\src'
sopSrcPath = sopSrcPath.replace( '\\', '/' )
os.chdir( sopSrcPath )

hcustomCommand = 'hcustom SOP_GroupPointsWithMatchingAttribs.cpp'
os.system( hcustomCommand )
