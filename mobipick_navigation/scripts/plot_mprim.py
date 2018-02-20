#!/usr/bin/env python

import math
import sys
import matplotlib.pyplot as plt
import numpy as np

def getValue( strline, name ):
    if strline.find( name ) < 0:
        raise Exception( "File format not matching the parser expectation", name )

    return strline.replace( name, "", 1 )
    
def getPose( line ):
    ss = line.split()
    np.array( [ float( ss[0] ), float( ss[1] ), float( ss[2] ) ] )

class MPrim:
    def __init__( self, f ):
        self.primID = int( getValue( f.readline(), "primID:" ) )
        self.startAngle = int( getValue( f.readline(), "startangle_c:" ) )
        self.endPose = getPose( getValue( f.readline(), "endpose_c:" ) )
        self.cost = float( getValue( f.readline(), "additionalactioncostmult:" ) )
        self.nrPoses = int( getValue( f.readline(), "intermediateposes:" ) )
        poses = []
        for _ in xrange( self.nrPoses ):
            poses.append( f.readline() )
        self.poses = np.loadtxt( poses, delimiter=" " )
        self.cmap = plt.get_cmap( "spectral" )

    def plot( self, nrAngles ):
        plt.plot( self.poses[:,0], self.poses[:,1], c=self.cmap( self.startAngle * 256 / nrAngles ) )

class MPrims:
    def __init__( self, filename ):
        f = open( filename, "r" )

        self.resolution = float( getValue( f.readline(), "resolution_m:" ) )
        self.nrAngles = int( getValue( f.readline(), "numberofangles:" ) )
        self.nrPrims = int( getValue( f.readline(), "totalnumberofprimitives:" ) )

        self.prims = []
        for _ in xrange( self.nrPrims ):
            self.prims.append( MPrim( f ) )
        
        f.close()

    def plot( self ):
        fig = plt.figure()
        ax = fig.add_subplot( 111 )
        ax.set_xticks( np.arange(-1, 1, self.resolution ) )
        ax.set_yticks( np.arange(-1, 1, self.resolution ) )
        for prim in self.prims:
            prim.plot( self.nrAngles )
        plt.grid()
        plt.show()

prims = MPrims( sys.argv[1] )
prims.plot()
