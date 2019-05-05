#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import scipy as sp
import scipy.misc as sp_misc
import cPickle, os, shutil, sys, gzip, argparse

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, JointState
from std_msgs.msg import String, Float64
import message_filters
import ros_numpy

defaultNumSamples = 10000
defaultPickleMod  = 1000
parser = argparse.ArgumentParser()
parser.add_argument( '-n', '--number_samples', type=int,          default=10000 )
parser.add_argument( '-m', '--pickle_mod',     type=int,          default=1000  )
parser.add_argument( '-r', '--resize',         action='store_true'    )
parser.add_argument( '-s', '--shape',          type=int, nargs=2, default=[32, 32],   help='Shape to resize to'    )
args = parser.parse_args()

_DBG          = False
_totalSamples = args.number_samples
_pickleMod    = args.pickle_mod
_resize       = args.resize
_shape        = args.shape
_data         = { 'pc': [], 'jc': [], 'sd': [] }
_fldr         = os.path.join(os.environ['ROS_PACKAGE_PATH'].split(':')[0].split('src')[0],'sampledData')
_pickleCount  = 0
_i            = 0

def shutdownHook():
	_data = None

def callback( pc, jc, sd ):
	global _totalSamples, _pickleMod, _resize, _shape, _data, _fldr, _pickleCount, _i
	_data['pc'].append( ros_numpy.numpify(pc).astype( np.float64 ) )
	_data['jc'].append( ros_numpy.numpify(jc) )
	_data['sd'].append( sd.data )
	
	if _DBG:
		print( '{}'.format(_i) )
		print( '\tpc_{i} type:  {pct}\n\tpc_{i} shape: {pcs}'.format(       i=_i, pct=type(_data['pc'][-1]), pcs=_data['pc'][-1].shape ) )
		print( '\tjc_{i} type:  {jct}\n\tjc_{i} shape: {jcs}'.format(       i=_i, jct=type(_data['jc'][-1]), jcs=_data['jc'][-1].shape ) )
		print( '\tsd_{i} type:  {sdt}\n\tsd_{i} value: {sdv}\t\n\n'.format( i=_i, sdt=type(_data['sd'][-1]), sdv=_data['sd'][-1]       ) )
	
	if( _i%_pickleMod==0 and not _i==0) or _i==_totalSamples-1:
		if _DBG: print( 'pc_len: {}\njc_len: {}\nsd_len: {}'.format(len(_data['pc']),len(_data['jc']),len(_data['sd'])) )
		if _resize:
			print( 'Resizing data.. ', end='' ); sys.stdout.flush();
			_data['pc'] = map(lambda arr : sp_misc.imresize(arr, _shape), _data['pc'])
		print( 'Pickling data group {}...'.format(_pickleCount), end='' );sys.stdout.flush();
		cPickle.dump(
			_data,
			open(os.path.join(_fldr,'n{}_s{}x{}_{}.p'.format(
					len(_data['pc']),
					_data['pc'][-1].shape[0], _data['pc'][-1].shape[1],
					_pickleCount)), 'wb'),
			protocol=-1 )
		_pickleCount+=1
		_data = { 'pc': [], 'jc': [], 'sd': [] }
		print( ' complete.\n' )
	if( _i==_totalSamples-1 ):
		rospy.signal_shutdown('Sample collection complete')
	_i+=1


def listener():
	global _totalSamples, _pickleMod, _data, _fldr, _pickleCount, _i
	print( 'Grabbing {} samples and storing in batches of {}...'.format(_totalSamples,_pickleMod) )
	
	rospy.init_node('data_grabber', anonymous=True)
	pc_sub = message_filters.Subscriber( '/camera/depth/points', ros_numpy.numpy_msg(PointCloud2) )
	jc_sub = message_filters.Subscriber( '/lbr4/joint_states',   ros_numpy.numpy_msg(JointState) )
	sd_sub = message_filters.Subscriber( '/sd_collision_dist',   Float64 )
	
	ts = message_filters.ApproximateTimeSynchronizer( 
		[pc_sub, jc_sub, sd_sub], 
		10, 0.1,
		allow_headerless=True )
	ts.registerCallback( callback )
	
	rospy.on_shutdown( shutdownHook )
	rospy.spin()


if __name__ == '__main__':
	if not os.path.exists(_fldr):
		os.mkdir(_fldr)
	listener()
