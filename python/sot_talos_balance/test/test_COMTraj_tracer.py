from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.utils.plot_utils import *
import sot_talos_balance.talos_conf as conf
import sot_talos_balance.motor_parameters as motor_params
import sot_talos_balance.control_manager_conf as control_manager_conf
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from numpy import eye
from time import sleep
import os
from IPython import embed

def main(robot):
	dt = robot.timeStep;
	robot.comTrajGen = create_com_trajectory_generator(dt,robot);
	
	# --- COM
	robot.taskCom = MetaTaskKineCom(robot.dynamic)
	robot.dynamic.com.recompute(0)
	robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
	robot.taskCom.task.controlGain.value = 10
	
	# --- CONTACTS
	#define contactLF and contactRF
	robot.contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF',robot.OperationalPointsMap['left-ankle'])
	robot.contactLF.feature.frame('desired')
	robot.contactLF.gain.setConstant(100)
	robot.contactLF.keep()
	locals()['contactLF'] = robot.contactLF
	
	robot.contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
	robot.contactRF.feature.frame('desired')
	robot.contactRF.gain.setConstant(100)
	robot.contactRF.keep()
	locals()['contactRF'] = robot.contactRF
	
	# --- SOT
	robot.sot = SOT('sot')
	robot.sot.setSize(robot.dynamic.getDimension())
	plug(robot.sot.control,robot.device.control)
	
	robot.sot.push(robot.contactRF.task.name)
	robot.sot.push(robot.taskCom.task.name)
	robot.sot.push(robot.contactLF.task.name)
	robot.device.control.recompute(0)
	
	# --- ESTIMATION
	robot.ctrl_manager            = create_ctrl_manager(control_manager_conf, motor_params, dt);
	robot.imu_offset_compensation = create_imu_offset_compensation(robot, dt)
	robot.device_filters          = create_device_filters(robot, dt)
	robot.imu_filters             = create_imu_filters(robot, dt)
	robot.base_estimator          = create_base_estimator(robot, dt, conf) 
	robot.be_filters              = create_be_filters(robot, dt)
	
	# --- TRACERS
	#~ outputs = ['robotState']
	#~ device_tracer    = create_tracer(robot,robot.device, 'device_tracer', outputs)
	outputs = ['q']
	estimator_tracer = create_tracer(robot,robot.base_estimator, 'estimator_tracer', outputs)
	
	# --- RUN SIMULATION
	plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN);
	sleep(1.0);
	os.system("rosservice call \start_dynamic_graph")
	sleep(2.0);
	robot.comTrajGen.move(1,-0.025,4.0);
	sleep(5.0);
	robot.comTrajGen.startSinusoid(1,0.05,8.0);
	sleep(0.2);
	
	#~ device_tracer.start();
	estimator_tracer.start();
	sleep(3.0);
	#~ dump_tracer(device_tracer);
	dump_tracer(estimator_tracer);
	print 'data dumped'
	
	return estimator_tracer	
	
	# --- DISPLAY
	device_data = read_tracer_file('/tmp/dg_'+robot.device.name+'-robotState.dat')
	estimator_data = read_tracer_file('/tmp/dg_'+robot.base_estimator.name+'-q.dat')
	plot_select_traj(device_data,[10,23,15])
	plot_select_traj(estimator_data,[10,23,15])
	write_pdf_graph('/tmp/')


