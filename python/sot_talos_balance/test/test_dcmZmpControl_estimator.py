'''Test CoM admittance control as described in paper.'''
from sot_talos_balance.utils.run_test_utils import *
from time import sleep

import matplotlib.pyplot as plt
import numpy as np

run_test('appli_dcmZmpControl_estimator.py')

raw_input("Wait before resetting madgwick")
runCommandClient('robot.imu_filters.setBeta(1e-3)')
c = ask_for_confirmation('Reset foot positions?')
if c:
    print('Resetting foot positions')
    runCommandClient('robot.base_estimator.reset_foot_positions()')
else:
    print('Not resetting foot positions')

run_ft_calibration('robot.ftc')
raw_input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

c = ask_for_confirmation("Execute a sinusoid?")
if c:
    print("Putting the robot in position...")
    runCommandClient('robot.comTrajGen.move(1,-0.025,1.0)')
    sleep(1.0)
    print("Robot is in position!")

    c2 = ask_for_confirmation("Confirm executing the sinusoid?")
    if c2:
        print("Executing the sinusoid...")
        runCommandClient('robot.comTrajGen.startSinusoid(1,0.025,2.0)')
        print("Sinusoid started!")
    else:
        print("Not executing the sinusoid")

    c3 = ask_for_confirmation("Put the robot back?")
    if c3:
        print("Stopping the robot...")
        runCommandClient('robot.comTrajGen.stop(1)')
        sleep(5.0)
        print("Putting the robot back...")
        runCommandClient('robot.comTrajGen.move(1,0.0,1.0)')
        sleep(1.0)
        print("The robot is back in position!")
    else:
        print("Not putting the robot back")
else:
    print("Not executing the sinusoid")

c = ask_for_confirmation("Raise the foot?")
if c:
    print("Putting the robot in position...")
    runCommandClient('robot.comTrajGen.move(1,-0.08,10.0)')
    sleep(10.0)
    print("Robot is in position!")

    c2 = ask_for_confirmation("Confirm raising the foot?")
    if c2:
        print("Raising the foot...")
        runCommandClient('h = robot.dynamic.LF.value[2][3]')
        runCommandClient('robot.lfTrajGen.move(2,h+0.05,10.0)')
        sleep(10.0)
        print("Foot has been raised!")
        c3 = ask_for_confirmation("Put the foot back?")
    else:
        print("Not raising the foot")
        c3 = False

    if c3:
        print("Putting the foot back...")
        runCommandClient('robot.lfTrajGen.move(2,h,10.0)')
        sleep(10.0)
        print("The foot is back in position!")
        c4 = ask_for_confirmation("Put the robot back?")
    else:
        print("Not putting the foot back")
        c4 = False

    if c4:
        print("Putting the robot back...")
        runCommandClient('robot.comTrajGen.move(1,0.0,10.0)')
        sleep(10.0)
        print("The robot is back in position!")
else:
    print("Not raising the foot")

raw_input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

