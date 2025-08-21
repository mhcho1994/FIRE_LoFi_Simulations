# load packages
import fmu
import modelica 
import logging 
import numpy as np 
import os 
import pandas as pd
import json
import utils
from math import gcd

"""
example of a closed/open loop simulation using FMUs generated from Modelica (component) models
notes:
   this script is only valid for simulating autonomous systems (time invariant dynamics)
   do not use if the system model has time-dependent elements 
   if you have time-dependent elements in the model, please covert them into time-dependent variables
   CoSIM (co-simulation): simulation considers controller, joystick, mission planner, quadrotor physics (multiple components)
   CLoopSIM (closed-loop simulation): simulation considers controller, quadrotor physics
   OLooopSIM (open-loop simulation): quadrotor physics-only

to do:
   1: change 'fidelity' parameter type (make it not to be 'final', modify modelica script)
   2: make 'final run statistics' silent (to decrease running time)
"""

class MFQuadrotorCoSIM:
    """
    simulation classes for a multi-fidelity quadrotor co-simulation, supports:
    - orchestrate different CPS components in whole drone system
    - retrieve the final value of all state variables
    """
    def __init__(self, physics_fidelity = 1, cyber_fidelity = 1, t0 = 0, fmu_type = "me", attack_scenario = 0, parameter_file = "dronesim.json", logging_file = None):
        """
        inputs:
            fidelity (int): physics/cyber fidelity level
                1: low-fidelity model
                2: high-fidelity model
        outputs:
            simulation class
                - fmus
                - data dictionary
                - attack scenario and parameters
        """
        # manual switch for generating fmu, will be deprecated after patching the following issue
        # important note: currently, the fidelity parameter is automatically set as a "final" parameter, which cannot be modified using Python API
        #                 please change it manually in the RoverExample.Components.Rover script (be careful that the fidelity parameter is also in RoverExample.ExampleScenario)
        fmu_gen = True

        # t0 should be 0
        self.t0 = t0
        
        # save fidelity level
        self.physics_fidelity = physics_fidelity
        self.cyber_fidelity = cyber_fidelity

        # read specification file (.json format)
        self.parameters_info = {}
        with open(parameter_file, 'r') as file:
            self.parameters_info['scneario'] = json.load(file)

        # check logging option
        if logging_file is None:
            self.logging = False
        else:
            self.logging = True
            self.logfile = logging_file

        # enable when you want to generate or update the FMU (e.g. Lo-Fi -> Hi-Fi, Hi-Fi -> Lo-Fi)
        # convert the modelica file into a fmu and set the ininital conditions and parameters 
        if fmu_gen:
            if self.physics_fidelity == 1:
                modelica.generate_fmu("GSQuad.Components.Quadrotor", f"{os.getcwd()}/model/MFQuadrotor_FMUgen.mo", fmu_type, {'fidelity': 1}) 
            elif self.physics_fidelity == 2:
                modelica.generate_fmu("GSQuad.Components.Quadrotor", f"{os.getcwd()}/model/MFQuadrotor_FMUgen.mo", fmu_type, {'fidelity': 2})
            if self.cyber_fidelity == 1:
                modelica.generate_fmu("GSQuad.Components.Controller", f"{os.getcwd()}/model/MFQuadrotor_FMUgen.mo", fmu_type, {'fidelity': 1}) 
            elif self.cyber_fidelity == 2:
                modelica.generate_fmu("GSQuad.Components.Controller", f"{os.getcwd()}/model/MFQuadrotor_FMUgen.mo", fmu_type, {'fidelity': 2})
            modelica.generate_fmu("GSQuad.Components.Joystick", f"{os.getcwd()}/model/MFQuadrotor_FMUgen.mo", fmu_type, {})
            modelica.generate_fmu("GSQuad.Components.MissionPlanner", f"{os.getcwd()}/model/MFQuadrotor_FMUgen.mo", fmu_type, {})

        # load fmu for chassis dynamics
        fmuDyn = "GSQuad.Components.Quadrotor/GSQuad.Components.Quadrotor.fmu"
        self.fmudyn = fmu.FMU(fmuDyn, fmu_type, self.t0, tol=1e-6)
        # retrieve parameter information before reset if needed
        self.parameters_info['chassis'] = {}
        if self.physics_fidelity == 1:
            self.parameters_info['chassis']['actuator_sample_period'] = self.fmudyn.get_param('actuator_sample_period')
            self.parameters_info['chassis']['sensor_sample_period'] = self.fmudyn.get_param('sensor_sample_period')
        elif self.physics_fidelity == 2:
            pass
        self.fmudyn.reset()
        # do not use setup_experiment and initialize for ME (CS only)
        # self.fmudyn.model.setup_experiment(start_time=0.0, stop_time=1.0)
        # self.fmudyn.model.initialize()
        if self.physics_fidelity == 1:
            self.fmudyn.set_param({'quad_low.position_w_p_w[1]': 0.0, 'quad_low.position_w_p_w[2]': 0.0, 'quad_low.position_w_p_w[3]': 0.0, # location
                                   'quad_low.velocity_w_p_b[1]': 0.0, 'quad_low.velocity_w_p_b[2]': 0.0, "quad_low.velocity_w_p_b[3]": 0.0, # body fixed velocity
                                   'quad_low.quaternion_wb[1]': 1.0, 'quad_low.quaternion_wb[2]':0.0, 'quad_low.quaternion_wb[3]': 0.0, 'quad_low.quaternion_wb[4]': 0.0, # quaternion from inertial to body
                                   'quad_low.velocity_w_p_b[1]': 0.0, 'quad_low.velocity_w_p_b[2]': 0.0, 'quad_low.velocity_w_p_b[3]': 0.0}) # rotation rates in body
        elif self.physics_fidelity == 2:
            self.fmudyn.set_param({'quad_high.position_w_p_w[1]': 0.0, 'quad_high.position_w_p_w[2]': 0.0, 'quad_high.position_w_p_w[3]': 0.0, # location
                                   'quad_high.velocity_w_p_b[1]': 0.0, 'quad_high.velocity_w_p_b[2]': 0.0, "quad_high.velocity_w_p_b[3]": 0.0, # body fixed velocity
                                   'quad_high.quaternion_wb[1]': 1.0, 'quad_high.quaternion_wb[2]':0.0, 'quad_high.quaternion_wb[3]': 0.0, 'quad_low.quaternion_wb[4]': 0.0, # quaternion from inertial to body
                                   'quad_high.velocity_w_p_b[1]': 0.0, 'quad_high.velocity_w_p_b[2]': 0.0, 'quad_high.velocity_w_p_b[3]': 0.0}) # rotation rates in body

        fmuController = "GSQuad.Components.Controller/GSQuad.Components.Controller.fmu" 
        self.fmuctrl = fmu.FMU(fmuController, fmu_type, self.t0, tol=1e-6)
        # retrieve parameter information before reset if needed
        self.parameters_info['controller'] = {}
        if self.cyber_fidelity == 1:
            self.parameters_info['controller']['update_period'] = self.fmuctrl.get_param('update_period')
        elif self.cyber_fidelity == 2:
            pass
        self.fmuctrl.reset()
        # do not use setup_experiment and initialize for ME (CS only)
        # self.fmuctrl.model.setup_experiment(start_time=0.0, stop_time=1.0)
        # self.fmuctrl.model.initialize()
        if self.cyber_fidelity == 1:
            self.fmuctrl.set_param({'euler_pid.vel_error_i[1]': 0.0, 'euler_pid.vel_error_i[2]': 0.0, 'euler_pid.vel_error_i[3]': 0.0, # velocity error integral
                                    'euler_pid.vel_error_last[1]': 0.0, 'euler_pid.vel_error_last[2]': 0.0, 'euler_pid.vel_error_last[3]': 0.0, # last velocity error
                                    'euler_pid.rate_error_i[1]': 0.0, 'euler_pid.rate_error_i[2]': 0.0, 'euler_pid.rate_error_i[3]': 0.0, # rate error integral
                                    'euler_pid.rate_error_last[1]': 0.0, 'euler_pid.rate_error_last[2]': 0.0, 'euler_pid.rate_error_last[3]': 0.0})  # last rate error
        elif self.cyber_fidelity == 2:
            pass

        fmuJoystick = "GSQuad.Components.Joystick/GSQuad.Components.Joystick.fmu"
        self.fmustick = fmu.FMU(fmuJoystick, fmu_type, self.t0, tol=1e-6)
        # retrieve parameter information before reset if needed
        self.parameters_info['joystick'] = {}
        self.parameters_info['joystick']['sample_period'] = self.fmustick.get_param('sample_period')
        self.fmustick.reset() 
        # do not use setup_experiment and initialize for ME (CS only)
        # self.fmustick.model.setup_experiment(start_time=0.0, stop_time=1.0)
        # self.fmustick.model.initialize()
        self.fmustick.set_param({'stick_cmd[1]': 0.0, 'stick_cmd[2]': 0.0, 'stick_cmd[3]': 0.0, 'stick_cmd[4]': 0.0}) # zero throttle, zero roll/pitch/yaw commands

        fmuMissionPlanner = "GSQuad.Components.MissionPlanner/GSQuad.Components.MissionPlanner.fmu"
        self.fmuplanner = fmu.FMU(fmuMissionPlanner, fmu_type, self.t0, tol=1e-6)
        # retrieve parameter information before reset if needed
        self.parameters_info['planner'] = {}
        self.parameters_info['planner']['sample_period'] = self.fmuplanner.get_param('sample_period')
        self.fmuplanner.reset()
        # do not use setup_experiment and initialize for ME (CS only)
        # self.fmuplanner.model.setup_experiment(start_time=0.0, stop_time=1.0)
        # self.fmuplanner.model.initialize()
        self.fmuplanner.set_param({'position_setpoint_w[1]': 0.0, 'position_setpoint_w[2]': 0.0, 'position_setpoint_w[3]': -30.0, 'position_setpoint_w[1]': 0.0,
                                   'yaw_setpoint_w': 0.0}) # position coordinates in NED, yaw setpoint with respect to north direction
        
        # closed loop simulation by calling all components
        self.data = {'t': np.zeros((0,)), 'missionplanner': {}, 'joystick': {}, 'controller': {}, 'quadrotor': {}}

        # set up the attack model
        self.attack_scenario = attack_scenario
        if self.attack_scenario == 0:  # [-] attack_scneario: 0 = no attack
            pass
            
    def simulate(self, tf = 10.0, dt = 0.1):
        """
        multi-rate, event-driven closed-loop simulation.
        - planner, joystick, controller run at their own rates.
        - chassis (plant) integrates at the base tick (gcd of all periods)
        - zero-order hold between updates for slower publishers.
        returns a pandas.DataFrame with stitched logs (time + selected signals).
        inputs:
            tf (np.float64): final time
            dt (np.float64): time step for simulation propagation

        outputs :
            pandas dataframe
                - time history of outputs
        """
        # compute gcd of update period to set it as step size
        dt_dyn = np.min(np.array([self.parameters_info['chassis'].get('actuator_sample_period'),
                                  self.parameters_info['chassis'].get('sensor_sample_period')],dtype=np.float64))
        dt_ctrl = np.float64(self.parameters_info['controller'].get('update_period'))
        dt_plan = np.float64(self.parameters_info['planner'].get('sample_period'))
        dt_stick = np.float64(self.parameters_info['joystick'].get('sample_period'))

        dt_base = _gcd_dt([dt_dyn, dt_ctrl, dt_plan, dt_stick], quantum=1e-6)
        if dt_base - min(dt_dyn, dt_ctrl, dt_plan, dt_stick) > 1e-12:
            dt_base = min(dt_dyn, dt_ctrl, dt_plan, dt_stick)

        # number of ticks
        n_ticks = int(round((tf-self.t0)/dt_base))+1 

        # compute stride
        k_dyn = np.max([1,np.int16(round(dt_dyn/dt_base))])
        k_ctrl = np.max([1,np.int16(round(dt_ctrl/dt_base))])
        k_plan = np.max([1,np.int16(round(dt_plan/dt_base))])
        k_stick = np.max([1,np.int16(round(dt_stick/dt_base))])

        # event-loop
        t_last_dyn = self.t0
        t_last_ctrl = self.t0
        t_last_plan = self.t0
        t_last_stick = self.t0

        # data
        t = [np.float64(self.t0)]

        for itick in range(n_ticks):
            t_cur = self.t0+itick*dt_base

            # propagate mission planner
            if itick%k_plan == 0:
                self.fmuplanner.step_time(t_last_plan, t_cur)
                t_last_plan = t_cur

            # propagate joystick
            if itick%k_stick == 0:
                self.fmustick.step_time(t_last_stick, t_cur)
                t_last_stick = t_cur

            # propagate controller
            if itick%k_ctrl == 0:
                self.fmuctrl.set_input(t_cur, {"x_ref": self.fmuplanner.get_output_value()['position_setpoint_w[1]'],
                                               "y_ref": self.fmuplanner.get_output_value()['position_setpoint_w[2]'],
                                               "z_ref": self.fmuplanner.get_output_value()['position_setpoint_w[3]'],
                                               "psi_ref": self.fmuplanner.get_output_value()['yaw_setpoint_w']})
                self.fmuctrl.set_param({'sample_interval': dt_base+1e-6})




        self.data['t'] = np.linspace(self.t0, tf, int((tf-self.t0)/dt)+1)

        for t_cur in self.data['t']:

            # get planner setpoint command
            self.fmuplanner.set_param({'sample_interval': dt+1e-6})  # avoid unnecessary update at t = dt, make update happen only at t=0
            self.fmuplanner.step_time(t_cur, t_cur+dt)

            # get joystick command
            #self.fmustick.
            
            # set turn command for controller
            # note: turn command is an integer input for controller; however, in FMU generation, it is not given as a parameter of the model
            #       'model.set' can be used to change the parameter 'turn' in the controller
            self.fctrl.model.set('turn', self.fweb.get_output_value()['turn'])
            if self.attack_scenario == 1 and self.fctrl.get_variable_value()['s'] >= 3.0:               # right turn command given + right turn mode check, initiate attack when rover is turning  
                self.fctrl.set_input(t_cur+dt, {"ax_acc": self.fmudyn.get_output_value()['ax_meas'],
                                                "ay_acc": self.fmudyn.get_output_value()['ay_meas'],
                                                "az_acc": self.fmudyn.get_output_value()['az_meas'],
                                                "mx_mag": self.fmudyn.get_output_value()['mx_meas'],
                                                "my_mag": self.fmudyn.get_output_value()['my_meas'],
                                                "mz_mag": self.fmudyn.get_output_value()['mz_meas'],
                                                "phi_gyro": self.fmudyn.get_output_value()['phi_meas'],
                                                "theta_gyro": self.fmudyn.get_output_value()['theta_meas'],
                                                "psi_gyro": self.fmudyn.get_output_value()['psi_meas']+(self.emi_atk_level)*(self.bias), 
                                                "p_gyro": self.fmudyn.get_output_value()['p_meas'], 
                                                "q_gyro": self.fmudyn.get_output_value()['q_meas'], 
                                                "r_gyro": self.fmudyn.get_output_value()['r_meas'], 
                                                "x": self.fmudyn.get_output_value()['x_meas'], 
                                                "y": self.fmudyn.get_output_value()['y_meas']})
            else:
                self.fctrl.set_input(t_cur+dt, {"ax_acc": self.fmudyn.get_output_value()['ax_meas'],
                                                "ay_acc": self.fmudyn.get_output_value()['ay_meas'],
                                                "az_acc": self.fmudyn.get_output_value()['az_meas'],
                                                "mx_mag": self.fmudyn.get_output_value()['mx_meas'],
                                                "my_mag": self.fmudyn.get_output_value()['my_meas'],
                                                "mz_mag": self.fmudyn.get_output_value()['mz_meas'],
                                                "phi_gyro": self.fmudyn.get_output_value()['phi_meas'],
                                                "theta_gyro": self.fmudyn.get_output_value()['theta_meas'],
                                                "psi_gyro": self.fmudyn.get_output_value()['psi_meas'], 
                                                "p_gyro": self.fmudyn.get_output_value()['p_meas'], 
                                                "q_gyro": self.fmudyn.get_output_value()['q_meas'], 
                                                "r_gyro": self.fmudyn.get_output_value()['r_meas'], 
                                                "x": self.fmudyn.get_output_value()['x_meas'], 
                                                "y": self.fmudyn.get_output_value()['y_meas']})
            self.fctrl.set_param({'sample_interval': dt+1e-6})  # avoid unnecessary update at t = dt, make update happen only at t=0
            self.fctrl.step_time(t_cur, t_cur+dt)

            # set pwm command for chassis dynamic model
            if self.attack_scenario == 2:
                self.fmudyn.set_input(t_cur+dt, {"pwm_steering": self.fctrl.get_output_value()['pwm_steering'], 
                                                 "pwm_throttle": np.int32(self.rollover_thr_pwm)})
            else:
                self.fmudyn.set_input(t_cur+dt, {"pwm_steering": self.fctrl.get_output_value()['pwm_steering'], 
                                                 "pwm_throttle": self.fctrl.get_output_value()['pwm_throttle']})

            if self.attack_scenario == 3:
                self.fmudyn.set_param({'W': self.gyro_atk_power, 
                                       'dist': self.speaker_dist,
                                       'w_ac': self.gyro_atk_freq, 
                                       'phi_0': self.gyro_atk_phase, 
                                       'psi_ac': self.gyro_atk_dir, 
                                       'epsilon': self.gyro_misalignment})
            else:
                self.fmudyn.set_param({'W': 0.0, 
                                       'dist': 0.01,
                                       'w_ac': 15000.0, 
                                       'phi_0': 0.0, 
                                       'psi_ac': 0.0, 
                                       'epsilon': 0.0})
                
            if self.attack_scenario == 4:
                self.fmudyn.set_param({'rover_8d.emi.wire_dir[1]': self.wire_dir[0],
                                       'rover_8d.emi.wire_dir[2]': self.wire_dir[1],
                                       'rover_8d.emi.wire_dir[3]': self.wire_dir[2],
                                       'rover_8d.emi.x_wire[1]': self.wire_dist[0],
                                       'rover_8d.emi.x_wire[2]': self.wire_dist[1],
                                       'rover_8d.emi.x_wire[3]': self.wire_dist[2]})
            else:
                self.fmudyn.set_param({'rover_8d.emi.wire_dir[1]': 0,
                                       'rover_8d.emi.wire_dir[2]': 0,
                                       'rover_8d.emi.wire_dir[3]': 1,
                                       'rover_8d.emi.x_wire[1]': 0,
                                       'rover_8d.emi.x_wire[2]': 0,
                                       'rover_8d.emi.x_wire[3]': -0.01})
            self.fmudyn.set_param({'sample_interval': dt+1e-6})  # avoid unnecessary update at t = dt, make update happen only at t=0
            self.fmudyn.step_time(t_cur, t_cur+dt)

            # correct internal time clock
            self.fweb.model.set(['timer_count'],[np.float64(t_cur/dt)])
            self.fmudyn.model.set(['gyroatk.timer_count'],[np.float64(t_cur/dt)])

            # check
            # print(np.float64(t_cur/dt))
            # print(self.fmudyn.variable[self.fmudyn.variableNames.index('timer_count')])
            # print(' -- ')

        # log the data
        self.data['webserver'] = self.fweb.data
        self.data['controller'] = self.fctrl.data
        self.data['rover'] = self.fmudyn.data

        # return the simulation output
        save_data = pd.DataFrame(np.hstack((self.data['t'][:,np.newaxis], 
                                            self.data['rover']['state']['values'], self.data['rover']['variable']['values'], 
                                            self.data['controller']['state']['values'], self.data['controller']['variable']['values'], 
                                            self.data['webserver']['state']['values'], self.data['webserver']['variable']['values'])), 
                                columns=np.hstack((['time'], self.data['rover']['state']['names'], self.data['rover']['variable']['names'],
                                                   self.data['controller']['state']['names'], self.data['controller']['variable']['names'], 
                                                   self.data['webserver']['state']['names'], self.data['webserver']['variable']['names'])))

        return save_data

    def sim_reset(self):
        """
        reset the simulation model
        inputs :
        outputs :
        """
        self.fmudyn.empty_data()
        self.fmudyn.reset()
        # do not use setup_experiment and initialize for ME (CS only)
        # self.fmudyn.model.setup_experiment(start_time=0.0, stop_time=1.0)
        # self.fmudyn.model.initialize()
        if self.fidelity == 1:
            self.fmudyn.set_param({'rover_3d.x': 0.0 , 'rover_3d.y': 0.0, 'rover_3d.z': 0.0, # location
                                   'rover_3d.vx': 0.0, 'rover_3d.vy': 0.0, "rover_3d.vz": 0.0, # body fixed velocity
                                   'rover_3d.psi': self.parameters_info['heading']/180*np.pi, 'rover_3d.theta':0.0 , 'rover_3d.phi': 0.0, # intrinsic rotation sequence in order
                                   'rover_3d.p': 0.0, 'rover_3d.q': 0.0, 'rover_3d.r': 0.0}) # rotation rates about x,y,z
        elif self.fidelity == 2:
            self.fmudyn.set_param({'rover_8d.x': 0.0 , 'rover_8d.y': 0.0, 'rover_8d.z': 0.0, # location
                                   'rover_8d.vx': 0.0, 'rover_8d.vy': 0.0, "rover_8d.vz": 0.0, # body fixed velocity
                                   'rover_8d.psi': self.parameters_info['heading']/180*np.pi, 'rover_8d.theta':0.0 , 'rover_8d.phi': 0.0, # intrinsic rotation sequence in order
                                   'rover_8d.p': 0.0, 'rover_8d.q': 0.0, 'rover_8d.r': 0.0, # rotation rates about x,y,z
                                   'rover_8d.omega_fl': 0.0, 'rover_8d.omega_fr': 0.0, 'rover_8d.omega_rl': 0.0, 'rover_8d.omega_rr': 0.0, # wheel rotational speeds
                                   'rover_8d.omega': 0.0}) # shaft rotational speeds

        self.fctrl.empty_data()
        self.fctrl.reset() 
        # do not use setup_experiment and initialize for ME (CS only)
        # self.fctrl.model.setup_experiment(start_time=0.0, stop_time=1.0)
        # self.fctrl.model.initialize()
        ic_quat = utils.eul2quat(np.array([0.0,0.0,self.parameters_info['heading']/180*np.pi]))
        self.fctrl.set_param({'s': 0 , # state of Lo-Fi FSM controller
                              'x_ref': 0.0, 'y_ref': 0.0, 'psi_ref': 0.0, # reference for turn logic
                              'v': 0.0, 'delta': 0.0, # commands
                              'quaternion_filtered[1]': ic_quat[0], 'quaternion_filtered[2]': ic_quat[1], 'quaternion_filtered[3]': ic_quat[2], 'quaternion_filtered[4]': ic_quat[3]}) 
        self.fctrl.set_param({'sample_interval': 0.1+0.00001})
        if self.fidelity == 1:
            self.fctrl.set_param({'delta_turn': np.atan(self.parameters_info['l_total']/self.parameters_info['turn_radius'])})
        elif self.fidelity == 2:
            self.fctrl.set_param({'delta_turn': np.atan(self.parameters_info['l_total']/self.parameters_info['turn_radius'])})

        self.fweb.empty_data()
        self.fweb.reset()
        # do not use setup_experiment and initialize for ME (CS only)
        # self.fweb.model.setup_experiment(start_time=0.0, stop_time=1.0)
        # self.fweb.model.initialize()

        # initialize data log
        self.data = {'t': np.zeros((0,)), 'webserver': {}, 'controller': {}, 'rover': {}}

        # randomly change attack parameters
        if self.attack_scenario == 1:
            self.bias = 2*(np.random.rand(1)-0.5)

        if self.attack_scenario == 3:
            self.gyro_atk_freq = (self.parameters_info['drive_freq']+self.parameters_info['acoustic_freq_range']*(2*np.random.rand(1)-1))*(2*np.pi)
            self.gyro_misalignment = self.parameters_info['gyro_misalignment']*np.random.rand(1)*(np.pi/180)
            self.gyro_atk_dir = np.random.rand(1)*(np.pi/2)
            self.gyro_atk_phase = np.random.rand(1)*2*np.pi-np.pi

# class MFRoverOnlySIM:
#     """
#     simulation classes for a multi-fidelity rover model, supports:
#     - rover-only simulation (input: throttle PWM duty cycle, )
#     - set input variables to inputs at given time instances
#     - be careful for
#     - retrieve the final value of all state variables
#     """


def _gcd_dt(dts, quantum=1e-6):
    """
    compute a robust gcd time step for floating periods
    - 'quantum' is the rounding quantum (e.g., 1e-6s = 1us)
    """
    us = [max(1, int(round(dt/quantum))) for dt in dts]  # [-] convert to integer ticks
    g = us[0]
    for u in us[1:]:
        g = gcd(g, u)
    return g*quantum