""" This module abstract the instantiation and configuration of the spacemouse.
"""

import xdefw.rtt
import rtt_interface
import deploy.deployer as ddeployer
import lgsm

sm = None     





class obj_move(xdefw.rtt.Task):
                   
	""" Orocos task that gathers the components to use the spacemouse: load driver, open device and configure.
	Two control mode are available: Normal and PDC.
	In Normal mode, the spacemouse control a cursor.
	In PDC mode, the user attach a body to the cursor with a PD coupling so that
	the body is attracted to the cursor.
	In both case the cursor is controlled in the camera frame.

	:param name: name of the Orocos task that will be associated to this task
	:param time_step: period of the Orocos task
	:param phy: the main physic agent
	:param graph: the main graphic agent
	:param cursor_name: name of the body that will represent the cursor
	:param pdc_enabled: set to True to enable PDC mode, False to enable Normal mode
	:param body_name: name of the body to be attached to the cursor in PDC mode

	"""
	def __init__(self, name, time_step, phy, graph, cursor_name, pdc_enabled, body_name):
		"""
		"""

		super(Obj_Mouse, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))

		self.s.setPeriod(time_step)
		self.smf = xdefw.rtt.Task(ddeployer.load("smf", "dio::obj_Movefactory", "dio-hw-obj_move", "dio/factory/"))
		self.smf.s.setPeriod(time_step)
		self.device = self.smf.s.scan()
		self.sm = xdefw.rtt.Task(self.smf.s.build(self.device[0], 'smf'))
		self.sm.s.setPeriod(time_step)
		self.phy = phy
		self.camera_interface = graph.s.Interface.CameraInterface("mainScene")

		self.sm_in_port = self.addCreateInputPort("sm_in", "Twistd" )
		self.sm_in_port.connectTo(self.sm.getPort("out_vel"))
		self.pos_out = self.addCreateOutputPort("pos_out", "Displacementd")

		self.cursor = None
		self.cursor = self.phy.s.GVM.RigidBody(cursor_name)
		self.cursor.disableWeight()
 



	def startHook(self):
		self.smf.s.start()
		self.sm.s.start()

	def stopHook(self):
		self.sm.s.stop()
		self.smf.s.stop()

	def updateHook(self):

		H_b_c = lgsm.Displacementd(0.0, 0.0, 0.0, 1, 0, 0, 0)
		H_b_c.setTranslation(lgsm.vector([0,0,0]))

		self.pos_out.write(H_b_c)
                
                




def createTask(name, time_step, phy, graph, cursor_name, pdc_enabled=False, body_name=None):
	"""
	Instantiate the Orocos task and set a proxy

	:param name: name of the Orocos task that will be associated to this task
	:param time_step: period of the Orocos task
	:param phy: the main physic agent
	:param graph: the main graphic agent
	:param cursor_name: name of the body that will represent the cursor
	:param pdc_enabled: set to True to enable PDC mode, False to enable Normal mode
	:param body_name: name of the body to be attached to the cursor in PDC mode
	:param PR: P parameter for rotation of the PD Coupling (PDC mode)
	:param PT: P parameter for translation of the PD Coupling (PDC mode)
	:param DR: D parameter for rotation of the PD Coupling (PDC mode)
	:param DT: D parameter for translation of the PD Coupling (PDC mode)
	"""
	sm = Obj_Move(name, time_step, phy, graph, cursor_name, pdc_enabled, body_name)
	setProxy(sm)
	return sm

def setProxy(_sm):
	"""
	Set the proxy
	"""
	global sm
	sm = _sm
