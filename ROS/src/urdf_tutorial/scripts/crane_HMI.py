#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from crane_interfaces.msg import MotionReference
from crane_interfaces.msg import Visualisation

import PySimpleGUI as sg


class GuiNode(Node):
	
	def __init__(self):
		super().__init__('gui_node')
		
		self.vis_subscription = self.create_subscription(Visualisation, "state_publisher", self.listener_callback, 10)
		
		self.motion_publisher = self.create_publisher(MotionReference, "motion_reference", 10)
	def listener_callback(self, message):
		piston_pressure = message.piston_pressure
		rod_pressure = message.rod_pressure
		self.window['piston_pressure'].update(str(piston_pressure))
		self.window['rod_pressure'].update(str(rod_pressure))
		
	def send_data(self, start, stop, position_reference, velocity_reference):
		message = MotionReference()
		message.start = start
		message.stop = stop
		message.cyl_vel_ref = velocity_reference
		message.cal_pos_ref = position_reference
		
		self.motion_publisher.publish(message)
		
def main(args = None):
	rclpy.init(args=args)
	gui_node = GuiNode()
	sg.theme('LightGrey1')
	
	layout = [[sg.Button('Start'), sg.Button('Stop')],
		[sg.Text('Position Reference'), sg.InputText(key = 'position_reference')],
		[sg.Text('Velocity Reference'), sg.InputText(key = 'velocity_reference')],
		[sg.Text('Piston Pressure'), sg.InputText(key = 'piston_pressure')],
		[sg.Text('Rod Pressure'), sg.InputText(key = 'rod_pressure')]]
	window = sg.Window('Green Crane Simulator', layout)
	gui_node.window = window
	
	while True:
		event, values = window.read()
		if event == sg.WIN_CLOSED or event == 'Exit':
			break
		if event == 'Start':
			try: 
				velocity_reference = float(values['velocity_reference'])
				position_reference = float(values['position_reference'])
				gui_node.send_data(True, False, position_reference, velocity_reference)
				rclpy.spin_once(self)
			except: 
				sg.popup('Please enter valid numbers!')
		if event == 'Stop':
			gui_node.send_data(False, True, 0, 0)
			rclpy.spin_once(self)
	window.close()
	gui_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
