#!/usr/bin/env python3

import rclpy
import rclpy.node

from hapthexa_msgs.msg import ForceSensor

import PySimpleGUI as sg

frontright = [
    [sg.Text('000',key=0),sg.ProgressBar(254,size=(5,10),key=18)],
    [sg.Text('000',key=1),sg.ProgressBar(254,size=(5,10),key=19)],
    [sg.Text('000',key=2),sg.ProgressBar(254,size=(5,10),key=20)]
]

frontleft = [
    [sg.Text('000',key=3),sg.ProgressBar(254,size=(5,10),key=21)],
    [sg.Text('000',key=4),sg.ProgressBar(254,size=(5,10),key=22)],
    [sg.Text('000',key=5),sg.ProgressBar(254,size=(5,10),key=23)]
]

middleright = [
    [sg.Text('000',key=15),sg.ProgressBar(254,size=(5,10),key=33)],
    [sg.Text('000',key=16),sg.ProgressBar(254,size=(5,10),key=34)],
    [sg.Text('000',key=17),sg.ProgressBar(254,size=(5,10),key=35)]
]

middleleft = [
    [sg.Text('000',key=6),sg.ProgressBar(254,size=(5,10),key=24)],
    [sg.Text('000',key=7),sg.ProgressBar(254,size=(5,10),key=25)],
    [sg.Text('000',key=8),sg.ProgressBar(254,size=(5,10),key=26)]
]

rearright = [
    [sg.Text('000',key=12),sg.ProgressBar(254,size=(5,10),key=30)],
    [sg.Text('000',key=13),sg.ProgressBar(254,size=(5,10),key=31)],
    [sg.Text('000',key=14),sg.ProgressBar(254,size=(5,10),key=32)]
]

rearleft = [
    [sg.Text('000',key= 9),sg.ProgressBar(254,size=(5,10),key=27)],
    [sg.Text('000',key=10),sg.ProgressBar(254,size=(5,10),key=28)],
    [sg.Text('000',key=11),sg.ProgressBar(254,size=(5,10),key=29)]
]

layout = [
    # [sg.Combo([],size=(25,10),key="ports"),sg.Button(button_text='Start/Stop')],
    [sg.Frame('Front Left', frontleft),sg.Frame('Front Right', frontright)],
    [sg.Frame('Middle Left', middleleft),sg.Frame('Middle Right', middleright)],
    [sg.Frame('Rear Left', rearleft),sg.Frame('Rear Right', rearright)]
]

leg_names = ['front_left', 'middle_left', 'rear_left', 'rear_right', 'middle_right', 'front_right']

legnum_dict = {
    'front_left': 1,
    'front_right': 0,
    'middle_right': 5,
    'middle_left': 2,
    'rear_right': 4,
    'rear_left': 3
}

class SensorChecker(rclpy.node.Node):

    def __init__(self):
        super().__init__('sensor_checker')
        self._window = sg.Window('HaptHexa Sensor Checker (ROS2)', layout)
        self._timer = self.create_timer(0.01, self.timer_callback)

        self._forcesensor_subs = []
        for leg_name in leg_names:
            self._forcesensor_subs.append   (   
                                                self.create_subscription(ForceSensor,
                                                'hapthexa/leg/'+leg_name+'/force_sensor',
                                                lambda msg,legnum=legnum_dict[leg_name]: self.forcesendor_callback(msg,legnum),
                                                10)
                                            )
    
    def timer_callback(self):
        event, values = self._window.read(timeout = 1)
        if event == sg.WIN_CLOSED:
            rclpy.shutdown()

    def forcesendor_callback(self, msg,legnum):
        self._window.FindElement(legnum*3+0).update(str(int(msg.radial_magnitude*100)).rjust(3,'0'))
        self._window.FindElement(legnum*3+1).update(str(0).rjust(3,'0'))
        self._window.FindElement(legnum*3+2).update(str(0).rjust(3,'0'))
        self._window.FindElement(legnum*3+0+18).update_bar(msg.loadcell1*127+127)
        self._window.FindElement(legnum*3+1+18).update_bar(msg.loadcell2*127+127)
        self._window.FindElement(legnum*3+2+18).update_bar(msg.piezo*254)



def main(args=None):
    rclpy.init(args=args)
    node = SensorChecker()
    rclpy.spin(node)
    # while rclpy.ok():
    #     rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()