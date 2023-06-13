import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from numpy.fft import fft, ifft, fftshift, fftfreq
from scipy.signal import max_len_seq

import rospy
from std_msgs.msg import Bool as Bool_msg
from std_msgs.msg import Empty as Empty_msg
from std_msgs.msg import Float64 as Float64_msg
from std_msgs.msg import String as String_msg

# interfaces
def prGreen(skk): print("\033[92m {}\033[00m" .format(skk))
def prRed(skk): print("\033[91m {}\033[00m" .format(skk))
def prBlue(skk): print("\033[94m {}\033[00m" .format(skk))
def prYellow(skk): print("\033[33m {}\033[00m" .format(skk))

class Printer(object):
    """ Base Printer class with ROS, Bekchoff and other connections """

    def __init__(self, namespace="beckhoff"):
        self.namespace = namespace
        self.init_temperature_control_publisher()
        self.init_temperature_control_publisher_bool()
        self.init_force_bias_publisher()

    #publisher
    def init_temperature_control_publisher(self):
        """ Resets the force sampler by clearing the queue """
        self.temperature_control_publisher = rospy.Publisher( self.namespace + '/temperature/current_heating_target', Float64_msg, queue_size=1)
        prBlue(self.namespace + "/current_heating_target topic advertised")

    def init_temperature_control_publisher_bool(self):
        """ Resets the force sampler by clearing the queue """
        self.temperature_control_publisher_bool = rospy.Publisher( self.namespace + '/temperature/current_heating_on', Bool_msg, queue_size=1)
        prBlue(self.namespace + "/current_heating_target topic advertised")

    def init_force_bias_publisher(self):
        """ Publishes force sampler topics """
        self.publisher_bias = rospy.Publisher( '/force_sampler/bias_sensor', Bool_msg, queue_size=1)

    def send_temperature_control_value(self, msg=0.1234):
        """ send temperature command to ads-cpp for external input

        Parameters:
            msg : temperature, defaults to 0.1234
        """

        self.temperature_control_publisher.publish(Float64_msg(msg))
        prGreen("Sent temperature command: " + str(msg))    


    def send_temperature_heater_on(self, msg=False):
        """ send temperature command to ads-cpp for external input

        Parameters:
            msg : temperature element on/off, defaults to false
        """

        self.temperature_control_publisher_bool.publish(Bool_msg(msg))
        prGreen("Sent temperature command: " + str(msg))    

    def bias_sensor(self, msg=True):
        """ Run sensor biasing

        Parameters:
            msg : bool, defaults to True
        """
        prGreen(msg)
        self.publisher_bias.publish(Bool_msg(msg))
        prGreen(("Biasing sensor ", msg))



#program

plot_on = False
samples_n = 6 # input length
rate_hz = 0.5



seq = max_len_seq(samples_n)[0]  # +1 and -1
spec = fft(seq)
N = len(seq)


print("input sequence " , seq)
print(N , " vs input: " ,2^samples_n)

if plot_on == True:
    plt.plot(fftshift(fftfreq(N)), fftshift(np.abs(spec)), '.-')
    plt.margins(0.1, 0.1)
    plt.grid(True)
    plt.show()

rospy.init_node('supervisor', anonymous=True)
rate = rospy.Rate( rate_hz )#

printer_beckhoff = Printer()

#printer_beckhoff.temperature_control_publisher_bool.publish(1)# preheat

def ros_loop_binary_signal():

    i = 0
    while not rospy.is_shutdown():
        i = i+1
        if(i < seq.size):
            printer_beckhoff.temperature_control_publisher_bool.publish(seq[i])
        else:
            printer_beckhoff.temperature_control_publisher_bool.publish(0)
 
        print("loop nr ", i)        
        rate.sleep()



if __name__ == '__main__':
    try:
        ros_loop_binary_signal()
    except rospy.ROSInterruptException:
        pass
