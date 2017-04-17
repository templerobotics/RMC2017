#!/usr/bin/env python


"""Encoder Decoder Class for TURMC Robot. ROS Related data transformations are not included. 
   Serves as main bridge to grab and return raw encoder data. Decodes rotary encoder pulses"""

import pigpio

class decoder:

    def __init__ (self, pi, channelA, channelB, callback):
        """ 
        Initiate class. channelA and channelB resemble GPIO pins on pi for input.
        Callback fn called when encoder makes a turn. It takes one parameter which is +1
        or -1 for counterclockwise

        """
        #Pigpio raspberry pi object:
        self.pi = pi

        #Encoder GPIO pins:
        self.channelA = channelA
        self.channelB = channelB

        #Callback fn:
        self.callback = callback

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.pi.set_mode(channelA,pigpio.INPUT)
        self.pi.set_mode(channelB, pigpio.INPUT)

        self.pi.set_pull_up_down(channelA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(channelB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)


        def _pulse(self, gpio, level, tick):
            if gpio == self.channelA:
                self.levA = level
            else:
                self.levB = level;

            if gpio != self.lastGpio: #debounce
               self.lastGpio = gpio

            if gpio == self.channelA and level == 1:
                if self.levB == 1:
                    self.callback(1)
            elif gpio == self.channelB and level == 1:
                if self.levA == 1:
                    self.callback(-1)
            """
            Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
          """
            def cancel(self):
                self.cbA.cancel()
                self.cbB.cancel()





