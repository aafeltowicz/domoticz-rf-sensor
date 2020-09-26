#!/usr/bin/env python3
"""
<plugin key="RFSensor" name="RFSensor" author="norf" version="0.0.1">
    <description>
    Plugin listens on selected GPIO pin for RF codes and, if spots any,
    sends them to text sensor.
    Based on: https://github.com/milaq/rpi-rf/blob/master/rpi_rf/rpi_rf.py
    </description>
    <params>
        <param field="Mode1" label="RF receiver GPIO port" width="50px" required="true" default="17">
        </param>
        <param field="Mode3" label="RF codes" width="1000px" default="">
        </param>
        <param field="Mode2" label="RF protocol" width="250px">
            <options>
                <option label="Protocol 1" value="0" default="true"/>
                <option label="Protocol 2" value="1"/>
                <option label="Protocol 3" value="2"/>
                <option label="Protocol 4" value="3"/>
                <option label="Protocol 5" value="4"/>
                <option label="Protocol 6" value="5"/>
            </options>
        </param>
    </params>
</plugin>
"""
import Domoticz
import time
from RPi import GPIO
from collections import namedtuple

MAX_CHANGES = 67

Protocol = namedtuple('Protocol',
                      ['pulselength',
                       'sync_high', 'sync_low',
                       'zero_high', 'zero_low',
                       'one_high', 'one_low'])

PROTOCOLS = (Protocol(350, 1, 31, 1, 3, 3, 1),
             Protocol(650, 1, 10, 1, 2, 2, 1),
             Protocol(100, 30, 71, 4, 11, 9, 6),
             Protocol(380, 1, 6, 1, 3, 3, 1),
             Protocol(500, 6, 14, 1, 2, 2, 1),
             Protocol(200, 1, 10, 1, 5, 1, 1))

class RFSensor:
    def rx_callback(self, gpio):
        timestamp = int(time.perf_counter() * 1000000)
        duration = timestamp - self._rx_last_timestamp

        if duration > 5000:
            if abs(duration - self._rx_timings[0]) < 200:
                self._rx_repeat_count += 1
                self._rx_change_count -= 1
                if self._rx_repeat_count == 2:
                    code = self._rx_waveform()
                    code_str = str(code)
                    if code > 0 and code_str in self.rf_codes:
                        self.device.Update(nValue=code, sValue=code_str)
                    self._rx_repeat_count = 0
            self._rx_change_count = 0

        if self._rx_change_count >= MAX_CHANGES:
            self._rx_change_count = 0
            self._rx_repeat_count = 0
        self._rx_timings[self._rx_change_count] = duration
        self._rx_change_count += 1
        self._rx_last_timestamp = timestamp

    def _rx_waveform(self):
        code = 0
        delay = int(self._rx_timings[0] / self.rf_protocol.sync_low)
        delay_tolerance = delay * .8

        for i in range(1, self._rx_change_count, 2):
            if (abs(self._rx_timings[i] - delay * self.rf_protocol.zero_high) < delay_tolerance and
                abs(self._rx_timings[i+1] - delay * self.rf_protocol.zero_low) < delay_tolerance):
                code <<= 1
            elif (abs(self._rx_timings[i] - delay * self.rf_protocol.one_high) < delay_tolerance and
                  abs(self._rx_timings[i+1] - delay * self.rf_protocol.one_low) < delay_tolerance):
                code <<= 1
                code |= 1
            else:
                return 0

        if self._rx_change_count < 6:
            return 0

        return code

    def onStart(self):
        self._rx_timings = [0] * (MAX_CHANGES + 1)
        self._rx_last_timestamp = 0
        self._rx_change_count = 0
        self._rx_repeat_count = 0

        self.gpio = int(Parameters["Mode1"])
        self.rf_protocol = PROTOCOLS[int(Parameters["Mode2"])]
        self.rf_codes = Parameters["Mode3"].split(',')
        self.device = None
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio, GPIO.IN)
        GPIO.add_event_detect(self.gpio, GPIO.BOTH)
        GPIO.add_event_callback(self.gpio, self.rx_callback)

        if (len(Devices) == 0):
            self.device = Domoticz.Device(Name='RFSensor', Unit=1, Type=243, Subtype=19, Used=1)
            self.device.Create()
        else:
            self.device = Devices[1]

    def onStop(self):
        GPIO.cleanup()

global _plugin
_plugin = RFSensor()

def onStart():
    global _plugin
    _plugin.onStart()

def onStop():
    global _plugin
    _plugin.onStop()