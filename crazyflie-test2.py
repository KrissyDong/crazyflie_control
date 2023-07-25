import logging
import time
from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class Main:
    def __init__(self, link_uri):
        self._cf = Crazyflie(rw_cache='./cache')
        self._height = 0

        with SyncCrazyflie(link_uri, cf=self._cf) as scf:

            try:
                self._reset_estimator(scf)
            
                # Create and add a log config for height estimation
                height_log_config = LogConfig(name='HeightEstimation', period_in_ms=100)
                height_log_config.add_variable('stateEstimate.z', 'float')
                
                with SyncLogger(scf, height_log_config) as logger:
                    
                    for log_entry in logger:
                        timestamp, data, log_conf = log_entry
                        self._height = data['stateEstimate.z']

                        with open('log.txt', 'a') as f:
                            self._take_off(scf, log_entry, f)
                            self._hover(scf, log_entry, f)
                            self._land(scf, log_entry, f)
                        print("Ran test with crazyflie!")
                        break

            except Exception as e:
                print(e)

    def _reset_estimator(self, scf):
        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

    def _take_off(self, scf, log_entry, f):
        print("taking off")
        for y in range(10):
            timestamp, data, log_conf = log_entry
            self._height = data['stateEstimate.z']
            f.write(f"Height: {self._height} m\n")
            scf.cf.commander.send_hover_setpoint(0, 0, 0, y / 10)
            time.sleep(0.3)

    def _hover(self, scf,log_entry,  f):
        print("hovering")
        start = time.time()
        while time.time() - start < 10:
            timestamp, data, log_conf = log_entry
            self._height = data['stateEstimate.z']
            f.write(f"Height: {self._height} m\n")
            scf.cf.commander.send_hover_setpoint(0, 0, 0, 1.0)
            time.sleep(0.3)

    def _land(self, scf, log_entry, f):
        print("landing")
        for y in range(10):
            timestamp, data, log_conf = log_entry
            self._height = data['stateEstimate.z']
            f.write(f"Height: {self._height} m\n")
            scf.cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 10)
            time.sleep(0.3)

        scf.cf.commander.send_stop_setpoint()

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    le = Main('radio://0/80/2M/E7E7E7E7E7')
