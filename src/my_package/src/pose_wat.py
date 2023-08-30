#!/usr/bin/python3
import asyncio
import math
import time
import xml.etree.cElementTree as ET
from threading import Thread

import qtm
from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import threading
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from functools import partial

# add cf as a global variable
cf = None
br = None
x = 0
y = 0
z = 0
rot_x = 0
rot_y = 0
rot_z = 0
rot_w = 0
orientation_std_dev = 8.0e-3

# URI to the Crazyflie to connect to
uri2 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E7E7')
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E705')


uris = {
    'radio://0/80/2M/E7E7E7E705',
    'radio://0/80/2M/E7E7E7E702',
}

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

f = [[0.702924, -5.0, 0.0, 0.0, 0.0, 2.09341, -4.93634, 4.3354, -1.3706, -5.0, 0.0, 0.0, 0.0, 2.0332,-4.65933, 3.93089, -1.18242, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.631857, -4.92929, 0.18923, -0.00146811, -0.100923, -2.59816, 8.7133, -10.2228, 4.20902, -4.92929, 0.186121, -0.0210391, -0.103419, -2.40996, 8.0826, -9.41695, 3.84697, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.654202, -4.85365, 0.0865392, 0.0242634, 0.0242582, 1.0378, -3.53545, 4.20436, -1.74143, -4.86086, 0.0686168, 0.0206248, 0.0250913, 1.08914, -3.83672, 4.66972, -1.96411, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.671924, -4.77314, 0.141022, 0.00743965, -0.0119174, -0.84067, 2.72376, -3.14457, 1.26962, -4.79503, 0.113392, 0.012031, 0.0122757, -1.19834, 3.5987, -4.00822, 1.58178, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.663234, -4.68782, 0.124532, 0.00602001, 0.00487211, 0.432213, -1.4602, 1.73132, -0.712945, -4.73212, 0.0849234, 0.0019344, 0.00504772, 0.398137, -1.48093, 1.87972, -0.814003, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.675522, -4.59781, 0.138958, 5.34195e-05, 0.0021467, 0.117311, -0.443482, 0.564433, -0.243455, -4.67245, 0.0883912, 0.00425172, 0.00219364, -0.158546, 0.323122, -0.231298, 0.0519502, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.675962, -4.5032, 0.139321, 0.00237811, 0.000867428, 0.477573, -1.65232, 2.00044, -0.834668, -4.61632, 0.0774118, 0.00165124, 0.000916317, 0.218653, -0.894399, 1.18924, -0.527043, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.662578, -4.40414, 0.145421, 0.00112315, 0.000421042, 1.10263, -3.97473, 4.98388, -2.14471, -4.56407, 0.0724343, 0.00285409, 0.000407567, 0.385576, -1.57923, 2.12197, -0.953802, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.695423, -4.30077, 0.148502, 0.00145843, 0.000136779, 0.435874, -1.47855, 1.75241, -0.71456, -4.51599, 0.0644837, 0.00239395, 0.000159664, 0.102329, -0.487743, 0.681177, -0.306351, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.681247, -4.19328, 0.152449, 0.00112505, 0.000116007, 1.07518, -3.76804, 4.59428, -1.9226, -4.47238, 0.0575243, 0.00264988, 7.76805e-05, 0.27727, -1.13402, 1.50225, -0.66218, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.714604, -4.08186, 0.155421, 0.00103218, 2.76717e-05, 0.410718, -1.36075, 1.57357, -0.62557, -4.43354, 0.0498565, 0.00257671, 1.2873e-05, 0.654244, -2.48845, 3.1006, -1.29199, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.699644, -3.96671, 0.158484, 0.000951867, 0.000113629, 1.02957, -3.52164, 4.18651, -1.70734, -4.39976, 0.0263773, -0.00132882, -2.47192e-06, 1.87663, -6.52924, 7.84252, -3.22054, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.733503, -3.84808, 0.16051, 0.000594741, 0.000133797, 0.383531, -1.23532, 1.39009, -0.537971, -4.3713, 0.0171957, -0.00130659, -1.30728e-05, 1.40517, -4.6196, 5.26323, -2.05399, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.71777, -3.7262, 0.16351, 0.000995278, 0.000400982, 0.990168, -3.33181, 3.88082, -1.54782, -4.34844, 0.0131221, -0.00136471, 3.16922e-05, 1.17162, -3.93994, 4.59002, -1.8313, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.752124, -3.60135, 0.162969, 0.000418337, 0.000820722, 0.321806, -0.984759, 1.06271, -0.396356, -4.33144, 0.00926366, -0.00124805, -7.68982e-05, 0.602282, -1.94523, 2.17058, -0.828378, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.735631, -3.47382, 0.169431, 0.00243011, 0.00199511, 1.04405, -3.57981, 4.16956, -1.64831, -4.32052, 0.00468496, -0.00151915, 0.000179646, 0.27153, -0.898836, 1.0276, -0.401677, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.770472, -3.3439, 0.157978, 0.00468583, 0.00447489, 0.0318202, 0.045043, -0.148463, 0.0809912, -4.31592, 0.00165963, -0.00087783, -0.000409222, -0.0986969, 0.270747, -0.270133, 0.0948099, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.753228, -3.21192, 0.187491, 0.011408, 0.0107981, 1.53997, -5.82395, 7.03787, -2.81719, -4.31784, -0.00516711, -0.00234972, 0.000982477, -0.540694, 1.75029, -1.95198, 0.743751, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.75779, -3.07819, 0.118948, 0.0268933, 0.0215711, 0.330653, -0.541827, 0.267016, -0.028309, -4.3265, -0.00311129, 0.00114153, -0.0019759, -0.828369, 2.49082, -2.65861, 0.983915, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.910633, -2.94309, 0.241872, 0.0295892, -0.0904369, 0.563339, -2.30402, 2.63977, -0.944137, -4.34205, -0.0217435, -0.00575652, 0.0080817, -0.541125, 1.51642, -1.44303, 0.464597, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

f2 = [[0.702924, -5.0, 0.0, 0.0, 0.0, 2.09341, -4.93634, 4.3354, -1.3706, -5.0, 0.0, 0.0, 0.0, 2.0332, -4.65933, 3.93089, -1.18242, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.631857, -4.92929, 0.18923, -0.00146811, -0.100923, -2.59816, 8.7133, -10.2228, 4.20902, -4.92929, 0.186121, -0.0210391, -0.103419, -2.40996, 8.0826, -9.41695, 3.84697, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.654202, -4.85365, 0.0865392, 0.0242634, 0.0242582, 1.0378, -3.53545, 4.20436, -1.74143, -4.86086, 0.0686168, 0.0206248, 0.0250913, 1.08914, -3.83672, 4.66972, -1.96411, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.671924, -4.77314, 0.141022, 0.00743965, -0.0119174, -0.84067, 2.72376, -3.14457, 1.26962, -4.79503, 0.113392, 0.012031, 0.0122757, -1.19834, 3.5987, -4.00822, 1.58178, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.663234, -4.68782, 0.124532, 0.00602001, 0.00487211, 0.432213, -1.4602, 1.73132, -0.712945, -4.73212, 0.0849234, 0.0019344, 0.00504772, 0.398137, -1.48093, 1.87972, -0.814003, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.675522, -4.59781, 0.138958, 5.34195e-05, 0.0021467, 0.117311, -0.443482, 0.564433, -0.243455, -4.67245, 0.0883912, 0.00425172, 0.00219364, -0.158546, 0.323122, -0.231298, 0.0519502, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.675962, -4.5032, 0.139321, 0.00237811, 0.000867428, 0.477573, -1.65232, 2.00044, -0.834668, -4.61632, 0.0774118, 0.00165124, 0.000916317, 0.218653, -0.894399, 1.18924, -0.527043, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.662578, -4.40414, 0.145421, 0.00112315, 0.000421042, 1.10263, -3.97473, 4.98388, -2.14471, -4.56407, 0.0724343, 0.00285409, 0.000407567, 0.385576, -1.57923, 2.12197, -0.953802, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.695423, -4.30077, 0.148502, 0.00145843, 0.000136779, 0.435874, -1.47855, 1.75241, -0.71456, -4.51599, 0.0644837, 0.00239395, 0.000159664, 0.102329, -0.487743, 0.681177, -0.306351, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.681247, -4.19328, 0.152449, 0.00112505, 0.000116007, 1.07518, -3.76804, 4.59428, -1.9226, -4.47238, 0.0575243, 0.00264988, 7.76805e-05, 0.27727, -1.13402, 1.50225, -0.66218, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.714604, -4.08186, 0.155421, 0.00103218, 2.76717e-05, 0.410718, -1.36075, 1.57357, -0.62557, -4.43354, 0.0498565, 0.00257671, 1.2873e-05, 0.654244, -2.48845, 3.1006, -1.29199, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.699644, -3.96671, 0.158484, 0.000951867, 0.000113629, 1.02957, -3.52164, 4.18651, -1.70734, -4.39976, 0.0263773, -0.00132882, -2.47192e-06, 1.87663, -6.52924, 7.84252, -3.22054, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.733503, -3.84808, 0.16051, 0.000594741, 0.000133797, 0.383531, -1.23532, 1.39009, -0.537971, -4.3713, 0.0171957, -0.00130659, -1.30728e-05, 1.40517, -4.6196, 5.26323, -2.05399, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.71777, -3.7262, 0.16351, 0.000995278, 0.000400982, 0.990168, -3.33181, 3.88082, -1.54782, -4.34844, 0.0131221, -0.00136471, 3.16922e-05, 1.17162, -3.93994, 4.59002, -1.8313, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.752124, -3.60135, 0.162969, 0.000418337, 0.000820722, 0.321806, -0.984759, 1.06271, -0.396356, -4.33144, 0.00926366, -0.00124805, -7.68982e-05, 0.602282, -1.94523, 2.17058, -0.828378, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.735631, -3.47382, 0.169431, 0.00243011, 0.00199511, 1.04405, -3.57981, 4.16956, -1.64831, -4.32052, 0.00468496, -0.00151915, 0.000179646, 0.27153, -0.898836, 1.0276, -0.401677, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.770472, -3.3439, 0.157978, 0.00468583, 0.00447489, 0.0318202, 0.045043, -0.148463, 0.0809912, -4.31592, 0.00165963, -0.00087783, -0.000409222, -0.0986969, 0.270747, -0.270133, 0.0948099, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.753228, -3.21192, 0.187491, 0.011408, 0.0107981, 1.53997, -5.82395, 7.03787, -2.81719, -4.31784, -0.00516711, -0.00234972, 0.000982477, -0.540694, 1.75029, -1.95198, 0.743751, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.75779, -3.07819, 0.118948, 0.0268933, 0.0215711, 0.330653, -0.541827, 0.267016, -0.028309, -4.3265, -0.00311129, 0.00114153, -0.0019759, -0.828369, 2.49082, -2.65861, 0.983915, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.910633, -2.94309, 0.241872, 0.0295892, -0.0904369, 0.563339, -2.30402, 2.63977, -0.944137, -4.34205, -0.0217435, -0.00575652, 0.0080817, -0.541125, 1.51642, -1.44303, 0.464597, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

def callback(data, cf_n, br_n):
    # This function gets called every time a new message arrives
    x = data.transform.translation.x
    y = data.transform.translation.y
    z = data.transform.translation.z
    rot_x = data.transform.rotation.x
    rot_y = data.transform.rotation.y
    rot_z = data.transform.rotation.z
    rot_w = data.transform.rotation.w

    br_n.sendTransform(data)

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "vicon/world"
    t.child_frame_id = "map"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    br_n.sendTransform(t)

    # print(f'{rospy.get_caller_id()} I received {x, y, z, rot_x, rot_y, rot_z, rot_w}')

    # Set up a callback to handle data from QTM
    cf_n.extpos.send_extpose(x, y, z, rot_x, rot_y, rot_z, rot_w)
    print(f"cf_n: {cf_n}, br_n: {br_n}, x: {x}, y: {y}, z: {z}")
    # cf_n.extpos.send_extpos(x, y, z)

    # Perform additional operations here
    # also do cf send_extpose with x,y... from data which is a TransformStamped

def create_callback(cf_n, i):
    def callback(data):
        # This function gets called every time a new message arrives
        x = data.transform.translation.x
        y = data.transform.translation.y
        z = data.transform.translation.z
        rot_x = data.transform.rotation.x
        rot_y = data.transform.rotation.y
        rot_z = data.transform.rotation.z
        rot_w = data.transform.rotation.w

        # Set up a callback to handle data from QTM
        # cf_n.extpos.send_extpose(x, y, z, rot_x, rot_y, rot_z, rot_w)
        # print(f"cf_n: {cf_n}, i: {i}, x: {x}, y: {y}, z: {z}")
        cf_n.extpos.send_extpos(x, y, z)
    return callback

    # Perform additional operations here
    # also do cf send_extpose with x,y... from data which is a TransformStamped
def listener():
    # Initialize the node with a name



    # Spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def adjust_orientation_sensitivity(cf_n):
    cf_n.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf_n):
    cf_n.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf_n.param.set_value('locSrv.extQuatStdDev', 0.06)



def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    trajectory_mem.write_data_sync()
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration

def run_sequence_basic(cf_n):
    commander = cf_n.high_level_commander

    print("Doing takeoff:")
    commander.takeoff(1.0, 2.0)
    time.sleep(6.0)
    print("Doing land")
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

# def run_sequence(cf):
#     commander = cf.high_level_commander

#     commander.takeoff(1.0, 2.0)
#     time.sleep(6.0)
#     commander.land(0.0, 2.0)
#     time.sleep(2)
#     commander.stop()

# def send_extpose_rot_matrix(cf, x, y, z, rot):
#     """
#     Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
#     rotaton matrix. This is going to be forwarded to the Crazyflie's
#     position estimator.
#     """
#     quat = Rotation.from_matrix(rot).as_quat()

#     cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=50)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')




    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def reset_estimator(cf_n):
    cf_n.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf_n.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.1)
    cf_n.param.set_value('locSrv.extPosStdDev', '0.01')
    # wait_for_position_estimator(cf)

def crazyflie_operation(cf_n, topic):
    rospy.Subscriber(topic, TransformStamped, lambda data: callback(data, cf_n))
    print("Subscribed to topic")

    listener_thread = threading.Thread(target=listener)
    listener_thread.start()
    time.sleep(1)

    log_config = LogConfig(name='Position', period_in_ms=50)
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')

    cf_n.log.add_config(log_config)
    def log_callback(timestamp, data, log_config):
        pass
    log_config.data_received_cb.add_callback(log_callback)

    log_config.start()

    cf_n.param.set_value('commander.enHighLevel', '1')
    adjust_orientation_sensitivity(cf_n)
    activate_kalman_estimator(cf_n)
    # duration = upload_trajectory(cf_n, 1, figure8)
    reset_estimator(cf_n)
    # run_sequence(cf_n, 1, duration)
    # print(f'duration: {duration}')
    print("Running basic sequence:")
    run_sequence_basic(cf_n) 

def crazyflie_operation_simple(cf_n):
    print("Running simple sequence:")
    run_sequence_basic(cf_n) 


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    trajectory_id = 1

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf1:
        with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2:
            cf1 = scf1.cf
            val = cf1.param.get_value('kalman.resetEstimation')
            print(f"Val: {val}")
            cf2 = scf2.cf
            val = cf2.param.get_value('kalman.resetEstimation')
            print(f"Val2: {val}")

            cfs = [cf1, cf2]
            # cfs = [cf1, cf2]
            topics = ['/vicon/kris_crazyflie/kris_crazyflie', '/vicon/CF1/CF1']

            # cfs = [cf1]
            # topics = ['/vicon/kris_crazyflie/kris_crazyflie']

            # # cfs = [cf2]
            # # topics = ['/vicon/CF1/CF1']



            print("Starting Listener" )

                # connect to crazytflie (cf object) through crazyradio
            rospy.init_node('kris_crazyflie_publisher', anonymous=True)
            print("Initialized node")
            
            brs = [tf2_ros.TransformBroadcaster(), tf2_ros.TransformBroadcaster()]

            threads = []

            # for i, cf_n in enumerate(cfs):
            #     t = threading.Thread(target=crazyflie_operation, args=(cf_n, topics[i]))
            #     threads.append(t)
            #     t.start()

            # # Optionally, wait for all threads to complete:
            # for t in threads:
            #     t.join()


            for i, cf_n in enumerate(cfs):
                print(f"cf_n: {cfs[i]}, br_n: {brs[i]}, i: {i}")
                # Subscribe to the 'transform_topic' topic and register the callback
                rospy.Subscriber(topics[i], TransformStamped, lambda data: callback(data, cf_n, brs[i]))         
                print("Subscribed to topic")

        
            
            listener_thread = threading.Thread(target=listener)
            listener_thread.start()
            time.sleep(1)

            

            for i, cf_n in enumerate(cfs):

                log_config = LogConfig(name='Position', period_in_ms=50)
                log_config.add_variable('stateEstimate.x', 'float')
                log_config.add_variable('stateEstimate.y', 'float')
                log_config.add_variable('stateEstimate.z', 'float')

                cf_n.log.add_config(log_config)
                def log_callback(timestamp, data, log_config):
                    # print(f'Estimate: {timestamp}: {data}')
                    pass
                log_config.data_received_cb.add_callback(log_callback)

                log_config.start()

                cf_n.param.set_value('commander.enHighLevel', '1')
                adjust_orientation_sensitivity(cf_n)
                activate_kalman_estimator(cf_n)
                # duration = upload_trajectory(cf_n, 1, figure8)
                reset_estimator(cf_n)

            for i, cf_n in enumerate(cfs):
                commander = cf_n.high_level_commander
                commander.takeoff(1, 2.0)

            time.sleep(4.0)

            for i, cf_n in enumerate(cfs):
                commander = cf_n.high_level_commander
                commander.land(0.0, 2.0)
            time.sleep(2)
            for i, cf_n in enumerate(cfs):
                commander = cf_n.high_level_commander
                commander.stop()


            # for i, cf_n in enumerate(cfs):
            #     duration = upload_trajectory(cf_n, 1, figure8)
            # for i, cf_n in enumerate(cfs):    
            #     reset_estimator(cf_n)
            # for i, cf_n in enumerate(cfs):
            #     run_sequence(cf_n, 1, duration)

            # for i, cf_n in enumerate(cfs):
            #     commander = cf.high_level_commander
            #     commander.takeoff(1.0, 2.0)

            # time.sleep(3.0)

            # for i, cf_n in enumerate(cfs):     
            #     commander = cf.high_level_commander
            #     relative = True
            #     commander.start_trajectory(trajectory_id, 1.0, relative)

            # time.sleep(duration)

            # for i, cf_n in enumerate(cfs): 
            #     commander = cf.high_level_commander     
            #     commander.land(0.0, 2.0)
            # time.sleep(2)
            # for i, cf_n in enumerate(cfs): 
            #     commander = cf.high_level_commander
            #     commander.stop()


            # for i, cf_n in enumerate(cfs):
            #     t = threading.Thread(target=crazyflie_operation_simple, args=(cf_n, ))
            #     threads.append(t)
            #     t.start()

            # # Optionally, wait for all threads to complete:
            # for t in threads:
            #     t.join()
            #     run_sequence(cf_n, 1, duration)
            #     print(duration)
            #     run_sequence_basic(cf_n)




    # once sending pose is done, add code to send waypoints to high level commander