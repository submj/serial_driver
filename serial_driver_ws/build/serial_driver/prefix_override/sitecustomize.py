import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/submj/Desktop/serial_driver/serial_driver_ws/install/serial_driver'
