#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
Author: LI Jinjie
File: tools_com.py
Date: 2021/7/8 10:14
LastEditors: LI Jinjie
LastEditTime: 2021/7/8 10:14
Description: 可以被外部调用的xbee命令
'''

import time
import random

from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.devices import XBeeDevice


def find_devices(device, finding_time):
    # 默认已经open()过设备了
    print(" +--------------------------------------+")
    print(" | XBee Python Library Discover Devices |")
    print(" +--------------------------------------+\n")

    # device = XBeeDevice(PORT, BAUD_RATE)

    try:
        xbee_network = device.get_network()

        xbee_network.set_discovery_timeout(finding_time)  # 15 seconds.

        xbee_network.clear()

        # Callback for discovered devices.
        def callback_device_discovered(remote):
            print("Device discovered: %s" % remote)

        # Callback for discovery finished.
        def callback_discovery_finished(status):
            if status == NetworkDiscoveryStatus.SUCCESS:
                print("Discovery process finished successfully.")
            else:
                print("There was an error discovering devices: %s" % status.description)

        xbee_network.add_device_discovered_callback(callback_device_discovered)

        xbee_network.add_discovery_process_finished_callback(callback_discovery_finished)

        xbee_network.start_discovery_process()

        print("Discovering remote XBee devices...")

        while xbee_network.is_discovery_running():
            time.sleep(0.1)

        device_list = xbee_network.get_devices()
        return device_list

    except:
        print("Something wrong when discovering devices")


car_nums = [[1, 1, 3],
            [0, 2, 3],
            [2, 2, 1]]


def generate_number(t):
    """
    调用一次，更新一次车辆的数字。更新完后保持在car_0, car_1和car_2三个变量中。
    """
    global car_nums
    # sum_num = 5
    # car_0 = random.randint(0, 3)
    # car_1 = random.randint(0, 3)
    # car_2 = sum_num - car_1 - car_0
    car_0 = car_nums[t][0]
    car_1 = car_nums[t][1]
    car_2 = car_nums[t][2]

    return car_0, car_1, car_2

# if __name__ == '__main__':
#     main()