#!/usr/bin/env python3
# ====================================================================================================================================
# @file       sick_generic_device_finder.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 26th April 2022 11:27:42 am
# @modified   Thursday, 26th May 2022 2:27:09 am
# @project    engineering-thesis
# @brief      Helper python script 
#    
#    
# @note The script has been based on <sick_scan>/tools/sick_generic_device_finder/sick_generic_device_finder.py tool
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

import socket
import random
import xml.etree.ElementTree as ET
from threading import Thread, Event
import sys

# ============================================================ Constants =========================================================== #

# Broadcast address of the UDP network
UDP_IP = "192.168.0.255"
# Broadcast port of the UDP network
UDP_PORT = 30718

# UDP message to be sent
RANDOM_KEY = random.randrange(2**32 - 1)
MESSAGE    = bytes.fromhex('10000008ffffffffffffc8f4b6270102c0a8007effffff00')
MESSAGE    = MESSAGE.replace(bytes.fromhex('c8f4b627'),RANDOM_KEY.to_bytes(4, byteorder='big', signed=False))

# Symbolic name meaning all available interfaces
HOST = ''                 
# Debug configuration
DEBUGMSGENABLED = False
# Timeout in seconds
ANSWERTIMEOUT=10 

# ========================================================= Globlal objects ======================================================== #

# Disable stderr
sys.stderr = object
# Data buffer for UDP response
rawData =[]
# Event object used to send signals from one thread to another
stop_event = Event()

# ============================================================ Functions =========================================================== #

def print_info():

    """Prints initial information about the tool to the user"""

    # Print opening message
    print(
        r'If you have more than one network interface it may happen that no scanner is found'
        r' because the broadcast ip address does not match and the discovery packets are sent to the wrong interface.'
        r'To fix this problem change the parameter <UDP_IP = "192.168.0.255"> '
        r'to the broadcast address that ifconfig returns for your network interface e.g. "192.168.178.255".'
    )
    
    # Print UDP configruation
    print("UDP target IP: {ip}".format(ip=UDP_IP))
    print("UDP target port: {port}".format(port=UDP_PORT))
    # Print debug configruation
    if(DEBUGMSGENABLED):
        print("Message: {message}".format(message=MESSAGE))
    # Print timeout configruation
    print("The scan result is available in "+str(ANSWERTIMEOUT)+" seconds.")
    # Print random key configruation
    print("Random key: {random_key}".format(random_key=RANDOM_KEY.to_bytes(4, byteorder='big', signed=False)))


def make_unique(input):

    """Filters `input` list to contain only unique items"""

    output = []
    for x in input:
        if x not in output:
            output.append(x)
    return output


def read_scanners_data():
    
    """Filters `input` list to contain only unique items"""
    
    global rawData

    # Create communication socker
    sock = socket.socket(
        # Internet
        socket.AF_INET,
        # UDP
        socket.SOCK_DGRAM
    )  

    # Configure the socket
    sock.settimeout(ANSWERTIMEOUT - 5)
    sock.bind((HOST, UDP_PORT))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    # Send request to the device
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

    # Read data from the socket
    while True:

        # Buffer size is 1024 bytes
        DATA, addr = sock.recvfrom(2048) 
        # Print debug info
        if(DEBUGMSGENABLED):
            print("received message: {data}".format(data=DATA))
        # Store data into the global buffer
        rawData.append(DATA)

        # If stop requested (timeout)
        if stop_event.is_set():

            # Close the socket
            sock.close()
            # Print timeout info
            print("Wait time out")
            if (DEBUGMSGENABLED):
                print(rawData)
            # Break routine
            break


def parse_response_data(DataListInput):

    """Parses response from the device"""

    # Filter incoming data
    DataList=make_unique(DataListInput)

    # Iterate over data elements (XML string)
    for Data in DataList:

        # Check if a valid XML data is given
        if b"<?xml"  in Data:
            
            # Print debug info
            if (DEBUGMSGENABLED):
                print('Scanner found:')

            # Find start of the XML string
            XML_INDEX=Data.find(b'<?xml')
            # Parse data
            XML_DATA=Data[XML_INDEX:]

            # Print debug info
            if (DEBUGMSGENABLED):
                print(XML_DATA)

            # Find root of the XML string            
            root = ET.fromstring(XML_DATA)
            # Prepare values to be found
            ipAddress = "???"
            IPMask= "???"
            IPGateway = "???"
            DeviceType = "???"
            SerialNumber = "???"
            DHCPClientEnabled= "???"
            FirmwareVersion= "????"

            # Iterate over XML tags
            for child in root:

                # Parse key and value
                k = child.attrib['key']
                v = child.attrib['value']
                # Parse target values
                if (k == 'IPAddress'):
                    ipAddress=v
                if (k == 'IPMask'):
                    IPMask=v
                if (k== 'IPGateway'):
                    IPGateway=v
                if (k=='DeviceType'):
                    DeviceType=v
                if (k=='SerialNumber'):
                    SerialNumber=v
                if (k=='DHCPClientEnabled'):
                    DHCPClientEnabled=v
                if (k=='FirmwareVersion'):
                    FirmwareVersion=v	

            # Print dbeug message
            print(
                'Device type = '  + DeviceType        + '\n'
                'SN = '           + SerialNumber      + '\n'
                'IP = '           + ipAddress         + '\n'
                'IPMask = '       + IPMask            + '\n'
                'Gatway = '       + IPGateway         + '\n'
                'DHCPEnable = '   + DHCPClientEnabled + '\n'
                'Firmwarevers.= ' + FirmwareVersion   + '\n'
            )


# ============================================================= Script ============================================================= #

def main():

    # Print opening message
    print_info()
    # Create another Thread to read data from the LIDAR
    action_thread= Thread(target=read_scanners_data)

    # Start the thread and wait 5 seconds before the code continues to execute.
    action_thread.start()
    action_thread.join(timeout=ANSWERTIMEOUT + 1)

    # Send a signal that the other thread should stop
    stop_event.set()
    # Parse response data
    parse_response_data(rawData)
    
    sys.exit()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
