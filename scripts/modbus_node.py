#!/usr/bin/env python3
import rospy
from pymodbus.client.sync import ModbusTcpClient
from ethernet_remote_io_module.msg import WriteCoil, WriteCoils, ReadDigitalInputs
from rospy import loginfo, logerr, logwarn, loginfo_once
import re
from typing import Union, List

def validate_ip(ip: str) -> bool:
    """[Validating IP addres]
    Args:
        ip (str): [IP address]
    Returns:
        bool: [True if validation succeeded, False otherwise]
    """
    regex = "^((25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])\.){3}(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])$"
    if(re.search(regex, ip)):
        return True
    else:
        return False

def shutdown_hook() -> None:
    """[Executes on shutdown, sets all DOs to False and closes modbus connection]
    """
    global connect_flag, client
    if connect_flag:
        flag = client.write_coils(address=0, values=[False]*8, unit=0x01)
        if flag:
            client.close()
    else:
        pass

class RemoteIO:
    
    """
        ROS wrapper for modbus client of remote IO module

        Note: wrapper reads/writes only digital inputs/outputs
    """

    def __init__(self,client: ModbusTcpClient) -> None:
        """[Initialize a wrapper instance]

        Args:
            client (ModbusTcpClient): [instanse of a modbus tcp client]
        
        Note: other parameters retrieves from rosparam server 
        """
        self._client = client
        self.freq = rospy.get_param('~rate', default=10)
        self.__rate = rospy.Rate(self.freq)
        self.__pub_topic = rospy.get_param('~inputs_topic_name', default='read_digital_inputs')
        self.__single_sub_topic = rospy.get_param('~single_coil_sub_topic_name', default='write_coil')
        self.__multi_sub_topic = rospy.get_param('~milti_coils_sub_topic_name', default='write_coils')
        self.__inputs_publisher = rospy.Publisher(self.__pub_topic, ReadDigitalInputs, queue_size=10, latch=True)
        self.__single_output_subscriber = rospy.Subscriber(self.__single_sub_topic, WriteCoil, callback=self.__write_coil_clbk)
        self.__multi_output_subscriber = rospy.Subscriber(self.__multi_sub_topic, WriteCoils, callback=self.__write_coils_clbk)
        self.__publish = self.__start_publish()
        
    def set_coil(self, address: int, value: bool, unit=0x01) -> bool:
        """
        Args:
            address (int): [The address to write to]
            value (bool): [The value to write to the specified address]
            unit (hexadecimal, optional): [The slave unit this request is targeting]. Defaults to 0x01.

        Returns:
            bool: [A deferred response handle ]
        """
        res = self._client.write_coil(address=address, value=value, unit=unit)
        if not res.isError():
            loginfo(f'Successfully set to DO{address+1} Value = {value}')
            return True
        else:
            logerr(f'Unable to set to DO{address+1}')
            return False

    def set_coils(self, start_address: int, end_address: int, value: bool, unit=0x01) -> bool:
        """
        Args:
            start_address (int): [The starting address to write to ]
            end_address (int): [The last address to write to]
            value (bool): [The value to write to the specified address]
            unit (hexadecimal, optional): [The slave unit this request is targeting]. Defaults to 0x01.

        Returns:
            bool: [A deferred response handle ]
        """
        res = self._client.write_coils(address=start_address, values=[value]*end_address, unit=unit)
        if not res.isError():
            loginfo(f'Successfully set to DO{start_address+1}- DO{end_address} Values = {value}')
            return True
        else:
            logerr(f'Unable to set to DO{start_address}- DO{end_address}')
            return False

    def __write_coil_clbk(self, msg: WriteCoil) -> None:
        """[Callback function for Subscriber]
        Args:
            msg (WriteCoil): [data]
        """
        address = msg.address
        value = msg.value
        self.set_coil(address=address, value=value)

    def __write_coils_clbk(self, msg: WriteCoils) -> None:
        """[Callback function for Subscriber]
        Args:
            msg (WriteCoils): [data]
        """
        start_address = msg.address
        end_address = msg.last
        value = msg.value
        self.set_coils(start_address=start_address, end_address=end_address, value=value)

    
    def get_inputs(self, address: int=0, count: int=8, unit=0x01) -> Union[List[bool], None]:
        """
        Args:
            address (int, optional): [The starting address to read from]. Defaults to 0.
            count (int, optional): [The number of discretes to read]. Defaults to 8.
            unit (hexadecimal, optional): [The slave unit this request is targeting]. Defaults to 0x01.

        Returns:
            Union[List[bool], None]: [If response is ok returns states of DINs otherwise None]
        """
        res = self._client.read_discrete_inputs(address=address, count=count, unit=unit)
        if not res.isError():
            loginfo_once(f'Reading inputs DIN{address+1} - DIN{count}')
            if all([type(val) == bool for val in res.bits[address:count]]):
                return res.bits[address:count]
            else:
                return None
        else:
            logerr(f'Unable to read inputs DIN{address+1} - DIN{count}')
            return None

    def __start_publish(self) -> None:
        """[Wraps DINs states to ros msg and publishes to topic]
        """
        while not rospy.is_shutdown():
            din_states = self.get_inputs()
            pub_msg = ReadDigitalInputs()
            if din_states is not None and len(din_states) == 8:
                try:
                    pub_msg.din_1 = din_states[0]
                    pub_msg.din_2 = din_states[1]
                    pub_msg.din_3 = din_states[2]
                    pub_msg.din_4 = din_states[3]
                    pub_msg.din_5 = din_states[4]
                    pub_msg.din_6 = din_states[5]
                    pub_msg.din_7 = din_states[6]
                    pub_msg.din_8 = din_states[7]
                    self.__inputs_publisher.publish(pub_msg)
                    self.__rate.sleep()
                except Exception as e:
                    logerr(e)
            else:
                logerr('Unable to read inputs')

if __name__ == '__main__':
    rospy.init_node('remote_io_node', anonymous=True, disable_signals=True)
    rospy.on_shutdown(shutdown_hook)
    connect_flag = False
    # get private parameters 
    if not rospy.has_param('~ip_address'):
        logwarn('IP addrees of modbus server not specified')
        logwarn('Try to connect to default address')
    if not rospy.has_param('~port'):
        logwarn('Modbus server port not specified')
        logwarn('Using default port')
    ip_address = rospy.get_param('~ip_address', default='192.168.1.110')
    port = rospy.get_param('~port', default=502)
    # validating IP address 
    if validate_ip(ip_address):
        loginfo(f'Modbus Server IP= {ip_address} port= {port}')
        loginfo(f'Connecting...')
        client = ModbusTcpClient(host=ip_address, port=port)
        connect_flag = client.connect()
        # check connection
        if connect_flag:
            loginfo(f'Connected successfully')
            remote_io = RemoteIO(client)
        else:
            logerr(f"Can't connect to {ip_address}")
            rospy.signal_shutdown('Wrong Modbus Server IP address or port')
    else:
        logerr(f'IP address validation error. Wrong IP {ip_address}')
        rospy.signal_shutdown('IP address validation error')
