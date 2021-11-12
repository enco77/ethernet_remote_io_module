#!/usr/bin/env python3
import rospy
from pymodbus.client.sync import ModbusTcpClient
from ethernet_remote_io_module.msg import WriteCoil, WriteCoils, ReadDigitalInputs
from rospy import loginfo, logerr, loginfo_once, logwarn
from rospy import ROSException, ROSInterruptException

def set_coil(client, address, value, unit=0x01):
    res = client.write_coil(address=address, value=value, unit=unit)
    if not res.isError():
        loginfo(f'Successfully set to DO{address+1} Value = {value}')
        return True
    else:
        logerr(f'Unable to set to DO{address+1}')
        return False

def set_coils(client, start_address, end_address, value, unit=0x01):
    res = client.write_coils(address=start_address, values=[value]*end_address, unit=unit)
    if not res.isError():
        loginfo(f'Successfully set to DO{start_address+1}- DO{end_address} Values = {value}')
        return True
    else:
        logerr(f'Unable to set to DO{start_address}- DO{end_address}')
        return False

def get_inputs(client, address=0, count=8, unit=0x01):
    res = client.read_discrete_inputs(address=address, count=count, unit=unit)
    if not res.isError():
        loginfo_once(f'Reading inputs DIN{address+1} - DIN{count}')
        return res.bits[address:count]
    else:
        logerr(f'Unable to read inputs DIN{address+1} - DIN{count}')
        return None

def write_coil_clbk(msg, *args):
    address = msg.address
    value = msg.value
    set_coil(args[0][0], address=address, value=value)


def write_coils_clbk(msg, *args):
    start_address = msg.address
    end_address = msg.last
    value = msg.value
    set_coils(args[0][0], start_address=start_address, end_address=end_address, value=value)

def shutdown_hook():
    if connect_flag:
        flag = set_coils(client=client, start_address=0, end_address=8, value=False)
        if flag:
            client.close()
    else:
        pass

if __name__ == '__main__':
    rospy.init_node('remote_io_node', anonymous=True, disable_signals=True)
    rospy.on_shutdown(shutdown_hook)
    rate = rospy.Rate(10)
    # get private parameters 
    if not rospy.has_param('~ip_address'):
        logwarn('IP addrees of modbus server not specified')
        logwarn('Try to connect to default address')
    if not rospy.has_param('~port'):
        logwarn('Modbus server port not specified')
        logwarn('Using default port')
    ip_address = rospy.get_param('~ip_address', default='192.168.1.110')
    port = rospy.get_param('~port', default=502)
    loginfo_once(f'Modbus Server IP= {ip_address} port= {port}')
    loginfo_once(f'Connecting...')
    client = ModbusTcpClient(host=ip_address, port=port)
    connect_flag = client.connect()
    if connect_flag:
        try:
            loginfo_once(f'Connected successfully')
            din_publisher = rospy.Publisher('read_digital_inputs', ReadDigitalInputs, queue_size=10)
            rospy.Subscriber('write_coil', WriteCoil, callback=write_coil_clbk, callback_args=(client,))
            rospy.Subscriber('write_coils', WriteCoils, callback=write_coils_clbk, callback_args=(client,))

            while not rospy.is_shutdown():
                din_states = get_inputs(client=client)
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
                        din_publisher.publish(pub_msg)
                        rate.sleep()
                    except Exception as e:
                        logerr(e)
                else:
                    logerr('Unable to read inputs')
            else:
                flag = set_coils(client=client, start_address=0, end_address=8, value=False)
                if flag:
                    client.close()
        except (KeyboardInterrupt, ROSException, ROSInterruptException): 
            shutdown_hook()
    else:
        logerr(f"Can't connect to {ip_address}")
        rospy.signal_shutdown('Wrong Modbus Server IP address or port')
