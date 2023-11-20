# -*- coding: utf-8 -*-

import serial
import time 

class Setrac_serial(object):
    def __init__(self,MODEL,COM_PORT,BAUD_RATE):
        self.COM_PORT=COM_PORT
        self.MODEL=MODEL
        print("Connecting to %s" %(self.COM_PORT))
        flag =False
        while flag==False:
            
            try:
                print("try to connect %s" %self.COM_PORT)
                self.ser = serial.Serial(self.COM_PORT, BAUD_RATE, timeout=0.050)
                flag = True
            except:
                print("Failed connected to  %s" %(self.COM_PORT))
                time.sleep(3)

        else :
            print("successfully connected to %s" %(self.COM_PORT))
            
        #command_id
        self.CID={
        "ST_CID_INVALID" : "00",
        # System Messages
        "ST_CID_SYS_ALIVE" :  "01",                			# Command sent to receive a simple alive message from the beacon.  
        "ST_CID_SYS_INFO" :  "02",                 			# Command sent to receive hardware & firmware identification information.  
        "ST_CID_SYS_REBOOT" :  "03",               			# Command sent to soft reboot the beacon.  
        "ST_CID_SYS_ENGINEERING" :  "04",          			# Command sent to perform engineering actions.  
        # Firmware Programming Messages
        "ST_CID_PROG_INIT" :  "0D",                			# Command sent to initialise a firmware programming sequence.  
        "ST_CID_PROG_BLOCK" :  "0E",               			# Command sent to transfer a firmware programming block.  
        "ST_CID_PROG_UPDATE" :  "0F",              			# Command sent to update the firmware once program transfer has completed.  
        # Status Messages
        "ST_CID_STATUS" :  "10",                   			# Command sent to request the current system status (AHRS   Depth   Temp   etc).  
        "ST_CID_STATUS_CFG_GET" :  "11",           			# Command sent to retrieve the configuration of the status system (message content and auto-output interval).  
        "ST_CID_STATUS_CFG_SET" :  "12",           			# Command sent to set the configuration of the status system (message content and auto-output interval).  
        # Settings Messages
        "ST_CID_SETTINGS_GET" :  "15",             			# Command sent to retrieve the working settings in use on the beacon.  
        "ST_CID_SETTINGS_SET" :  '16' ,              			# Command sent to set the working settings and apply them. They are NOT saved to permanent memory until ST_CID_ SETTINGS_SAVE is issued. The device will need to be rebooted after this to apply some of the changes.  
        "ST_CID_SETTINGS_LOAD" :  '17',              			# Command sent to load the working settings from permanent storage and apply them. Not all settings can be loaded and applied as they only affect the device on start-up.  
        "ST_CID_SETTINGS_SAVE" :  "18",            			# Command sent to save the working settings into permanent storage.  
        "ST_CID_SETTINGS_RESET" :  "19",           			# Command sent to restore the working settings to defaults   store them into permanent memory and apply them.  
        # Calibration Messages
        "ST_CID_CAL_ACTION" :  "20",               			# Command sent to perform specific calibration actions.  
        "ST_CID_CAL_AHRS_GET" :  "21",             			# Command sent to retrieve the current AHRS calibration.  
        "ST_CID_CAL_AHRS_SET" :  "22",             			# Command sent to set the contents of the current AHRS calibration (and store to memory)  
        # Acoustic Transceiver Messages
        "ST_CID_XCVR_ANALYSE" :  "30",             			# Command sent to instruct the receiver to perform a noise analysis and report the results.  
        "ST_CID_XCVR_TX_MSG" :  "31",              			# Message sent when the transceiver transmits a message.  
        "ST_CID_XCVR_RX_ERR" :  "32",              			# Message sent when the transceiver receiver encounters an error.  
        "ST_CID_XCVR_RX_MSG" :  "33",              			# Message sent when the transceiver receives a message (not requiring a response).  
        "ST_CID_XCVR_RX_REQ" :  "34",              			# Message sent when the transceiver receives a request (requiring a response).  
        "ST_CID_XCVR_RX_RESP" :  "35",             			# Message sent when the transceiver receives a response (to a transmitted request).  
        "ST_CID_XCVR_RX_UNHANDLED" :	 "37",       			# Message sent when a message has been received but not handled by the protocol stack.  
        "ST_CID_XCVR_USBL" :	 "38",               			# Message sent when a USBL signal is decoded into an angular bearing.  
        "ST_CID_XCVR_FIX" :	 "39",               			# Message sent when the transceiver gets a position/range fix on a beacon from a request/response.  
        "ST_CID_XCVR_STATUS" :	 "3A",           			# Message sent to query the current transceiver state.  
        # PING Protocol Messages
        "ST_CID_PING_SEND" :  "40",                			# Command sent to transmit a PING message.  
        "ST_CID_PING_REQ" :  "41",                 			# Message sent when a PING request is received.  
        "ST_CID_PING_RESP" :  "42",                			# Message sent when a PING response is received   or timeout occurs   with the echo response data.  
        "ST_CID_PING_ERROR" :  "43",               			# Message sent when a PING response error/timeout occurs.  
        # ECHO Protocol Messages
        "ST_CID_ECHO_SEND" :  "48",                			# Command sent to transmit an ECHO message.  
        "ST_CID_ECHO_REQ" :  "49",                 			# Message sent when an ECHO request is received.  
        "ST_CID_ECHO_RESP" :  "4A",                			# Message sent when an ECHO response is received   or timeout occurs   with the echo response data.  
        "ST_CID_ECHO_ERROR" :  '4B',                 			# Message sent when an ECHO response error/timeout occurs.  
        # NAV Protocol Messages
        "ST_CID_NAV_QUERY_SEND" :  "50",           			# Message sent to query navigation information from a remote beacon.  
        "ST_CID_NAV_QUERY_REQ" :  "51",            			# Message sent from a beacon that receives a NAV_QUERY.  
        "ST_CID_NAV_QUERY_RESP" :  "52",           			# Message generated when the beacon received a response to a NAV_QUERY.  
        "ST_CID_NAV_ERROR" :  "53",                			# Message generated if there is a problem with a NAV_QUERY - i.e. timeout etc.  
        "ST_CID_NAV_QUEUE_SET" :  "58",                        # Message sent to queue a packet of data ready for remote interrogation  
        "ST_CID_NAV_QUEUE_CLR" :  "59",                        # Message sent to query the packet queue  
        "ST_CID_NAV_QUEUE_STATUS" :  "5A",                     # Message sent to query the status of queued packets for each beacon  
        "ST_CID_NAV_STATUS_SEND" :  "5B",                      # Message issued to broadcast status information to all other beacons  
        "ST_CID_NAV_STATUS_RECEIVE" :  "5C",                   # Message generated when a beacon receives a NAV_STATUS message  
        # DAT Protocol Messages
        "ST_CID_DAT_SEND" :  "60",                 			# Message sent to transmit a datagram to another beacon  
        "ST_CID_DAT_RECEIVE" :  "61",              			# Message generated when a beacon receives a datagram.  
        "ST_CID_DAT_ERROR" :  "63",                			# Message generated when a beacon response error/timeout occurs for ACKs.  
        "ST_CID_DAT_QUEUE_SET" :  "64",            			# Message sent to set the contents of the packet data queue.  
        "ST_CID_DAT_QUEUE_CLR" :  "65",            			# Message sent to clear the contents of the packet data queue.  
        "ST_CID_DAT_QUEUE_STATUS" :  "66",                  # Message sent to obtain the current status of the packet data queue.  
        }
        #command_status
        self.CST={
            #General Status Codes
        "ST_CST_OK" :   "00",                       			 # Returned if a command or operation is completed successful without error.  #
        "ST_CST_FAIL" :   "01",                     			 # Returned if a command or operation cannot be completed.  #
        "ST_CST_EEPROM_ERROR" :   "03",             			 # Returned if an error occurs while reading or writing EEPROM data.  #
            #Command Processor Status Codes
        "ST_CST_CMD_PARAM_MISSING" :   "04",        			 # Returned if a command message is given that does not have enough defined fields for the specified CID code.  #
        "ST_CST_CMD_PARAM_INVALID" :   "05",        			 # Returned if a data field in a message does not contain a valid or expected value.  #
            #Firmware Programming Status Codes
        "ST_CST_PROG_FLASH_ERROR" :   "0A",         			 # Returned if an error occurs while writing data into the processors flash memory.  #
        "ST_CST_PROG_FIRMWARE_ERROR" :   "0B",      			 # Returned if firmware cannot be programmed due to incorrect firmware credentials or signature.  #
        "ST_CST_PROG_SECTION_ERROR" :   "0C",       			 # Returned if the firmware cannot be programmed into the specified memory section.  #
        "ST_CST_PROG_LENGTH_ERROR" :   "0D",        			 # Returned if the firmware length is too large to fit into the specified memory section   or not what the current operation is expecting.  #
        "ST_CST_PROG_DATA_ERROR" :   "0E",          			 # Returned if there is an error decoding data in a firmware block.  #
        "ST_CST_PROG_CHECKSUM_ERROR" :   "0F",      			 # Returned if the specified checksum for the firmware does not match the checksum computed prior to performing the update.  #
            #Acoustic Transceiver Status Codes
        "ST_CST_XCVR_BUSY" :   "30",                			 # Returned if the transceiver cannot perform a requested action as it is currently busy (i.e. transmitting a message).  #
        "ST_CST_XCVR_ID_REJECTED" :   "31",         			 # Returned if the received message did not match the specified transceiver ID (and wasn’t a Sent-To-All)   and the message has been rejected.  #
        "ST_CST_XCVR_CSUM_ERROR" :   "32",          			 # Returned if received acoustic message’s checksum was invalid   and the message has been rejected.  #
        "ST_CST_XCVR_LENGTH_ERROR" :   "33",        			 # Returned if an error occurred with message framing   meaning the end of the message has not been received within the expected time.  #
        "ST_CST_XCVR_RESP_TIMEOUT" :   "34",        			 # Returned if the transceiver has sent a request message to a beacon   but no response has been returned within the allotted waiting period.  #
        "ST_CST_XCVR_RESP_ERROR" :   "35",          			 # Returned if the transceiver has send a request message to a beacon   but an error occurred while receiving the response.  #
        "ST_CST_XCVR_RESP_WRONG" :   "36",          			 # Returned if the transceiver has sent a request message to a beacon   but received an unexpected response from another beacon while waiting.  #
        "ST_CST_XCVR_PLOAD_ERROR" :   "37",         			 # Returned by protocol payload decoders   if the payload can’t be parsed correctly.  #
        "ST_CST_XCVR_STATE_STOPPED" :   "3A",       			 # Indicates the transceiver is in a stopped state.  #
        "ST_CST_XCVR_STATE_IDLE" :   "3B",          			 # Indicates the transceiver is in an idle state waiting for reception or transmission to start.  #
        "ST_CST_XCVR_STATE_TX" :   "3C",            			 # Indicates the transceiver is in a transmitting states.  #
        "ST_CST_XCVR_STATE_REQ" :   "3D",           			 # Indicates the transceiver is in a requesting state   having transmitted a message and is waiting for a response to be received.  #
        "ST_CST_XCVR_STATE_RX" :   "3E",            			 # Indicates the transceiver is in a receiving state.  #
        "ST_CST_XCVR_STATE_RESP" :   "3F",          			 # Indicates the transceiver is in a responding state   where a message is being composed and the “response time” period is being observed.  #
            #DEX Protocol Status Codes
        "ST_CST_DEX_SOCKET_ERROR" :   "70",         			 # Returned by the DEX protocol handler if an error occurred trying to open   close or access a specified socket ID.  #
        "ST_CST_DEX_RX_SYNC" :   "71",              			 # Returned by the DEX protocol handler when receiver synchronisation has occurred with the socket master and data transfer is ready to commence.  #
        "ST_CST_DEX_RX_DATA" :   "72",              			 # Returned by the DEX protocol handler when data has been received through a socket.  #
        "ST_CST_DEX_RX_SEQ_ERROR" :   "73",         			 # Returned by the DEX protocol handler when data transfer synchronisation has been lost with the socket master.  #
        "ST_CST_DEX_RX_MSG_ERROR" :   "74",         			 # Returned by the DEX protocol handler to indicate an unexpected acoustic message type with the DEX protocol has been received and cannot be processed.  #
        "ST_CST_DEX_REQ_ERROR" :   "75",            			 # Returned by the DEX protocol handler to indicate a error has occurred while responding to a request (i.e. lack of data).  #
        "ST_CST_DEX_RESP_TMO_ERROR" :   "76",       			 # Returned by the DEX protocol handler to indicate a timeout has occurred while waiting for a response back from a remote beacon with requested data.  #
        "ST_CST_DEX_RESP_MSG_ERROR" :   "77",       			 # Returned by the DEX protocol handler to indicate an error has occurred while receiving response back from a remote beacon.  #
        "ST_CST_DEX_RESP_REMOTE_ERROR" :   "78",    			 # Returned by the DEX protocol handler to indicate the remote beacon has encountered an error and cannot return the requested data or perform the required operation.  #
        }
        #beacon_id
        self.BID = {
        "beacon_id_00" : "00",
        "beacon_id_01":"01",
        "beacon_id_02":"02",
        "beacon_id_03":"03",
        "beacon_id_04":"04",
        "beacon_id_05":"05",
        "beacon_id_06":"06",
        "beacon_id_07":"07",
        "beacon_id_08":"08",
        "beacon_id_09":"09",
        "beacon_id_10":"0A",
        "beacon_id_11":"0B",
        "beacon_id_12":"0C",
        "beacon_id_13":"0D",
        "beacon_id_14":"0E",
        "beacon_id_15" :"0F"
        }
        #Acoustic_message_type
        self.AMSG = {
        "ST_AMSG_OWAY" : "00",                 				 # Indicates an acoustic message is sent One-Way, and does not require a response. One-Way messages may also be broadcast to all beacons if required. No USBL information is sent.  #
        'ST_AMSG_OWAYU' : "01",                				 # Indicates an acoustic message is sent One-Way, and does not require a response. One-Way messages may also be broadcast to all beacons if required. Additionally, the message is sent with USBL acoustic information allowing an incoming bearing to be determined by USBL receivers, although range cannot be provided.  #
        'ST_AMSG_REQ' :  "02",                  				 # Indicates an acoustic message is sent as a Request type. This requires the receiver to generate and return a Response (MSG_RESP) message. No USBL information is requested.  #
        "ST_AMSG_RESP" :  "03",                 				 # Indicates an acoustic message is sent as a Response to a previous Request message (MSG_REQ). No USBL information is returned.  #
        'ST_AMSG_REQU' :  "04",                 				 # Indicates an acoustic message is sent as a Request type. This requires the receiver to generate and return a Response (MSG_RESP) message. Additionally, the Response message should be returned with USBL acoustic information allowing a position fix to be computed by USBL receivers through the range and incoming signal angle.  #
        'ST_AMSG_RESPU' :  "05",                				 # Indicates an acoustic message is sent as a Response to a previous Request message (MSG_REQ). Additionally, the message is sent with USBL acoustic information allowing the position of the sender to be determined through the range and incoming signal angle.  #
        'ST_AMSG_REQX' :  "06",                	 				 # Indicates an acoustic message is sent as a Request type. This requires the receiver to generate and return a Response (MSG_RESP) message. Additionally, the Response message should be returned with extended Depth and USBL acoustic information allowing a more accurate position fix to be computed by USBL receivers through the range, remote depth and incoming signal angle.  #
        "ST_AMSG_RESPX" :  '07',                				 # Indicates an acoustic message is sent as a Response to a previous Request message (MSG_REQ). Additionally, the message is sent with extended depth and USBL acoustic information allowing a more accurate position of the sender to be determined through the range, remote depth and incoming signal angle.  #
        "ST_AMSG_UNKNOWN" :  "FF"
        }
        #payload_type
        self.APLOAD={
        "ST_APLOAD_PING" : "00",                   			 # Specified an acoustic message payload should be interpreted by the PING protocol handler. PING messages provide the simplest (and quickest) method of validating the presence of a beacon, and determining its position.  #
        "ST_APLOAD_ECHO" : "01",                   			 # Specified an acoustic message payload should be interpreted by the ECHO protocol handler. ECHO messages allow the function and reliability of a beacon to be tested, by requesting the payload contents of the message be returned back to the sender.  #
        'ST_APLOAD_NAV' : "02",                    			 # Specified an acoustic message payload should be interpreted by the NAV (Navigation) protocol handler. NAV messages allow tracking and navigation systems to be built that use enhanced positioning and allow remote parameters of beacons (such as heading, attitude, water temperature etc) to be queried.  #
        "ST_APLOAD_DAT" : "03",                    			 # Specified an acoustic message payload should be interpreted by the DAT (Datagram) protocol handler. DAT messages for the simplest method of data exchange between beacons, and provide a method of acknowledging data reception.  #
        'ST_APLOAD_DEX' : "04",                  			  	 # Specified an acoustic message payload should be interpreted by the DEX (Data Exchange) protocol handler. DEX messages implement a protocol that allows robust bi-directional socket based data exchange with timeouts, acknowledgments and retry schemes.  #
        'ST_APLOAD_UNKNOWN' : "FF", 
        }
        #STATUS_BITS_T
        self.STB={
        "AHRS_COMP_DATA":"05",
        "AHRS_RAW_DATA":"04",
        "ACC_CAL":"03",
        "MAG_CAL":"02",
        "ATTITUDE":"01",
        "ENVIRONMENT":"00",
        }
    
    def reverse_bit(self,bit):
        return bytearray.fromhex(bit)[::-1].hex()

    def hex_to_int(self,val,types):
        if val == "":
            val="00"
        out=int(self.reverse_bit(val),16)
        bit=len(val)*4
        half_bit=(pow(2, bit)/2)
        if (out >= half_bit) and bool(types):
            out=out-pow(2, bit)
        return out 

    def hex_to_string(self,hex):
        return bytes.fromhex((hex)).decode('utf-8')
    
    def crc16(self,data):
        #from checksum import crc16
        data = str(data)
        offset=0
        length=int(len(data)/2)
        data=bytes.fromhex(data)
        if data is None or offset < 0 or offset > len(data) - 1 and offset+length > len(data):
            return 0
        crc = 0x0000
        for i in (range(0, length)):
            # crc = crc << 8
            crc ^= data[i]
            for j in range(0, 8):
                if (crc & 0x0001) > 0:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return str(hex(crc))[4:6]+str(hex(crc))[2:4]

    def serial_read(self):
        self.data_in_raw = self.ser.readline()
        self.data_in = str(self.data_in_raw)[2:-5]
        return self.data_in

    def serial_write(self,input):
            if input=="ping":
                self.ser.write(b'#4001068046\r\n')
            elif input=="data":
                self.ser.write(b'#5B010454455354263E\r\n')
            else:
                data=str(input)+'\r\n'
                self.ser.write(bytes(data,'UTF-8'))

    def check_serial_to_command(sefl,message__type,data):
        return list(message__type.keys())[list(message__type.values()).index(data)]

    def encrypt_command(self,command):
        self.encrypt="#"+command+self.crc16(command)
        return self.encrypt
      
    def ping_x150_command(self,ping_type,id,acoutic):
        
        ping=self.CID[ping_type]+self.BID[id]+self.AMSG[acoutic]
        self.ping_encrypt=self.encrypt_command(ping)
        
        return self.ping_encrypt

    def send_data_x150_command(self,NAV_STATUS_TYPE,id,data):
        if len(data)<16:
            data_size="0"+hex((len(data)))[2:]
        else:
            data_size=hex(len(data))[2:]
        send_data=self.CID[NAV_STATUS_TYPE]+self.BID[id]+data_size+str(data.encode("utf-8").hex())
        self.send_data_encrypt=self.encrypt_command(send_data)
        return self.send_data_encrypt

    def decrypt_command(self,status,command):
        if self.MODEL == "x150":
            Command=self.data_in[1:3]
            if status is True:
                if Command == self.CID["ST_CID_STATUS"]:
                    status_encrypt={
                    "CmdId":Command,
                    "STATUS_OUTPUT" :self.data_in[3:5],
                    "TIMESTAMP":self.data_in[5:21],
                    "ENV_SUPPLY" : self.data_in[21:25],
                    "ENV_TEMP" :self.data_in[25:29],
                    "ENV_PRESSURE":self.data_in[29:37],
                    "ENV_DEPTH":self.data_in[37:45],
                    "ENV_VOS":self.data_in[45:49],
                    "ATT_YAW":self.data_in[49:53],
                    "ATT_PITCH":self.data_in[53:57],
                    'ATT_ROLL':self.data_in[57:61],
                    'MAG_CAL_BUF':self.data_in[61:63],
                    'MAG_CAL_VALID':self.data_in[63:65],
                    'MAG_CAL_AGE':self.data_in[65:73],
                    'MAG_CAL_FIT':self.data_in[73:75],
                    'ACC_CAL':self.data_in[75:79],
                    'ACC_LIM_MIN_X':self.data_in[79:83],
                    'ACC_LIM_MIN_Y':self.data_in[83:87],
                    'ACC_LIM_MIN_Z':self.data_in[87:91],
                    'ACC_LIM_MAX_X':self.data_in[91:95],
                    'ACC_LIM_MAX_Y':self.data_in[95:99],
                    'ACC_LIM_MAX_Z':self.data_in[99:103],
                    'AHRS_RAW_ACC_X':self.data_in[103:107],
                    'AHRS_RAW_ACC_Y':self.data_in[107:111],
                    'AHRS_RAW_ACC_Z':self.data_in[111:115],
                    'AHRS_RAW_MAG_X':self.data_in[115:119],
                    'AHRS_RAW_MAG_Y' :self.data_in[119:123],
                    'AHRS_RAW_MAG_Z':self.data_in[123:127],
                    'AHRS_RAW_GYRO_X':self.data_in[127:131],
                    'AHRS_RAW_GYRO_Y':self.data_in[131:135],
                    'AHRS_RAW_GYRO_Z':self.data_in[135:139]
                    }    
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),                             # Command_ID , Command identification code (CID_STATUS)
                    "STATUS_OUTPUT" : self.hex_to_int(status_encrypt["STATUS_OUTPUT"],1) ,                                   # STATUS_OUTPUT , A bit-mask specifying which information is included in the following status output message, so any decoding algorithm can correctly parse the message record
                    "TIMESTAMP": (self.hex_to_int(status_encrypt["TIMESTAMP"],0)) /1000,                                    # Seconds , The value of the beacon’s clock (set to zero when the beacon was powered up – not rebooted!). Values are encoded in milliseconds, so divide by 1000 for seconds.
                    "ENV_SUPPLY" : self.hex_to_int(status_encrypt["ENV_SUPPLY"],0) /1000,                   # Volts , The beacons supply voltage. Values are encoded in milli-volts, so divide by 1000 for a value in Volts.
                    "ENV_TEMP" : self.hex_to_int(status_encrypt["ENV_TEMP"],0) /10,           # Celsius , The temperature of air/water in contact with the diaphragm of the pressure sensor. Values are encoded in deci-Celsius, so divide by 10 to obtain a value in Celsius.
                    "ENV_PRESSURE": self.hex_to_int(status_encrypt["ENV_PRESSURE"],1)/1000 ,                # Bar , The external pressure measured on the diaphragm of the pressure sensor. Values are encoded in milli-bars, so divide by 1000 to obtain a value in Bar. Please note, the specification of pressure reading is 0.5% of the sensors full-scale value, so for a 200 Bar sensor the tolerance applicable to this value is ±1 Bar (~10m).
                    "ENV_DEPTH": self.hex_to_int(status_encrypt["ENV_DEPTH"],1)/10 ,                        # Metres ,The computed depth based on the measured environmental pressure. Values are encoded in deci-metres, so divide by 10 for a value in metres.
                    "ENV_VOS": self.hex_to_int(status_encrypt["ENV_VOS"],1)/10 ,                            # metres-per-second , The value of Velocity-of-Sound currently being used for computing ranges. This may be either the manually specified VOS from settings, or the auto-computed value based on pressure, temperature and salinity. Values are encoded in deci-metres-per-second, so divide by 10 to obtain a value in metres-per-second.
                    "ATT_YAW": self.hex_to_int(status_encrypt["ATT_YAW"],1) /10,                                  # degrees , The current Yaw angle of the beacon, relative to magnetic north, measured by the beacons AHRS system. Values are encoded as deci-degrees, so divide the value by 10 to obtain a value in degrees.
                    "ATT_PITCH": self.hex_to_int(status_encrypt["ATT_PITCH"],1) /10,                              # degrees , The current Pitch angle of the beacon, relative to magnetic north, measured by the beacons AHRS system. Values are encoded as deci-degrees, so divide the value by 10 to obtain a value in degrees.
                    'ATT_ROLL': self.hex_to_int(status_encrypt["ATT_ROLL"],1) /10,                                # degrees ,The current Roll angle of the beacon, relative to magnetic north, measured by the beacons AHRS system. Values are encoded as deci-degrees, so divide the value by 10 to obtain a value in degrees.
                    'MAG_CAL_BUF': self.hex_to_int(status_encrypt["MAG_CAL_BUF"],1) ,                                         # percentage , Value that indicates how full the data buffer is that holds magnetometer values describing the surrounding magnetic environment. Values are encoded as a percentage from 0 to 100 representing empty (where no magnetic calibration is possible) to full (where the best magnetic calibration can be computed).
                    'MAG_CAL_VALID':bool(int(status_encrypt["MAG_CAL_VALID"],16)) ,                                                  # Boolean , The flag is True if a magnetic calibration has been computed and is currently in use, compensating magnetometer readings.
                    'MAG_CAL_AGE': self.hex_to_int(status_encrypt["MAG_CAL_AGE"],0) ,                                         # Seconds , The number of seconds that have elapsed since the magnetometer calibration was last computed. When dynamic calibration is enabled, and there is sufficient data in the magnetic calibration buffer, then calibrations should be computed every 30 seconds.
                    'MAG_CAL_FIT': self.hex_to_int(status_encrypt["MAG_CAL_FIT"],1) ,                                         # percentage , Value indicating how well the current magnetometer calibration can fit the measured data to an ideal “sphere” (or perfect calibration). Values are encoded as a percentage from 0 to 100.                                          
                    'ACC_CAL': self.hex_to_int(status_encrypt["ACC_CAL"],1) ,
                    'ACC_LIM_MIN_X': self.hex_to_int(status_encrypt["ACC_LIM_MIN_X"],1) ,
                    'ACC_LIM_MIN_Y': self.hex_to_int(status_encrypt["ACC_LIM_MIN_Y"],1) ,
                    'ACC_LIM_MIN_Z': self.hex_to_int(status_encrypt["ACC_LIM_MIN_Z"],1) ,
                    'ACC_LIM_MAX_X': self.hex_to_int(status_encrypt["ACC_LIM_MAX_X"],1) ,
                    'ACC_LIM_MAX_Y': self.hex_to_int(status_encrypt["ACC_LIM_MAX_Y"],1) ,
                    'ACC_LIM_MAX_Z': self.hex_to_int(status_encrypt["ACC_LIM_MAX_Z"],1) ,
                    'AHRS_RAW_ACC_X': self.hex_to_int(status_encrypt["AHRS_RAW_ACC_X"],1) ,
                    'AHRS_RAW_ACC_Y': self.hex_to_int(status_encrypt["AHRS_RAW_ACC_Y"],1) ,
                    'AHRS_RAW_ACC_Z': self.hex_to_int(status_encrypt["AHRS_RAW_ACC_Z"],1) ,
                    'AHRS_RAW_MAG_X': self.hex_to_int(status_encrypt["AHRS_RAW_MAG_X"],1) ,
                    'AHRS_RAW_MAG_Y' : self.hex_to_int(status_encrypt["AHRS_RAW_MAG_Y"],1) ,
                    'AHRS_RAW_MAG_Z': self.hex_to_int(status_encrypt["AHRS_RAW_MAG_Z"],1) ,
                    'AHRS_RAW_GYRO_X': self.hex_to_int(status_encrypt["AHRS_RAW_GYRO_X"],1) ,
                    'AHRS_RAW_GYRO_Y': self.hex_to_int(status_encrypt["AHRS_RAW_GYRO_Y"],1) ,
                    'AHRS_RAW_GYRO_Z': self.hex_to_int(status_encrypt["AHRS_RAW_GYRO_Z"],1),
                    }
                else : 
                    status_encrypt,status_decrypt=None,None

            elif command is True:           
                if Command == self.CID["ST_CID_PING_RESP"] or (Command == self.CID["ST_CID_XCVR_FIX"] ):  
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],
                    "RANGE_COUNT" : self.data_in[35:43],
                    "RANGE_TIME": self.data_in[43:51],
                    "RANGE_DIST" : self.data_in[51:55],
                    "USBL_CHANNELS" : self.data_in[55:57],
                    "USBL_RSSI_1": self.data_in[57:61],
                    "USBL_RSSI_2": self.data_in[61:65],
                    "USBL_RSSI_3": self.data_in[65:69],
                    "USBL_RSSI_4": self.data_in[69:73],
                    "USBL_AZIMUTH" : self.data_in[73:77],
                    "USBL_ELEVATION": self.data_in[77:81],
                    "USBL_FIT_ERROR" : self.data_in[81:85],
                    "POSITION_EASTING" : self.data_in[85:89],
                    "POSITION_NORTHING" : self.data_in[89:93],
                    "POSITION_DEPTH": self.data_in[93:97],
                    } 
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : self.hex_to_int(status_encrypt["FLAGS"],1) ,
                    "MSG_TYPE" : self.hex_to_int(status_encrypt["MSG_TYPE"],1)  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 ,
                    "RANGE_COUNT" : self.hex_to_int(status_encrypt["RANGE_COUNT"],1) ,
                    "RANGE_TIME": self.hex_to_int(status_encrypt["RANGE_TIME"],1)/10000000 ,
                    "RANGE_DIST" : self.hex_to_int(status_encrypt["RANGE_DIST"],1)/10 ,
                    "USBL_CHANNELS" : self.hex_to_int(status_encrypt["USBL_CHANNELS"],1) ,
                    "USBL_RSSI_1": self.hex_to_int(status_encrypt["USBL_RSSI_1"],1)/10 ,
                    "USBL_RSSI_2": self.hex_to_int(status_encrypt["USBL_RSSI_2"],1)/10 ,
                    "USBL_RSSI_3": self.hex_to_int(status_encrypt["USBL_RSSI_3"],1)/10 ,
                    "USBL_RSSI_4": self.hex_to_int(status_encrypt["USBL_RSSI_4"],1)/10 ,
                    "USBL_AZIMUTH" : self.hex_to_int(status_encrypt["USBL_AZIMUTH"],1) /10,
                    "USBL_ELEVATION": self.hex_to_int(status_encrypt["USBL_ELEVATION"],1) /10,
                    "USBL_FIT_ERROR" : self.hex_to_int(status_encrypt["USBL_FIT_ERROR"],1) /100,
                    "POSITION_EASTING" : self.hex_to_int(status_encrypt["POSITION_EASTING"],1) /10,
                    "POSITION_NORTHING" : self.hex_to_int(status_encrypt["POSITION_NORTHING"],1) /10,
                    "POSITION_DEPTH": self.hex_to_int(status_encrypt["POSITION_DEPTH"],1) /10,
                    }
                
                elif Command == self.CID["ST_CID_XCVR_TX_MSG"]:
                    if self.data_in[17:19] =="8F":
                        status_encrypt={
                        "CmdId": Command,
                        "DEST_ID" : self.data_in[3:5],
                        "SRC_ID": self.data_in[5:7],
                        "MSG_TYPE" : self.data_in[7:9],
                        "MSG_DEPTH": self.data_in[9:13],
                        "MSG_PAYLOAD_ID" : self.data_in[13:15],
                        "MSG_PAYLOAD_LEN" : self.data_in[15:17],
                        "MSG_PAYLOAD" : self.data_in[19:-4],
                        }
                    else:
                        status_encrypt={
                        "CmdId": Command,
                        "DEST_ID" : self.data_in[3:5],
                        "SRC_ID": self.data_in[5:7],
                        "MSG_TYPE" : self.data_in[7:9],
                        "MSG_DEPTH": self.data_in[9:13],
                        "MSG_PAYLOAD_ID" : self.data_in[13:15],
                        "MSG_PAYLOAD_LEN" : self.data_in[15:17],
                        "MSG_PAYLOAD" : self.data_in[17:-4],
                        }  
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "MSG_TYPE" : self.hex_to_int(status_encrypt["MSG_TYPE"],1)  ,
                    "MSG_DEPTH": self.hex_to_int(status_encrypt["MSG_DEPTH"],1)  ,
                    "MSG_PAYLOAD_ID" : self.hex_to_int(status_encrypt["MSG_PAYLOAD_ID"],1)  ,
                    "MSG_PAYLOAD_LEN" : self.hex_to_int(status_encrypt["MSG_PAYLOAD_LEN"],1)  ,
                    "MSG_PAYLOAD" : self.hex_to_string(status_encrypt["MSG_PAYLOAD"])  ,
                    }                   
                
                elif Command == self.CID["ST_CID_XCVR_RX_RESP"] :
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "HDR_MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],
                    "RANGE_COUNT" : self.data_in[35:43],
                    "RANGE_TIME": self.data_in[43:51],
                    "RANGE_DIST" : self.data_in[51:55],
                    "USBL_CHANNELS" : self.data_in[55:57],
                    "USBL_RSSI_1": self.data_in[57:61],
                    "USBL_RSSI_2": self.data_in[61:65],
                    "USBL_RSSI_3": self.data_in[65:69],
                    "USBL_RSSI_4": self.data_in[69:73],
                    "USBL_AZIMUTH" : self.data_in[73:77],
                    "USBL_ELEVATION": self.data_in[77:81],
                    "USBL_FIT_ERROR" : self.data_in[81:85],
                    "POSITION_EASTING" : self.data_in[85:89],
                    "POSITION_NORTHING" : self.data_in[89:93],
                    "POSITION_DEPTH": self.data_in[93:97],
                    "MSG_DEST_ID": self.data_in[97:99],
                    "MSG_SRC_ID": self.data_in[99:101],
                    "MSG_TYPE": self.data_in[101:103],
                    "MSG_DEPTH": self.data_in[103:107],
                    "MSG_PAYLOAD_ID": self.data_in[107:109],
                    "MSG_PAYLOAD_LEN": self.data_in[109:111],
                    #"MSG_PAYLOAD": self.data_in[111:-4],                
                    }   
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : self.hex_to_int(status_encrypt["FLAGS"],1) ,
                    "HDR_MSG_TYPE" : self.hex_to_int(status_encrypt["HDR_MSG_TYPE"],1)  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 ,
                    "RANGE_COUNT" : self.hex_to_int(status_encrypt["RANGE_COUNT"],1) ,
                    "RANGE_TIME": self.hex_to_int(status_encrypt["RANGE_TIME"],1)/10000000 ,
                    "RANGE_DIST" : self.hex_to_int(status_encrypt["RANGE_DIST"],1)/10 ,
                    "USBL_CHANNELS" : self.hex_to_int(status_encrypt["USBL_CHANNELS"],1) ,
                    "USBL_RSSI_1": self.hex_to_int(status_encrypt["USBL_RSSI_1"],1)/10 ,
                    "USBL_RSSI_2": self.hex_to_int(status_encrypt["USBL_RSSI_2"],1)/10 ,
                    "USBL_RSSI_3": self.hex_to_int(status_encrypt["USBL_RSSI_3"],1)/10 ,
                    "USBL_RSSI_4": self.hex_to_int(status_encrypt["USBL_RSSI_4"],1)/10 ,
                    "USBL_AZIMUTH" : self.hex_to_int(status_encrypt["USBL_AZIMUTH"],1) /10,
                    "USBL_ELEVATION": self.hex_to_int(status_encrypt["USBL_ELEVATION"],1) /10,
                    "USBL_FIT_ERROR" : self.hex_to_int(status_encrypt["USBL_FIT_ERROR"],1) /100,
                    "POSITION_EASTING" : self.hex_to_int(status_encrypt["POSITION_EASTING"],1) /10,
                    "POSITION_NORTHING" : self.hex_to_int(status_encrypt["POSITION_NORTHING"],1) /10,
                    "POSITION_DEPTH": self.hex_to_int(status_encrypt["POSITION_DEPTH"],1) /10,
                    "MSG_DEST_ID": self.hex_to_int(status_encrypt["MSG_DEST_ID"],1) ,
                    "MSG_SRC_ID": self.hex_to_int(status_encrypt["MSG_SRC_ID"],1) ,
                    "MSG_TYPE": self.hex_to_int(status_encrypt["MSG_TYPE"],1) ,
                    "MSG_DEPTH": self.hex_to_int(status_encrypt["MSG_DEPTH"],1) ,
                    "MSG_PAYLOAD_ID": self.hex_to_int(status_encrypt["MSG_PAYLOAD_ID"],1) ,
                    "MSG_PAYLOAD_LEN": self.hex_to_int(status_encrypt["MSG_PAYLOAD_LEN"],1) ,
                    #"MSG_PAYLOAD":status_encrypt["MSG_PAYLOAD"]  ,
                    } 

                elif Command == self.CID["ST_CID_SYS_ALIVE"]:
                    status_encrypt={
                    "CmdId": Command,
                    "Timestamp" : self.data_in[3:11],
                    }    
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "Timestamp" : self.hex_to_int(status_encrypt["Timestamp"],1) 
                    }

                elif Command == self.CID["ST_CID_XCVR_RX_REQ"] :
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],
                    "RANGE_COUNT" : self.data_in[35:39],
                    "RANGE_TIME" : self.data_in[39:43],
                    "RANGE_DIST": self.data_in[43:47],
                    "MSG_PAYLOAD_LEN": self.data_in[47:49],
                    "MSG_PAYLOAD": self.data_in[49:-4],  
                    }   
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : self.hex_to_int(status_encrypt["FLAGS"],1) ,
                    "MSG_TYPE" : self.hex_to_int(status_encrypt["MSG_TYPE"],1)  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 ,
                    "RANGE_COUNT" : self.hex_to_int(status_encrypt["RANGE_COUNT"],1) ,
                    "RANGE_TIME": self.hex_to_int(status_encrypt["RANGE_TIME"],1)/10000000 ,
                    #"RANGE_DIST" : self.hex_to_int(status_encrypt["RANGE_DIST"],1)/10 ,
                    "MSG_PAYLOAD_LEN": self.hex_to_int(status_encrypt["MSG_PAYLOAD_LEN"],1) ,
                    "MSG_PAYLOAD":self.hex_to_string(status_encrypt["MSG_PAYLOAD"])  ,
                    }

                elif Command == self.CID["ST_CID_PING_REQ"] :
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],                
                    }   
                    status_decrypt={      
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : self.hex_to_int(status_encrypt["FLAGS"],1) ,
                    "MSG_TYPE" : self.hex_to_int(status_encrypt["MSG_TYPE"],1)  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 ,
                    }  

                elif Command == self.CID["ST_CID_XCVR_TX_MSG"]:
                    print("Data")
                    if self.data_in[17:19] =="8F":
                        status_encrypt={
                        "CmdId": Command,
                        "DEST_ID" : self.data_in[3:5],
                        "SRC_ID": self.data_in[5:7],
                        "MSG_TYPE" : self.data_in[7:9],
                        "MSG_DEPTH": self.data_in[9:13],
                        "MSG_PAYLOAD_ID" : self.data_in[13:15],
                        "MSG_PAYLOAD_LEN" : self.data_in[15:17],
                        "FLAG" : self.data_in[17:19],
                        "MSG_PAYLOAD" : self.data_in[19:-4],
                        }
                    else:
                        status_encrypt={
                        "CmdId": Command,
                        "DEST_ID" : self.data_in[3:5],
                        "SRC_ID": self.data_in[5:7],
                        "MSG_TYPE" : self.data_in[7:9],
                        "MSG_DEPTH": self.data_in[9:13],
                        "MSG_PAYLOAD_ID" : self.data_in[13:15],
                        "MSG_PAYLOAD_LEN" : self.data_in[15:17],
                        "MSG_PAYLOAD" : self.data_in[17:-4],
                        }
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "MSG_TYPE" : self.hex_to_int(status_encrypt["MSG_TYPE"],1)  ,
                    "MSG_DEPTH": self.hex_to_int(status_encrypt["MSG_DEPTH"],1)  ,
                    "MSG_PAYLOAD_ID" : self.hex_to_int(status_encrypt["MSG_PAYLOAD_ID"],1)  ,
                    "MSG_PAYLOAD_LEN" : self.hex_to_int(status_encrypt["MSG_PAYLOAD_LEN"],1)  ,
                    "MSG_PAYLOAD" : self.hex_to_string(status_encrypt["MSG_PAYLOAD"])  ,
                    }                   
                
                elif Command == self.CID["ST_CID_NAV_STATUS_SEND"] :
                    status_encrypt={
                    "CmdId": Command,
                    "STATUS" : self.data_in[3:5],
                    "BEACON_ID": self.data_in[5:7],
                    }   
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "STATUS" : self.check_serial_to_command(self.CST,status_encrypt["STATUS"]),
                    "BEACON_ID": self.hex_to_int(status_encrypt["BEACON_ID"],1) ,
                    }

                elif Command == self.CID["ST_CID_XCVR_RX_MSG"] :
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "HDR_MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],
                    "MSG_DEST_ID": self.data_in[35:37],
                    "MSG_SRC_ID": self.data_in[37:39],
                    "MSG_TYPE": self.data_in[39:41],
                    "MSG_DEPTH": self.data_in[41:45],
                    "MSG_PAYLOAD_ID": self.data_in[45:47],
                    "MSG_PAYLOAD_LEN": self.data_in[47:49],
                    "MSG_PAYLOAD": self.data_in[49:-4],
                    }   
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : self.hex_to_int(status_encrypt["FLAGS"],1) ,
                    "HDR_MSG_TYPE" : self.hex_to_int(status_encrypt["HDR_MSG_TYPE"],1)  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 ,
                    "MSG_DEST_ID": self.hex_to_int(status_encrypt["MSG_DEST_ID"],1) ,
                    "MSG_SRC_ID": self.hex_to_int(status_encrypt["MSG_SRC_ID"],1) ,
                    "MSG_TYPE": self.hex_to_int(status_encrypt["MSG_TYPE"],1) ,
                    "MSG_DEPTH": self.hex_to_int(status_encrypt["MSG_DEPTH"],1) ,
                    "MSG_PAYLOAD_ID": self.hex_to_int(status_encrypt["MSG_PAYLOAD_ID"],1) ,
                    "MSG_PAYLOAD_LEN": self.hex_to_int(status_encrypt["MSG_PAYLOAD_LEN"],1) ,
                    "MSG_PAYLOAD" : self.hex_to_string(status_encrypt["MSG_PAYLOAD"])  ,
                    } 

                elif Command == self.CID["ST_CID_NAV_STATUS_RECEIVE"] :
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],
                    "BEACON_ID": self.data_in[35:37],
                    "MSG_PAYLOAD_LEN": self.data_in[37:39],
                    "MSG_PAYLOAD": self.data_in[39:-6],
                    "LOCAL_FLAG": self.data_in[-6:-4],
                    }   
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : status_encrypt["FLAGS"] ,
                    "MSG_TYPE" : status_encrypt["MSG_TYPE"]  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 ,
                    "BEACON_ID": self.hex_to_int(status_encrypt["BEACON_ID"],1) ,
                    "MSG_PAYLOAD_LEN": self.hex_to_int(status_encrypt["MSG_PAYLOAD_LEN"],1) ,
                    "MSG_PAYLOAD" : self.hex_to_string(status_encrypt["MSG_PAYLOAD"])  ,
                    "LOCAL_FLAG" : bool(int(status_encrypt["LOCAL_FLAG"],16)),
                    }
                
                else : 
                    status_encrypt,status_decrypt=None,None

            else : 
                    status_encrypt,status_decrypt=None,None

            return status_encrypt,status_decrypt
        
        elif self.MODEL=="x010":
            Command=self.data_in[1:3]
            if status is True:
                if Command == self.CID["ST_CID_STATUS"]: 
                    status_encrypt={
                    "CmdId":Command,
                    "STATUS_OUTPUT" :self.data_in[3:5],
                    "TIMESTAMP":self.data_in[5:21],
                    "ENV_SUPPLY" : self.data_in[21:25],
                    "ENV_TEMP" :self.data_in[25:29],
                    "ENV_PRESSURE":self.data_in[29:37],
                    "ENV_DEPTH":self.data_in[37:45],
                    "ENV_VOS":self.data_in[45:49],
                    }    
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),                             # Command_ID , Command identification code (CID_STATUS)
                    "STATUS_OUTPUT" : self.hex_to_int(status_encrypt["STATUS_OUTPUT"],1) ,                                   # STATUS_OUTPUT , A bit-mask specifying which information is included in the following status output message, so any decoding algorithm can correctly parse the message record
                    "TIMESTAMP": (self.hex_to_int(status_encrypt["TIMESTAMP"],0)) /1000,                                    # Seconds , The value of the beacon’s clock (set to zero when the beacon was powered up – not rebooted!). Values are encoded in milliseconds, so divide by 1000 for seconds.
                    "ENV_SUPPLY" : self.hex_to_int(status_encrypt["ENV_SUPPLY"],0) /1000,                   # Volts , The beacons supply voltage. Values are encoded in milli-volts, so divide by 1000 for a value in Volts.
                    "ENV_TEMP" : self.hex_to_int(status_encrypt["ENV_TEMP"],0) /10,           # Celsius , The temperature of air/water in contact with the diaphragm of the pressure sensor. Values are encoded in deci-Celsius, so divide by 10 to obtain a value in Celsius.
                    "ENV_PRESSURE": self.hex_to_int(status_encrypt["ENV_PRESSURE"],1)/1000 ,                # Bar , The external pressure measured on the diaphragm of the pressure sensor. Values are encoded in milli-bars, so divide by 1000 to obtain a value in Bar. Please note, the specification of pressure reading is 0.5% of the sensors full-scale value, so for a 200 Bar sensor the tolerance applicable to this value is ±1 Bar (~10m).
                    "ENV_DEPTH": self.hex_to_int(status_encrypt["ENV_DEPTH"],1)/10 ,                        # Metres ,The computed depth based on the measured environmental pressure. Values are encoded in deci-metres, so divide by 10 for a value in metres.
                    "ENV_VOS": self.hex_to_int(status_encrypt["ENV_VOS"],1)/10 ,                            # metres-per-second , The value of Velocity-of-Sound currently being used for computing ranges. This may be either the manually specified VOS from settings, or the auto-computed value based on pressure, temperature and salinity. Values are encoded in deci-metres-per-second, so divide by 10 to obtain a value in metres-per-second. 
                    }

                else : 
                    status_encrypt,status_decrypt=None,None
                
            elif command is True:
                if Command == self.CID["ST_CID_PING_REQ"] :
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],
                    }   
                    status_decrypt={  
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : self.hex_to_int(status_encrypt["FLAGS"],1) ,
                    "MSG_TYPE" : self.hex_to_int(status_encrypt["MSG_TYPE"],1)  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 , 
                    } 

                elif Command == self.CID["ST_CID_XCVR_FIX"]:
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],
                    "RANGE_COUNT" : self.data_in[35:43],
                    "RANGE_TIME" : self.data_in[43:51],
                    "RANGE_DIST": self.data_in[51:55],
                    }   
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : self.hex_to_int(status_encrypt["FLAGS"],1) ,
                    "MSG_TYPE" : self.hex_to_int(status_encrypt["MSG_TYPE"],1)  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 ,
                    "RANGE_COUNT" : self.hex_to_int(status_encrypt["RANGE_COUNT"],1) ,
                    "RANGE_TIME": self.hex_to_int(status_encrypt["RANGE_TIME"],1)/10000000 ,
                    "RANGE_DIST" : self.hex_to_int(status_encrypt["RANGE_DIST"],1)/10 ,
                    }

                elif Command == self.CID["ST_CID_PING_RESP"] :
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],
                    "RANGE_COUNT" : self.data_in[35:43],
                    "RANGE_TIME": self.data_in[43:51],
                    "RANGE_DIST" : self.data_in[51:55],
                    }   
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : self.hex_to_int(status_encrypt["FLAGS"],1) ,
                    "MSG_TYPE" : self.hex_to_int(status_encrypt["MSG_TYPE"],1)  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 ,
                    "RANGE_COUNT" : self.hex_to_int(status_encrypt["RANGE_COUNT"],1) ,
                    "RANGE_TIME": self.hex_to_int(status_encrypt["RANGE_TIME"],1)/10000000 ,
                    "RANGE_DIST" : self.hex_to_int(status_encrypt["RANGE_DIST"],1)/10 ,
                    }

                elif Command == self.CID["ST_CID_PING_SEND"] :
                    status_encrypt={
                    "CmdId": Command,
                    "STATUS" : self.data_in[3:5],
                    "BEACON_ID": self.data_in[5:7],
                    }   
                    status_decrypt={ 
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "STATUS" : self.check_serial_to_command(self.CST,status_encrypt["STATUS"]),
                    "BEACON_ID": self.hex_to_int(status_encrypt["BEACON_ID"],1) ,
                    }
                
                elif Command == self.CID["ST_CID_NAV_STATUS_RECEIVE"] :
                    status_encrypt={
                    "CmdId": Command,
                    "DEST_ID" : self.data_in[3:5],
                    "SRC_ID": self.data_in[5:7],
                    "FLAGS" : self.data_in[7:9],
                    "MSG_TYPE" : self.data_in[9:11],
                    "ATTITUDE_YAW": self.data_in[11:15],
                    "ATTITUDE_PITCH" : self.data_in[15:19],
                    "ATTITUDE_ROLL": self.data_in[19:23],
                    "DEPTH_LOCAL" : self.data_in[23:27],
                    "VOS" : self.data_in[27:31],
                    "RSSI": self.data_in[31:35],
                    "BEACON_ID": self.data_in[35:37],
                    "MSG_PAYLOAD_LEN": self.data_in[37:39],
                    "MSG_PAYLOAD": self.data_in[39:-6],
                    "LOCAL_FLAG": self.data_in[-6:-4],
                    }   
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "DEST_ID" : self.hex_to_int(status_encrypt["DEST_ID"],1) ,
                    "SRC_ID": self.hex_to_int(status_encrypt["SRC_ID"],1) ,
                    "FLAGS" : status_encrypt["FLAGS"] ,
                    "MSG_TYPE" : status_encrypt["MSG_TYPE"]  ,
                    "ATTITUDE_YAW" : self.hex_to_int(status_encrypt["ATTITUDE_YAW"],1)/10 ,
                    "ATTITUDE_PITCH" : self.hex_to_int(status_encrypt["ATTITUDE_PITCH"],1)/10 ,
                    "ATTITUDE_ROLL": self.hex_to_int(status_encrypt["ATTITUDE_ROLL"],1) /10,
                    "DEPTH_LOCAL" : self.hex_to_int(status_encrypt["DEPTH_LOCAL"],1) /10,
                    "VOS" : self.hex_to_int(status_encrypt["VOS"],1) /10,
                    "RSSI": self.hex_to_int(status_encrypt["RSSI"],1)/10 ,
                    "BEACON_ID": self.hex_to_int(status_encrypt["BEACON_ID"],1) ,
                    "MSG_PAYLOAD_LEN": self.hex_to_int(status_encrypt["MSG_PAYLOAD_LEN"],1) ,
                    "MSG_PAYLOAD" : self.hex_to_string(status_encrypt["MSG_PAYLOAD"])  ,
                    "LOCAL_FLAG" : bool(int(status_encrypt["LOCAL_FLAG"],16)),
                    } 

                elif Command == self.CID["ST_CID_NAV_STATUS_SEND"] :
                    status_encrypt={
                    "CmdId": Command,
                    "STATUS" : self.data_in[3:5],
                    "BEACON_ID": self.data_in[5:7],
                    }   
                    status_decrypt={
                    "CmdId":self.check_serial_to_command(self.CID,status_encrypt["CmdId"]),
                    "STATUS" : self.check_serial_to_command(self.CST,status_encrypt["STATUS"]),
                    "BEACON_ID": self.hex_to_int(status_encrypt["BEACON_ID"],1) ,
                    }

                else : 
                    status_encrypt,status_decrypt=None,None

            else : 
                    status_encrypt,status_decrypt=None,None
                    
            return status_encrypt,status_decrypt

    def serial_close(self):
        self.ser.close()
        print("Close serial port %s"%self.COM_PORT)