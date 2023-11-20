import seatrac_serial
import time
import pandas as pd

if __name__ == "__main__":
    seatrac=seatrac_serial.Setrac_serial(MODEL="x010",COM_PORT="/dev/ttyS0",BAUD_RATE=115200)
    try: 
        while 1:
            
                #seatrac.serial_write(seatrac.ping_x150_command("ST_CID_PING_SEND","beacon_id_15","ST_AMSG_REQX"))
                seatrac.serial_write(seatrac.send_data_x150_command("ST_CID_NAV_STATUS_SEND","beacon_id_15","TEST"))

                time.sleep(5)
                
    except KeyboardInterrupt:
        print("pressed ctrl-c button.")
        print("Exit program")