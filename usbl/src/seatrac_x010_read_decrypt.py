#!/usr/bin/env python3
import seatrac_serial
import time
import pandas as pd

if __name__ == "__main__":

    seatrac=seatrac_serial.Setrac_serial(MODEL="x010",COM_PORT="/dev/x010",BAUD_RATE=115200)
    try:
        while 1:
                serial_input=(seatrac.serial_read())
                
                status_encrypt,status_decrypt=seatrac.decrypt_command(status=False,command=False)
                command_encrypt,command_decrypt=seatrac.decrypt_command(status=False,command=True)
                #print(serial_input)
                if (status_encrypt is not None and status_decrypt is not None) :
                    df_status_decrypt=pd.DataFrame([status_encrypt,status_decrypt],index=['encrypt',"decrypt"]).T
                    print(df_status_decrypt)
                    
                if (command_encrypt is not None and command_decrypt is not None):
                    df_command_decrypt=pd.DataFrame([command_encrypt,command_decrypt],index=['encrypt',"decrypt"]).T
                    print(df_command_decrypt)

    except KeyboardInterrupt:
        print("pressed ctrl-c button.")
        print("Exit program")
        
