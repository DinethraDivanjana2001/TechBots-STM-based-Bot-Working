import serial

def read_from_com_port(com_port, baud_rate, output_file):
    try:
        # Open COM port
        ser = serial.Serial(com_port, baud_rate)
        
        # Open file for writing
        with open(output_file, 'w') as f:
            while True:
                # Read data from COM port
                data = ser.readline().decode('utf-8').strip()
                
                # Write data to file
                f.write(data + '\n')
                f.flush()  # Flush the buffer to ensure data is written immediately
                
    except serial.SerialException as e:
        print("Error:", e)

if __name__ == "__main__":
    com_port = 'COM13'  # Change this to the appropriate COM port on your system
    baud_rate = 115200   # Change this to match the baud rate of your device
    output_file = 'output3.txt'  # Change this to the desired output file name
    
    read_from_com_port(com_port, baud_rate, output_file)