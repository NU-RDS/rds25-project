import serial
import matplotlib.pyplot as plt 
import numpy as np

class SerialGUI:
    def __init__(self, port, baud):
       self.port = port
       self.baud = baud
       self.buffer_len = 1000
       self.ref_ls = []
       self.data_ls = []
       self.Ff = 0.
       self.Kp = 0.
       self.Ki = 0.
       self.Kd = 0.
       self.ref_type = 0
       self.ser = None
       
    def initSerial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud)
            print(f'Opening port: {self.ser.name}')
            print("Serial connection established successfully!")
            return True
        except Exception as e:
            print(f"Error connecting to serial port: {e}")
            return False

    def parsingSerial(self):
        data_str = self.ser.read_until(b'\n').decode('utf-8')
        values = data_str.strip().split(" ")
        if len(values) >= 2:
            ref = float(values[0])
            data = float(values[1])
        else:
            print("Warning: Received incomplete data")
            ref = 0.0
            data = 0.0
        return ref, data

    def livePlot(self):
        self.ref_ls = []
        self.data_ls = []
        
        print("Collecting data...")
        while True:
            ref, data = self.parsingSerial()
            self.ref_ls.append(ref)
            self.data_ls.append(data)
            
            if len(self.data_ls) >= self.buffer_len:
                break
        
        x_axis = np.arange(len(self.ref_ls))
        plt.figure(figsize=(10, 6))
        plt.plot(x_axis, self.ref_ls, 'r-', label="reference")
        plt.plot(x_axis, self.data_ls, 'b-', label="load cell")
        plt.title("Reference vs. Measurement")
        plt.legend()
        plt.show()
        
    def run(self):
        if not self.initSerial():
            print("Failed to open serial port. Exiting...")
            return
            
        while True:
            print("\nHere are the commands you can select: ")
            print("0. Set Feedforward term")
            print("1. Set Kp")
            print("2. Set Ki")
            print("3. Set Kd")
            print("4. Set reference type")
            print("5. Run PID and plot")
            print("6. Show PID params")
            print("7. Show encoder reading")
            print("8. Quit")
            print("-"*20)
            
            selection = input("Choose a number (int) and hit enter: ")
            selection_endline = selection+'\n'
            self.ser.write(selection_endline.encode())
            
            if selection == '0':
                self.Ff = float(input("Enter Feedforward: "))
                self.ser.write((str(self.Ff)+'\n').encode())
            
            elif selection == '1':
                self.Kp = float(input("Enter Kp: "))
                self.ser.write((str(self.Kp)+'\n').encode())
            
            elif selection == '2':
                self.Ki = float(input("Enter Ki: "))
                self.ser.write((str(self.Ki)+'\n').encode())
            
            elif selection == '3':
                self.Kd = float(input("Enter Kd: "))
                self.ser.write((str(self.Kd)+'\n').encode())
            
            elif selection == '4':
                print("0. STEP")
                print("1. SIN")
                self.ref_type = int(input("Enter reference type (int): "))
                self.ser.write((str(self.ref_type)+'\n').encode())
            
            elif selection == '5':
                self.livePlot()
            
            elif selection == '6':
                ff = float(self.ser.read_until(b'\n').decode('utf-8').strip())
                kp = float(self.ser.read_until(b'\n').decode('utf-8').strip())
                ki = float(self.ser.read_until(b'\n').decode('utf-8').strip())
                kd = float(self.ser.read_until(b'\n').decode('utf-8').strip())
                # force_type = int(self.ser.read_until(b'\n').decode('utf-8').strip())
                force_type = int(float(self.ser.read_until(b'\n').decode('utf-8').strip()))
                
                self.Ff = ff
                self.Kp = kp
                self.Ki = ki
                self.Kd = kd
                self.ref_type = force_type
                
                # Display the parameters
                print("Current PID parameters from controller:")
                print(f"Feedforward (Kp): {self.Ff}")
                print(f"Proportional (Kp): {self.Kp}")
                print(f"Integral (Ki): {self.Ki}")
                print(f"Derivative (Kd): {self.Kd}")
                print(f"Reference type: {'STEP' if self.ref_type == 0 else 'SIN'}")
                
            elif selection == '7':
                print("Reading 100 encoder values...")
                for _ in range(100):
                    encoder_str = self.ser.read_until(b'\n').decode('utf-8').strip()
                    encoder = float(encoder_str)
                    print(f"Encoder value: {encoder}")
                
            else:
                print("Exiting program...")
                self.ser.close()
                break
    
if __name__ == "__main__":
    serialGUI = SerialGUI('/dev/tty.usbmodem166393701', 115200)
    serialGUI.run()