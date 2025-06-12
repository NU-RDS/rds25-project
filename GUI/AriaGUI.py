import serial
import matplotlib.pyplot as plt
import numpy as np
import time
import sys
import threading


class SerialGUI:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.buffer_len = 1000
        self.time_ls = []
        self.ref_ls = []
        self.data_ls = []
        self.Ff = 0.
        self.Kp = 0.
        self.Ki = 0.
        self.Kd = 0.
        self.ref_type = 0
        self.ser = None
        self.stop_flag = False

    def initSerial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print(f'Opening port: {self.ser.name}')
            print("Serial connection established successfully!")
            return True
        except Exception as e:
            print(f"Error connecting to serial port: {e}")
            return False

    def parsingSerial(self):
        try:
            data_str = self.ser.read_until(b'\n').decode('utf-8')
            values = data_str.strip().split(" ")
            if len(values) >= 3:
                timestamp = float(values[0])  # ms since start
                ref = float(values[1])
                data = float(values[2])
                return timestamp, ref, data
            else:
                print(f"Unexpected data format: {data_str}")
                return None, None, None
        except Exception as e:
            print(f"Error parsing data: {e}")
            return None, None, None

    def key_capture_thread(self):
        """Thread to capture keyboard input to stop data collection"""
        input("Press Enter to stop data collection...\n")
        self.stop_flag = True

    def livePlot(self, duration=3.0):  # Default to 3 seconds
        # Reset collections
        self.time_ls = []
        self.ref_ls = []
        self.data_ls = []
        self.stop_flag = False

        # Clear any pending data
        self.ser.reset_input_buffer()

        # Send command to start PID
        self.ser.write(b'5\n')

        # Wait for start signal
        print("Waiting for data stream to start...")
        while True:
            line = self.ser.read_until(b'\n').decode('utf-8').strip()
            if line == "START_DATA_STREAM":
                break

        # Start a thread to capture keyboard input
        capture_thread = threading.Thread(target=self.key_capture_thread)
        capture_thread.daemon = True
        capture_thread.start()

        print(
            f"Collecting data for {duration} seconds... (Press Enter to stop earlier)")
        start_time = time.time()
        sample_count = 0

        # Collection loop
        try:
            while not self.stop_flag:
                # Check if there's data to read
                if self.ser.in_waiting > 0:
                    timestamp, ref, data = self.parsingSerial()

                    # Only process valid data
                    if timestamp is not None:
                        self.time_ls.append(timestamp)
                        self.ref_ls.append(ref)
                        self.data_ls.append(data)
                        sample_count += 1

                        # Display progress every 100 samples
                        if sample_count % 100 == 0:
                            print(f"Collected: {sample_count} samples")
                            print(
                                f"Time: {timestamp:.1f}ms, Ref: {ref:.3f}, Act: {data:.3f}")

                # Stop after specified duration
                if time.time() - start_time > duration:
                    print(f"{duration} seconds elapsed - stopping data collection")
                    break

                # Small sleep to prevent excessive CPU usage
                time.sleep(0.001)

        except KeyboardInterrupt:
            print("Data collection interrupted")
        finally:
            # Send command to stop PID
            self.ser.write(b'9\n')
            print("Sent stop command to Teensy")

            # Wait for stop acknowledgement with a timeout
            timeout = time.time() + 2.0  # 2 second timeout
            while time.time() < timeout:
                if self.ser.in_waiting > 0:
                    line = self.ser.read_until(b'\n').decode('utf-8').strip()
                    if line == "DATA_STREAM_STOPPED":
                        print("Data stream stopped successfully")
                        break
                time.sleep(0.1)

        # Plot the collected data if we have enough samples
        if len(self.time_ls) > 1:
            # Convert time to seconds for better readability
            time_sec = [t/1000 for t in self.time_ls]

            plt.figure(figsize=(12, 8))

            # Plot reference vs actual force
            plt.subplot(2, 1, 1)
            plt.plot(time_sec, self.ref_ls, 'r-', label="Reference Force")
            plt.plot(time_sec, self.data_ls, 'b-', label="Actual Force")
            plt.title("Force Control Performance")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Force (N)")
            plt.legend()
            plt.grid(True)

            # Plot error
            plt.subplot(2, 1, 2)
            error = [ref - act for ref, act in zip(self.ref_ls, self.data_ls)]
            plt.plot(time_sec, error, 'g-')
            plt.title("Force Error")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Error (N)")
            plt.grid(True)

            plt.tight_layout()
            plt.show()

            # Print statistics
            rms_error = np.sqrt(np.mean(np.array(error)**2))
            max_error = np.max(np.abs(error))
            print("\nPerformance Statistics:")
            print(f"Total samples: {len(self.time_ls)}")
            print(f"Test duration: {time_sec[-1]:.2f} seconds")
            print(f"RMS Error: {rms_error:.4f} N")
            print(f"Maximum Error: {max_error:.4f} N")
        else:
            print("Not enough data points collected for plotting")

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

            try:
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
                    duration = 3.0
                    print(
                        f"Running PID for {duration} seconds (Press Enter to stop early)")
                    self.livePlot(duration)

                elif selection == '6':
                    try:
                        # Clear buffer first to ensure we get fresh values
                        self.ser.reset_input_buffer()

                        # Read parameters
                        ff = float(self.ser.read_until(
                            b'\n').decode('utf-8').strip())
                        kp = float(self.ser.read_until(
                            b'\n').decode('utf-8').strip())
                        ki = float(self.ser.read_until(
                            b'\n').decode('utf-8').strip())
                        kd = float(self.ser.read_until(
                            b'\n').decode('utf-8').strip())
                        force_type = int(
                            float(self.ser.read_until(b'\n').decode('utf-8').strip()))

                        self.Ff = ff
                        self.Kp = kp
                        self.Ki = ki
                        self.Kd = kd
                        self.ref_type = force_type

                        # Display the parameters
                        print("Current PID parameters from controller:")
                        print(f"Feedforward (Ff): {self.Ff}")
                        print(f"Proportional (Kp): {self.Kp}")
                        print(f"Integral (Ki): {self.Ki}")
                        print(f"Derivative (Kd): {self.Kd}")
                        print(
                            f"Reference type: {'STEP' if self.ref_type == 0 else 'SIN'}")
                    except Exception as e:
                        print(f"Error reading PID parameters: {e}")

                elif selection == '7':
                    print("Reading 100 encoder values...")
                    for i in range(100):
                        try:
                            encoder_str = self.ser.read_until(
                                b'\n').decode('utf-8').strip()
                            encoder = float(encoder_str)
                            print(f"Encoder {i+1}: {encoder}")
                        except Exception as e:
                            print(f"Error reading encoder value: {e}")
                            break

                elif selection == '8':
                    print("Exiting program...")
                    self.ser.close()
                    break

                else:
                    # Handle invalid input
                    print(f"Invalid selection: {selection}")

            except Exception as e:
                print(f"Error processing command: {e}")


if __name__ == "__main__":
    # Use the correct port for your system
    default_port = '/dev/tty.usbmodem160567101'

    serialGUI = SerialGUI(default_port, 115200)
    serialGUI.run()
