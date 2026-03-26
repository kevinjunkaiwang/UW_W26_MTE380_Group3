import serial
import threading
import time
import curses


class SerialCommunication:
    def __init__(self, port="/dev/ttyACM0", baud=115200, auto_connect=True):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.read_thread = None
        self.current_speed = 150

        if auto_connect:
            self.connect()

    # ===============================
    # CONNECT / DISCONNECT
    # ===============================
    def connect(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2)

        self.running = True
        self.read_thread = threading.Thread(
            target=self._read_loop, daemon=True
        )
        self.read_thread.start()

        print(f"[SerialCommunication] Connected to {self.port}")

    def disconnect(self):
        self.running = False
        if self.ser:
            self.ser.close()
            print("[SerialCommunication] Disconnected")

    # ===============================
    # BACKGROUND SERIAL READER
    # ===============================
    def _read_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line:
                        self.on_message(line)
            except Exception as e:
                print("[SerialCommunication] Read error:", e)
                break

    def on_message(self, message):
        print(f"[Arduino] {message}")

    # ===============================
    # LOW LEVEL SEND
    # ===============================
    def send_raw(self, text):
        if self.ser:
            self.ser.write((text + "\n").encode())

    # ===============================
    # HIGH LEVEL COMMANDS
    # ===============================
    def forward(self, speed=None):
        if speed is None:
            speed = self.current_speed
        self.send_raw(f"F {speed}")

    def backward(self, speed=None):
        if speed is None:
            speed = self.current_speed
        self.send_raw(f"B {speed}")

    def left(self, speed=None):
        if speed is None:
            speed = self.current_speed
        self.send_raw(f"L {speed}")

    def right(self, speed=None):
        if speed is None:
            speed = self.current_speed
        self.send_raw(f"R {speed}")

    def stop(self):
        self.send_raw("S")

    def detect_and_press(self):
        self.send_raw("P")

    def release(self):
        self.send_raw("D")

    # ===============================
    # TELEOP MODE
    # ===============================
    def teleop(self):
        print("\nStarting Teleop Mode")
        print("w/s/a/d move | space stop | +/- speed | p press | r release | q quit\n")

        curses.wrapper(self._teleop_loop)


    def _teleop_loop(self, stdscr):
        stdscr.nodelay(True)
        stdscr.clear()

        last_motion = None  # track current direction

        while True:
            key = stdscr.getch()

            if key == ord('w'):
                last_motion = 'F'
                self.forward()
            elif key == ord('s'):
                last_motion = 'B'
                self.backward()
            elif key == ord('a'):
                last_motion = 'L'
                self.left()
            elif key == ord('d'):
                last_motion = 'R'
                self.right()
            elif key == ord(' '):
                last_motion = None
                self.stop()
            elif key == ord('p'):
                self.detect_and_press()
            elif key == ord('r'):
                self.release()
            elif key == ord('+'):
                self.current_speed = min(255, self.current_speed + 10)
                # resend motion if currently moving
                if last_motion:
                    self.send_raw(f"{last_motion} {self.current_speed}")
            elif key == ord('-'):
                self.current_speed = max(0, self.current_speed - 10)
                if last_motion:
                    self.send_raw(f"{last_motion} {self.current_speed}")
            elif key == ord('q'):
                self.stop()
                break

            stdscr.clear()
            stdscr.addstr(0, 0, "=== TELEOP MODE ===")
            stdscr.addstr(2, 0, f"Speed: {self.current_speed}")
            stdscr.addstr(4, 0, f"Direction: {last_motion if last_motion else 'Stopped'}")
            stdscr.addstr(6, 0, "w/s/a/d move")
            stdscr.addstr(7, 0, "space stop")
            stdscr.addstr(8, 0, "+/- speed")
            stdscr.addstr(9, 0, "p press")
            stdscr.addstr(10, 0, "r release")
            stdscr.addstr(11, 0, "q quit")

            stdscr.refresh()
            time.sleep(0.05)`