# OO_Controller.py: xbox controller as an extra e-stop (windows only). X = e-stop, B = reset, A = restart, unplugged = e-stop
# calls the same functions as the GUI buttons, so the same reset-then-restart rules apply. run this file on its own to test it

import ctypes
import time

A_BUTTON = 0x1000
B_BUTTON = 0x2000
X_BUTTON = 0x4000


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [("wButtons", ctypes.c_uint16), ("bLeftTrigger", ctypes.c_uint8),
                ("bRightTrigger", ctypes.c_uint8), ("sThumbLX", ctypes.c_int16),
                ("sThumbLY", ctypes.c_int16), ("sThumbRX", ctypes.c_int16),
                ("sThumbRY", ctypes.c_int16)]

class XINPUT_STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", ctypes.c_uint32), ("Gamepad", XINPUT_GAMEPAD)]


class XboxController:

    # Give it the three functions to run: for e-stop, back (reset) and start (restart).
    def __init__(self, on_estop, on_back, on_start):
        self.on_estop = on_estop              # runs when X is pressed
        self.on_back = on_back                # runs when B is pressed
        self.on_start = on_start              # runs when A is pressed
        self.connected = False                # is a controller plugged in right now?
        self.x_before = False                 # was X held down last time we checked?
        self.b_before = False                 # was B held down last time?
        self.a_before = False                 # was A held down last time?
        self.last_try = 0.0                   # last time we looked for a missing controller
        try:
            self.xinput = ctypes.windll.LoadLibrary("xinput1_4.dll")   # Windows controller library
        except (AttributeError, OSError):     # not Windows, or library missing
            self.xinput = None
            print("Xbox controller: not available on this computer")

    # Returns (is_connected, buttons_held).
    def read(self):
        if self.xinput is None:                           # no library, so no controller
            return False, 0
        state = XINPUT_STATE()                            # empty box for Windows to fill in
        result = self.xinput.XInputGetState(0, ctypes.byref(state))   # ask for controller number 0
        if result != 0:                                   # anything but 0 means not connected
            return False, 0
        return True, state.Gamepad.wButtons               # connected, and the buttons held

    # Called automatically by Workcell.step() every animation step.
    def poll(self):
        # If no controller, only look again once a second (looking is slow).
        if not self.connected and time.monotonic() - self.last_try < 1.0:
            return
        self.last_try = time.monotonic()

        connected, buttons = self.read()                  # what is the controller doing now?

        if connected and not self.connected:              # it has just been plugged in
            print("Xbox controller connected")
        if self.connected and not connected:              # it has just disappeared
            print("Xbox controller DISCONNECTED - emergency stop")
            self.on_estop()                               # fail-safe: stop everything
        self.connected = connected                        # remember for next time

        # Which buttons are held down right now?
        x_now = bool(buttons & X_BUTTON)
        b_now = bool(buttons & B_BUTTON)
        a_now = bool(buttons & A_BUTTON)

        # only act when a button is first pressed, not while held or on release
        if x_now and not self.x_before:
            print("Xbox controller: X -> EMERGENCY STOP")
            self.on_estop()
        if b_now and not self.b_before:
            print("Xbox controller: B -> back (reset)")
            self.on_back()
        if a_now and not self.a_before:
            print("Xbox controller: A -> start (restart)")
            self.on_start()

        # Remember the buttons for the next check.
        self.x_before = x_now
        self.b_before = b_now
        self.a_before = a_now


# Prints a message when you press X, B or A, so you can check the controller works.
if __name__ == "__main__":
    test = XboxController(lambda: print("  (e-stop)"), lambda: print("  (back)"), lambda: print("  (start)"))
    print("Press X, B or A on the controller. Ctrl+C to quit.")
    try:
        while True:
            test.poll()                                   # check the controller
            time.sleep(0.02)                              # short sleep so this test loop is gentle on the CPU
    except KeyboardInterrupt:
        print("Done")