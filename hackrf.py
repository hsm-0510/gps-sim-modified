import subprocess
import pyautogui
import pygetwindow as gw
import time

# Step 1: Launch the startHackrf.bat file in a new command prompt
process = subprocess.Popen(['start', 'cmd', '/k', 'hackrf_repeated.bat'], shell=True)

# Step 2: Wait for the batch file to start running and the window to appear
time.sleep(5)  # Adjust based on initialization time

# Step 3: Locate the Command Prompt window by its title
target_window = None
for window in gw.getAllTitles():
    # Check if the title contains the specific command pattern
    if "HackRF Repeated" in window:
        target_window = gw.getWindowsWithTitle(window)[0]
        break

if target_window:
    # Step 4: Bring the found window to the foreground
    target_window.activate()
    time.sleep(1)  # Brief pause to ensure focus

    # Step 5: Simulate Ctrl+C to stop the running batch process
    pyautogui.hotkey('ctrl', 'c')
    print("Sent Ctrl+C to the target command prompt window.")
else:
    print("Could not find the command prompt window with the specified title.")
