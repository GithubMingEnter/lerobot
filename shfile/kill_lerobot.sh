
# Find the PID of the process
PID=$(pgrep -f "lerobot/scripts/control_robot.py")

# Check if the PID was found
if [ ! -z "$PID" ]; then
    echo "Killing process with PID: $PID"
    kill -9 $PID
else
    echo "No matching process found."
fi