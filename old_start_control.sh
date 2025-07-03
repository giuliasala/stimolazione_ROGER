sudo -S cpufreq-set -g performance
sudo -S ifconfig can0 down
sudo -S ifconfig can0 up
echo "Starting FEATHER control script"
python3 FEATHER_control.py
