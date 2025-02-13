sudo -S cpufreq-set -g performance
sudo -S ifconfig can0 down
sudo -S ifconfig can0 up

human_loop=$(python3 -c "import json; print(json.load(open('/home/debian/FEATHER/feather_bbai/settings.json'))['preference'][0]['Human_loop'])")


if [[ $human_loop == "True" ]]; then
	echo "Starting demo script"
	python3 /home/debian/FEATHER/feather/FEATHER_demo.py
else
	echo "Starting tuning script"
	python3 /home/debian/FEATHER/feather/FEATHER_optimization.py
fi
