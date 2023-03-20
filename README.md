# A Web App for the Keynetics Robotics Project

Part of the 2022-2023 senior capstone design project by:
* Austin Kugler
* Taylor Martin
* Zach Preston

# Setup (Linux or WSL)
Install ROS 2 Humble:
* Main installation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
* Add ROS source command to setup scripts:
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Install repository:
```
git clone https://github.com/Trmart/Keynetics-Web-App.git
cd Keynetics-Web-App
python3 -m venv env
source env/bin/activate
pip install -r requirements.txt
```

Create Database:
In the root directory, create a file named `.env` with the contents:
```
FLASK_SECRET_KEY='fredthebot'
DATABASE_URL='sqlite:///app.db'
APP_URL='http://127.0.0.1:5000/'
```
Then, run the below command.
```
python3 create_db.py
```

Run Web App Locally:
```
python3 run.py
```