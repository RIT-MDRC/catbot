<img src="https://pbs.twimg.com/profile_images/661962002/logo_400x400.png" align="right" alt="MDRC LOGO" title="MDRC LOGO" width="100">

# Catbot
Quadruped Hybrid Pneumatic-Electric Robot. Utilizes McKibben Muscles for bio-inpired locomotion.

## Setup

### Docker Dev Container
* Install [Docker](https://docs.docker.com/get-docker/)
* `ctrl+shift+p` -> `Dev Containers: Reopen in Container`

### Setup Docker environment
* Run the following
* `source .venv/bin/activate`
* `pip install -r requirements.txt`
* `source /opt/ros/jazzy/setup.bash`
* `colcon build`
* `source install/local_setup.bash`

### Run script
Build the packages:
* `colcon build`
To run the UI package:
* `ros2 run ui ui`

### Install dependencies
* install [python](https://www.python.org/downloads/)
* install [nodejs](https://nodejs.org/en/download/)
* install [C/C++ extention](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) for VSCode
* install [Arduino extention](https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino) for VSCode
### Install python dependencies
Run the following command in the terminal:
```bash
pip install -r requirements.txt
```
for macos:
```bash
pip3 install -r requirements.txt
```
### Install nodejs dependencies and catbot-cli
Run the following command in the terminal:
```bash
npm -g install
catbot
```

For more command information goto [catbot-cli](cli/README.md)

If all above fails:
* Consider using the arduino-ide and uploading the code through the ide

## Contributing
### Before contributing
* Make sure to read the documentation in the [wiki](https://github.com/RIT-MDRC/Catbot/wiki)