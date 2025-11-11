# CommonRoad Interactive Scenarios
This package provides the functionality of simulating  interactive scenarios by coupling CommonRoad with [SUMO](https://sumo.dlr.de/docs/index.html) traffic simulator. 

The code is written in Python 3.7 and has been tested on Ubuntu 18.04. If you encounter any issue, please raise it in our [forum](https://commonroad.in.tum.de/forum/c/sumo-interface).

## Installation
This project uses Conda, thus it should be installed before proceeding with the installation.

1. Creating a new environment
    You can either use your existing environment which has Python version 3.7, or create a new one by running
    
    ```bash
    conda create -n cr37 python=3.7
   ```
   
   Here the environment is named `cr37`. If your Conda environment has a name different from `cr37`, it should be replaced in the following commands.
   
2. Configuring Git

    If you wish to avoid entering your Git credentials many times, it can be configured to save the credentials:
    ```bash
    git config --global credential.helper store
    ```

3. Installing the dependencies

   The following commands install the dependencies of this package, the main dependencies include:

   - [CommonRoad-SUMO Interface](https://gitlab.lrz.de/tum-cps/commonroad-sumo-interface)
   - [SUMO](https://sumo.dlr.de/docs/index.html)

   Install the package using the following command:

   ```bash
   bash install.sh -e cr37 --sumo
   ```
   It will create a folder `install/`, pull all the dependencies and install them there.
   
   

4. Updating the environment variables

    If you have just installed SUMO, the `SUMO_PATH` environment variable has been written into the `~/.profile` file. To reach this variable from an IDE (e.g., PyCharm), you must **reboot your system**.
    
    *Alternatively*, you can choose to use the dockerized sumo installation instead of installing SUMO by the command```bash install.sh -e cr37``` . To use dockerized sumo simulation, you have to install [Docker](https://docs.docker.com/engine/install/ubuntu/) and follow the [postinstall instructions](https://docs.docker.com/engine/install/linux-postinstall/) as well, to ensure that **you can use the Docker without root privileges**.
   
   Run the following to check that your docker is successfully installed.
   ```bash
   docker run hello-world
   ```
   
   The rest

## Usage

Follow `tutorial/1_tutorial_scenario_simulation.ipynb` to learn how to simulate interactive scenarios without ego vehicle, with plugged-in motion planners, and with given solution trajectories. Additionally, a minimal example python script is provided at `tutorial/minimal_example.py`.

## Example

The `Main` and `Secondary` plots show the scenario with and without the ego vehicle, respectively.

<img src="outputs/gifs/README/USA_US101-26_2_I-1.gif" alt="USA_US101-26_2_I-1" style="zoom:80%;" />

## Known Issues
1. If you would like to build the SUMO-GUI, you can follow the procedure in the install script; however, you might encounter the following issues

   - The GUI is not in the list of enabled features
   - FOX is not found
   - DSO missing from command line

   Under Ubuntu 18.04, these problems can be resolved by copying the content of the folder `misc/sumo_gui` into the root folder of `sumo`. This will fix the modules missing issues. After overwriting the existing CMake files, you should rebuild the CMake files and build the SUMO again. 

2. If you encounter errors with `ffmpeg` while saving GIFs, you can install `imagemagick` and substitute the backend when saving them:

   Install with:

   ```bash
   sudo apt-get install imagemagick
   ```

   Then navigate to `install/sumo_interface/sumocr/visualization/gif.py`, in line 152, set writer to `imagemagick`.
