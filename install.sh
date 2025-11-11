#!/usr/bin/env bash

# Based on the implementation in the CommonRoad-RL and Commonroad-drivability-checker repository

# Constants
INSTALL_SUMO="FALSE"

USAGE="
$(basename "$0") [options] -- installs the dependencies for the commonroad-interactive-benchmark repo.
Options:
    -h | --help   show this help text
    -e ANACONDA_ENV | --env ANACONDA_ENV   name of the environment
    --sumo           install SUMO, default: false
"
# Parse args
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
  -h | --help)
    echo -e "${USAGE}"
    exit 1
    ;;

  -e | --env)
    ENVIRONMENT="$2"
    shift # past argument
    shift # past value
    ;;

  --sumo)
    INSTALL_SUMO="TRUE"
    shift # past argument
    shift # past value
    ;;

  *) # unknown option
    shift              # past argument
    ;;
  esac
done

source activate "$ENVIRONMENT"
which python

PYTHON_VERSION=$(python -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
JOBS=$(($(nproc) - 1))

function safe_cd() {
  cd "${@}" || exit 255
}

function require_sudo() {
  if [[ $EUID -ne 0 ]]; then
    print_info "Permission required, using root."
    sudo ${@}
  else
    ${@}
  fi
}

function extract() {
  FILE="${1}"
  TYPE="${2}"
  if [ "$TYPE" = "zip" ]; then
    unzip -q -o "$FILE"
  elif [ "$TYPE" = "tar.gz" ]; then
    tar -xf --overwrite "$FILE"
  else
    echo "Unsupported archive format"
  fi
}

function get_ifnexist() {
  LINK=$2
  FILE="${LINK##*/}"
  TYPE="$1"
  wget -nv -nc "$LINK"
  extract "$FILE" "$TYPE"
}

mkdir -p install
safe_cd install

BASEDIR="$(pwd)"
function back_to_basedir() {
  safe_cd "${BASEDIR}"
}

echo "Install python requirements"
pip install -r ../requirements.txt

echo "Installing build dependencies"
require_sudo apt-get install -y git unzip cmake


echo "Installing ffmpeg"
require_sudo apt-get install -y ffmpeg

cp pathConfig_DEFAULT.py pathConfig.py
search="SUMO_BINARY = '/home/user/sumo/bin/sumo'"
replace="SUMO_BINARY = '${BASEDIR}/sumo/bin/sumo'"
sed -i "s+${search}+${replace}+g" pathConfig.py
if [ $? -eq 0 ]; then
    echo "SUMO_BINARY has been set"
else
    fail "Could not set SUMO_BINARY in pathConfig.py"
fi
back_to_basedir


if [ "${INSTALL_SUMO}" == "TRUE" ]; then
  echo "Installing SUMO"
  require_sudo apt-get install python3 wget curl g++ libxerces-c-dev libfox-1.6-0 libfox-1.6-dev cmake libsqlite3-dev libgdal-dev libproj-dev libgl2ps-dev
  git clone --recursive https://github.com/eclipse/sumo.git --branch v1_12_0 --depth 1 --single-branch
  safe_cd sumo
  mkdir -p build/cmake-build
  safe_cd build/cmake-build
  cmake ../..
  make -j $JOBS
  safe_cd ../..
  export SUMO_HOME="$PWD"
  export PATH=$PATH:$SUMO_HOME/bin
  echo "export SUMO_HOME=$SUMO_HOME" >> ~/.profile
  echo "export PATH=$PATH:$SUMO_HOME/bin" >> ~/.profile
  safe_cd tools
  pwd >> "${CONDA_PREFIX}/lib/python${PYTHON_VERSION}/site-packages/commonroad.pth"
  safe_cd ..
  back_to_basedir
fi

back_to_basedir



echo "Done"
