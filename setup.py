import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="commonroad-interactive-scenarios",
    version="0.5",
    author="Cyber-Physical Systems Group, Technical University of Munich",
    author_email="commonroad@lists.lrz.de",
    description="Functionality of simulating interactive scenarios by coupling CommonRoad with SUMO traffic simulator.",
    long_description=long_description,
    url="https://gitlab.lrz.de/tum-cps/commonroad-interactive-scenarios",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3.7",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
