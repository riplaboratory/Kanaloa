# Comms Messenger

## About

`comms_messenger` is a pure python2.7 module that implements the Communications Protocol spec and requirement given by RobotX for the 2018 RobotX Maritime Challenge. It contains a single class `CommsMessenger` that is used for all communication with the Technical Director server. The RobotX spec sheet can be found [here](https://www.robotx.org/images/RobotX-2018-Communications-Protocol_v1.0.pdf).

## Examples

## Installation

### Prerequisites

This module is meant to be `import`ed into your python script.
To test and run the code from this folder, you will need to:

Install [pipenv](https://pipenv.readthedocs.io/en/latest/install/#installing-pipenv).

### Installing

There is nothing to install. You can however setup your environment by running `pipenv install` from the folder this README.md file is contained in.

## Testing

* Download all testing packages from this root folder using `pipenv install --dev`.
* Run `pipenv run pytest` from this folder.
* If you want to see print statements, run `pipenv run pytest -s` from this folder instead.
