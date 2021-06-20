#!/bin/bash

virtualenv venv

source ./venv/bin/activate

pip install --upgrade pip west

west init --manifest-url https://github.com/nrfconnect/sdk-nrf.git \
          --manifest-rev v1.6.0 \
          --manifest-file west.yml
west update

# Install python required packages
pip install --requirement zephyr/scripts/requirements.txt \
            --requirement bootloader/mcuboot/scripts/requirements.txt \
            --requirement nrf/scripts/requirements.txt

# Leave the python virtual environment
deactivate
