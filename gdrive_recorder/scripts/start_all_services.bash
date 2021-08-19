#!/usr/bin/env bash

set -x
sudo systemctl start jsk-pr1040-gdrive-recorder.service
sudo systemctl start jsk-fetch15-gdrive-recorder.service
sudo systemctl start jsk-fetch1075-gdrive-recorder.service
set +x
