#!/bin/bash

sudo openocd -f openocd.cfg -c "init" -c "halt" -c "reset halt"
