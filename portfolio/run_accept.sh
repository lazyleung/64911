#!/bin/bash

FILES="../portfolio/acceptance_test"
for f in "$FILES"/*.pass
do
    java simulator.framework.Elevator -pf $f -b 200 -fs 5.0 -monitor RuntimeRequirementsMonitor
done