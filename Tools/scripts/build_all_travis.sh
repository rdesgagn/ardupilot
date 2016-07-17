#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. config.mk

set -e
set -x

. ~/.profile

# If TRAVIS_BUILD_TARGET is not set, default to all of them
if [ -z "$TRAVIS_BUILD_TARGET" ]; then
    TRAVIS_BUILD_TARGET="sitl linux apm2 atmega navio"
fi

declare -A build_platforms
declare -A build_concurrency
declare -A build_extra_clean

build_platforms=(  ["ArduPlane"]="apm2 atmega navio sitl linux"
                   ["ArduCopter"]="navio sitl linux"
                   ["APMrover2"]="apm2 atmega navio sitl linux"
                   ["AntennaTracker"]="apm2 atmega navio sitl linux"
                   ["Tools/Replay"]="linux")

build_concurrency=(["apm2"]="-j2"
                   ["atmega"]="-j2"
                   ["navio"]="-j2"
                   ["sitl"]="-j2"
                   ["linux"]="-j2")

#build_extra_clean=(["px4-v2"]="make px4-cleandep")

echo "Targets: $TRAVIS_BUILD_TARGET"
for t in $TRAVIS_BUILD_TARGET; do
    for v in ${!build_platforms[@]}; do
        if [[ ${build_platforms[$v]} != *$t* ]]; then
            continue
        fi
        echo "Building $v for ${t}..."

        pushd $v
        make clean
        if [ ${build_extra_clean[$t]+_} ]; then
            ${build_extra_clean[$t]}
        fi

        make $t ${build_concurrency[$t]}
        popd
    done
done
