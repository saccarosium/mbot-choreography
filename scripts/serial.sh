#!/bin/sh

set -u

stty -F "$1" \
    cs8 9600 \
    ignbrk \
    -brkint \
    -imaxbel \
    -opost \
    -onlcr \
    -isig \
    -icanon \
    -iexten \
    -echo \
    -echoe \
    -echok \
    -echoctl \
    -echoke noflsh \
    -ixon \
    -crtscts
