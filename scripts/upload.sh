#!/bin/sh

while read -r type port; do
    if [ "$type" = 'arduino' ]; then
        if [ -e "$port" ]; then
            echo "Flashing $port"
            arduino-cli compile -b arduino:avr:mega --port "$port" --upload mbot/firmware
        else
            echo "$port: doesn't exists" >&2
        fi
    elif [ "$type" = 'mindstorm' ]; then
        echo "TODO: fai commando per mindstorm"
    else
        echo "$type: is not supported" >&2
    fi

    # Delimiter '------'
    for _ in $(seq 0 100); do printf '-'; done
    printf '\n'
done <boards
