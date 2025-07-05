# mBot Choreography

This is a simple project to orchestrate 2 mBot Ranger and 1 lego robot to do a
coreography while they cannot comunicate with eatch other.

| Name | Quantity | Board |
|---|---|---|
| [mBot Ranger](https://www.makeblock.com/pages/mbot-ranger-robot-building-kit) | 2 | Arduino Mega or Mega 2560 |
| [Lego Mindstorm](https://en.wikipedia.org/wiki/Lego_Mindstorms) | 1 | EV3 Intelligent Brick |

## Install

For setupping the project with the libraries installed use the the `setup.py`.

> [!TIP]
> To see the port to use do a `arduino-cli board list` to check what ports are
> open.

mBot Ranger:
```
arduino-cli compile -b arduino:avr:mega --port <port> --upload <path-to-sketch>
```

Lego Mindstorm, connect via Bluetooth and run:
```
ssh robot@ev3dev
# Default password is maker

gcc file.c -lev3dev-c -o file
```

## Docs

For Lego Mindstorms, follow the documentation on ev3dev:
- [Getting Started with ev3dev](https://www.ev3dev.org/docs/getting-started/)
- [Devices' drivers](https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/) for complete interface of sensors and motors.
- [ev3dev-C library](https://github.com/in4lio/ev3dev-c) for interacting with the drivers

For mBots:
- [MeAuriga library](https://github.com/Makeblock-official/Makeblock-Libraries/blob/master/src/MeAuriga.h)
