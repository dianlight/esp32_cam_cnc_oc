#!/bin/bash
socat  pty,link=/tmp/virtualcom0,raw  UDP4-LISTEN:1337 &
pio device monitor --port=/tmp/virtualcom0
kill $!