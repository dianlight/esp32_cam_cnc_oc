#!/bin/bash
#socat  pty,link=/tmp/virtualcom0,raw  UDP4-LISTEN:1337,ignoreeof,broadcast &
#pio device monitor --port=/tmp/virtualcom0
#kill $!
socat UDP4-RECVFROM:16666,broadcast,fork STDIO
