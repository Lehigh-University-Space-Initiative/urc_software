#!/bin/bash

cd "$(dirname "$0")"

if ["$1" == "-i"]; then
    clang-format
else

fi