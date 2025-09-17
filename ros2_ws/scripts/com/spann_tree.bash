#!/bin/bash

tree -L 3 -d -I \
    ".cache*|build*|install*|log*|__pycache__*|output*"
