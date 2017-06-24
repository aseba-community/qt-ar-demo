#!/bin/bash
git submodule foreach -q --recursive 'echo "Processing $name" && git pull && echo ""'
echo ""
echo "Processing top-level"
git pull
