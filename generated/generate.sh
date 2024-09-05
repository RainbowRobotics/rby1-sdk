#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

##

rm -rf "$SCRIPT_DIR"/python/*
python -m grpc_tools.protoc -I "$SCRIPT_DIR"/../protos/ --python_out="$SCRIPT_DIR"/python/ \
  --pyi_out="$SCRIPT_DIR"/python/ --grpc_python_out="$SCRIPT_DIR"/python/ "$SCRIPT_DIR"/../protos/rb/api/*.proto