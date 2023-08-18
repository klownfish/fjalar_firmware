#!/bin/sh
nanopb_generator.py -D application/common/protocol/ schema.proto -I /usr/lib/python3/dist-packages/proto/ -I . -C
protoc --python_out=dashboard/ nanopb.proto schema.proto -I /usr/lib/python3/dist-packages/proto/ -I .