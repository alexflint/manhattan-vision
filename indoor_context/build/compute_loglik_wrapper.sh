#!/bin/bash

LD_PRELOAD=/usr/lib/liblapack.so progs/compute_loglik "$@"
