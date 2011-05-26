#!/bin/bash

LD_PRELOAD=/usr/lib/liblapack.so ./compute_loglik "$@"
