#!/bin/bash

export NODENAME=$1
export DB_ADDRESS=${1}db.internal.wambacluster.com

for NUM_OPERATIONS in 2048; do
	for NUM_ELEMENTS in 16 32 64 128 256 512 1024 2048; do
		for FREQUENCY in 5 10; do
			for ((i=0; i < $2; i++))
			do
				echo "Executing for NUM_OPERATIONS=$NUM_OPERATIONS NUM_ELEMENTS=$NUM_ELEMENTS FREQUENCY=$FREQUENCY
				ed-hypertable profile_query.yaml  _name:=$1  &> test-${NUM_OPERATIONS}o-${NUM_ELEMENTS}e-${FREQUENCY}hz-a$i.txt
			done
		done
	done
done
