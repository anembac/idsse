#!/bin/bash

# Check if an argument (timestamp X) is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <timestamp>"
    exit 1
fi

# Extract the timestamp from the command line argument
timestamp="$1"

# Define the file names based on the timestamp
misbehaving_reporter_file="misbehaving_reporter_${timestamp}.csv"
received_reporter_file="received_reporter_${timestamp}.csv"

# Check if the files exist
if [ ! -f "$misbehaving_reporter_file" ] || [ ! -f "$received_reporter_file" ]; then
    echo "One or both of the specified files do not exist."
    exit 1
fi

# Execute the grep commands
echo "Total flagged:  $misbehaving_reporter_file: $(grep -c '^' "$misbehaving_reporter_file")"
echo "True Positives: $misbehaving_reporter_file: $(grep -c '^0' "$misbehaving_reporter_file")"
echo "Total attacker messages: $received_reporter_file: $(grep -c '^0' "$received_reporter_file")"
echo "Total messages: $received_reporter_file: $(grep -c '^' "$received_reporter_file")"

