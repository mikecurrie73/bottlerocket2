#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Check if a commit message was provided.
if [ -z "$1" ]; then
  echo "Error: No commit message provided."
  echo "Usage: ./push.sh \"Your commit message\""
  exit 1
fi

# Git commands
git add .
git commit -m "$1"
git push origin master
