#!/bin/bash

REPO_DIR="/home/kaikeller/Purdue/AOL/MAV"
BRANCH="Kai"

cd "$REPO_DIR" || exit

echo "Watching for changes in $REPO_DIR on branch $BRANCH (Two-way sync active)..."

# Initial pull to start in sync
git pull --rebase origin "$BRANCH"

while true; do
    # Wait for a change or every 5 minutes regardless to check for remote updates
    inotifywait -r -t 300 -e close_write,moved_to,create "$REPO_DIR" --exclude '\.git'
    
    # 1. Pull changes from GitHub first
    echo "Checking for remote changes..."
    git pull --rebase origin "$BRANCH"
    
    # 2. Add and Commit local changes
    if [[ -n $(git status -s) ]]; then
        echo "Local changes detected. Syncing..."
        git add .
        git commit -m "Auto-save: $(date +'%Y-%m-%d %H:%M')"
        git push origin "$BRANCH"
    fi
    
    sleep 2
done
