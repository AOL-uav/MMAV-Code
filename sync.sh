#!/bin/bash

REPO_DIR="/home/kaikeller/Purdue/AOL/MAV"
BRANCH="Kai"

cd "$REPO_DIR" || exit

echo "Watching for changes in $REPO_DIR on branch $BRANCH..."

while inotifywait -r -e close_write,moved_to,create "$REPO_DIR" --exclude '\.git'; do
    git add .
    git commit -m "Auto-save: $(date +'%Y-%m-%d %H:%M')"
    git push origin "$BRANCH"
done
