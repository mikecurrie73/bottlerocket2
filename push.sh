#!/bin/bash

REPO_DIR="/home/mike/mikerocket"
COMMIT_MSG="Auto update on $(date '+%Y-%m-%d %H:%M:%S')"

cd "$REPO_DIR" || {
    echo "❌ Failed to find directory: $REPO_DIR"
    exit 1
}

if [ ! -d ".git" ]; then
    echo "⚙️ Git not initialized — setting up..."
    git init
fi

git remote set-url origin git@github.com:mikecurrie73/bottlerocket.git

echo "📦 Staging changes..."
git add .

if git diff --cached --quiet; then
    echo "⚠️ No changes to commit."
else
    echo "📝 Committing with message: $COMMIT_MSG"
    git commit -m "$COMMIT_MSG"
fi

echo "📥 Pulling latest from GitHub..."
git pull origin main --rebase

echo "🚀 Pushing to GitHub..."
git push origin main

echo "✅ Push complete!"
