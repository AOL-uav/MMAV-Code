# Windows Auto-Sync Script for Gemini/MAV Project
# Save this as 'win_sync.ps1' in your project root

$REPO_DIR = Get-Location
$BRANCH = "Kai"

Write-Host "Starting Windows Auto-Sync on branch $BRANCH..." -ForegroundColor Cyan

# Initial pull to ensure we are starting in sync
git pull --rebase origin $BRANCH

while($true) {
    Write-Host "Checking for updates..." -ForegroundColor Gray
    
    # 1. Pull changes from GitHub (Other machines -> Windows)
    git pull --rebase origin $BRANCH
    
    # 2. Check for local changes
    $status = git status --porcelain
    if ($status) {
        Write-Host "Local changes detected! Syncing to GitHub..." -ForegroundColor Yellow
        git add .
        $date = Get-Date -Format "yyyy-MM-dd HH:mm"
        git commit -m "Auto-save (Windows): $date"
        git push origin $BRANCH
        Write-Host "Sync Complete." -ForegroundColor Green
    }
    
    # Wait 60 seconds before checking again
    # (Windows doesn't have a built-in 'inotify', so we poll frequently)
    Start-Sleep -Seconds 60
}
