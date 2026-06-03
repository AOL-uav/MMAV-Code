$BRANCH = "Kai"
while($true) {
    git pull --rebase origin $BRANCH
    if (git status --porcelain) {
        git add .
        $date = Get-Date -Format "yyyy-MM-dd HH:mm"
        git commit -m "Auto-save (Windows): $date"
        git push origin $BRANCH
    }
    Start-Sleep -Seconds 60
}
