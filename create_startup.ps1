$startupFolder = [System.IO.Path]::Combine($env:APPDATA, 'Microsoft\Windows\Start Menu\Programs\Startup')
$shortcutPath = [System.IO.Path]::Combine($startupFolder, 'MAV_Sync.lnk')
$ws = New-Object -ComObject WScript.Shell
$s = $ws.CreateShortcut($shortcutPath)
$s.TargetPath = 'wscript.exe'
$s.Arguments = '"C:\Users\Kai\OneDrive - purdue.edu\Purdue\AOL\MAV-2026\Controls_PCB\silent_launch.vbs"'
$s.Save()
