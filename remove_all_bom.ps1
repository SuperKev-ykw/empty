$dirs = @(
    'D:\STUDY\MSPM0G3507\Myworkspace\empty\Hardware',
    'D:\STUDY\MSPM0G3507\Myworkspace\empty\Test',
    'D:\STUDY\MSPM0G3507\Myworkspace\empty\System',
    'D:\STUDY\MSPM0G3507\Myworkspace\empty\system'
)
$count = 0
foreach ($dir in $dirs) {
    if (Test-Path $dir) {
        $files = Get-ChildItem -Path $dir -Recurse -File -Include '*.c','*.h'
        foreach ($f in $files) {
            $bytes = [System.IO.File]::ReadAllBytes($f.FullName)
            $changed = $false
            # Keep removing BOM at the start until there's no more
            while ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
                $bytes = $bytes[3..($bytes.Length-1)]
                $changed = $true
            }
            if ($changed) {
                [System.IO.File]::WriteAllBytes($f.FullName, $bytes)
                $relPath = $f.FullName.Substring('D:\STUDY\MSPM0G3507\Myworkspace\empty\'.Length)
                Write-Host "Fixed: $relPath"
                $count++
            }
        }
    }
}
Write-Host "`nTotal fixed: $count files"
