﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿﻿# 清除工程中所有 .c / .h 文件的 UTF-8 BOM
Get-ChildItem -Recurse -Include *.c,*.h | ForEach-Object {
    $raw = [System.IO.File]::ReadAllBytes($_.FullName)
    $count = 0
    while ($count -lt 10 -and $raw.Length -ge 3 -and $raw[0] -eq 0xEF -and $raw[1] -eq 0xBB -and $raw[2] -eq 0xBF) {
        $raw = $raw[3..($raw.Length-1)]
        $count++
    }
    if ($count -gt 0) {
        [System.IO.File]::WriteAllBytes($_.FullName, $raw)
        Write-Host "Cleaned $count BOM(s): $($_.Name)"
    }
}
Write-Host "Done"
