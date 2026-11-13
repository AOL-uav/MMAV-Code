Add-Type -AssemblyName System.Drawing
$src = "<HOME>\AppData\Local\Temp\claude\MAV-2026-Codes\e78baad8-7bd4-4c2e-8073-2f0b681b4bcd\scratchpad\zanonia_dl\pattern.jpg"
$dst = "<HOME>\AppData\Local\Temp\claude\MAV-2026-Codes\e78baad8-7bd4-4c2e-8073-2f0b681b4bcd\scratchpad\zanonia_dl\pattern.pgm"
$bmp = New-Object System.Drawing.Bitmap($src)
$w = $bmp.Width; $h = $bmp.Height
"size $w x $h  dpi $($bmp.HorizontalResolution) x $($bmp.VerticalResolution)"
$rect = New-Object System.Drawing.Rectangle(0, 0, $w, $h)
$data = $bmp.LockBits($rect, [System.Drawing.Imaging.ImageLockMode]::ReadOnly, [System.Drawing.Imaging.PixelFormat]::Format24bppRgb)
$stride = $data.Stride
$buf = New-Object byte[] ($stride * $h)
[System.Runtime.InteropServices.Marshal]::Copy($data.Scan0, $buf, 0, $buf.Length)
$bmp.UnlockBits($data)
$out = New-Object byte[] ($w * $h)
for ($y = 0; $y -lt $h; $y++) {
  $o = $y * $stride
  $p = $y * $w
  for ($x = 0; $x -lt $w; $x++) {
    $i = $o + 3 * $x
    $out[$p + $x] = [byte]((299 * $buf[$i + 2] + 587 * $buf[$i + 1] + 114 * $buf[$i]) / 1000)
  }
}
$hdr = [System.Text.Encoding]::ASCII.GetBytes("P5`n$w $h`n255`n")
$fs = [System.IO.File]::Create($dst)
$fs.Write($hdr, 0, $hdr.Length)
$fs.Write($out, 0, $out.Length)
$fs.Close()
"wrote $dst"
