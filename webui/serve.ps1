param(
  [int]$Port = 8000,
  [string]$Bind = '0.0.0.0'
)

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

Write-Host "Serving webui from: $root" -ForegroundColor Cyan

function Get-LocalIPv4 {
  try {
    $ips = Get-NetIPAddress -AddressFamily IPv4 |
      Where-Object { $_.IPAddress -notmatch '^169\.254\.' -and $_.IPAddress -ne '127.0.0.1' } |
      Select-Object -ExpandProperty IPAddress
    if (-not $ips) { throw 'no ip' }
    return $ips
  } catch {
    # Fallback to ipconfig parsing
    $out = ipconfig | Select-String -Pattern 'IPv4 Address' -Context 0,0 | ForEach-Object { $_.Line }
    $ips = @()
    foreach ($line in $out) {
      if ($line -match '(\d+\.\d+\.\d+\.\d+)') { $ips += $Matches[1] }
    }
    if ($ips.Count -eq 0) { return @('127.0.0.1') } else { return $ips }
  }
}

function Show-AccessInfo {
  $ips = Get-LocalIPv4
  foreach ($ip in $ips) {
    Write-Host (" -> http://{0}:{1}" -f $ip, $Port) -ForegroundColor Green
  }
  Write-Host "Ensure Windows Firewall allows inbound TCP on port $Port for python/node." -ForegroundColor Yellow
  Write-Host ("Quick rule (admin): netsh advfirewall firewall add rule name=webui dir=in action=allow protocol=TCP localport={0}" -f $Port) -ForegroundColor DarkYellow
}

function Try-Run($exe, $args) {
  $p = Get-Command $exe -ErrorAction SilentlyContinue
  if ($null -ne $p) {
    Show-AccessInfo
    Write-Host "Launching: $exe $args" -ForegroundColor Cyan
    & $exe @args
    return $true
  }
  return $false
}

# Prefer Python http.server if available
if (Try-Run 'python' @('-m','http.server',$Port,'--bind',$Bind)) { exit }
if (Try-Run 'py' @('-3','-m','http.server',$Port,'--bind',$Bind)) { exit }

# Try Node-based static server (requires Node + network access for first run of npx)
if (Try-Run 'npx' @('-y','http-server','-a',$Bind,'-p',"$Port")) { exit }
if (Try-Run 'npx' @('-y','serve','-l',"$Bind:$Port",'.')) { exit }

Write-Warning 'No static server found (python/py/node/npx). Install Python from python.org or Node.js, or host this folder with IIS.'
Show-AccessInfo
Start-Sleep -Seconds 10

