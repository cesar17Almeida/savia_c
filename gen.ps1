$lines = Get-Content 'data_hourly_agg.csv'
$ta = @()
$hs10 = @()
$hs30 = @()
for ($i=49; $i -lt 121; $i++) {
    $p = $lines[$i] -split ','
    $ta += $p[13]
    $hs10 += $p[1]
    $hs30 += $p[3]
}
Write-Host "static const float m_ta[72] = {$($ta -join ',')};"
Write-Host "static const float m_hs10[72] = {$($hs10 -join ',')};"
Write-Host "static const float m_hs30[72] = {$($hs30 -join ',')};"
