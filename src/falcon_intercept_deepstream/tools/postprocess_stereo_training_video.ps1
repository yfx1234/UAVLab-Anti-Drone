<#
.SYNOPSIS
Applies the production stereo color recipe to previously recorded training videos.

.EXAMPLE
powershell.exe -ExecutionPolicy Bypass -File .\postprocess_stereo_training_video.ps1 `
  -InputPath 'D:\recordings\left.mkv','D:\recordings\right.mkv' `
  -OutputDirectory 'D:\recordings\corrected' -FfmpegPath 'C:\ffmpeg\bin\ffmpeg.exe'

.EXAMPLE
powershell.exe -ExecutionPolicy Bypass -File .\postprocess_stereo_training_video.ps1 `
  -InputPath 'D:\recordings' -Recurse -Codec h264 -Crf 16 `
  -FfmpegPath 'C:\ffmpeg\bin\ffmpeg.exe'

.NOTES
The default H.264 MP4 output is compatible with Windows players and training tools.
Use -Codec ffv1 only when a large lossless Matroska archive is explicitly required.
Input files are never modified. Existing outputs require -Overwrite.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string[]] $InputPath,

    [string] $OutputDirectory = "",

    [ValidateSet("ffv1", "h264")]
    [string] $Codec = "h264",

    [ValidateRange(0, 51)]
    [int] $Crf = 16,

    [ValidateSet("ultrafast", "superfast", "veryfast", "faster", "fast", "medium", "slow")]
    [string] $Preset = "medium",

    [string] $Suffix = "_color_corrected",

    [string] $FfmpegPath = "",

    [switch] $Recurse,

    [switch] $Overwrite,

    [switch] $DryRun
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$recipe = [ordered]@{
    name = "falcon_stereo_color_2026_07_24"
    matrix_rgb = @(0.92, 0.0, 0.0, 0.0, 2.08, 0.0, 0.0, 0.0, 0.91)
    contrast_pivot = 116.0
    contrast = 1.10
    magenta_suppression = 1.0
    magenta_red = -0.52
    magenta_green = 0.62
    magenta_blue = -0.20
    gamma = 0.55
    color_mix = 0.85
    gray_mix = 0.15
}

function Resolve-FfmpegExecutable {
    if ($FfmpegPath) {
        $resolved = Resolve-Path -LiteralPath $FfmpegPath -ErrorAction Stop
        return $resolved.Path
    }

    if ($env:IMAGEIO_FFMPEG_EXE -and (Test-Path -LiteralPath $env:IMAGEIO_FFMPEG_EXE)) {
        return (Resolve-Path -LiteralPath $env:IMAGEIO_FFMPEG_EXE).Path
    }

    $command = Get-Command ffmpeg -CommandType Application -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }

    if ($DryRun) {
        return "ffmpeg"
    }

    throw @"
FFmpeg was not found. Install FFmpeg for Windows and add it to PATH, or pass:
  -FfmpegPath 'C:\path\to\ffmpeg.exe'
The original videos have not been modified.
"@
}

function Get-VideoInputs {
    $extensions = @(".avi", ".mkv", ".mov", ".mp4", ".m4v", ".mpg", ".mpeg", ".ts")
    $files = [System.Collections.Generic.List[System.IO.FileInfo]]::new()

    foreach ($candidate in $InputPath) {
        $item = Get-Item -LiteralPath $candidate -ErrorAction Stop
        if ($item -is [System.IO.FileInfo]) {
            if ($extensions -notcontains $item.Extension.ToLowerInvariant()) {
                throw "Unsupported video extension: $($item.FullName)"
            }
            $files.Add($item)
            continue
        }

        $children = Get-ChildItem -LiteralPath $item.FullName -File -Recurse:$Recurse
        foreach ($child in $children) {
            if ($extensions -contains $child.Extension.ToLowerInvariant() -and
                -not $child.BaseName.EndsWith($Suffix, [System.StringComparison]::OrdinalIgnoreCase)) {
                $files.Add($child)
            }
        }
    }

    $unique = @($files | Sort-Object FullName -Unique)
    if ($unique.Count -eq 0) {
        throw "No supported input videos were found."
    }
    return $unique
}

function New-FalconColorFilter {
    $matrix = "lutrgb=" +
        "r='clip((val*0.92-116)*1.10+116,0,255)':" +
        "g='clip((val*2.08-116)*1.10+116,0,255)':" +
        "b='clip((val*0.91-116)*1.10+116,0,255)'"

    $magenta = "max(0,min(r(X,Y),b(X,Y))-g(X,Y))"
    $repair = "geq=" +
        "r='clip(r(X,Y)-0.52*$magenta,0,255)':" +
        "g='clip(g(X,Y)+0.62*$magenta,0,255)':" +
        "b='clip(b(X,Y)-0.20*$magenta,0,255)'"

    $gamma = "lutrgb=" +
        "r='255*pow(val/255,0.55)':" +
        "g='255*pow(val/255,0.55)':" +
        "b='255*pow(val/255,0.55)'"

    # 0.85 * color + 0.15 * (0.299 R + 0.587 G + 0.114 B)
    $grayBlend = "colorchannelmixer=" +
        "rr=0.89485:rg=0.08805:rb=0.01710:" +
        "gr=0.04485:gg=0.93805:gb=0.01710:" +
        "br=0.04485:bg=0.08805:bb=0.86710"

    return "format=gbrp,$matrix,$repair,$gamma,$grayBlend,format=yuv420p"
}

function Format-ArgumentForDisplay([string] $Value) {
    if ($Value -match '[\s\"'']') {
        return '"' + $Value.Replace('"', '\"') + '"'
    }
    return $Value
}

$ffmpeg = Resolve-FfmpegExecutable
$inputs = @(Get-VideoInputs)
$filter = New-FalconColorFilter

if ($OutputDirectory) {
    $outputRoot = [System.IO.Path]::GetFullPath($OutputDirectory)
    if (-not $DryRun) {
        New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
    }
} else {
    $outputRoot = ""
}

Write-Host "Falcon stereo training-video post-processing"
Write-Host "  recipe: $($recipe.name)"
Write-Host "  codec:  $Codec"
Write-Host "  inputs: $($inputs.Count)"

foreach ($inputFile in $inputs) {
    $targetDirectory = if ($outputRoot) { $outputRoot } else { $inputFile.DirectoryName }
    $outputExtension = if ($Codec -eq "ffv1") { ".mkv" } else { ".mp4" }
    $outputPath = Join-Path $targetDirectory ($inputFile.BaseName + $Suffix + $outputExtension)
    $sidecarPath = $outputPath + ".color_recipe.json"

    if ([System.IO.Path]::GetFullPath($outputPath) -eq $inputFile.FullName) {
        throw "Output would overwrite the input: $($inputFile.FullName)"
    }
    if ((Test-Path -LiteralPath $outputPath) -and -not $Overwrite -and -not $DryRun) {
        throw "Output already exists (use -Overwrite to replace it): $outputPath"
    }

    $arguments = [System.Collections.Generic.List[string]]::new()
    $arguments.Add("-hide_banner")
    $arguments.Add("-nostdin")
    $arguments.Add($(if ($Overwrite) { "-y" } else { "-n" }))
    $arguments.Add("-i")
    $arguments.Add($inputFile.FullName)
    $arguments.Add("-map")
    $arguments.Add("0:v:0")
    $arguments.Add("-map")
    $arguments.Add("0:a?")
    $arguments.Add("-map_metadata")
    $arguments.Add("0")
    $arguments.Add("-map_chapters")
    $arguments.Add("0")
    $arguments.Add("-vf")
    $arguments.Add($filter)
    $arguments.Add("-fps_mode")
    $arguments.Add("passthrough")

    if ($Codec -eq "ffv1") {
        $arguments.Add("-c:v")
        $arguments.Add("ffv1")
        $arguments.Add("-level")
        $arguments.Add("3")
        $arguments.Add("-coder")
        $arguments.Add("1")
        $arguments.Add("-context")
        $arguments.Add("1")
        $arguments.Add("-g")
        $arguments.Add("1")
    } else {
        $arguments.Add("-c:v")
        $arguments.Add("libx264")
        $arguments.Add("-preset")
        $arguments.Add($Preset)
        $arguments.Add("-crf")
        $arguments.Add($Crf.ToString([System.Globalization.CultureInfo]::InvariantCulture))
        $arguments.Add("-tag:v")
        $arguments.Add("avc1")
        $arguments.Add("-movflags")
        $arguments.Add("+faststart")
        $arguments.Add("-colorspace")
        $arguments.Add("bt709")
        $arguments.Add("-color_primaries")
        $arguments.Add("bt709")
        $arguments.Add("-color_trc")
        $arguments.Add("bt709")
    }

    $arguments.Add("-pix_fmt")
    $arguments.Add("yuv420p")
    $arguments.Add("-c:a")
    $arguments.Add("copy")
    $arguments.Add($outputPath)

    Write-Host ""
    Write-Host "Input:  $($inputFile.FullName)"
    Write-Host "Output: $outputPath"

    if ($DryRun) {
        $displayArguments = @($arguments | ForEach-Object { Format-ArgumentForDisplay $_ })
        Write-Host ("Command: " + (Format-ArgumentForDisplay $ffmpeg) + " " +
            ($displayArguments -join " "))
        continue
    }

    & $ffmpeg @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "FFmpeg failed with exit code $LASTEXITCODE for $($inputFile.FullName)"
    }

    $manifest = [ordered]@{
        recipe = $recipe
        generated_at = (Get-Date).ToString("o")
        input = [ordered]@{
            path = $inputFile.FullName
            bytes = $inputFile.Length
            modified_at = $inputFile.LastWriteTimeUtc.ToString("o")
        }
        output = [ordered]@{
            path = $outputPath
            codec = $Codec
            pixel_format = "yuv420p"
            bytes = (Get-Item -LiteralPath $outputPath).Length
        }
        ffmpeg = $ffmpeg
        video_filter = $filter
    }
    $manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $sidecarPath -Encoding utf8
    Write-Host "Recipe: $sidecarPath"
}

if ($DryRun) {
    Write-Host ""
    Write-Host "Dry run complete; no files were written."
} else {
    Write-Host ""
    Write-Host "All videos completed. Original files were not modified."
}
