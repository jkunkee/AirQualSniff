
$testinput = @"
{"data":"/wEBAQEBAQEBAQEBAQEB8Qn5CQn1AQH/AQEBfYWFhYV9BXkBAQEBAQEBAQEBAf8BAQFxiYmJiYkBAQF5hQU5QYGB/QEB/wEBgQEBgUFBgQEBAYGB8YmFhYWFAQH/ARmlhYmJkaE9Af8BGaWhoaGhpTkB/wAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAA+AQE+AABAQAAAP8AAAA8Qn5AQjwAAACAAAAAAAAAAAAA/wAAAI9IKCgoyAAAAHiEAgICAoR4AAD/ABgkJRoFChIBAAAAAAA8IiAgICAAAP8AY5SEhISElOMA/wCTlJSUlKSkQwD/AAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAACBgfGJhYWFhQAA/wABAR8hISEhHwAAAB8gID8gICAgAAD/AAAAxyQUFBTnBAQA+AQBAQEBBPgAAP8AABg4OHh4uLg4ODg4OP84ODg4/wAA/wAAAAwSFhgQDgD/AAQEDgQEBAQCAP8AAAAAAAAAAAAAAAAA/wAAgICAgICAgICAgICA8ICAgIDwAAD/AAD4BAICAgLE+Pz+fx8HAwMCBPgAAP8AACAzkvISIiNCwoCAAQICAgIBAAAA/wAAAAAAAAAAAQEDA","ttl":60,"published_at":"2023-07-15T06:05:12.624Z","coreid":"29001a000a47373336323230","name":"fb_0"}
"@,@"
{"data":"gYEDwAAgIABAAD/AAAAaHhISEhIAP8AEBCcUlJSUpwA/wAAAAAAAAAAAAAAAAD/AAABAwMHBwsbEzMjY0P/AwMDAx8AAP8AAAECBAwMDw8HBwMGDAwMDIaHAQAA/wAATl9/f0BAQAAAAQMDBw8PDw8GAAD/AAAAAAAAAACAgICAgIAAAAMHBwMAAP8AAADOKSkpKc4I/wAAAHNKSkpKc0L/AAAAAAAAAAAAAAAAAP8AAIDAYHBwcHh4eHh4eHBwcGDAgAAA/wAAAAAAAAAAAAAAAAAAAAADBwcDAAD/AACAwGBwcHB4eHh4eHhwcHBgwIAAAP8AAHjMhoeHh4eHh4eHh4eHh4bMeAAA/wDEDA3FJSUlxQH/ADFLCxERIUF5AP8AAAAAAAAAAAAAAAAA/wAABwwYODh4eHh4eHh4eDg4GAwHAAD/AAAwcPBwcHBwcHBwcHBwcHBwcP4AAP8AAAcMGDg4eHh4eHh4eHg4OBgMBwAA/wAAAAABAwMHBwcHBwcHxwMDAQDAAAD/ADkhITkEBAQ5AP8AAQMDAQEBAQEA/wAAAAAAAAAAAAAAAAD/AADAYDA4ODg8PDw8PDw4ODgwYMAAAP8AAAAAgYCAgAAAAI","ttl":60,"published_at":"2023-07-15T06:05:13.060Z","coreid":"29001a000a47373336323230","name":"fb_1"}
"@,@"
{"data":"DAwMDAwIAAAQAA/wAAYODg4ODg4ODg4ODg/ODg4OD8AAD/AAAGDg4eHi5uTs6Ojg7/Dg4ODn8AAP+AgICAgICAgICA/4CAgICAgICAgID/AAAAAAAAAAAAAAAAAP8AAAMGDBwcPDw8PDw8PDwcHAwGAwAA/wAAfoEAAADA8f7//58HAQAAgMF+AAD/AAAAAAABAQIGBAwIGBA/AAAAAAcAAP8AAAAAAAAAAAAAAAABAQMAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA/wAAwMDAwMDAwMDAwMDAwMDAwMD4AAD/AAAAAAEDAwMDAQEAAQMDAwMBAQAAAP8AAAAAAAAAAAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/AAAAAQcBAQEBAQEBAQEBAQEBAQcAAP8AAAAAAAAAAAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA","ttl":60,"published_at":"2023-07-15T06:05:13.560Z","coreid":"29001a000a47373336323230","name":"fb_2"}
"@,@"
{"data":"AAAAAAAAAAAAAAP+AgICAgICAgICAgICAgICAgICAgICA/4CAgICAgICAgICAgICAgICAgICAgID/gICAgICAgICAgICAgICAgICAgICAgP+AgICAgICAgICAgICAgICAgICAgICA/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA","ttl":60,"published_at":"2023-07-15T06:05:14.062Z","coreid":"29001a000a47373336323230","name":"fb_3"}
"@,@"
{"data":"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAgGAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAgAAAAAAAAAAAAAAAAAAAAAAAAAAAACDAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=","ttl":60,"published_at":"2023-07-15T06:05:14.564Z","coreid":"29001a000a47373336323230","name":"fb_4"}
"@

$accretedString = ""

function BufferToString {
    param (
        [System.Byte[]]$raw
    )
    for ($row = 0; $row -lt 128; $row += 2) {
        $line = ""
        for ($col = 0; $col -lt 128; $col += 1) {
            $byte = $raw[($row/8)*128 + $col]
            $bits = ($byte -shr ($row % 8)) -band 3
            switch ($bits) {
                0 {
                    $line += '0'
                }
                1 {
                    $line += "'"
                }
                2 {
                    $line += ','
                }
                3 {
                    $line += '8'
                }
                default {
                    $line += '?'
                }
            }
        }
        write-host $line
    }
}

#particle subscribe fb_ |
$testInput |
? { return $_.startswith('{') } |
% {
    $json = $_ | convertfrom-json
    switch ($json.name) {
        "fb_0" {
            $accretedString = $json.data
            Write-Host "0 $($accretedString.Length)"
        }
        "fb_1" {
            $accretedString += $json.data
            Write-Host "1 $($accretedString.Length)"
        }
        "fb_2" {
            $accretedString += $json.data
            Write-Host "2 $($accretedString.Length)"
        }
        "fb_3" {
            $accretedString += $json.data
            Write-Host "3 $($accretedString.Length)"
        }
        "fb_4" {
            $accretedString += $json.data
            Write-Host "3 $($accretedString.Length)"
            $fb = [System.Convert]::FromBase64String($accretedString)
            write-host $fb[0]
            BufferToString($fb)
        }
    }
}
