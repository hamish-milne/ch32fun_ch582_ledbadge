-- Run this with: xmake lua font.lua
local download = import("net.http.download")
local bytes = import("core.base.bytes")


local source = os.tmpfile()
print("Downloading source image...")
download(
    "https://raw.githubusercontent.com/idispatch/raster-fonts/refs/heads/master/06x08_Terminal_Microsoft.png",
    source
)
print("Converting image...")
local rawimage = os.tmpfile()
os.execv(
    "ffmpeg",
    {
        "-i", source,
        "-f", "rawvideo",
        "-pix_fmt", "gray8",
        "-y", rawimage,
    }
)
os.rm(source)
print("Reading raw image data...")
local rawbytes = bytes(io.readfile(rawimage, { encoding = "binary" }))
os.rm(rawimage)
print("Raw image size (bytes): " .. rawbytes:size())

print("Converting to 1bit font format...")
local CHR_W = 6
local NCHARS = 16
local outbytes = bytes(NCHARS * NCHARS * CHR_W)

for y = 0, NCHARS - 1 do
    for x = 0, NCHARS - 1 do
        local char_index = y * NCHARS + x
        local out_offset = char_index * CHR_W
        for cx = 0, CHR_W - 1 do
            local out_byte = 0
            for cy = 0, 7 do
                local in_x = (x * CHR_W) + cx
                local in_y = (y * 8) + cy
                local in_idx = in_x + (in_y * NCHARS * CHR_W) + 1
                if rawbytes[in_idx] > 0xf0 then
                    out_byte = out_byte | (1 << cy)
                end
            end
            outbytes[out_offset + cx + 1] = out_byte
        end
    end
end

print("Writing font data to font.hex...")

local f = io.open("font.hex", "wb")
for i = 1, outbytes:size() do
    f:write(string.format("0x%02X,", outbytes[i]))
    if i % (NCHARS * CHR_W) == 0 then
        f:write("\n")
    end
end
f:close()

print("Done.")
