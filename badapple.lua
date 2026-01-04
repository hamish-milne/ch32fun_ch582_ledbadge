-- Run this with: xmake lua badapple.lua
local download = import("net.http.download")
local bytes = import("core.base.bytes")

local source = os.tmpfile()
print("Downloading source video...")
download(
    "https://github.com/bad-apple-lab/Bad-Apple/raw/refs/heads/main/badapple.mp4",
    source
)

print("Converting video...")
local rawvideo = os.tmpfile()
os.execv(
    "ffmpeg",
    {
        "-i", source,
        "-vf", "crop=1444:361:0:400,scale=44:11",
        "-f", "rawvideo",
        "-video_size", "44x11",
        "-pixel_format", "gray8",
        "-y", rawvideo,
    },
    {
        stdout = rawvideo,
    }
)
os.rm(source)

print("Reading raw video frames...")
local rawbytes = bytes(io.readfile(rawvideo, { encoding = "binary" }))
os.rm(rawvideo)
print("Raw video size (bytes): " .. rawbytes:size())

local F_START = 40
local F_END = 6509

print("Converting to 1bit format...")
-- ffmpeg has a bug that causes it to output 44x17 frames instead of 44x11
-- frames when scaling down to small sizes.
local frame_size_in = 44 * 17
local frame_size_out = 22 * 3 -- 1-bit, 2 columns in 3 bytes
local num_frames = F_END - F_START + 1
local outbytes = bytes(num_frames * frame_size_out)
for i = 1, outbytes:size() do
    outbytes[i] = 0
end
local out_frame = 0
for i = F_START, F_END do -- Cut empty video
    local in_offset = i * frame_size_in
    local out_offset = out_frame * frame_size_out
    for x = 0, 21 do -- process 2 columns at a time
        local in_col = in_offset + (x * 2)
        local out_col = out_offset + (x * 3)
        for y = 0, 21 do
            local in_byte_idx = in_col + ((y % 11) * 44) + (y // 11) + 1
            if rawbytes[in_byte_idx] > 0x30 then
                local out_byte_idx = out_col + (y // 8) + 1
                local out_bit_idx = y % 8
                outbytes[out_byte_idx] = outbytes[out_byte_idx] | (1 << out_bit_idx)
            end
        end
    end
    out_frame = out_frame + 1
end

print("Writing output file...")
local file = io.open("badapple_1bit.hex", "w")
for i = 1, outbytes:size() do
    file:write(string.format("0x%02X,", outbytes[i]))
    if i % 66 == 0 then
        file:write("\n")
    end
end
file:close()

print("Done.")
