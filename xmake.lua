toolchain("embed", function()
    set_kind("standalone")
    local PREFIX = "riscv64-unknown-none-elf-"
    set_cross(PREFIX)
    on_check(function(toolchain)
        import("lib.detect.find_program")
        return find_program(PREFIX.."gcc")
    end)
end)

local function includes(haystack, needle, ...)
    if not needle and #{...} == 0 then
        return false
    end
    if string.find(haystack, needle) then
        return true
    end
    return includes(haystack, ...)
end

local function startswith(haystack, needle, ...)
    if not needle and #{...} == 0 then
        return false
    end
    if string.find(haystack, needle) == 1 then
        return true
    end
    return startswith(haystack, ...)
end

function ch32fun_config(config)
    local targetMcu = config.TARGET_MCU
    if not targetMcu then error("no TARGET_MCU") end
    local cFlags = {}
    local defines = {}
    local targetMcuPackage = config.TARGET_MCU_PACKAGE
    local mcuPackage = config.MCU_PACKAGE
    local targetMcuLd
    local extOrigin
    local targetMcuMemorySplit
    local enableFpu = config.ENABLE_FPU
    local isCh5xx
    local arch = "rv32"
    local abi = "ilp32"
    if startswith(targetMcu, "CH32V00") then
        if #targetMcu ~= 8 then
            error("Unknown MCU "..targetMcuPackage)
        end
        mcuPackage = mcuPackage or 1
        if targetMcu == "CH32V003" then
            arch = "rv32ec"
        else
            arch = "rv32ec_zmmul"
            defines.CH32V00x = 1
        end
        abi = "ilp32e"
        if startswith(targetMcu, "CH32V003") then
            targetMcuLd = 0
        elseif startswith(targetMcu, "CH32V002") then
            targetMcuLd = 5
        elseif includes(targetMcu, "CH32V004", "CH32V005") then
            targetMcuLd = 6
        elseif includes(targetMcu, "CH32V006", "CH32V007") then
            targetMcuLd = 7
        else
            error("unknown CH32V00x variant: "..targetMcu)
        end
    elseif startswith(targetMcu, "CH32V10") then
        targetMcuPackage = targetMcuPackage or "CH32V103R8T6"
        if #targetMcuPackage ~= 12 then
            error("Unknown MCU "..targetMcuPackage)
        end
        arch = "rv32imac"
        defines.CH32V10x = 1
        if includes(targetMcuPackage, "R8", "C8") then
            mcuPackage = 1
        elseif includes(targetMcuPackage, "C6") then
            mcuPackage = 2
        end
        targetMcuLd = 1
    elseif startswith(targetMcu, "CH32X03") then
        targetMcuPackage = targetMcuPackage or "CH32X035F8U6"
        if #targetMcuPackage ~= 12 then
            error("Unknown MCU "..targetMcuPackage)
        end
        arch = "rv32imac"
        defines.CH32X03x = 1
        if includes(targetMcuPackage, "F8", "R8", "K8", "C8", "G8", "G6", "F7") then
            mcuPackage = 1
        end
        targetMcuLd = 4
    elseif startswith(targetMcu, "CH32L103") then
        targetMcuPackage = targetMcuPackage or "CH32L103C8T6"
        if #targetMcuPackage ~= 12 then
            error("Unknown MCU "..targetMcuPackage)
        end
        arch = "rv32imac"
        defines.CH32L103 = 1
        if includes(targetMcuPackage, "F8", "K8", "C8", "G8") then
            mcuPackage = 1
        end
        targetMcuLd = 4
    elseif startswith(targetMcu, "CH32V20") then
        targetMcuPackage = targetMcuPackage or "CH32V203F6P6"
        if #targetMcuPackage ~= 12 then
            error("Unknown MCU "..targetMcuPackage)
        end
        arch = "rv32imac"
        if includes(targetMcuPackage, "203RB") then
            defines.CH32V20x_D8 = 1
        elseif includes(targetMcuPackage, "308") then
            defines.CH32V20x_D8W = 1
            mcuPackage = 3
        elseif includes(targetMcuPackage, "F8", "G8", "K8", "C8") then
            mcuPackage = 1
        elseif includes(targetMcuPackage, "F6", "G6", "K6", "C6") then
            mcuPackage = 2
        elseif includes(targetMcuPackage, "RB", "GB", "CB", "WB") then
            mcuPackage = 3
        else
            defines.CH32V20x_D6 = 1
        end
        if mcuPackage == 1 then
            extOrigin = "0x08010000"
        elseif mcuPackage == 2 then
            extOrigin = "0x08008000"
        elseif mcuPackage == 3 then
            extOrigin = "0x08020000"
        end
        targetMcuLd = 2
    elseif startswith(targetMcu, "CH32V30") then
        targetMcuPackage = targetMcuPackage or "CH32V307VCT6"
        if #targetMcuPackage ~= 12 then
            error("Unknown MCU "..targetMcuPackage)
        end
        mcuPackage = mcuPackage or 1
        targetMcuMemorySplit = targetMcuMemorySplit or 3
        enableFpu = enableFpu or 1
        if enableFpu then
            arch = "rc32imafc"
            abi = "ilp32f"
        else
            arch = "rv32imac"
            defines.DISABLED_FLOAT = 1
        end
        defines.CH32V30x = 1
        defines.TARGET_MCU_MEMORY_SPLIT=targetMcuMemorySplit
        if includes(targetMcuPackage, "RC", "VC", "WC") then
            mcuPackage = 1
        elseif includes(targetMcuPackage, "CB", "FB", "RB") then
            mcuPackage = 2
        end
        if includes(targetMcuPackage, "303") then
            defines.CH32V30x_D8 = 1
        else
            defines.CH32V30x_D8C = 1
        end
        targetMcuLd = 3
    elseif startswith(targetMcu, "CH57") then
        targetMcuPackage = targetMcuPackage or "CH570E"
        if #targetMcuPackage ~= 6 then
            error("Unknown MCU "..targetMcuPackage)
        end
        arch = "rv32imac"
        defines.CH57x = 1
        defines.CH5xx = 1
        mcuPackage = tonumber(targetMcu:sub(5, 6))
        defines['CH57x'..targetMcuPackage:sub(6, 7)] = 1
        targetMcuLd = 10
        isCh5xx = true
    elseif startswith(targetMcu, "CH58") then
        targetMcuPackage = targetMcuPackage or "CH582F"
        if #targetMcuPackage ~= 6 then
            error("Unknown MCU "..targetMcuPackage)
        end
        arch = "rv32imac"
        defines.CH58x = 1
        defines.CH5xx = 1
        mcuPackage = tonumber(targetMcu:sub(5, 6))
        defines['CH58x'..targetMcuPackage:sub(6, 7)] = 1
        targetMcuLd = 8
        isCh5xx = true
    elseif startswith(targetMcu, "CH59") then
        targetMcuPackage = targetMcuPackage or "CH592F"
        if #targetMcuPackage ~= 6 then
            error("Unknown MCU "..targetMcuPackage)
        end
        arch = "rv32imac"
        defines.CH59x = 1
        defines.CH5xx = 1
        mcuPackage = tonumber(targetMcu:sub(5, 6))
        defines['CH59x'..targetMcuPackage:sub(6, 7)] = 1
        targetMcuLd = 9
        isCh5xx = true
    elseif startswith(targetMcu, "CH32H41") then
        targetMcuPackage = targetMcuPackage or "CH32H417"
        enableFpu = enableFpu or 1
        mcuPackage = mcuPackage or 1
        if enableFpu then
            arch = "rc32imafc"
            abi = "ilp32f"
        else
            arch = "rv32imac"
            defines.DISABLED_FLOAT = 1
        end
        if includes(targetMcuPackage, "416") then
            mcuPackage = 2
        elseif includes(targetMcuPackage, "415") then
            mcuPackage = 3
        end
        defines.CH32H41x = 1
        targetMcuLd = 11
    else
        error("Unknown MCU "..targetMcu)
    end
    defines.TARGET_MCU = targetMcu
    defines.MCU_PACKAGE = mcuPackage
    defines.TARGET_MCU_PACKAGE = targetMcuPackage
    defines.TARGET_MCU_LD = targetMcuLd
    local cFlags_array = {
        "-g",
        "-Os",
        "-flto",
        "-ffunction-sections",
        "-fdata-sections",
        "-fmessage-length=0",
        "-msmall-data-limit=8",
        "-static-libgcc",
        "-nostdlib",
        "-Wall"
    }
    for k,v in pairs(cFlags) do
        cFlags_array[#cFlags_array+1] = "-"..k.."="..v
    end
    local defines_array = {}
    for k,v in pairs(defines) do
        defines_array[#defines_array+1] = k.."="..v
    end
    -- Due to cross-compiling, xmake incorrectly thinks the flags are wrong:
    local archFlags = {"-march="..arch, "-mabi="..abi}
    add_cflags(archFlags, { force = true })
    add_ldflags(archFlags, { force = true })

    add_cflags(cFlags_array)
    add_ldflags(cFlags_array)
    add_ldflags(
        "-lgcc",
        "-Wl,--print-memory-usage",
        "-Wl,--gc-sections"
    )
    add_defines(defines_array)
end

-- Add a file to be preprocessed with the compiler, then used as a linker script
rule("generate-ld", function ()
    -- Need to use `add_rules("generate-ld")` for this to work
    on_load(function (target)
        -- For each input file, add its output path as a link 
        local sourcebatch = target:sourcebatches()["generate-ld"]
        if sourcebatch then
            for _, sourcefile in ipairs(sourcebatch.sourcefiles) do
                target:add(
                    "files",
                    target:autogenfile(sourcefile),
                    { always_added = true }
                )
            end
        end
    end)

    -- Then use `add_files("./script.ld.in", { rules = "generate-ld" })`
    on_buildcmd_file(function (target, batchcmds, sourcefile, opt)
        local out = target:autogenfile(sourcefile)
        local compiler = target:compiler("cc")
        local cmd = compiler:compcmd(
            sourcefile,
            out,
            {
                table.unpack(opt),
                compflags = {
                    "-E", -- Preprocess only
                    "-P", -- No #line directives
                    "-xc", -- Set language to C to force it to work
                    table.unpack(compiler:compflags(opt))
                }
            }
        )
        batchcmds:mkdir(path.directory(out))
        batchcmds:vrunv(cmd)
    end)
end)

-- Creates a .bin file from a given .elf source.
-- The source is derived from this target's dependency.
rule("firmware-image", function ()
    on_config(function (target)
        local project = import("core.project.project")
        -- Note that this doesn't work if we have multiple dependencies!
        local input = project.target(target:get("deps"))
        -- Make sure that the input gets fully built before running.
        -- This has to be done this way because we depend on the output binary, not a source file.
        input:set("policy", "build.fence", true)
        -- This ensures that 'on_buildcmd_file' is run for our input file:
        target:add("files", input:targetfile(), { rules = "firmware-image", always_added = true })
    end)
    on_buildcmd_file(function (target, batchcmds, sourcefile, opt)
        local objcopy = target:tool("objcopy")
        batchcmds:vrunv(objcopy, {
            "-R.storage", -- Exclude the external flash section
            "-Obinary", -- Output raw binary
            sourcefile,
            sourcefile..".bin"
        })
    end)
    on_link(function ()
        -- Make sure xmake doesn't try to run the linker
    end)
end)

target("ledbadge", function ()
    add_rules("generate-ld")
    set_toolchains("embed")
    set_kind("binary")
    ch32fun_config {
        TARGET_MCU = "CH582",
        TARGET_MCU_PACKAGE = "CH582F"
    }
    add_files("./ch32fun/ch32fun/ch32fun.ld", { rules = {"generate-ld", override = true} })
    add_files("./ledbadge2.c", "./ch32fun/ch32fun/ch32fun.c")
    add_includedirs(".", "./ch32fun/ch32fun")
end)

target("ledbadge-bin", function ()
    set_toolchains("embed")
    add_deps("ledbadge")
    add_rules("firmware-image")
end)
