-- By default xmake checks whether flags are supported by the host compiler.
-- This actually works fine for our configuration, but it does slow down
-- the first build a little, so you can uncomment this line to disable it.
--set_policy("check.auto_ignore_flags", false)

-- We're only building for one target, so disable intermediate directories
set_policy("build.intermediate_directory", false)

-- Define a custom toolchain for RISC-V embedded targets.
-- This lets us compile for both the host and the target.
toolchain("embed", function()
    set_kind("standalone")
    local PREFIX = "riscv64-unknown-none-elf-"
    -- Let xmake detect the tools based on the prefix
    set_cross(PREFIX)
    -- Normally xmake expects cross-toolchains to specify an sdkdir,
    -- but our nixpkgs toolchains just add everything to the PATH.
    -- So we just tell xmake to find the tools directly.
    on_check(function(toolchain)
        import("lib.detect.find_program")
        return find_program(PREFIX.."gcc")
    end)
    -- We don't want any standard libraries or startup files.
    -- This option is added here so that check_flags works correctly.
    add_ldflags("-nostdlib")

    -- Pull in include and library paths from the environment,
    -- which are set by the nix shell.
    add_includedirs(os.getenv("TARGET_INCDIR"))
    add_linkdirs(os.getenv("TARGET_LIBDIR"))
end)

option("memory_split", function()
    set_default(0)
    set_showmenu(true)
    set_description("Memory split option for CH32V30xC devices")
    set_values(0, 1, 2, 3)
end)

option("target_mcu", function()
    set_default("CH582F")
    set_showmenu(true)
    set_category("ch32fun")
    set_description("Target MCU to build for")
    add_deps("memory_split")
    after_check(function (option)

        local function run_shell(cmd, args)
            local pipe = import("core.base.pipe")
            local bytes = import("core.base.bytes")
            local rpipe, wpipe = pipe.openpair()
            os.execv(cmd, args, { stdout = wpipe })
            wpipe:close()
            local output = bytes(1024)
            rpipe:read(output, 1024)
            rpipe:close()
            return output:str()
        end 

        local targetMcu = option:value()
        local vars = run_shell("sh", {"./ch32fun/ch32fun/parse_mcu_package.sh", targetMcu, option:dep("memory_split"):value()})
        local mcuInfo = {}
        for line in vars:gmatch("[^\n]+") do
            local key, value = line:match("([^=]+)=(.+)")
            if key and value then
                mcuInfo[key] = value
            end
        end
        local archFlags = "-march="..mcuInfo.ARCH.." -mabi="..mcuInfo.ABI
        option:add("cflags", archFlags)
        option:add("ldflags", archFlags)
        for _, key in ipairs({"FLASH_SIZE_KB", "RAM_SIZE_KB", "EXT_ORIGIN", "EXT_SIZE_KB"}) do
            if mcuInfo[key] then
                option:add("defines", key.."="..mcuInfo[key])
            end
        end
        option:add("defines", mcuInfo.TARGET_MCU)
        for define in string.gmatch(mcuInfo.DEFINES, "%S+") do
            option:add("defines", define.."=1")
        end
    end)
end)

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
        batchcmds:show_progress(opt.progress, "${color.build.object}processing linker script %s", sourcefile)
        batchcmds:vrunv(cmd)
    end)
end)

target("minichlink-static", function()
    set_kind("static")
    local PREFIX = "./ch32fun/minichlink/"
    add_files(
        PREFIX.."minichlink.c",
        PREFIX.."pgm-wch-linke.c",
        PREFIX.."pgm-wch-isp.c",
        PREFIX.."pgm-esp32s2-ch32xx.c",
        PREFIX.."nhc-link042.c",
        PREFIX.."ardulink.c",
        PREFIX.."serial_dev.c",
        PREFIX.."pgm-b003fun.c",
        PREFIX.."minichgdb.c",
        PREFIX.."chips.c",
        PREFIX.."ch5xx.c"
    )
    add_includedirs(PREFIX)
    add_defines("MINICHLINK", "CH32V003")
    add_cflags("-O0", "-g3", "-Wall", "-Wno-unused-function", "-fmessage-length=0")
    add_cflags("-fPIC")
    if is_plat("windows") then
        add_defines("_WIN32_WINNT=0x0600")
        add_links("pthread", "usb-1.0", "setupapi", "ws2_32")
        add_cflags("-Os", "-s")
    elseif is_plat("linux") then
        add_links("pthread", "usb-1.0", "udev")
    elseif is_plat("macosx") then
        add_defines("__MACOSX__")
        add_links("pthread")
        add_frameworks("CoreFoundation", "IOKit", "Security")
        add_cflags("-Wno-asm-operand-widths", "-Wno-deprecated-declarations", "-Wno-deprecated-non-prototype")
    end
end)

target("minichlink", function()
    set_kind("binary")
    add_deps("minichlink-static")
    set_policy("build.fence", true)
end)

target("minichlink-shared", function()
    set_kind("shared")
    add_deps("minichlink-static")
    set_basename("minichlink")
end)

-- Creates a .bin file from a given .elf source.
-- The source is derived from this target's dependency.
rule("firmware-image", function ()
    on_buildcmd(function (target, batchcmds, opt)
        for _,input in pairs(target:deps()) do
            -- Make sure that the input gets fully built before running.
            -- This has to be done this way because we depend on the output binary, not a source file.
            input:set("policy", "build.fence", true)
            local sourcefile = input:targetfile()
            local objcopy = target:tool("objcopy")
            batchcmds:show_progress(opt.progress, "${color.build.target}generating %s", sourcefile..".bin")
            batchcmds:vrunv(objcopy, {
                "-R.storage", -- Exclude the external flash section
                "-Obinary", -- Output raw binary
                sourcefile,
                sourcefile..".bin"
            })
        end
    end)
end)

target("ledbadge", function ()
    add_rules("generate-ld")
    set_toolchains("embed")
    set_kind("binary")
    add_options("target_mcu")
    add_files("./ch32fun/ch32fun/ch32fun.ld", { rules = {"generate-ld", override = true} })
    add_files("./ledbadge2.c", "./badapple.c", "./font.c", "./ch32fun/ch32fun/ch32fun.c")
    add_includedirs(".", "./ch32fun/ch32fun")
    add_configfiles("./config.h.in", { prefixdir = ".." })

    local cFlags_array = {
        "-g",
        "-Os",
        "-flto",
        "-ffunction-sections",
        "-fdata-sections",
        "-fmessage-length=0",
        "-msmall-data-limit=8",
        "-nostdlib",
        "-Wall",
    }
    add_cflags(cFlags_array)
    add_ldflags(cFlags_array)
    add_ldflags(
        "-Wl,--print-memory-usage",
        "-Wl,--gc-sections"
    )
end)

target("ledbadge-bin", function ()
    set_toolchains("embed")
    add_deps("ledbadge")
    add_rules("firmware-image")
    set_kind("phony")
end)

target("flash", function ()
    set_kind("phony")
    add_deps("ledbadge", "ledbadge-bin", "minichlink")
    on_buildcmd(function (target, batchcmds, opt)
        local binfile = target:dep("ledbadge"):targetfile()..".bin"
        local minichlink = target:dep("minichlink"):targetfile()
        batchcmds:show_progress(opt.progress, "${color.build.object}flashing %s", binfile)
        batchcmds:vexecv(minichlink, {
            "-w", binfile,
            "flash", "-b",
        })
    end)
end)
