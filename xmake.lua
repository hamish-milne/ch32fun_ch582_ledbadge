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

local arch_abi = {
    rv32ec = "ilp32e",
    rv32ec_zmmul = "ilp32e",
    rv32imac = "ilp32",
    rv32imafc = "ilp32f",
}

local ch32_packages = {
    ["CHV300([234567])"] = {
        arch = "rv32ec_zmmul",
        defines = { CH32V00x = 1 },
        variants = {{
            ["3"] = {
                flash_size_kb = 16,
                ram_size_kb = 2,
                arch = "rv32ec",
            },
            ["2"] = {
                flash_size_kb = 16,
                ram_size_kb = 4,
            },
            ["[45]"] = {
                flash_size_kb = 32,
                ram_size_kb = 6,
            },
            ["[67]"] = {
                flash_size_kb = 62,
                ram_size_kb = 8,
            },
        }},
    },
    ["CHV103([RC][68])"] = {
        arch = "rv32imac",
        defines = { CH32V10x = 1 },
        variants = {{
            ["[RC]8"] = {
                flash_size_kb = 64,
                ram_size_kb = 20,
            },
            ["C6"] = {
                flash_size_kb = 32,
                ram_size_kb = 10,
            },
        }}
    },
    ["CHX03[35][CFGR][678]"] = {
        arch = "rv32imac",
        defines = { CH32X03x = 1 },
        flash_size_kb = 62,
        ram_size_kb = 20,
    },
    ["CHL103[CFGK]8"] = {
        arch = "rv32imac",
        defines = { CH32L103 = 1 },
        flash_size_kb = 62,
        ram_size_kb = 20,
    },
    ["CHV20([38])[CFGKRW]([68BW])"] = {
        arch = "rv32imac",
        defines = { CH32V20x = 1 },
        variants = {{
            ["3"] = {
                defines = { CH32V20x_D6 = 1 },
            },
            ["8"] = {
                defines = { CH32V20x_D8 = 1 },
            },
        },{
            ["6"] = {
                flash_size_kb = 32,
                ram_size_kb = 10,
                ext_origin = "0x08008000",
                ext_size_kb = 192,
            },
            ["8"] = {
                flash_size_kb = 64,
                ram_size_kb = 20,
                ext_origin = "0x08010000",
                ext_size_kb = 160,
            },
            ["[BW]"] = {
                flash_size_kb = 128,
                ram_size_kb = 32,
                ext_origin = "0x08020000",
                ext_size_kb = 352,
            },
        }},
    },
    ["CH32V30([37])[CFRVW]([BC])"] = {
        arch = "rv32imafc",
        defines = { CH32V30x = 1 },
        variants = {{
            ["3"] = {
                defines = { CH32V30x_D8 = 1 },
            },
            ["7"] = {
                defines = { CH32V30x_D8C = 1 },
            },
        },{
            ["C"] = function (option)
                local memory_split = option:dep("memory_split"):value()
                return ({{
                    flash_size_kb = 192,
                    ram_size_kb = 128,
                },{
                    flash_size_kb = 224,
                    ram_size_kb = 96,
                },{
                    flash_size_kb = 256,
                    ram_size_kb = 64,
                },{
                    flash_size_kb = 288,
                    ram_size_kb = 32,
                }})[memory_split]
            end,
            ["B"] = {
                flash_size_kb = 128,
                ram_size_kb = 32,
            },
        }},
    },
    ["CH5([789])"] = {
        arch = "rv32imac",
        defines = { CH5xx = 1 },
        has_highcode = true,
        variants = {{
            ["7([0123])"] = {
                defines = { CH57x = 1 },
                variants = {{
                    ["[02]"] = {
                        flash_size_kb = 240,
                        ram_size_kb = 12,
                        defines = { CH570_CH572 = 1 },
                    },
                    ["1"] = {
                        flash_size_kb = 192,
                        ram_size_kb = 18,
                        has_dma_quirk = true,
                        defines = { CH571_CH573 = 1 },
                    },
                    ["3"] = {
                        flash_size_kb = 448,
                        ram_size_kb = 18,
                        has_dma_quirk = true,
                        defines = { CH571_CH573 = 1 },
                    },
                }},
            },
            ["8([2345])"] = {
                defines = { CH58x = 1 },
                flash_size_kb = 448,
                variants = {{
                    ["[23]"] = {
                        ram_size_kb = 32,
                        defines = { CH582_CH583 = 1 },
                    },
                    ["4"] = {
                        ram_size_kb = 96,
                        defines = { CH584_CH585 = 1 },
                    },
                    ["5"] = {
                        ram_size_kb = 128,
                        defines = { CH584_CH585 = 1 },
                    },
                }},
            },
            ["9([12])"] = {
                defines = { CH59x = 1, CH591_CH592 = 1 },
                ram_size_kb = 26,
                variants = {{
                    ["1"] = {
                        flash_size_kb = 192,
                    },
                    ["2"] = {
                        flash_size_kb = 448,
                    },
                }},
            },
        }},
    },
    ["CH32H41([567])"] = {
        arch = "rv32imafc",
        defines = { CH32H4x = 1 },
        ram_size_kb = 512,
        has_coupled_ram = true,
        variants = {{
            ["[57]"] = {
                flash_size_kb = 960,
            },
            ["6"] = {
                flash_size_kb = 480,
            },
        }},
    },
}

local function getFindResult(s, e, ...)
    return s, e, {...}
end

local function match_ch32_package_part(target, variants, str, idx, parent)
    for pattern, info in pairs(variants) do
        local s, e, specs = getFindResult(string.find(str, "^"..pattern, idx))
        if s then
            local result = table.join(parent, info)
            result.defines = table.join(parent.defines, info.defines)
            result.variants = nil
            if info.variants then
                for vidx, spec in ipairs(specs) do
                    local variant = info.variants[vidx]
                    if type(variant) == "function" then
                        variant = variant(target)
                    end
                    -- all variant specs need to be matched; no backtracking
                    result, e = match_ch32_package_part(target, variant, str, e, result)
                    if not result then
                        return nil
                    end
                end
            end
            return result, e
        end
    end
    return nil
end

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
        local targetMcu = option:value()
        local mcuInfo = match_ch32_package_part(option, ch32_packages, targetMcu, 1, {})
        if not mcuInfo then
            raise("Unknown MCU "..targetMcu)
        end
        local archFlags = "-march="..mcuInfo.arch.." -mabi="..arch_abi[mcuInfo.arch]
        option:add("cflags", archFlags)
        option:add("ldflags", archFlags)
        local defines = table.join({
            TARGET_MCU_PACKAGE = targetMcu,
            HAS_HIGHCODE = mcuInfo.has_highcode or nil,
            HAS_DMA_QUIRK = mcuInfo.has_dma_quirk or nil,
            EXT_ORIGIN = mcuInfo.ext_origin or nil,
            EXT_SIZE_KB = mcuInfo.ext_size_kb or nil,
            FLASH_SIZE_KB = mcuInfo.flash_size_kb,
            RAM_SIZE_KB = mcuInfo.ram_size_kb,
        }, mcuInfo.defines)
        local defines_array = {}
        for k, v in pairs(defines) do
            defines_array[#defines_array + 1] = k.."="..tostring(v)
        end
        option:add("defines", defines_array)
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

target("minichlink-obj", function()
    set_kind("object")
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
    add_deps("minichlink-obj")
    set_policy("build.fence", true)
end)

target("minichlink-shared", function()
    set_kind("shared")
    add_deps("minichlink-obj")
    set_basename("minichlink")
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
        batchcmds:show_progress(opt.progress, "${color.build.target}generating %s", sourcefile..".bin")
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
    add_options("target_mcu")
    add_files("./ch32fun.ld", { rules = {"generate-ld", override = true} })
    add_files("./ledbadge2.c", "./badapple.c", "./ch32fun/ch32fun/ch32fun.c")
    add_includedirs(".", "./ch32fun/ch32fun")

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
end)
