let
  nixpkgs = <nixpkgs>;
  host = import nixpkgs {};
  cross = import nixpkgs {
    crossSystem = {
      config = "riscv64-unknown-none-elf";
    };
  };
in
host.mkShell rec {
  depsBuildBuild = [
    host.libusb1
    host.libudev-zero
    host.xmake
  ];
  depsBuildTarget = [
    # We use 'unwrapped' because we're not compiling for the host system.
    cross.gcc-unwrapped
    cross.binutils
  ];
  depsTargetTarget = [
    cross.newlib
  ];
  # Configure xmake to find any target dependencies. This is necessary
  # because we are using 'unwrapped' toolchains, which do not
  # automatically set up the include and library paths.
  shellHook = with builtins; ''xmake config \
    --includedirs=${concatStringsSep ":" (map (x: x.outPath + x.incdir) depsTargetTarget)} \
    --linkdirs=${concatStringsSep ":" (map (x: x.outPath + x.libdir) depsTargetTarget)} \
'';
}
