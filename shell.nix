let
  host = import <nixpkgs> {};
  cross = import <nixpkgs> {
    crossSystem = {
      config = "riscv64-unknown-none-elf";
    };
  };
in
host.mkShell {
  hardeningDisable = [ "all" ];
  depsBuildBuild = [
    host.libusb1
    host.libudev-zero
    host.xmake
  ];
  depsBuildTarget = [
    cross.gcc
  ];
  depsTargetTarget = [
    cross.newlib
  ];
}
