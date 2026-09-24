{ pkgs ? import <nixpkgs> {} }:

let
  renode-17 = pkgs.stdenv.mkDerivation {
    name = "renode-17";
    
    # Replace with your actual tar.gz URL and SHA256 hash
    src = pkgs.fetchurl {
      url = "https://builds.renode.io/renode-1.17.0.linux-portable.tar.gz";
      sha256 = "sha256-kqM9aqedPIv3TA2TREegSNvHYQLe9nXS5T3pAazzAOs="; 
    };

    installPhase = ''
      mkdir -p $out/bin
      mkdir -p $out/share

      cp -r * $out/share
      ln -s $out/share/renode $out/bin/renode
    '';
  };
  renode-test = pkgs.writeShellScriptBin "renode-test" ''
    exec ${pkgs.uv}/bin/uv run \
      --with-requirements ${renode-17}/share/tests/requirements.txt \
      ${renode-17}/share/renode-test "$@"
  '';

in pkgs.mkShell {
  buildInputs = with pkgs; [
    uv
    cppcheck
    python3
    renode-17
    renode-test
    gcc-arm-embedded
    openocd
    git
    picocom
    cmakeCurses
  ];
}
