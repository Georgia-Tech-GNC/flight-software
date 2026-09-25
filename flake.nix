{
    description = "Flight software development environment";

    inputs = {
        nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    };

    outputs = { self, nixpkgs }:
    let
        systems = [
            "x86_64-linux"
            "aarch64-linux"
            "aarch64-darwin"
        ];

        forAllSystems = nixpkgs.lib.genAttrs systems;
    in {
    devShells = forAllSystems (system:
        let
        pkgs = import nixpkgs {
            inherit system;
        };

        renode-17 = pkgs.stdenv.mkDerivation {
  pname = "renode";
  version = "1.17.0";

  src = pkgs.fetchurl {
    url = "https://builds.renode.io/renode-1.17.0.multiplatform.zip";
    hash = "sha256-us1hpfebrTxPDWLQjp29fHDQbeTqUaLbGJdDsrdMyCY=";
  };

  nativeBuildInputs = [
    pkgs.unzip
  ];

  unpackPhase = ''
    unzip "$src"
  '';

  installPhase = ''
    mkdir -p $out/bin
    mkdir -p $out/share

    cp -r renode_1.17.0-multiplatform/. $out/share/
  '';
};
        renode = pkgs.writeShellScriptBin "renode" ''
        export RENODE_ROOT=${renode-17}/share/renode
  export PATH=${renode-17}/share/renode:$PATH

        exec ${renode-17}/share/renode "$@"
'';

        renode-test = pkgs.writeShellScriptBin "renode-test" ''
            exec ${pkgs.uv}/bin/uv run \
            --with-requirements ${renode-17}/share/tests/requirements.txt \
            ${renode-17}/share/renode-test "$@"
        '';

        in {
        default = pkgs.mkShell {
            packages = with pkgs; [
            uv
            glib
            gtk-sharp-3_0
            gtk3
            cppcheck
            python3
            renode-17
            renode
            renode-test
            gcc-arm-embedded
            openocd
            git
            picocom
            cmakeCurses
            dotnet-sdk_10
            ];

            shellHook = ''
    export LD_LIBRARY_PATH="${
      pkgs.lib.makeLibraryPath [
        pkgs.gtk3
        pkgs.glib
        pkgs.gtk-sharp-3_0
      ]
    }:$LD_LIBRARY_PATH"
  '';
        };
        });
    };
}
