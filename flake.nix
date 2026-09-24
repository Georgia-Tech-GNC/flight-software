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
            url = "https://builds.renode.io/renode-1.17.0.linux-portable.tar.gz";
            hash = "sha256-kqM9aqedPIv3TA2TREegSNvHYQLe9nXS5T3pAazzAOs=";
            };

            installPhase = ''
            mkdir -p $out/bin
            mkdir -p $out/share

            cp -r ./* $out/share
            ln -s $out/share/renode $out/bin/renode
            '';
        };

        renode-test = pkgs.writeShellScriptBin "renode-test" ''
            exec ${pkgs.uv}/bin/uv run \
            --with-requirements ${renode-17}/share/tests/requirements.txt \
            ${renode-17}/share/renode-test "$@"
        '';

        in {
        default = pkgs.mkShell {
            packages = with pkgs; [
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
        };
        });
    };
}
