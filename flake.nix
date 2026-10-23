{
  description = "Flight software development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-darwin"
      ];

      forAllSystems = nixpkgs.lib.genAttrs systems;
    in {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs {
            inherit system;
          };

          libraryPathVar =
            if pkgs.stdenv.hostPlatform.isDarwin
            then "DYLD_LIBRARY_PATH"
            else "LD_LIBRARY_PATH";

          renode-17 = pkgs.stdenv.mkDerivation {
            pname = "renode";
            version = "1.17.0";

            src = pkgs.fetchurl {
              url = "https://builds.renode.io/renode-1.17.0.multiplatform.zip";
              hash = "sha256-us1hpfebrTxPDWLQjp29fHDQbeTqUaLbGJdDsrdMyCY=";
            };

            nativeBuildInputs = [
              pkgs.unzip
            ] ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isDarwin [
              pkgs.darwin.sigtool
            ];

            unpackPhase = ''
              unzip "$src"
            '';

            installPhase = ''
              runHook preInstall

              mkdir -p "$out/bin" "$out/share"
              cp -r renode_1.17.0-multiplatform/. "$out/share/"

              ${pkgs.lib.optionalString pkgs.stdenv.hostPlatform.isDarwin ''
                codesign --force --sign - \
                  "$out/share/bin/platform-lib/osx-arm64/renode-ui"

                ln -s /tmp/renode-ui-tmp \
                  "$out/share/bin/platform-lib/osx-arm64/.tmp"
              ''}

              ${pkgs.lib.optionalString (pkgs.stdenv.hostPlatform.isLinux && pkgs.stdenv.hostPlatform.isx86_64) ''
                ln -s /tmp/renode-ui-tmp \
                  "$out/share/bin/platform-lib/linux-x64/.tmp"
              ''}

              runHook postInstall
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
              gtk3
              glib
              atk
              pango
              cairo
              gdk-pixbuf
              stdenv.cc.cc.lib
              cppcheck
              python3
              ninja
              renode-17
              renode
              renode-test
              gcc-arm-embedded
              openocd
              git
              pre-commit
              picocom
              cmakeCurses
              dotnet-sdk_10
              doxygen

              libxcb
              libXrandr
              libXrandr
              libpng
            ] ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
              webkitgtk_4_1
            ];

            shellHook = ''
              export ${libraryPathVar}="${
              pkgs.lib.makeLibraryPath (
                [
                  pkgs.gtk3
                  pkgs.glib
                  pkgs.atk
                  pkgs.pango
                  pkgs.cairo
                  pkgs.gdk-pixbuf
                  pkgs.stdenv.cc.cc.lib
                ]
                ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
                  pkgs.libxcb
                  pkgs.libX11
                  pkgs.libXrandr
                  pkgs.libpng
                  pkgs.webkitgtk_4_1
                ]
              )}"

              export UV_PYTHON="${pkgs.python3}/bin/python3"
              export UV_PYTHON_PREFERENCE="only-managed"

              mkdir -p /tmp/renode-ui-tmp

              uv sync
              pre-commit install
            '';
          };
        });
    };
}
