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
          pkgs.patchelf
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
            UI="$out/share/bin/platform-lib/linux-x64/renode-ui"

            patchelf \
              --set-interpreter "${pkgs.stdenv.cc.bintools.dynamicLinker}" \
              "$UI"

            ln -s /tmp/renode-ui-tmp \
              "$out/share/bin/platform-lib/linux-x64/.tmp"
          ''}

          runHook postInstall
        '';
      };

      renode = pkgs.writeShellScriptBin "renode" ''
        export RENODE_ROOT=${renode-17}/share/renode
        export PATH=${renode-17}/share/renode:$PATH
        ${pkgs.lib.optionalString pkgs.stdenv.hostPlatform.isLinux ''
          export GDK_BACKEND=x11
          export LIBGL_ALWAYS_SOFTWARE=1
          export GALLIUM_DRIVER=llvmpipe
        ''}
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
          mesa
          libglvnd
          fontconfig
        ] ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
          libxcb
          libX11
          libXrandr
          libpng
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
                pkgs.fontconfig
              ]
              ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
                pkgs.libxcb
                pkgs.libX11
                pkgs.libXrandr
                pkgs.libpng
                pkgs.webkitgtk_4_1
                pkgs.libglvnd
                pkgs.mesa
              ]
            )}"

          export FONTCONFIG_FILE="${pkgs.fontconfig.lib}/etc/fonts/fonts.conf"
          export FONTCONFIG_PATH="${pkgs.fontconfig.lib}/etc/fonts"

          ${pkgs.lib.optionalString pkgs.stdenv.hostPlatform.isLinux ''
            export LIBGL_DRIVERS_PATH="${pkgs.mesa}/lib/dri"
            export __EGL_VENDOR_LIBRARY_DIRS="${pkgs.mesa}/share/glvnd/egl_vendor.d"
          ''}

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
