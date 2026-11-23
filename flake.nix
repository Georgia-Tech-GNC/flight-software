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

      fontsConf = pkgs.makeFontsConf {
        fontDirectories = with pkgs; [
          dejavu_fonts
          liberation_ttf
          noto-fonts
        ];
      };

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
          ffmpeg
          nodejs_24
          bashInteractive
          parallel
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

		      export FONTCONFIG_FILE="${fontsConf}"          
          
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
