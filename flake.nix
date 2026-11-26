{
  description = "Flight software development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      shellSystems = [
        "x86_64-linux"
        "aarch64-darwin"
      ];

      packageSystems = [
        "x86_64-linux"
        "aarch64-darwin"
      ];

      forSystems = systems:
        nixpkgs.lib.genAttrs systems;

      renodeBinPath = builtins.getEnv "RENODE_BIN";
      renodeSrcPath = builtins.getEnv "RENODE_SRC";

      renodeBin = builtins.path {
        path = renodeBinPath;
        name = "renode-build-output";
      };

      renodeSource = builtins.path {
        path = renodeSrcPath;
        name = "renode-source";
      };

      mkRenode = system:
        let
          pkgs = import nixpkgs {
            inherit system;
          };

          lib = pkgs.lib;

          dotnetRuntime =
            pkgs.dotnetCorePackages.runtime_10_0;

          pythonLibs =
            with pkgs.python313Packages;
            makePythonPath [
              construct
              psutil
              pyyaml
              requests
              tkinter
              telnetlib3
              pyelftools

              (robotframework.overridePythonAttrs (oldAttrs: {
                version = "6.1";

                src = pkgs.fetchFromGitHub {
                  owner = "robotframework";
                  repo = "robotframework";
                  rev = "v6.1";
                  hash = "sha256-l1VupBKi52UWqJMisT2CVnXph3fGxB63mBVvYdM1NWE=";
                };

                patches = (oldAttrs.patches or [ ]) ++ [
                  (pkgs.fetchpatch {
                    name = "python3.13-support.patch";
                    url = "https://github.com/robotframework/robotframework/commit/921e352556dc8538b72de1e693e2a244d420a26d.patch";
                    hash = "sha256-aSaror26x4kVkLVetPEbrJG4H1zstHsNWqmwqOys3zo=";
                  })
                ];
              }))
            ];

          runtimeLibs = with pkgs; [
            gtk3
            gtk3-x11
            glib
            atk
            pango
            cairo
            gdk-pixbuf
            stdenv.cc.cc.lib
            fontconfig
            libglvnd
            mesa
            dconf
          ] ++ lib.optionals pkgs.stdenv.hostPlatform.isLinux [
            libxcb
            libX11
            libXrandr
            libpng
            webkitgtk_4_1
          ];

          runtimeLibPath =
            lib.makeLibraryPath runtimeLibs;

          rpath =
            "${runtimeLibPath}:$ORIGIN:$ORIGIN/..:$ORIGIN/../..:$ORIGIN/../../..";

          gioModules =
            "${lib.getLib pkgs.dconf}/lib/gio/modules";

        in
        pkgs.stdenvNoCC.mkDerivation {
          pname = "renode";
          version = "unstable";

          src = renodeBin;
          dontUnpack = true;

          nativeBuildInputs = with pkgs; [
            makeWrapper
            patchelf
          ]
          ++ lib.optionals pkgs.stdenv.hostPlatform.isDarwin [
            darwin.sigtool
            darwin.cctools
          ];
          
          installPhase = ''
            runHook preInstall

            mkdir -p "$out/lib/renode"

            # Preserve Renode's source-tree layout.
            mkdir -p "$out/lib/renode/output/bin/Release"

            # Install the locally built Renode binaries.
            cp -R "$src/." \
              "$out/lib/renode/output/bin/Release/"

            chmod -R u+w "$out/lib/renode"

            # Redirect Renode's native platform cache to a writable temp directory.
            for rid in linux-x64 osx-x64 osx-arm64; do
                platformTmp="$out/lib/renode/output/bin/Release/platform-lib/$rid/.tmp"

                mkdir -p "$(dirname "$platformTmp")"

                if [ -e "$platformTmp" ] || [ -L "$platformTmp" ]; then
                    rm -rf "$platformTmp"
                fi

                ln -s /tmp/renode-ui-tmp "$platformTmp"
            done

            cp "${renodeSource}/.renode-root" \
              "$out/lib/renode/.renode-root"

            cp -R "${renodeSource}/platforms" \
              "$out/lib/renode/"

            mkdir -p "$out/lib/renode/lib/resources"

            cp -R "${renodeSource}/lib/resources/." \
                "$out/lib/renode/lib/resources/"

            cp "${renodeSource}/renode-test" \
              "$out/lib/renode/renode-test"

            cp -R "${renodeSource}/tests" \
              "$out/lib/renode/"

            cp -R "${renodeSource}/tools" \
              "$out/lib/renode/"

            ${lib.optionalString pkgs.stdenv.hostPlatform.isDarwin ''
              ui="$out/lib/renode/output/bin/Release/platform-lib/osx-arm64/renode-ui"

              if [ -f "$ui" ]; then
                codesign --force --sign - "$ui"
              else
                echo "Renode UI executable not found: $ui" >&2
                exit 1
              fi
            ''}

            runHook postInstall
          '';

          postFixup = ''
            echo "Patching Renode ELF files..."

            ${lib.optionalString pkgs.stdenv.hostPlatform.isLinux ''
              find "$out/lib/renode" -type f -print0 |
                while IFS= read -r -d $'\0' file; do
                  if ! patchelf --print-needed "$file" >/dev/null 2>&1; then
                    continue
                  fi

                  echo "Patching: $file"

                  patchelf --set-rpath "${rpath}" "$file"

                  interpreter="$(
                    patchelf --print-interpreter "$file" 2>/dev/null || true
                  )"

                  if [ -n "$interpreter" ]; then
                    patchelf \
                      --set-interpreter \
                      "${pkgs.stdenv.cc.bintools.dynamicLinker}" \
                      "$file"
                  fi
                done
            ''}

            echo "Creating Renode launcher..."

            makeWrapper "${dotnetRuntime}/bin/dotnet" \
              "$out/bin/renode" \
              --add-flags \
                "$out/lib/renode/output/bin/Release/Renode.dll" \
              --set RENODE_ROOT "$out/lib/renode" \
              --prefix LD_LIBRARY_PATH ":" "${runtimeLibPath}" \
              --prefix PYTHONPATH ":" "${pythonLibs}" \
              --prefix GIO_EXTRA_MODULES ":" "${gioModules}" \
              --set WEBKIT_DISABLE_DMABUF_RENDERER 1 \
              ${lib.optionalString pkgs.stdenv.hostPlatform.isLinux ''
                --set GDK_BACKEND x11 \
                --set LIBGL_ALWAYS_SOFTWARE 1
              ''}

            echo "Creating Renode test launcher..."

            makeWrapper "$out/lib/renode/renode-test" \
              "$out/bin/renode-test" \
              --prefix PATH ":" \
                "$out/bin:$out/lib/renode:${lib.makeBinPath [ dotnetRuntime ]}" \
              --set RENODE_ROOT "$out/lib/renode" \
              --prefix LD_LIBRARY_PATH ":" "${runtimeLibPath}" \
              --prefix PYTHONPATH ":" "${pythonLibs}" \
              --prefix GIO_EXTRA_MODULES ":" "${gioModules}" \
          '';

          meta = {
            description = "Renode embedded systems simulator";
            mainProgram = "renode";
            platforms = packageSystems;
          };
        };

    in
    {
      packages = forSystems packageSystems (system:
        let
          renode = mkRenode system;
        in
        {
          inherit renode;
          default = renode;
        });

      devShells = forSystems shellSystems (system:
        let
          pkgs = import nixpkgs {
            inherit system;
          };

          fontsConf = pkgs.makeFontsConf {
            fontDirectories = with pkgs; [
              dejavu_fonts
              liberation_ttf
              noto-fonts
            ];
          };

        in
        {
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
              patchelf
            ] ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
              libxcb
              libX11
              libXrandr
              libpng
              webkitgtk_4_1
            ];

            shellHook = ''
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
