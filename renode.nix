{
  buildDotnetModule,
  cmake,
  dconf,
  dotnetCorePackages,
  fetchFromGitHub,
  fetchpatch,
  gcc,
  glibcLocalesUtf8,
  gtk3-x11,
  gtk3,
  lib,
  python313Packages,
  stdenv,
}:

let
  hostArch =
    if stdenv.hostPlatform.isx86 then
      "i386"
    else if stdenv.hostPlatform.isAarch64 then
      "aarch64"
    else
      throw "renode: unsupported host architecture ${stdenv.hostPlatform.system}";

  resources = fetchFromGitHub {
    owner = "renode";
    repo = "renode-resources";
    rev = "d3d69f8f17ed164ee23e46f0c06844a69bf4c004";
    hash = "sha256-wR3heL58NOQLENwCzL4lPM4KuvT/ON7dlc/KUqrlRjg=";
  };

  pythonLibs =
    with python313Packages;
    makePythonPath [
      construct
      psutil
      pyyaml
      requests
      tkinter

      # from tools/csv2resd/requirements.txt
      construct

      # from tools/execution_tracer/requirements.txt
      pyelftools

      (robotframework.overridePythonAttrs (oldAttrs: {
        version = "6.1";
        src = fetchFromGitHub {
          owner = "robotframework";
          repo = "robotframework";
          rev = "v6.1";
          hash = "sha256-l1VupBKi52UWqJMisT2CVnXph3fGxB63mBVvYdM1NWE=";
        };
        patches = (oldAttrs.patches or [ ]) ++ [
          (fetchpatch {
            # utest: Improve filtering of output sugar for Python 3.13+
            name = "python3.13-support.patch";
            url = "https://github.com/robotframework/robotframework/commit/921e352556dc8538b72de1e693e2a244d420a26d.patch";
            hash = "sha256-aSaror26x4kVkLVetPEbrJG4H1zstHsNWqmwqOys3zo=";
          })
        ];
      }))
    ];

in
buildDotnetModule rec {
  pname = "renode";
  version = "1.17.1";

  src = fetchFromGitHub {
    owner = "renode";
    repo = "renode";
    rev = "ab721d88e135a1bcb8ed2ecc5a38f51cbe61fdd2";
    hash = "sha256-vVd1ZTz5+iuKjZh7gxgVvX496+FrTxVSV+3neWUy9/Y=";
    fetchSubmodules = true;
  };

  disallowedReferences = [
    cmake
    gcc
    dotnet-sdk
  ];

  projectFile = "Renode.sln";

  dotnet-sdk = dotnetCorePackages.sdk_10_0;
  dotnet-runtime = dotnetCorePackages.runtime_10_0;

  nugetDeps = ./deps.json;

  patches = [ ./renode-test.patch ];

  dotnetFlags = [ 
    "-p:TargetFrameworks=net10.0"
    "-p:RemoveWPF=true"
  ];

  postPatch = ''
    # https://github.com/dotnet/roslyn/issues/37379#issuecomment-513371985
    cat <<'EOF' > Directory.Build.props
    <Project>
      <ItemGroup>
        <SourceRoot Include="$(MSBuildThisFileDirectory)/"/>
    </ItemGroup>
    </Project>
    EOF

    patchShebangs build.sh tools/

    # Fixes determinism build error
    sed -i 's/AssemblyVersion("1.0.*")/AssemblyVersion("1.0.0.0")/g' lib/AntShell/AntShell/Properties/AssemblyInfo.cs lib/CxxDemangler/CxxDemangler/Properties/AssemblyInfo.cs
  '';

  nativeBuildInputs = [
    cmake
    gcc
  ];
  runtimeDeps = [
    gtk3
  ];

  dontUseCmakeConfigure = true;

  enableParallelBuilding = false;

  preBuild = ''
    mkdir -p lib/resources
    ln -s ${resources}/* lib/resources/

    mkdir output

    cat <<'EOF' > output/properties.csproj
<?xml version="1.0" encoding="utf-8"?>
<Project DefaultTargets="Build" xmlns="http://schemas.microsoft.com/developer/msbuild/2003" ToolsVersion="14.0">
<PropertyGroup>
  <CompilerPath Condition="$(CompilerPath) == &apos;&apos;">/usr/bin/gcc</CompilerPath>
  <LinkerPath Condition="$(LinkerPath) == &apos;&apos;">/usr/bin/gcc</LinkerPath>
  <ArPath Condition="$(ArPath) == &apos;&apos;">/usr/bin/ar</ArPath>
  <CurrentPlatform>Linux</CurrentPlatform>
  <DefineConstants>$(DefineConstants);PLATFORM_LINUX</DefineConstants>
</PropertyGroup>
</Project>
EOF

    sed -i "s#/usr/bin/gcc#${gcc}/bin/gcc#g" output/properties.csproj
    sed -i "s#/usr/bin/ar#${gcc}/bin/ar#g" output/properties.csproj

    # To fix value "" error in element <Import>
    rm -rf src/Directory.Build.targets

    CORES=(arm.le arm.be arm64.le arm-m.le arm-m.be ppc.le ppc.be ppc64.le ppc64.be i386.le x86_64.le riscv.le riscv64.le sparc.le sparc.be xtensa.le)
    for core_config in ''${CORES[@]}
    do
      CORE="$(echo $core_config | cut -d '.' -f 1)"
      ENDIAN="$(echo $core_config | cut -d '.' -f 2)"
      BITS=32

      if [[ $CORE =~ "64" ]]; then
        BITS=64
      fi

      SOURCE="${src}/src/Infrastructure/src/Emulator/Cores"
      CMAKE_CONF_FLAGS="-DTARGET_ARCH=$CORE -DTARGET_WORD_SIZE=$BITS -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$out/lib"
      CORE_DIR=build/$CORE/$ENDIAN
      mkdir -p $CORE_DIR
      pushd $CORE_DIR

      if [[ $ENDIAN == "be" ]]; then
          CMAKE_CONF_FLAGS+=" -DTARGET_WORDS_BIGENDIAN=1"
      fi

      cmake $CMAKE_CONF_FLAGS -DHOST_ARCH=${hostArch} $SOURCE
      cmake --build . -j$NIX_BUILD_CORES
      popd
    done

    mkdir -p src/Infrastructure/src/Emulator/Cores/bin/Release/lib
    ln -s $out/lib/*.so src/Infrastructure/src/Emulator/Cores/bin/Release/lib
  '';

  dotnetInstallFlags = [ 
    "-p:TargetFramework=net10.0"
    "-p:RemoveWPF=true"
  ];

  postInstall = ''
    rm -rf build output/properties.csproj
    find . -type d -name obj -exec rm -rf {} +
    mkdir -p $out/lib/renode
    mv * .renode-root $out/lib/renode

    makeWrapper "$out/lib/renode/renode-test" "$out/bin/renode-test" \
      --prefix PATH : "$out/lib/renode:${lib.makeBinPath [ dotnet-runtime ]}" \
      --prefix GIO_EXTRA_MODULES : "${lib.getLib dconf}/lib/gio/modules" \
      --suffix LD_LIBRARY_PATH : "${lib.makeLibraryPath [ gtk3-x11 ]}" \
      --prefix PYTHONPATH : "${pythonLibs}" \
      --set LOCALE_ARCHIVE "${glibcLocalesUtf8}/lib/locale/locale-archive"
  '';

  postFixup = ''
    mv $out/bin/Renode $out/bin/renode
  '';

  executables = [ "Renode" ];

  passthru.updateScript = ./update.sh;

  meta = {
    changelog = "https://github.com/renode/renode/blob/${version}/CHANGELOG.rst";
    description = "Virtual development framework for complex embedded systems";
    downloadPage = "https://github.com/renode/renode";
    homepage = "https://renode.io";
    license = lib.licenses.bsd3;
    maintainers = with lib.maintainers; [
      otavio
      znaniye
    ];
    platforms = [
      "x86_64-linux"
      "aarch64-linux"
    ];
  };
}
