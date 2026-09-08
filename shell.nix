{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = with pkgs; [
    gtk3
    gtk-sharp-3_0
    glib
    xorg.libX11
    uv
    python3Packages.virtualenv
    python3Packages.pip
  ];

  shellHook = ''
    export LD_LIBRARY_PATH="${pkgs.gtk3}/lib:${pkgs.gtk-sharp-3_0}/lib:${pkgs.glib.out}/lib:${pkgs.xorg.libX11}/lib:$LD_LIBRARY_PATH"
  '';
}
