# default.nix
{ pkgs ? import <nixpkgs> {} }:

pkgs.callPackage ./renode.nix { }
