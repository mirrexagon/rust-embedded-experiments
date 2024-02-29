{ pkgs ? import <nixpkgs> { } }:

with pkgs;

mkShell {
  name = "rust-embedded-experiments-shell";

  buildInputs = [
    rustup
    probe-rs

    # For cargo-embassy
    pkg-config
    udev
  ];
}
