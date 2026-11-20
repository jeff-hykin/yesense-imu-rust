{
  description = "yesense-imu: Rust driver for Yesense IMUs over a serial port";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        name = "yesense-imu";
        src = nix-filter.lib { root = ./.; exclude = [ "target" "result" ]; };
        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };
        # the library (`lib` output) and yesense-dump (`bin`)
        crate = (import generated { inherit pkgs; }).rootCrate.build;
      in {
        packages.default = crate;
        checks.tests = crate.override { runTests = true; };
        devShells.default = pkgs.mkShell { packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ]; };
      });
}
