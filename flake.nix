{
  description = "Efficient Task Space Inverse Dynamics (TSID) based on Pinocchio.";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        { pkgs, self', ... }:
        {
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
          };
          devShells.default = pkgs.mkShell {
            inputsFrom = [ self'.packages.default ];
            packages = [ (pkgs.python3.withPackages (p: [ p.tomlkit ])) ]; # for "make release"
          };
          packages = {
            default = self'.packages.tsid;
            tsid = pkgs.python3Packages.tsid.overrideAttrs (_: {
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./bindings
                  ./CMakeLists.txt
                  ./doc
                  ./include
                  ./models
                  ./package.xml
                  ./src
                  ./tests
                ];
              };
            });
          };
        };
    };
}
