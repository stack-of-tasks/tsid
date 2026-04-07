{
  description = "Efficient Task Space Inverse Dynamics (TSID) based on Pinocchio.";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    systems.follows = "gepetto/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          {
            flakoboros.extraDevPyPackages = [ "tsid" ];
            flakoboros.overrideAttrs.tsid = _: {
              src = lib.fileset.toSource {
                root = ./.;
                fileset = lib.fileset.unions [
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
            };
          }
        ];
      }
    );
}
