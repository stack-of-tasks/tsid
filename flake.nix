{
  description = "Efficient Task Space Inverse Dynamics (TSID) based on Pinocchio.";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        extraDevPyPackages = [ "tsid" ];
        overrideAttrs.tsid =
          { ... }:
          {
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
    );
}
