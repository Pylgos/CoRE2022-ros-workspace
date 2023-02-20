{ inputs
, cell
}:
let
  ros2nix = inputs.ros2nix;
  rosLibs = ros2nix.humble.lib;
  rosPkgs = ros2nix.legacyPackages.humble;
  nixpkgs = inputs.nixpkgs.legacyPackages;
  l = inputs.nixpkgs.lib // builtins;
in 
{
  default = rosPkgs.mkRosWorkspace {
    pkgs = with rosPkgs; [
      desktop
    ];
  };
}