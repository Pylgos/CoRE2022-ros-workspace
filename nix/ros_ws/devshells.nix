{ inputs
, cell
}:
let
  ros2nix = inputs.ros2nix;
  rosPkgs = ros2nix.legacyPackages.humble;
  nixpkgs = inputs.nixpkgs;
  l = inputs.nixpkgs.lib // builtins;
in
{
  default = rosPkgs.mkRosWorkspace {
    pkgs = with rosPkgs; [
      desktop
      foxglove_bridge
    ];
    shellHook = ''
      set +u
      alias bld='colcon build'
      alias setup='source install/local_setup.bash'
    '';
    COLCON_DEFAULTS_FILE = ./defaults.yaml;
  };
}
