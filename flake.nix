{
  description = "ROS2 Workspace For CoRE 2022 Team B";

  inputs = {
    ros2nix.url = "github:Pylgos/ros2nix";
    nixpkgs.follows = "ros2nix/nixpkgs";
    std.url = "github:divnix/std";
    std.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = {self, std, ...} @ inputs:
    std.growOn {
      inherit inputs;
      cellsFrom = ./nix;
      cellBlocks = with std.blockTypes; [
        (installables "packages")
        (devshells "devshells")
      ];
    }
    {
      devShells = std.harvest self ["ros_ws" "devshells"];
    };
}
