# Worlds

These are example configs you can load into the sim to use.
All of which are .yaml files.

Create your own by creating a file with the following format:
```yaml
lane_boundary:
    left:
        - {x: 0.0, y: 0.0, z: 0.0}
        - ...
    right:
        - {x: 0.0, y: 0.0, z: 0.0}
        - ...

entities:
    - x: 1.0
      y: 1.0
      z: 0.0
      gamma: 0.0 # i.e., yaw in rads
    - ...
```