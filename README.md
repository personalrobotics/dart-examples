# DART Examples for the Personal Robotics Lab

## Installation
Checkout and build this package,
[DART](https://github.com/dartsim/dart.git) (version 5.1 or above),
[dart_rviz](https://github.com/personalrobotics/dart_rviz.git), and
[r3](https://github.com/personalrobotics/r3.git) from source. You can automate
the checkout and build by following the
[development environment](https://www.personalrobotics.ri.cmu.edu/software/development-environment)
instructions with this `.rosinstall` file:

```yaml
- git:
    local-name: dart
    uri: https://github.com/dartsim/dart.git
    version: master
- git:
    local-name: dart_examples
    uri: https://github.com/personalrobotics/dart-examples.git
    version: master
- git:
    local-name: dart_rviz
    uri: https://github.com/personalrobotics/dart_rviz.git
    version: master
- git:
    local-name: r3
    uri: https://github.com/personalrobotics/r3.git
    version: master
```
