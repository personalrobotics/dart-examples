# Personal Robotics Lab DART Examples

## Installation
Checkout and build this package,
[DART](https://github.com/dartsim/dart.git) (version 6.0 or above)
and [aikido](https://github.com/personalrobotics/aikido.git) from source. You
can automate the checkout and build by following the
[development environment](https://www.personalrobotics.ri.cmu.edu/software/development-environment)
instructions with this `.rosinstall` file:
```yaml
- git:
    local-name: aikido
    uri: https://github.com/personalrobotics/aikido.git
    version: master
- git:
    local-name: dart
    uri: https://github.com/dartsim/dart.git
    version: release-6.0
- git:
    local-name: dart_examples
    uri: https://github.com/personalrobotics/dart_examples.git
    version: master
- git:
    local-name: herb_description
    uri: https://github.com/personalrobotics/herb_description.git
    version: master
```

## Usage
To load HERB:
```shell
$ rosrun dart_examples load_urdf package://herb_description/robots/herb.urdf
```
