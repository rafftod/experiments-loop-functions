# README
experiments-loop-functions
=====================

This package contains the source for unified handling of loop
fonctions to assess performances in automatic design methods.
It also contains the specific implementation of loop functions for
missions from different AutoMoDe related studies.

All useful information about the AutoMoDe package, including
installation and utilization instructions, are regrouped in the
following technical report ([techrep](#bibliography)). Please cite
this report if you any ARGoS3-AutoMoDe related package.

## Package content
- `loop-functions/` contains all sources for loop functions for
    specific missions from different AutoMoDe related studies.
- `src/` contains the core source files and mother class from which
    all loop functions are derived.

## Installation
### Dependencies
- [ARGoS3](https://github.com/ilpincy/argos3) (3.0.0-beta48)
- [argos3-epuck](https://github.com/demiurge-project/argos3-epuck) (v48)

### Compiling and installing
    $ git clone https://github.com/demiurge-project/experiments-loop-functions.git
    $ cd experiments-loop-functions
    $ mkdir build
    $ cmake ..
    $ make
    $ sudo make install

Once compiled and installed the shared library and header files
are installed in the system.

If you do not have root access off what to install in a your local
argos distribution folder use (if your argos installation is located
in `~/argos-dist`):

    $ cmake .. -DCMAKE_INSTALL_PREFIX=~/argos-dist
    $ make
    $ make install

### How to use
This package is meant to be used as a library to allow unified
definition of loop functions for performance assessment for different
automatic design methods. It is used in AutoMoDe, Evostick and other
automatic design methods (see [references](#bibliography)).

## References
### Bibliography

- [techrep] Ligot, A., Hasselmann, K., Delhaisse, B., Garattoni, L., Francesca, G., & Birattari, M. (2017). AutoMoDe, NEAT, and EvoStick: implementations for the E-puck robot in ARGoS3. Technical report TR/IRIDIA/2017-002, IRIDIA, Université libre de Bruxelles, Belgium.
- [chocolate] Francesca, G., Brambilla, M., Brutschy, A., Garattoni, L., Miletitch, R., Podevijn, G., ... & Mascia, F. (2015). AutoMoDe-Chocolate: automatic design of control software for robot swarms. Swarm Intelligence, 9(2-3), 125-152.
- [gianduja] Hasselmann K., Robert F., Birattari M. (2018) Automatic Design of Communication-Based Behaviors for Robot Swarms. In: Dorigo M., Birattari M., Blum C., Christensen A., Reina A., Trianni V. (eds) Swarm Intelligence. ANTS 2018. Lecture Notes in Computer Science, vol 11172. Springer, Cham
