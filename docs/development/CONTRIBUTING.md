---
title: Contributing
has_children: true
nav_order: 10
---

# Contributing to the ARCHES PiCar-X

The ARCHES PiCar-X is open-source research software to exemplify digital twin research. We welcome everyone to contribute to this project.
Contributions are not limited to code contributions, instead we welcome and recognize everything concerning:

* Raising issues, questions and suggestions for using the ARCHES PiCar-X
* Fixing bugs or implementing new features
* Improving the documentation
* Using the ARCHES PiCar-X as part of your (not necessarily scientific) digital twin research


## We Develop with Docker
Keep in mind that all contributions should be able to run in an environment with [Docker](https://www.docker.com/) containers. The CI/CD pipelines use Docker to execute all steps in an fully automated way. Especially, the integration tests are executed in Docker containers.

## Start Contributing

If you have bug reports, feature requests, questions or suggestions, you may create a [GitHub issue](https://github.com/cau-se/arches-picar-x/issues) or directly [contact maintainers](../project-info).
You can also create a [GitHub pull request](https://github.com/cau-se/arches-picar-x/pulls) if you have already implemented bug fixes and improvements.

If you would like to get more involved in ARCHES PiCar-X development and maintenance, you may contact us as well.

## Any Contributions You Make Will be Under the Apache License Version 2.0
In short, when you submit code changes, your submissions are understood to be under the same [Apache License 2.0](https://choosealicense.com/licenses/apache-2.0/) that covers the project. Feel free to contact the maintainers if that's a concern.

## Internal Project Structure

The ARCHES PiCar-X is organized as a monorepo containing multiple largely independent modules in subdirectories.
See the project's [`README.md`](https://github.com/cau-se/arches-picar-x/blob/main/README.md#project-structure) for an overview of all modules.
Each module directory provides a dedicated `README.md` file describing how to build, test, package,... the corresponding module.
