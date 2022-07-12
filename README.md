# M2 Common Code in Python

## Platform

- CentOS 7
- python: 3.9.12

## Needed Package

- [black](https://github.com/psf/black) (22.3.0, optional)
- [documenteer](https://github.com/lsst-sqre/documenteer) (optional)
- [sphinxcontrib-plantuml](https://github.com/sphinx-contrib/plantuml/) (optional, by `pip`)
- pytest (optional, install by `conda`)
- pytest-flake8 (optional, install by `conda -c conda-forge`)

## Code Format

This code is automatically formatted by `black` using a git pre-commit hook.
To enable this:

1. Install the `black` Python package.
2. Run `git config core.hooksPath .githooks` once in this repository.

## Build the Document

To build project documentation, run `package-docs build` to build the documentation.
To clean the built documents, use `package-docs clean`.
See [Building single-package documentation locally](https://developer.lsst.io/stack/building-single-package-docs.html) for further details.

## Unit Tests

You can run the unit tests by:

```bash
pytest tests/
```

## Class Diagrams

The class diagrams are in [here](doc/uml).
You can use the [PlantUML](https://plantuml.com) to read them.
