# Polymetis: a real-time PyTorch controller manager

[![CircleCI](https://circleci.com/gh/fair-robotics/polymetis/tree/main.svg?style=svg&circle-token=8f484b4a044fb2836a354477de4cf7c7cc3cb23d)](https://circleci.com/gh/fair-robotics/polymetis/tree/main)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

Infrastructure for running robot experiments at Facebook AI Research. See [documentation](https://polymetis-docs.github.io/).
## For Developers
### Formatting

We enforce linters for our code. The `formatting` test will not pass if your code does not conform.

To make this easy for yourself, you can either
- Add the formattings to your IDE
- Install the git [pre-commit](https://pre-commit.com/) hooks by running
    ```bash
    pip install pre-commit
    pre-commit install
    ```

#### Python

We use [black](https://github.com/psf/black).

To enforce this in VSCode, install [black](https://github.com/psf/black), [set your Python formatter to black](https://code.visualstudio.com/docs/python/editing#_formatting) and [set Format On Save to true](https://code.visualstudio.com/updates/v1_6#_format-on-save).

To format manually, run: `black .`

#### C++

We use [clang-format](https://clang.llvm.org/docs/ClangFormat.html).

To automatically format in VSCode, install [clang-format](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format) and [set Format On Save to true](https://code.visualstudio.com/updates/v1_6#_format-on-save).

To format manually, run: `./scripts/format_cpp.sh format all`

### Code collaboration conventions
- Trunk-based development: Branches should be short-lived and a PR should be submitted when a feature is complete.
- Run `pytest` and make sure it passes before pushing anything to `main` branch. Remember to have good test coverage for new features.
