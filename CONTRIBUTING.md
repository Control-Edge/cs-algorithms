# Contributing to Control System Algorithms

Thanks for your interest in contributing! This repository collects C and C++ algorithms for control-system and embedded software development, validated through simulations built on the [CppModel](https://cppmodel.com) framework.

## What belongs here

In scope:
- Reusable algorithms for control logic, signal processing, filtering, numerical methods, diagnostics, and related simulation utilities.
- Both C and C++ implementations are welcome — picking one is fine, providing both is encouraged when it's natural.
- A simulation under `simulations/` that exercises each new algorithm.

Out of scope:
- Application code, GUIs, or end-user tooling.
- Changes to the CppModel framework itself — those belong upstream at [cppmodel.com](https://cppmodel.com).

## Repository layout

```
algorithms/<algorithm-name>/   one folder per algorithm, kebab-case
simulations/                   one .c and/or .cpp per scenario
dependencies/                  prebuilt CppModel libraries (gitignored)
```

Naming conventions already in use, please match them:
- **Folders**: `kebab-case` (e.g. `pid-controller`, `first-order`).
- **C sources/headers**: prefix with `C` — e.g. `CPid.c` / `CPid.h`, `CFirstOrder.c` / `CFirstOrder.h`.
- **C++ sources/headers**: match the algorithm name in PascalCase — e.g. `PIDController.cpp` / `PIDController.h`, `FirstOrder.cpp` / `FirstOrder.h`.
- **Simulation executables**: `<Thing>Sim` — e.g. `CPidSystemSim` (C), `CppPidSystemSim` (C++).

## Building and running locally

See the [How to use](README.md#how-to-use) section in the README for the build steps.

The simulations connect to a CppModel workspace at runtime, so they need credentials:

```bash
export CPPMODEL_USERNAME=your-username
export CPPMODEL_PASSWORD=your-password
ctest --output-on-failure   # from the build/ directory
```

Without credentials the simulations will fail when they reach the API connector — that's expected.

## Adding a new algorithm

1. Create `algorithms/<your-algorithm-name>/` with your C and/or C++ sources.
2. Add a simulation under `simulations/` that exercises the algorithm against a meaningful scenario.
3. Register the simulation in [simulations/CMakeLists.txt](simulations/CMakeLists.txt) using `add_simulation_test(<name> <sources...>)`.
4. Build locally and run `ctest --output-on-failure` to confirm it passes.
5. Open a PR and confirm all three CI workflows (Linux / macOS / Windows) pass before requesting review.

Add a short header comment at the top of each source file describing inputs, outputs, and any assumptions (sample rate, units, saturation behavior, etc.).

## Coding conventions

- **C**: C99, snake_case for functions and struct fields.
- **C++**: C++17, PascalCase for types and methods, camelCase for local variables. Match the style already in [PIDController.cpp](algorithms/pid-controller/PIDController.cpp) and similar files.
- **External dependencies**: limited to CppModel, OpenSSL, zlib, and ixwebsocket. Don't introduce new third-party libraries without discussing first.
- **Formatting**: no `.clang-format` is enforced — match the surrounding file's indentation and brace style.

## Pull request rules

- Branch from `main`, open the PR against `main`.
- **One algorithm per PR.** Bundling several makes review harder and bisecting failures much harder. Bug fixes and small refactors can ride along with the algorithm they relate to.
- All three CI workflows must be green before merge.
- Keep commit messages descriptive — first line under ~70 characters, a paragraph of context below if needed.

## Developer Certificate of Origin (DCO)

This project uses the [Developer Certificate of Origin](https://developercertificate.org) for all contributions. The DCO is a lightweight, per-commit attestation that you have the right to submit the code under this project's MIT license. There is no separate form to sign — you accept the DCO by adding a `Signed-off-by` line to every commit:

```
Signed-off-by: Jane Doe <jane.doe@example.com>
```

Git can do this for you automatically:

```bash
git commit -s -m "Add Kalman filter algorithm"
```

The full text of the DCO is reproduced below for reference. By signing off on a commit, you are attesting to it.

> ### Developer Certificate of Origin 1.1
>
> By making a contribution to this project, I certify that:
>
> **(a)** The contribution was created in whole or in part by me and I have the right to submit it under the open source license indicated in the file; or
>
> **(b)** The contribution is based upon previous work that, to the best of my knowledge, is covered under an appropriate open source license and I have the right under that license to submit that work with modifications, whether created in whole or in part by me, under the same open source license (unless I am permitted to submit under a different license), as indicated in the file; or
>
> **(c)** The contribution was provided directly to me by some other person who certified (a), (b) or (c) and I have not modified it.
>
> **(d)** I understand and agree that this project and the contribution are public and that a record of the contribution (including all personal information I submit with it, including my sign-off) is maintained indefinitely and may be redistributed consistent with this project or the open source license(s) involved.

PRs without sign-off on every commit will be asked to amend before merge. If you forget, you can fix the most recent commit with `git commit --amend -s` or rewrite a range with `git rebase --signoff <base>`.

## Reporting issues and asking questions

- **Bugs and feature requests**: open an issue on [GitHub](https://github.com/Control-Edge/cs-algorithms/issues).
- **Design discussion and general questions**: join the [Discord](https://discord.gg/p8X7EJdW).

## License

All contributions are licensed under the [MIT License](LICENSE), matching the rest of the project.
