<!--
Thanks for contributing! Please make sure you've read CONTRIBUTING.md.
For new algorithms, please open an Algorithm proposal issue first if you haven't already.
-->

## Summary

<!-- What does this PR do? One or two sentences. -->

## Related issue

<!-- e.g. Closes #123, Refs #456. Leave blank if no issue. -->

## Type of change

- [ ] New algorithm
- [ ] New simulation / test
- [ ] Bug fix in an existing algorithm
- [ ] Build / CI / tooling change
- [ ] Documentation

## Validation

<!--
How did you verify this works? For new algorithms, describe the simulation scenario.
For bug fixes, describe what was broken and how the test exercises the fix.
-->

## Checklist

- [ ] All commits are signed off (`git commit -s`) per the [DCO](../blob/main/CONTRIBUTING.md#developer-certificate-of-origin-dco).
- [ ] CI is green on all three platforms (Linux, macOS, Windows).
- [ ] If this adds a new algorithm: a simulation under `simulations/` exercises it and is registered via `add_simulation_test(...)`.
- [ ] If this adds a new algorithm: only **one** algorithm per PR.
- [ ] Naming follows the conventions in [CONTRIBUTING.md](../blob/main/CONTRIBUTING.md#repository-layout) (kebab-case folders, `C`-prefixed C files, PascalCase C++ files).
