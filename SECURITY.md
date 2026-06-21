# Security Policy

BobLib is an engineering simulation library. Most incorrect-model, numerical,
documentation, validation, or physical-fidelity concerns should be reported as
normal GitHub issues so they can be discussed and regression-tested in public.

Use a private security report only for vulnerabilities that could affect a
user's machine, credentials, CI secrets, package-distribution integrity, or
other non-public data.

## Supported Versions

Security fixes are handled on the active `main` branch. This project does not
currently publish versioned security-maintenance releases.

## Reporting a Vulnerability

Please report security-sensitive issues using GitHub's private vulnerability
reporting flow if it is enabled for the repository. If that is not available,
contact the maintainers through the BobDyn project channels listed in the
official documentation:

https://bobdyn.com/boblib

Do not include secrets, proprietary vehicle data, or private test logs in a
public issue.

## Safety Notice

BobLib is not safety-certified software. Simulation results must be validated
against measured data before being used for design, controls, or operating
decisions.
