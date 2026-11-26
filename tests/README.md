# Tests

- `unit/` contains deterministic tests without hardware.
- `fixtures/` contains only small, reviewed, sanitized test data.

Future integration and UI tests may be added when they contain real behavior.
Hardware-in-the-loop tests must remain explicitly separated and must never be
reported as passing without an attached device and retained evidence.
