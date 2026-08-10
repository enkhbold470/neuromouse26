# Open-source follow-ups (manual / human-gated)

Tracked after the multi-agent OSS audit ([PR #11](https://github.com/enkhbold470/neuromouse26/pull/11)).
These were intended for Linear; create Linear issues from this list when convenient.

## Security (do first)

- [ ] **Rotate WiFi password** previously committed in git history (`NETGEAR38` / old password era). Tip of tree is scrubbed (`WifiDebug.h` placeholders); history is not.
- [ ] **Optional:** purge secret blobs from history with `git filter-repo` / BFG **after** rotation. Coordinate with all clones/forks before force-pushing.

## Release / GitHub settings

- [ ] Merge PR #11, then cut **`v2.0.1`** (or retag) from current `main` — `v2.0.0` is behind audit fixes.
- [ ] Fix or clear the repo **homepage / documentation URL** (community profile currently 404s on `enkhbold470.github.io/neuromouse26`).
- [ ] Enable **`main` branch protection**: require PR; require PlatformIO CI status check; disallow force-push.

## Optional polish

- [ ] Compress or move competition GIFs under `docs/images/` to Git LFS (~9MB total).
- [ ] Prune stale remote branches (`elijah`, `trust-me-bro`, etc.) after review.
- [ ] Disable empty Wiki or add content.

## Already done in PR #11

Community health files, CI matrix, dead env cleanup, docs accuracy sync, EXIF strip, Pixabay/competition-rules removal, tools docs, `ble-test` NimBLE 2.x fix.
