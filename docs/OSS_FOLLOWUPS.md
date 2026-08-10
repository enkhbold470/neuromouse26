# Open-source follow-ups (manual / human-gated)

Tracked after the multi-agent OSS audit ([PR #11](https://github.com/enkhbold470/neuromouse26/pull/11)).

Linear project: [NeuroMouse26 OSS launch follow-ups](https://linear.app/1nky/project/neuromouse26-oss-launch-follow-ups-7cc37aa2239c)

## Security (do first)

- [ ] [INK-9](https://linear.app/1nky/issue/INK-9/rotate-compromised-wifi-password-from-git-history-era) — **Rotate WiFi password** previously committed in git history (`NETGEAR38` / old password era). Tip of tree is scrubbed; history is not.
- [ ] [INK-10](https://linear.app/1nky/issue/INK-10/purge-wifi-secrets-from-git-history-filter-repo-bfg) — **Optional:** purge secret blobs from history with `git filter-repo` / BFG **after** rotation (blocked by INK-9).

## Release / GitHub settings

- [ ] [INK-8](https://linear.app/1nky/issue/INK-8/cut-v201-release-after-merging-oss-audit-pr) — Merge PR #11, then cut **`v2.0.1`** from updated `main`.
- [ ] [INK-5](https://linear.app/1nky/issue/INK-5/fix-or-clear-broken-github-pages-homepage-url) — Fix or clear the repo **homepage / documentation URL** (currently 404s).
- [ ] [INK-6](https://linear.app/1nky/issue/INK-6/enable-main-branch-protection-require-pr-platformio-ci) — Enable **`main` branch protection** (require PR + PlatformIO CI).

## Optional polish

- [ ] [INK-7](https://linear.app/1nky/issue/INK-7/compress-or-git-lfs-competition-demo-gifs-9mb) — Compress or move competition GIFs under `docs/images/` to Git LFS (~9MB).

## Already done in PR #11

Community health files, CI matrix, dead env cleanup, docs accuracy sync, EXIF strip, Pixabay/competition-rules removal, tools docs, `ble-test` NimBLE 2.x fix.
