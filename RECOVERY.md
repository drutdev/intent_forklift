# Recover shelved work

## Saved local state

The geometric-only cleanup on 2026-11-21 preserved the previous history and working files before changing the repository.

| Saved state | Stable local Git reference |
| --- | --- |
| Previous committed history at `bee228b` | `archive/pre-geometric-cleanup-20261121T180109Z` |
| Complete original tracked and untracked working state | `refs/archive/geometric-cleanup-20261121T180109Z/checkpoint` |
| Shelved search/iLQR, RL design, tests, conversion scripts, drafts, and legacy assets | `refs/archive/geometric-cleanup-20261121T180109Z/excluded` |

The original full checkpoint is stash commit `e383ab1bdff5fdf0e2ae3c4bd021b0959075e8d1`. The excluded-files stash is `8f98f11556099bd7493bf5ab9a9b4a8eaff156a4`. The fixed references retain these objects even if stash indices change. The shared first-week hardware note also has a copy in the excluded-files stash and remains in the active repository.

`master` was soft-reset from `bee228b` to the shared-runtime commit `b26ad17`. The geometric implementation and retained runtime changes remain uncommitted. The cleanup did not create a project commit or push anything.

The original file hashes, patches, cleanup metadata, and verified file copies are under `.git/geometric-cleanup-20261121T180109Z/`. Git stashes do not include ignored recordings or `.venv`; those directories remain in the original workspace. These references and stashes are local recovery data.

## Restore the complete original workspace

Create a separate worktree so restoration leaves the geometric workspace intact. Run these commands from the geometric repository root. Use a new directory and branch name.

```sh
git worktree add -b recovery/pre-geometric-cleanup ../intent_forklift-recovery archive/pre-geometric-cleanup-20261121T180109Z
git -C ../intent_forklift-recovery stash apply --index refs/archive/geometric-cleanup-20261121T180109Z/checkpoint
```

This restores the pre-cleanup tracked edits, untracked implementations, drafts, and legacy files on the original commit. Install dependencies in the recovered worktree before running it. The original workspace retains the existing simulation recordings.

## Inspect the excluded files

```sh
git stash list
git stash show --include-untracked --stat refs/archive/geometric-cleanup-20261121T180109Z/excluded
git ls-tree -r --name-only refs/archive/geometric-cleanup-20261121T180109Z/excluded^3
```

After the soft reset, the excluded files were untracked relative to the shared-runtime commit. Their contents are in the stash's third parent. To recover one file, substitute its repository-relative path below.

```sh
git show refs/archive/geometric-cleanup-20261121T180109Z/excluded^3:pallet_engagement/search_ilqr.py > /tmp/recovered-search_ilqr.py
```

Recovering individual files does not restore their former CLI registration or documentation. Use the complete-workspace procedure to recover the original runnable multi-method stack.
