# Dev Branch History Cleanup Summary

## Status: COMPLETE (local branches ready to push)

The git history cleanup has been completed locally. The following branches are ready:

- `kemjensak/dev-cleanup-backup` — backup of original `dev` branch HEAD (49bc01c)
- `kemjensak/dev-history-cleanup` — cleaned history (535 → 529 commits)

## Changes Made

### Squashed (Groups A, B, C)
| SHA(s) | New Message |
|--------|-------------|
| `b4475d8` + `5ffc8d7` + `015a594` + `5493bbb` | `ci(workflow): configure Conan HTTP retry with global.conf and bash shell` |
| `a6f9488` + `46a4fac` | `feat(examples/cpp): add C++ SDK examples` |
| `891f3d4` + `9e88df6` | `fix(api+examples): add kDeviceCount alias, fix LeaderArm device name resolution, brake usage, and related example fixes` |

### Dropped
| SHA | Reason |
|-----|--------|
| `98032536` | Empty merge commit (0 additions, 0 deletions) |

### Reworded
| Old SHA | Old Message | New Message |
|---------|-------------|-------------|
| `bb4aec8` | `Merge remote-tracking branch 'origin/main'` | `fix(examples/python): fix usage comment numbering in Python examples` |
| `54ec517` | `Merge remote-tracking branch 'origin/feature/master-arm' into dev` | `feat(api+ci): merge LeaderArm feature branch; bump pyproject version, adjust torque limits` |
| `76aece3` | `chore(merge): merge origin/main into main` | `feat(examples/python): add real_time_control and dynamics_robot examples` |

## File Content
✅ File content is identical to `origin/dev` (verified with `git diff HEAD origin/dev`)

## Push Commands
Run in the repository:
```bash
git push origin kemjensak/dev-cleanup-backup
git push origin kemjensak/dev-history-cleanup
```

To merge into dev:
```bash
git checkout dev
git merge --ff-only kemjensak/dev-history-cleanup
git push --force-with-lease origin dev
```

⚠️ **Note:** Force push will overwrite remote `dev` history. Verify the cleaned branch matches expected state before pushing (`git diff origin/dev kemjensak/dev-history-cleanup` should show only history differences).

Or open a PR from `kemjensak/dev-history-cleanup` targeting `dev`.
