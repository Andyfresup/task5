# Branching Strategy

## Default Branch
- Use main as the default long-lived branch.
- Keep main always releasable.

## Branch Naming
- feature/<module>-<short-description>
- fix/<module>-<short-description>
- hotfix/<module>-<short-description>
- release/<version>

Examples:
- feature/navigation-task5-bridge
- fix/local-planner-cmdvel-conflict
- hotfix/tf-loop
- release/v1.2.0

## Workflow
1. Create branch from main.
2. Commit with clear messages.
3. Open PR to main.
4. Require review and CI pass before merge.
5. Prefer squash merge to keep history concise.

## Commit Message Style
- feat: new feature
- fix: bug fix
- chore: maintenance
- docs: documentation
- refactor: code refactor
- perf: performance update
- test: tests
