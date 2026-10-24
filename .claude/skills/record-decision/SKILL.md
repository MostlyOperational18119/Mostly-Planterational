---
name: record-decision
description: Record a notable programming decision in the team notebook (../Mostly-Noterational) and commit and push it. Use as soon as a design choice is settled in a session — an approach chosen over alternatives, an approach rejected, or a measured result that changes how the code works — without waiting to be asked.
---

# Record a programming decision

The team notebook is `../Mostly-Noterational` (GitHub `MostlyOperational18119/Mostly-Noterational`).
Its `CLAUDE.md` has the rules; follow them. This skill is the programming side of it.

## What counts

Record it if a teammate reading the portfolio would ask "why is it done this way?":

- an approach chosen over another (a measured speed table instead of a physics model; robot-relative
  tags instead of botpose; hold-to-shoot instead of toggle)
- an approach tried and dropped, with why
- a measurement that changed the design (the hood is fixed, so speed must vary with distance)

Not a decision: a bug fix with one obvious answer, a refactor, a re-tuned number. A re-tuned value
the robot now runs with goes in the day's log and the matching `robot/<subsystem>.md` table instead.

Only record what was actually decided in the session, with the reasons actually given. If the user
made the call, say so; if it was your recommendation that they accepted, say that.

## Write it

1. `decisions/YYYY-MM-DD-slug.md` from `templates/decision.md` — subteam "programming", the user's
   name only if they gave it. Link the code: file names, constants, commit or PR if there is one.
2. A **Programming** entry in `log/YYYY-MM-DD.md` (append; create from `templates/log.md`) pointing
   at the decision file.
3. If it changes a value or part listed in `robot/`, update that file and its "Last updated" line.

## Save it — automatically

The team has authorized committing and pushing notebook entries without asking:

```bash
cd ../Mostly-Noterational
git pull --rebase && git add -A && git commit -m "programming: <decision in a few words>" && git push
```

On a rejected push, `git pull --rebase` and push again; keep both entries in a `log/` conflict. This
authorization is for the notebook only — commits and pushes in this code repo still need the user to
ask.

Tell the user in one line what you recorded and where.

## If the notebook isn't there

In a cloud session (claude.ai/code on this repo) or a machine without it cloned alongside,
`../Mostly-Noterational` won't exist. Don't clone it yourself. Put the finished decision record in your
reply instead and say it still needs saving to the notebook.
