@AGENTS.md

## Claude Code specifics

Shared rules live in `AGENTS.md` (imported above); edit that file, not this one, unless a rule only applies to Claude Code.

- Claude Code loads nested `CLAUDE.md` files automatically, but not nested `AGENTS.md` files. If a package folder has its own `AGENTS.md`, read it before editing files there.
- Use plan mode, and get the human's approval of the plan, before any change that touches more than one package, a topic name, or a file in `src/cross_pkg_messages_urc/msg/`.
- After `/compact`, re-read the header of any file you are still editing before changing it again.
