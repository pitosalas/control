Run a full checkpoint in this order:

- Run all tests
- Run `ruff check . && ruff format --check .`; fix any violations before continuing
- Update `02-doc/current.md` with your latest context: revise `## Snapshot`
  and `## Known Issues / Pending`/`## Likely Next Steps` for what changed
  this session; this project keeps one running handoff doc, no
  `02-doc/history.md` split
- Review changed code against judgment rules in @.claude/style_guide.md (design, error handling, naming semantics)
- Update any changed literate md files
- Create a commit (no need for a pull request)
- Push to github