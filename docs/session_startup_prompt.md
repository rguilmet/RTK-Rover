# Session Startup Prompt: RTK Rover Decoupling & Cadastral Engine

This document contains the exact startup prompt and briefing context to initialize your next AI pair-programming session.

---

## 📋 Copy & Paste Startup Prompt

Copy and paste the text block below as your very first message in the new session:

```text
Hello Antigravity. We are continuing development on the RTK Rover project. 

Please read and anchor your context on the following key project documents before taking any action:
1. GEMINI.md (project rules, versioning, semver standards, and software engineering principles)
2. docs/hyper-context-anchor.md (core technical architecture, FreeRTOS/LVGL constraints, and hard roadblocks)
3. docs/session_handoff.md (detailed recent milestones and hardware states)
4. docs/architecture_plan_public_private_decoupling.md (approved master plan for Option B Sibling Directory)

Current State Summary:
- Git repo is clean on master branch at commit 0e6eccd.
- Git commit history has been 100% sanitized of personal tokens/coordinates and force-pushed to GitHub.
- Pre-push security verification hook is active in .git/hooks/pre-push.
- Sibling repository model (Option B) was approved: public RTK_Rover will contain generic schema-driven tools; private RTK_Rover_Private will contain real property deeds and survey coordinates under local Git version control.

Our immediate task:
Execute Phase 1 of docs/architecture_plan_public_private_decoupling.md:
1. Design and parameterize scripts/check_secrets.py so all generic security checks remain public while personal search tokens are loaded dynamically from an unversioned .secrets.local file.
2. Add .secrets.local to .gitignore and provide sample.secrets.local.
3. Update .git/hooks/pre-push and scripts/install_git_hooks.py to call scripts/check_secrets.py.
4. Verify 100% clean scan and zero leakage.

Once Phase 1 is verified, we will proceed directly into Phase 2 (setting up ../RTK_Rover_Private and migrating reference/ assets).

Please confirm you have read the documents, summarize your understanding, and begin Phase 1 execution.
```

---

## 🧭 Context Brief for the Incoming Agent

### Key Technical Coordinates
* **Working Directory:** `./`
* **Sibling Target:** `../RTK_Rover_Private` (to be created in Phase 2)
* **Firmware Version:** `v0.2.54` (`ROVER_E220_PROFILE_REV = 2`)
* **Base Station Software:** `e220_pacer.py v2.8.1` on `pi4b-rtkbase` (`tcp://127.0.0.1:5016` filtered to `1006(5),1074,1084,1094,1230(10)`)
* **Radio Configuration:** Channel `75` (`925.125 MHz`), `115,200 baud`, `38.4 kbps` air data rate, `200B` subpackets, `21 dBm` (Level 3), Transparent Mode 0.

### Critical Behavioral Rules
* **Strict Code vs. Data Decoupling:** Never hardcode personal names, amateur radio callsigns, homestead coordinates, or specific deed calls into public Python scripts or documentation.
* **Pure ASCII Requirement:** Ensure all UI labels and documentation use pure ASCII (`+/-`, `deg`, `->`).
* **Zero Truncation Rule:** Provide complete drop-in file implementations without placeholders (`// ...`).
* **Living Documentation Sync:** Keep `project_overview.md`, `session_handoff.md`, `hyper-context-anchor.md`, `GEMINI.md`, and `README.md` synchronized across changes.
