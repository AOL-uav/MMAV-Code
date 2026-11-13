# Practice folder for the modelling walkthrough

Copies and variants only. Nothing here is read by any project result, so you cannot damage one.
Open `../index.html` in a browser for the explanations. Commands are PowerShell, run from this folder:

```powershell
cd "<MAV-2026>\Codes\docs\modeling_walkthrough\practice"
```

## Lesson 1: AVL by hand (section 5)

```powershell
..\..\..\tools\avl\avl352.exe f45_bare.avl        # start AVL interactively; type PLOP, G, Enter, Enter, OPER, ... (see lesson1.in)
Get-Content lesson1.in | ..\..\..\tools\avl\avl352.exe f45_bare.avl > my_screen.txt    # the same session from a file
```

`lesson1_screen.txt` is the output of that second command, saved when the walkthrough was written.
Always end a hand session with a blank line (back to the top menu) and `QUIT`; ending by closing the window gives a harmless Fortran "End of file" message.

## Lesson 2 and the checks: scripted variants (sections 6 and 7)

```powershell
py -B run_case.py f45_bare.avl --cl 0.3               # one case: prints alpha, CL, Xnp, Cm, Clb, Cnb, e, static margin
py -B run_case.py f45_bare.avl --cl 0.3 --xcg 0.040   # same, reference point moved to x = 40 mm
py -B run_case.py variants\tail_bend05.avl --alpha 0  # the bent-tail example
py -B lesson2_batch.py                                # every variant of the walkthrough, about a minute; rewrites variants\ and lesson2_results.txt
py -B ..\build_walkthrough.py                         # rebuilds ..\index.html from the files above (quotes are read from them)
```

## Files

| file | what it is |
|---|---|
| `f45_bare.avl`, `f45_fins.avl` | the F45 flat-test case (250 mm span, sweep 25 deg), bare and with tip fins; copies from `results/runs/rigid_glider/flat_test_v2/avl_work/F45_span250/`, title line changed |
| `lesson1.in` | the keystrokes of lesson 1 |
| `lesson1_screen.txt` | the real screen output of lesson 1 |
| `run_case.py` | small stdlib-only script that types the keystrokes and reads the numbers |
| `review_checks.py`, `review_checks_results.txt` | added after review: the CG move at one operating point (CL 0.3), and the NACA 0000 / 0006 / 0012 thickness check (decks `variants/naca*.avl`); `py -B review_checks.py`, about 10 s |
| `lesson2_batch.py`, `lesson2_results.txt`, `variants/` | the sweep / fins / tail-bend exercises, the panel-count and known-case checks, and the deliberate mistakes |
