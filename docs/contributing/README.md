# Adding and revising PCBs

For a new PCB concept, create a folder under `pcb/` named after the board. Add
a `README.md` explaining its purpose and what it connects to.

Inside the board folder, create an iteration folder named
`{BOARDNAME}_V{number}`, starting at `1`. Create the KiCad project inside that
iteration folder and keep its project files and custom libraries together.

For example:

```text
pcb/
  PiHat/
    README.md
    PiHat_V1/
      README.md
      PiHat.kicad_pro
      PiHat.kicad_sch
      PiHat.kicad_pcb
    PiHat_V2/
      README.md
      ...
```

The board README explains the overall purpose and links to its iterations.
Each iteration README briefly describes what changed from the previous version
and its status, such as in progress, tested, or failed.

For another iteration of the same board, add the next numbered folder inside
the existing board folder. For a different PCB concept, create a new board
folder. Add new boards to the [PCB overview](../../pcb/README.md).
