@echo off
echo AXF to BIN file generation ...

"C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe" --bin --output ..\DZ_logic\DZ_logic.bin ..\DZ_logic\DZ_logic.axf 

echo AXF to BIN file generation completed.

pause

