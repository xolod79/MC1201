# [DVK MC1201](https://en.wikipedia.org/wiki/DVK) for [MiSTer Board](https://github.com/MiSTer-devel/Main_MiSTer/wiki).

Port of DVK MC1201 (PDP-11 compatible Soviet computer) [DVK-fpga core by forth32](https://github.com/forth32/dvk-fpga).

Based on [PDP-11 microprocessors reverse engineering](https://github.com/1801BM1/cpu11).

Virtual SDcard from files work.

[Disks image](https://github.com/xolod79/MC1201/blob/master/disk/initdisk.7z) write to SDcard [tools like](https://sourceforge.net/projects/win32diskimager/)

For boot OS in terminal press
B
RK0
or
DX0
or
MY0

# Quartus version
Cores must be developed in **Quartus v17.0.x**. It's recommended to have updates, so it will be **v17.0.2**. Newer versions won't give any benefits to FPGA used in MiSTer, however they will introduce incompatibilities in project settings and it will make harder to maintain the core and collaborate with others. **So please stick to good old 17.0.x version.** You may use either Lite or Standard license.

