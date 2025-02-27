# tkinter package
---------------
The tkinter package (“Tk interface”) is the standard Python interface to the Tcl/Tk GUI toolkit.

Both Tk and tkinter are available on most Unix platforms, including macOS, as well as on Windows systems.
 * https://docs.python.org/3/library/tkinter.html

This sample also requires the Tix Extension widgets for Tk.
 * https://docs.python.org/3/library/tkinter.tix.html

# Installation

Warning: 
 * By default, Tk and Tix packages are not installed on Unix platforms.
 * The Tix Tk extension is unmaintained, and the tkinter.tix wrapper module is deprecated in favor of tkinter.ttk.

## Ubuntu (20.04)
### Python version 2: 
```
sudo apt install tk tix python3-tk
```
### Python version 3:
```
sudo apt install tk tix python-tk
```

## Arch Linux (2023.12.01)
Tix is deprecated but is available via AUR packages.
### Python version 3:
```
sudo pacman -S tk
git clone https://aur.archlinux.org/tix.git
cd tix
makepkg
sudo pacman -U tix-8.4.3-5-x86_64.pkg.tar.zst
```