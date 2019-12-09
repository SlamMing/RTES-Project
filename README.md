# RTES-Project
PROGETTO REAL TIME EMBEDDED SYSTEMS #3
	Car​ ​race.​ ​Simulate​ ​a​ ​car​ ​race​ ​with​ ​N​ ​cars​ ​on​ ​a​ ​circuit​ ​that​ ​can​ ​be​ ​drawn​ ​using​ ​a
	mouse​ ​and​ ​saved​ ​in​ ​a​ ​file.​ ​N-1​ ​cars​ ​must​ ​be​ ​autonomous​ ​(hence​ ​equipped​ ​by
	sensors​ ​that​ ​detect​ ​curves​ ​and​ ​other​ ​cars)​ ​and​ ​one​ ​is​ ​controlled​ ​by​ ​the​ ​player
	through​ ​the​ ​keyboard​ ​arrows.

	HOW TO PLAY:
	1) drawing phase: using the mouse you can drag and draw a road from the starting
	point, making a loop as a track ensures the good functioning of the game; 
	you can press G to save the current track or you can press H and select an existing track file.
	2) racing phase: after finishing the track you can press ENTER and the race will 
	start immediately, you can accelerate or decelerate with respectively W and S 
	while you can turn the car using A for right turn and D for left turn.

	P.S.: it might take a while for the NN to learn how to drive, do not worry if they look
	stuck, they will eventually get over it, usually in under 2 minutes at least one of the cars completes
	any given track.

## Dependencies

To play this game you will need :
- g++ compiler
- [Allegro 5.1+](https://liballeg.org) GFX library
- pthreads support by your OS

## How to install Allegro
Depending on your distro, you have different ways to install Allegro:
- [Ubuntu with PPAs](https://wiki.allegro.cc/index.php?title=Install_Allegro_from_Ubuntu_PPAs) 
- [Git](https://wiki.allegro.cc/index.php?title=Install_Allegro5_From_Git/Linux/Debian)

other methods can be find [here](https://wiki.allegro.cc/index.php?title=Getting_Started)

## How to play

First of all you need to compile the game :
```bash
$ make clean
$ make Game
```

Then launch the game :
```bash
$ ./Game
```
If you are lacking privileges use **chmod**

### Keys
 - **LMB**(hold): draw the track.
 - **G**: save current track.
 - **H**: load track.
 - **ENTER**: starts the race.
 - **A/D**: steer car.
 - **W/S**: accelerate and decelerate.
