#!/bin/bash

export FG_ROOT=/usr/share/games/flightgear
export PATH=$PATH:$FG_ROOT/../bin
export FG_SCENERY=$FG_ROOT/Scenery:$FG_ROOT/WorldScenery

fgfs --fdm=null --native-fdm=socket,in,30,localhost,5502,udp   --prop:/sim/rendering/shaders/quality-level=0 --aircraft=c172p --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --airport=LIME --runway=28 --altitude=15000 --heading=0 --offset-distance=4.72 --offset-azimuth=0   &
